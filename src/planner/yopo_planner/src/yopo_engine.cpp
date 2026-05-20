#include "yopo_planner/yopo_engine.h"

#include <fstream>
#include <iostream>
#include <numeric>

namespace yopo_planner {

namespace {

bool checkCuda(cudaError_t status, const char* what) {
  if (status == cudaSuccess) return true;
  std::cerr << what << " failed: " << cudaGetErrorString(status) << std::endl;
  return false;
}

bool hasDynamicDim(const nvinfer1::Dims& dims) {
  for (int i = 0; i < dims.nbDims; ++i) {
    if (dims.d[i] < 0) return true;
  }
  return false;
}

}  // namespace

void TrtLogger::log(Severity severity, const char* msg) noexcept {
  if (severity > severity_) return;
  std::cerr << "[TensorRT] " << msg << std::endl;
}

YopoEngine::YopoEngine(Config config)
    : config_(std::move(config)), logger_(TrtLogger::Severity::kWARNING) {}

YopoEngine::~YopoEngine() { releaseDeviceBuffers(); }

bool YopoEngine::build() {
  if (deserializeEngine()) {
    std::cout << "Loaded YOPO TensorRT engine: " << config_.engine_file << std::endl;
    return prepareBindings();
  }

  std::cout << "Failed to load YOPO engine, building from ONNX: " << config_.onnx_file << std::endl;
  if (!buildFromOnnx()) return false;
  if (!saveEngine()) return false;
  return prepareBindings();
}

bool YopoEngine::infer(const std::array<float, 1 * 1 * 96 * 160>& depth,
                       const std::array<float, 1 * 9 * 3 * 5>& obs,
                       std::array<float, 1 * 9 * 3 * 5>* endstate,
                       std::array<float, 1 * 3 * 5>* score) {
  if (!context_ || !endstate || !score) return false;

  if (!checkCuda(cudaMemcpy(depth_device_, depth.data(), depth.size() * sizeof(float),
                            cudaMemcpyHostToDevice),
                 "cudaMemcpy depth H2D")) {
    return false;
  }
  if (!checkCuda(cudaMemcpy(obs_device_, obs.data(), obs.size() * sizeof(float),
                            cudaMemcpyHostToDevice),
                 "cudaMemcpy obs H2D")) {
    return false;
  }

  if (!context_->executeV2(device_bindings_.data())) {
    std::cerr << "YOPO TensorRT executeV2 failed" << std::endl;
    return false;
  }

  if (!checkCuda(cudaMemcpy(endstate->data(), endstate_device_,
                            endstate->size() * sizeof(float), cudaMemcpyDeviceToHost),
                 "cudaMemcpy endstate D2H")) {
    return false;
  }
  if (!checkCuda(cudaMemcpy(score->data(), score_device_, score->size() * sizeof(float),
                            cudaMemcpyDeviceToHost),
                 "cudaMemcpy score D2H")) {
    return false;
  }
  return true;
}

bool YopoEngine::deserializeEngine() {
  if (!fileExists(config_.engine_file)) return false;

  std::ifstream file(config_.engine_file, std::ios::binary);
  if (!file) return false;
  file.seekg(0, std::ifstream::end);
  const size_t size = static_cast<size_t>(file.tellg());
  file.seekg(0, std::ifstream::beg);
  std::vector<char> data(size);
  file.read(data.data(), size);

  TrtUniquePtr<nvinfer1::IRuntime> runtime{nvinfer1::createInferRuntime(logger_)};
  if (!runtime) return false;
  engine_ = std::shared_ptr<nvinfer1::ICudaEngine>(
      runtime->deserializeCudaEngine(data.data(), data.size()), InferDeleter());
  return static_cast<bool>(engine_);
}

bool YopoEngine::buildFromOnnx() {
  if (!fileExists(config_.onnx_file)) {
    std::cerr << "YOPO ONNX file does not exist: " << config_.onnx_file << std::endl;
    return false;
  }

  TrtUniquePtr<nvinfer1::IBuilder> builder{nvinfer1::createInferBuilder(logger_)};
  if (!builder) return false;

  const auto explicit_batch =
      1U << static_cast<uint32_t>(nvinfer1::NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);
  TrtUniquePtr<nvinfer1::INetworkDefinition> network{builder->createNetworkV2(explicit_batch)};
  if (!network) return false;

  TrtUniquePtr<nvinfer1::IBuilderConfig> config{builder->createBuilderConfig()};
  if (!config) return false;

  TrtUniquePtr<nvonnxparser::IParser> parser{nvonnxparser::createParser(*network, logger_)};
  if (!parser) return false;

  if (!parser->parseFromFile(config_.onnx_file.c_str(),
                             static_cast<int>(nvinfer1::ILogger::Severity::kWARNING))) {
    std::cerr << "Failed to parse YOPO ONNX: " << config_.onnx_file << std::endl;
    for (int i = 0; i < parser->getNbErrors(); ++i) {
      std::cerr << parser->getError(i)->desc() << std::endl;
    }
    return false;
  }

  if (network->getNbInputs() != 2 || network->getNbOutputs() != 2) {
    std::cerr << "Unexpected YOPO ONNX IO count. inputs=" << network->getNbInputs()
              << " outputs=" << network->getNbOutputs() << std::endl;
    return false;
  }

  nvinfer1::IOptimizationProfile* profile = builder->createOptimizationProfile();
  if (!profile) return false;

  const nvinfer1::Dims4 depth_dims{1, 1, 96, 160};
  const nvinfer1::Dims4 obs_dims{1, 9, 3, 5};
  auto set_profile_dims = [&](const std::string& name, const nvinfer1::Dims& dims) -> bool {
    return profile->setDimensions(name.c_str(), nvinfer1::OptProfileSelector::kMIN, dims) &&
           profile->setDimensions(name.c_str(), nvinfer1::OptProfileSelector::kOPT, dims) &&
           profile->setDimensions(name.c_str(), nvinfer1::OptProfileSelector::kMAX, dims);
  };

  if (!set_profile_dims(config_.depth_input_name, depth_dims) ||
      !set_profile_dims(config_.obs_input_name, obs_dims)) {
    std::cerr << "Failed to set YOPO TensorRT optimization profile" << std::endl;
    return false;
  }
  if (!profile->isValid()) {
    std::cerr << "Invalid YOPO TensorRT optimization profile" << std::endl;
    return false;
  }
  config->addOptimizationProfile(profile);

  if (config_.fp16 && builder->platformHasFastFp16()) {
    config->setFlag(nvinfer1::BuilderFlag::kFP16);
  }

  TrtUniquePtr<nvinfer1::IHostMemory> plan{builder->buildSerializedNetwork(*network, *config)};
  if (!plan) return false;

  TrtUniquePtr<nvinfer1::IRuntime> runtime{nvinfer1::createInferRuntime(logger_)};
  if (!runtime) return false;
  engine_ = std::shared_ptr<nvinfer1::ICudaEngine>(
      runtime->deserializeCudaEngine(plan->data(), plan->size()), InferDeleter());
  return static_cast<bool>(engine_);
}

bool YopoEngine::saveEngine() const {
  if (!engine_) return false;
  TrtUniquePtr<nvinfer1::IHostMemory> serialized{engine_->serialize()};
  if (!serialized) return false;

  std::ofstream file(config_.engine_file, std::ios::binary);
  if (!file) {
    std::cerr << "Failed to open YOPO engine for writing: " << config_.engine_file << std::endl;
    return false;
  }
  file.write(static_cast<const char*>(serialized->data()), serialized->size());
  std::cout << "Saved YOPO TensorRT engine: " << config_.engine_file << std::endl;
  return true;
}

bool YopoEngine::prepareBindings() {
  if (!engine_) return false;

  context_ = std::shared_ptr<nvinfer1::IExecutionContext>(engine_->createExecutionContext(),
                                                          InferDeleter());
  if (!context_) return false;

  depth_binding_ = engine_->getBindingIndex(config_.depth_input_name.c_str());
  obs_binding_ = engine_->getBindingIndex(config_.obs_input_name.c_str());
  endstate_binding_ = engine_->getBindingIndex(config_.endstate_output_name.c_str());
  score_binding_ = engine_->getBindingIndex(config_.score_output_name.c_str());
  if (endstate_binding_ < 0) endstate_binding_ = engine_->getBindingIndex("endstate_pred");
  if (score_binding_ < 0) score_binding_ = engine_->getBindingIndex("score_pred");
  if (depth_binding_ < 0 || obs_binding_ < 0 || endstate_binding_ < 0 || score_binding_ < 0) {
    std::cerr << "Failed to find YOPO TensorRT bindings. Check ONNX tensor names." << std::endl;
    return false;
  }

  const nvinfer1::Dims4 depth_dims{1, 1, 96, 160};
  const nvinfer1::Dims4 obs_dims{1, 9, 3, 5};
  if (hasDynamicDim(engine_->getBindingDimensions(depth_binding_)) &&
      !context_->setBindingDimensions(depth_binding_, depth_dims)) {
    std::cerr << "Failed to set YOPO depth binding dimensions" << std::endl;
    return false;
  }
  if (hasDynamicDim(engine_->getBindingDimensions(obs_binding_)) &&
      !context_->setBindingDimensions(obs_binding_, obs_dims)) {
    std::cerr << "Failed to set YOPO obs binding dimensions" << std::endl;
    return false;
  }
  if (!context_->allInputDimensionsSpecified()) {
    std::cerr << "YOPO TensorRT input dimensions are not fully specified" << std::endl;
    return false;
  }

  const int nb = engine_->getNbBindings();
  device_bindings_.assign(nb, nullptr);

  auto check_binding = [&](int index, size_t expected_volume, const char* label) -> bool {
    const auto dims = context_->getBindingDimensions(index);
    const auto dtype = engine_->getBindingDataType(index);
    if (dtype != nvinfer1::DataType::kFLOAT) {
      std::cerr << label << " must be FP32 binding, got " << dtypeName(dtype) << std::endl;
      return false;
    }
    const size_t got = volume(dims);
    if (got != expected_volume) {
      std::cerr << label << " volume mismatch. got=" << got
                << " expected=" << expected_volume << std::endl;
      return false;
    }
    return true;
  };

  if (!check_binding(depth_binding_, 1 * 1 * 96 * 160, "depth")) return false;
  if (!check_binding(obs_binding_, 1 * 9 * 3 * 5, "obs")) return false;
  if (!check_binding(endstate_binding_, 1 * 9 * 3 * 5, "endstate")) return false;
  if (!check_binding(score_binding_, 1 * 3 * 5, "score")) return false;

  releaseDeviceBuffers();
  if (!checkCuda(cudaMalloc(&depth_device_, 1 * 1 * 96 * 160 * sizeof(float)),
                 "cudaMalloc depth")) {
    return false;
  }
  if (!checkCuda(cudaMalloc(&obs_device_, 1 * 9 * 3 * 5 * sizeof(float)), "cudaMalloc obs")) {
    return false;
  }
  if (!checkCuda(cudaMalloc(&endstate_device_, 1 * 9 * 3 * 5 * sizeof(float)),
                 "cudaMalloc endstate")) {
    return false;
  }
  if (!checkCuda(cudaMalloc(&score_device_, 1 * 3 * 5 * sizeof(float)), "cudaMalloc score")) {
    return false;
  }

  device_bindings_[depth_binding_] = depth_device_;
  device_bindings_[obs_binding_] = obs_device_;
  device_bindings_[endstate_binding_] = endstate_device_;
  device_bindings_[score_binding_] = score_device_;
  return true;
}

void YopoEngine::releaseDeviceBuffers() {
  if (depth_device_) cudaFree(depth_device_);
  if (obs_device_) cudaFree(obs_device_);
  if (endstate_device_) cudaFree(endstate_device_);
  if (score_device_) cudaFree(score_device_);
  depth_device_ = nullptr;
  obs_device_ = nullptr;
  endstate_device_ = nullptr;
  score_device_ = nullptr;
}

bool YopoEngine::fileExists(const std::string& path) {
  std::ifstream file(path);
  return file.good();
}

size_t YopoEngine::volume(const nvinfer1::Dims& dims) {
  size_t value = 1;
  for (int i = 0; i < dims.nbDims; ++i) {
    if (dims.d[i] < 0) return 0;
    value *= static_cast<size_t>(dims.d[i]);
  }
  return value;
}

const char* YopoEngine::dtypeName(nvinfer1::DataType dtype) {
  switch (dtype) {
    case nvinfer1::DataType::kFLOAT:
      return "float";
    case nvinfer1::DataType::kHALF:
      return "half";
    case nvinfer1::DataType::kINT8:
      return "int8";
    case nvinfer1::DataType::kINT32:
      return "int32";
    case nvinfer1::DataType::kBOOL:
      return "bool";
    default:
      return "unknown";
  }
}

}  // namespace yopo_planner
