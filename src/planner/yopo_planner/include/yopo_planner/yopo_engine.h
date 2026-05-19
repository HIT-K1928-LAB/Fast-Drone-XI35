#ifndef YOPO_PLANNER_YOPO_ENGINE_H_
#define YOPO_PLANNER_YOPO_ENGINE_H_

#include <NvInfer.h>
#include <NvOnnxParser.h>
#include <cuda_runtime_api.h>

#include <array>
#include <memory>
#include <string>
#include <vector>

namespace yopo_planner {

class TrtLogger final : public nvinfer1::ILogger {
 public:
  explicit TrtLogger(Severity severity = Severity::kWARNING) : severity_(severity) {}
  void log(Severity severity, const char* msg) noexcept override;

 private:
  Severity severity_;
};

class YopoEngine {
 public:
  struct Config {
    std::string onnx_file;
    std::string engine_file;
    std::string depth_input_name = "depth";
    std::string obs_input_name = "obs";
    std::string endstate_output_name = "endstate";
    std::string score_output_name = "score";
    bool fp16 = true;
  };

  explicit YopoEngine(Config config);
  ~YopoEngine();

  bool build();
  bool infer(const std::array<float, 1 * 1 * 96 * 160>& depth,
             const std::array<float, 1 * 9 * 3 * 5>& obs,
             std::array<float, 1 * 9 * 3 * 5>* endstate,
             std::array<float, 1 * 3 * 5>* score);

 private:
  struct InferDeleter {
    template <typename T>
    void operator()(T* obj) const {
      if (obj) obj->destroy();
    }
  };

  template <typename T>
  using TrtUniquePtr = std::unique_ptr<T, InferDeleter>;

  bool deserializeEngine();
  bool buildFromOnnx();
  bool saveEngine() const;
  bool prepareBindings();
  void releaseDeviceBuffers();
  static bool fileExists(const std::string& path);
  static size_t volume(const nvinfer1::Dims& dims);
  static const char* dtypeName(nvinfer1::DataType dtype);

  Config config_;
  TrtLogger logger_;
  std::shared_ptr<nvinfer1::ICudaEngine> engine_;
  std::shared_ptr<nvinfer1::IExecutionContext> context_;

  int depth_binding_ = -1;
  int obs_binding_ = -1;
  int endstate_binding_ = -1;
  int score_binding_ = -1;
  std::vector<void*> device_bindings_;
  void* depth_device_ = nullptr;
  void* obs_device_ = nullptr;
  void* endstate_device_ = nullptr;
  void* score_device_ = nullptr;
};

}  // namespace yopo_planner

#endif  // YOPO_PLANNER_YOPO_ENGINE_H_
