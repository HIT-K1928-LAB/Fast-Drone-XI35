#pragma once
#include <NvInfer.h>
#include <array>
#include <memory>
#include <string>
#include <vector>
namespace yopo_minco_planner {
using Depth = std::array<float,15360>;
using Observation = std::array<float,135>;
struct NetworkOutput {
  std::array<float,210> endstate{};
  std::array<float,15> score{};
  std::array<float,300> radius{};
};
class TrtLogger : public nvinfer1::ILogger {
 public: void log(Severity severity,const char* msg) noexcept override;
};
class YopoEngine {
 public:
  ~YopoEngine();
  bool load(const std::string& path);
  bool infer(const Depth& depth,const Observation& obs,NetworkOutput* output);
 private:
  TrtLogger logger_;
  // Destruction must be context -> engine -> runtime.
  std::unique_ptr<nvinfer1::IRuntime> runtime_;
  std::unique_ptr<nvinfer1::ICudaEngine> engine_;
  std::unique_ptr<nvinfer1::IExecutionContext> context_;
  std::vector<void*> buffers_;
  std::array<int,5> indices_{{-1,-1,-1,-1,-1}};
  bool ready_=false;
};
}
