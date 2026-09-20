#include "yopo_minco_planner/yopo_engine.h"
#include <cuda_runtime_api.h>
#include <fstream>
#include <iostream>
#include <iterator>
#include <cmath>
#include <algorithm>
namespace yopo_minco_planner {
void TrtLogger::log(Severity s,const char* msg) noexcept {
  if(s<=Severity::kWARNING) std::cerr<<"[TensorRT] "<<msg<<'\n';
}
static bool cudaOK(cudaError_t e) {
  if(e==cudaSuccess) return true;
  std::cerr<<"CUDA: "<<cudaGetErrorString(e)<<'\n'; return false;
}
YopoEngine::~YopoEngine() { for(void* p:buffers_) if(p) cudaFree(p); }
bool YopoEngine::load(const std::string& path) {
  if(engine_) return false; // One immutable engine per instance.
  std::ifstream f(path,std::ios::binary);
  if(!f) { std::cerr<<"Cannot read engine: "<<path<<'\n'; return false; }
  std::vector<char> bytes((std::istreambuf_iterator<char>(f)),{});
  runtime_.reset(nvinfer1::createInferRuntime(logger_));
  if(!runtime_) return false;
  engine_.reset(runtime_->deserializeCudaEngine(bytes.data(),bytes.size()));
  if(!engine_ || engine_->getNbBindings()!=5 || engine_->hasImplicitBatchDimension()) return false;
  context_.reset(engine_->createExecutionContext());
  if(!context_) return false;
  const char* names[]={"depth","obs","endstate","score","radius"};
  const std::vector<std::vector<int>> shapes={{1,1,96,160},{1,9,3,5},{1,14,3,5},{1,3,5},{1,20,3,5}};
  buffers_.assign(5,nullptr);
  for(int k=0;k<5;++k) {
    int i=engine_->getBindingIndex(names[k]); indices_[k]=i;
    if(i<0 || engine_->bindingIsInput(i)!=(k<2) || engine_->getBindingDataType(i)!=nvinfer1::DataType::kFLOAT ||
       engine_->getBindingFormat(i)!=nvinfer1::TensorFormat::kLINEAR) return false;
    auto d=engine_->getBindingDimensions(i);
    if(d.nbDims!=static_cast<int>(shapes[k].size())) return false;
    size_t n=1;
    for(int j=0;j<d.nbDims;++j) { if(d.d[j]!=shapes[k][j]) return false; n*=d.d[j]; }
    if(!cudaOK(cudaMalloc(&buffers_[i],n*sizeof(float)))) return false;
  }
  ready_=true; return true;
}
bool YopoEngine::infer(const Depth& depth,const Observation& obs,NetworkOutput* out) {
  if(!ready_ || !out) return false;
  for(float x:depth) if(!std::isfinite(x)) return false;
  for(float x:obs) if(!std::isfinite(x)) return false;
  if(!cudaOK(cudaMemcpy(buffers_[indices_[0]],depth.data(),sizeof(depth),cudaMemcpyHostToDevice)) ||
     !cudaOK(cudaMemcpy(buffers_[indices_[1]],obs.data(),sizeof(obs),cudaMemcpyHostToDevice)) ||
     !context_->executeV2(buffers_.data())) return false;
  if(!cudaOK(cudaMemcpy(out->endstate.data(),buffers_[indices_[2]],sizeof(out->endstate),cudaMemcpyDeviceToHost)) ||
     !cudaOK(cudaMemcpy(out->score.data(),buffers_[indices_[3]],sizeof(out->score),cudaMemcpyDeviceToHost)) ||
     !cudaOK(cudaMemcpy(out->radius.data(),buffers_[indices_[4]],sizeof(out->radius),cudaMemcpyDeviceToHost))) return false;
  for(float x:out->endstate) if(!std::isfinite(x)) return false;
  for(float x:out->score) if(!std::isfinite(x)) return false;
  for(float x:out->radius) if(!std::isfinite(x)) return false;
  return true;
}
}
