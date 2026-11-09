// 独立实验：同一 BGR 图像、项目 CUDA 预处理、只读引擎；不执行 DS 检测/NMS。
#include <NvInfer.h>
#include <cuda_runtime_api.h>
#include <dlfcn.h>
#include <fcntl.h>
#include <gst/app/gstappsrc.h>
#include <gst/gst.h>
#include <gstnvdsinfer.h>
#include <gstnvdsmeta.h>
#include <nvdspreprocess_meta.h>
#include <nvbufsurface.h>
#include <opencv2/imgcodecs.hpp>
#include <yolos/core/trt_session_base.hpp>
#include <yolos/core/cuda_preprocessing.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

namespace fs = std::filesystem;
using Clock = std::chrono::steady_clock;
using Time = Clock::time_point;
double ms(Time a, Time b) { return std::chrono::duration<double, std::milli>(b-a).count(); }
void require(bool condition, const std::string& message) {
  if (!condition) throw std::runtime_error(message);
}
void cudaCheck(cudaError_t status) {
  if (status != cudaSuccess) throw std::runtime_error(cudaGetErrorString(status));
}
std::string quote(const std::string& text) {
  std::string result = "\"";
  for (const unsigned char c : text) {
    if (c == '"' || c == '\\') { result += '\\'; result += c; }
    else if (c < 32) throw std::runtime_error("Control character in JSON string");
    else result += c;
  }
  return result + '"';
}
void checkGuard() {
  using Identity = const char* (*)();
  const auto identity = reinterpret_cast<Identity>(dlsym(RTLD_DEFAULT, "ds_compare_guard_identity"));
  const auto builder = dlsym(RTLD_DEFAULT, "createInferBuilder_INTERNAL");
  require(identity && builder, "Mandatory LD_PRELOAD guard is absent");
  require(std::string(identity()) == "deepstream_compare_no_engine_build_v1", "Wrong guard ABI");
  Dl_info guard_info{}, builder_info{};
  require(dladdr(reinterpret_cast<void*>(identity), &guard_info) &&
          dladdr(builder, &builder_info), "Cannot resolve loaded guard symbol origin");
  const char* expected = std::getenv("DS_COMPARE_GUARD_PATH");
  require(expected && fs::canonical(expected) == fs::canonical(guard_info.dli_fname) &&
          fs::canonical(expected) == fs::canonical(builder_info.dli_fname),
          "Builder symbol is not resolved to the expected preload library");
  void* trt = dlopen("libnvinfer.so.10", RTLD_NOW | RTLD_LOCAL);
  require(trt, "Cannot inspect loaded TensorRT 10 library");
  void* actual = dlsym(trt, "createInferBuilder_INTERNAL");
  require(actual && actual != builder, "Cannot verify the real TensorRT builder symbol");
  dlclose(trt);
  std::cerr << "Guard verified: " << guard_info.dli_fname << '\n';
}
struct Logger final : nvinfer1::ILogger {
  void log(Severity s, const char* message) noexcept override {
    if (s <= Severity::kWARNING) std::cerr << "TRT: " << message << '\n';
  }
};
struct Tensor { std::string name; std::vector<int> shape; size_t count=0; };
struct Model { Tensor input; std::vector<Tensor> outputs; };
Model inspect(const std::string& path) {
  std::ifstream input(path, std::ios::binary);
  require(input.good(), "Cannot read sealed engine");
  std::vector<char> bytes{std::istreambuf_iterator<char>(input), {}};
  Logger logger;
  std::unique_ptr<nvinfer1::IRuntime> runtime(nvinfer1::createInferRuntime(logger));
  require(bool(runtime), "createInferRuntime failed");
  std::unique_ptr<nvinfer1::ICudaEngine> engine(runtime->deserializeCudaEngine(bytes.data(),bytes.size()));
  require(bool(engine), "Existing engine failed deserialization; no fallback is permitted");
  Model model; unsigned inputs=0;
  for (int i=0;i<engine->getNbIOTensors();++i) {
    const char* name=engine->getIOTensorName(i);
    require(engine->getTensorDataType(name)==nvinfer1::DataType::kFLOAT &&
            engine->getTensorFormat(name)==nvinfer1::TensorFormat::kLINEAR &&
            engine->getTensorLocation(name)==nvinfer1::TensorLocation::kDEVICE,
            "Experiment requires static FLOAT/LINEAR device tensors");
    Tensor tensor; tensor.name=name; tensor.count=1;
    const auto dims=engine->getTensorShape(name);
    for (int d=0;d<dims.nbDims;++d) {
      require(dims.d[d]>0, "Dynamic engine is outside this experiment");
      tensor.shape.push_back(dims.d[d]); tensor.count*=dims.d[d];
    }
    if (engine->getTensorIOMode(name)==nvinfer1::TensorIOMode::kINPUT) {
      model.input=tensor; ++inputs;
    } else model.outputs.push_back(tensor);
  }
  require(inputs==1 && model.input.shape.size()==4 && model.input.shape[0]==1 &&
          model.input.shape[1]==3 && model.outputs.size()==2,
          "Expected existing batch-1 segmentation engine (1 input, 2 outputs)");
  return model;
}
using Values=std::map<std::string,std::vector<float>>;
struct Result { double raw_ready_ms=0,completed_ms=0; Values tensors; };
class Direct final : public yolos::TrtSessionBase {
 public:
  explicit Direct(const std::string& engine) : TrtSessionBase(engine,-1,10) {}
  Result run(const cv::Mat& image,bool capture) {
    const auto start=Clock::now();
    inferGpu(image);  // 真正生产依赖的 H2D、CUDA letterbox、CUDA graph、D2H。
    const auto ready=Clock::now();
    Result result; result.raw_ready_ms=ms(start,ready); result.completed_ms=result.raw_ready_ms;
    if (capture) {
      for (const auto& tensor:outputTensors_) {
        const float* data=tensor.hostBuf.as<float>();
        result.tensors[tensor.name]={data,data+tensor.bytes/sizeof(float)};
      }
      auto& tensor=result.tensors["__input__"]; tensor.resize(inputTensors_[0].bytes/sizeof(float));
      cudaCheck(cudaMemcpy(tensor.data(),inputTensors_[0].deviceBuf.data(),
                           inputTensors_[0].bytes,cudaMemcpyDeviceToHost));
    }
    return result;
  }
};

class DeepStream {
 public:
  DeepStream(const std::string& engine,const Model& model,cv::Size size,const fs::path& output)
      : model_(model),size_(size) {
    cudaCheck(cudaStreamCreate(&stream_));
    const size_t bytes=static_cast<size_t>(size_.width)*size_.height*3;
    cudaCheck(cudaMallocHost(&host_src_,bytes));
    cudaCheck(cudaMalloc(&device_src_,bytes));
    cudaCheck(cudaMalloc(&tensor_,model_.input.count*sizeof(float)));
    NvBufSurfaceCreateParams params{};
    params.gpuId=0; params.width=size_.width; params.height=size_.height;
    params.colorFormat=NVBUF_COLOR_FORMAT_RGBA; params.layout=NVBUF_LAYOUT_PITCH;
    params.memType=NVBUF_MEM_DEFAULT;
    require(NvBufSurfaceCreate(&surface_,1,&params)==0,"NvBufSurfaceCreate failed");
    surface_->numFilled=1;
    // tensor-input 模式只使用这份真实 NVMM surface 的尺寸/批次元数据，不读它的像素。
    const auto config=output/"nvinfer.ini";
    std::ofstream file(config);
    file << "[property]\ngpu-id=0\nbatch-size=1\ninterval=0\nprocess-mode=1\nnetwork-type=100\n"
            "input-tensor-from-meta=1\noutput-tensor-meta=0\ngie-unique-id=1\nnetwork-mode=2\n";
    file.close();
    pipeline_=gst_pipeline_new("existing-engine-comparison");
    source_=gst_element_factory_make("appsrc","source");
    infer_=gst_element_factory_make("nvinfer","inference");
    sink_=gst_element_factory_make("fakesink","completed");
    require(pipeline_ && source_ && infer_ && sink_,"Required GStreamer elements missing");
    GstCaps* caps=gst_caps_new_simple("video/x-raw","format",G_TYPE_STRING,"RGBA",
        "width",G_TYPE_INT,size_.width,"height",G_TYPE_INT,size_.height,
        "framerate",GST_TYPE_FRACTION,0,1,nullptr);
    gst_caps_set_features(caps,0,gst_caps_features_new("memory:NVMM",nullptr));
    g_object_set(source_,"caps",caps,"format",GST_FORMAT_TIME,"is-live",TRUE,
                 "block",TRUE,"max-buffers",static_cast<guint64>(1),nullptr);
    gst_caps_unref(caps);
    // 7.1 的 INI 路径解析会 realpath()，不能用于匿名 memfd；属性 setter 原样保留 fd 路径。
    g_object_set(infer_,"model-engine-file",engine.c_str(),"config-file-path",config.c_str(),
                 "input-tensor-meta",TRUE,
                 "raw-output-generated-callback",reinterpret_cast<gpointer>(&outputCallback),
                 "raw-output-generated-userdata",this,nullptr);
    g_object_set(sink_,"sync",FALSE,"async",FALSE,"signal-handoffs",TRUE,nullptr);
    g_signal_connect(sink_,"handoff",G_CALLBACK(handoff),this);
    gst_bin_add_many(GST_BIN(pipeline_),source_,infer_,sink_,nullptr);
    require(gst_element_link_many(source_,infer_,sink_,nullptr),"Cannot link nvinfer pipeline");
    bus_=gst_element_get_bus(pipeline_);
    require(gst_element_set_state(pipeline_,GST_STATE_PLAYING)!=GST_STATE_CHANGE_FAILURE,
            "nvinfer startup failed; inspect preserved log");
    pollBus();
    // INI 键 input-tensor-from-meta 与 GObject 属性 input-tensor-meta 名称不同。
    // 核对实际生效值，避免未知配置键被 DS 忽略后静默走 surface 默认预处理。
    gboolean uses_tensor_meta=FALSE;
    guint batch_size=0,interval=1;
    g_object_get(infer_,"input-tensor-meta",&uses_tensor_meta,
                 "batch-size",&batch_size,"interval",&interval,nullptr);
    require(uses_tensor_meta && batch_size==1 && interval==0,
            "nvinfer effective settings must be input-tensor-meta=true, batch-size=1, interval=0");
    std::ofstream(output/"effective-nvinfer.json")
        << "{\"input_tensor_meta\":true,\"batch_size\":" << batch_size
        << ",\"interval\":" << interval << ",\"preprocessing\":\"external_project_cuda\"}\n";
    std::cerr << "Verified nvinfer: external tensor metadata, batch=1, interval=0\n";
  }
  ~DeepStream() {
    if (pipeline_) { gst_element_set_state(pipeline_,GST_STATE_NULL); gst_object_unref(pipeline_); }
    if (bus_) gst_object_unref(bus_);
    if (surface_) NvBufSurfaceDestroy(surface_);
    if (tensor_) cudaFree(tensor_);
    if (device_src_) cudaFree(device_src_);
    if (host_src_) cudaFreeHost(host_src_);
    if (stream_) cudaStreamDestroy(stream_);
  }
  Result run(const cv::Mat& image,bool capture) {
    require(image.size()==size_,"Ordered images must have identical source dimensions");
    {
      std::lock_guard<std::mutex> lock(mutex_);
      capture_=capture; completed_=false; callback_seen_=false; failure_.clear(); values_.clear();
    }
    const auto start=Clock::now();
    const size_t bytes=image.total()*image.elemSize();
    std::memcpy(host_src_,image.data,bytes);
    cudaCheck(cudaMemcpyAsync(device_src_,host_src_,bytes,cudaMemcpyHostToDevice,stream_));
    yolos::cuda::letterboxPreprocess(static_cast<const uint8_t*>(device_src_),
        image.cols,image.rows,static_cast<float*>(tensor_),
        model_.input.shape[3],model_.input.shape[2],stream_);
    cudaCheck(cudaGetLastError());
    // DS 使用自己的流；交接前同步，不能把尚未就绪的张量交给另一个流。
    cudaCheck(cudaStreamSynchronize(stream_));
    GstBuffer* buffer=gst_buffer_new_wrapped_full(GST_MEMORY_FLAG_READONLY,surface_,
        sizeof(NvBufSurface),0,sizeof(NvBufSurface),nullptr,nullptr);
    require(buffer,"Cannot allocate GstBuffer");
    GST_BUFFER_PTS(buffer)=++frame_*GST_MSECOND; GST_BUFFER_DURATION(buffer)=GST_MSECOND;
    NvDsBatchMeta* batch=nvds_create_batch_meta(1);
    require(batch,"Cannot allocate batch meta");
    NvDsMeta* meta=gst_buffer_add_nvds_meta(buffer,batch,nullptr,
        nvds_batch_meta_copy_func,nvds_batch_meta_release_func);
    require(meta,"Cannot attach batch meta");
    meta->meta_type=NVDS_BATCH_GST_META;
    NvDsFrameMeta* fm=nvds_acquire_frame_meta_from_pool(batch);
    require(fm,"Cannot allocate frame meta");
    fm->batch_id=0; fm->pad_index=0; fm->source_id=0; fm->frame_num=frame_;
    fm->source_frame_width=image.cols; fm->source_frame_height=image.rows;
    fm->buf_pts=GST_BUFFER_PTS(buffer); fm->num_surfaces_per_frame=1;
    nvds_add_frame_meta_to_batch(batch,fm);
    auto* prepared=new GstNvDsPreProcessBatchMeta{};
    prepared->target_unique_ids={1}; prepared->tensor_meta=new NvDsPreProcessTensorMeta{};
    auto& tm=*prepared->tensor_meta;
    tm.raw_tensor_buffer=tensor_; tm.buffer_size=model_.input.count*sizeof(float);
    tm.tensor_shape=model_.input.shape; tm.data_type=NvDsDataType_FP32;
    tm.tensor_name=model_.input.name; tm.gpu_id=0; tm.meta_id=0; tm.maintain_aspect_ratio=TRUE;
    NvDsRoiMeta roi{}; roi.frame_meta=fm; roi.roi.width=image.cols; roi.roi.height=image.rows;
    roi.scale_ratio_x=roi.scale_ratio_y=std::min(
        static_cast<float>(model_.input.shape[3])/image.cols,
        static_cast<float>(model_.input.shape[2])/image.rows);
    roi.offset_left=std::round((model_.input.shape[3]-std::round(image.cols*roi.scale_ratio_x))/2.0-0.1);
    roi.offset_top=std::round((model_.input.shape[2]-std::round(image.rows*roi.scale_ratio_y))/2.0-0.1);
    prepared->roi_vector.push_back(roi);
    NvDsUserMeta* user=nvds_acquire_user_meta_from_pool(batch);
    require(user,"Cannot allocate tensor user meta");
    user->user_meta_data=prepared; user->base_meta.meta_type=NVDS_PREPROCESS_BATCH_META;
    user->base_meta.release_func=releasePreprocess;
    nvds_add_user_meta_to_batch(batch,user);
    require(gst_app_src_push_buffer(GST_APP_SRC(source_),buffer)==GST_FLOW_OK,"appsrc rejected frame");
    std::unique_lock<std::mutex> lock(mutex_);
    const auto deadline=start+std::chrono::seconds(30);
    while (!completed_) {
      condition_.wait_for(lock,std::chrono::milliseconds(50));
      lock.unlock(); pollBus(); lock.lock();
      require(Clock::now()<deadline,"Timed out waiting for nvinfer frame; no fallback");
    }
    require(failure_.empty(),failure_);
    require(callback_seen_,"nvinfer completed without raw output; interval or metadata is wrong");
    Result result; result.raw_ready_ms=ms(start,ready_); result.completed_ms=ms(start,done_);
    result.tensors=std::move(values_);
    lock.unlock();
    if (capture) {
      auto& input=result.tensors["__input__"]; input.resize(model_.input.count);
      cudaCheck(cudaMemcpy(input.data(),tensor_,input.size()*sizeof(float),cudaMemcpyDeviceToHost));
    }
    return result;
  }
 private:
  static void releasePreprocess(gpointer data,gpointer) {
    auto* user=static_cast<NvDsUserMeta*>(data);
    auto* prepared=static_cast<GstNvDsPreProcessBatchMeta*>(user->user_meta_data);
    if (prepared) { delete prepared->tensor_meta; delete prepared; }
  }
  static void outputCallback(GstBuffer* buffer,NvDsInferNetworkInfo*,NvDsInferLayerInfo* layers,
                             guint count,guint batch_size,gpointer data) noexcept {
    auto& self=*static_cast<DeepStream*>(data);
    const auto ready=Clock::now();
    std::lock_guard<std::mutex> lock(self.mutex_);
    try {
      require(!self.callback_seen_ && batch_size==1 &&
          GST_BUFFER_PTS(buffer)==self.frame_*GST_MSECOND,"Unexpected frame pairing/batch/repeated output");
      self.callback_seen_=true; self.ready_=ready;
      unsigned outputs=0;
      for (guint i=0;i<count;++i) if (!layers[i].isInput) {
        const auto& layer=layers[i]; ++outputs;
        const auto found=std::find_if(self.model_.outputs.begin(),self.model_.outputs.end(),
            [&](const Tensor& t){return t.name==layer.layerName;});
        require(found!=self.model_.outputs.end() && layer.dataType==FLOAT && layer.buffer &&
                layer.inferDims.numElements==found->count,"Output tensor metadata mismatch");
        if (self.capture_) {
          const auto* values=static_cast<const float*>(layer.buffer);
          self.values_[found->name]={values,values+found->count};
        }
      }
      require(outputs==self.model_.outputs.size(),"Missing output tensors");
    } catch (const std::exception& e) { self.failure_=e.what(); }
    catch (...) { self.failure_="Unknown output callback exception"; }
  }
  static void handoff(GstElement*,GstBuffer*,GstPad*,gpointer data) noexcept {
    auto& self=*static_cast<DeepStream*>(data);
    std::lock_guard<std::mutex> lock(self.mutex_);
    self.done_=Clock::now(); self.completed_=true; self.condition_.notify_all();
  }
  void pollBus() {
    while (GstMessage* msg=gst_bus_pop_filtered(bus_,
        static_cast<GstMessageType>(GST_MESSAGE_ERROR|GST_MESSAGE_EOS))) {
      std::string message="Unexpected pipeline EOS";
      if (GST_MESSAGE_TYPE(msg)==GST_MESSAGE_ERROR) {
        GError* error=nullptr; gchar* debug=nullptr;
        gst_message_parse_error(msg,&error,&debug);
        message=error?error->message:"Unknown GStreamer error";
        if (debug) message+="; "+std::string(debug);
        if (error) g_error_free(error);
        g_free(debug);
      }
      gst_message_unref(msg);
      throw std::runtime_error(message);
    }
  }
  Model model_; cv::Size size_;
  cudaStream_t stream_{}; void* host_src_{}; void* device_src_{}; void* tensor_{};
  NvBufSurface* surface_{};
  GstElement *pipeline_{},*source_{},*infer_{},*sink_{}; GstBus* bus_{};
  std::mutex mutex_; std::condition_variable condition_;
  bool capture_=false,completed_=false,callback_seen_=false;
  uint64_t frame_=0; Time ready_,done_; Values values_; std::string failure_;
};
void writeModel(const Model& model,const fs::path& path) {
  std::ofstream out(path);
  out << "{\"input_name\":" << quote(model.input.name) << ",\"tensors\":[";
  std::vector<Tensor> tensors=model.outputs;
  Tensor input=model.input; input.name="__input__"; tensors.push_back(input);
  bool first=true;
  for (const auto& tensor:tensors) {
    if (!first) out << ','; first=false;
    out << "{\"name\":" << quote(tensor.name) << ",\"count\":" << tensor.count << ",\"shape\":[";
    for (size_t i=0;i<tensor.shape.size();++i) { if (i) out << ','; out << tensor.shape[i]; }
    out << "]}";
  }
  out << "]}\n";
}
template<class Backend>
void execute(Backend& backend,const std::vector<cv::Mat>& images,const Model& model,
             const fs::path& output,unsigned warmup,double window_seconds) {
  for (unsigned i=0;i<warmup;++i) backend.run(images[i%images.size()],false);
  // 保存一遍有序输入的完整张量；此处写盘不计入后续性能窗口。
  std::ofstream captures(output/"captures.csv");
  captures << "frame,tensor_index,path\n";
  for (size_t f=0;f<images.size();++f) {
    const auto result=backend.run(images[f],true);
    std::vector<Tensor> tensors=model.outputs;
    Tensor input=model.input; input.name="__input__"; tensors.push_back(input);
    for (size_t i=0;i<tensors.size();++i) {
      const auto& values=result.tensors.at(tensors[i].name);
      require(values.size()==tensors[i].count,"Captured tensor size mismatch");
      const auto name="frame-"+std::to_string(f)+"-tensor-"+std::to_string(i)+".f32";
      std::ofstream binary(output/name,std::ios::binary);
      binary.write(reinterpret_cast<const char*>(values.data()),values.size()*sizeof(float));
      require(binary.good(),"Cannot save raw tensor");
      captures << f << ',' << i << ',' << name << '\n';
    }
  }
  captures.close();
  std::ofstream frames(output/"frames.csv"),windows(output/"windows.csv");
  frames << "window,sequence,image_index,raw_ready_ms,completed_ms\n" << std::setprecision(17);
  windows << "window,completed,wall_seconds,completed_per_second\n" << std::setprecision(17);
  for (unsigned w=0;w<3;++w) {
    struct Frame { size_t index; Result result; };
    std::vector<Frame> recorded;
    const auto begin=Clock::now();
    while (std::chrono::duration<double>(Clock::now()-begin).count()<window_seconds) {
      const size_t index=recorded.size()%images.size();
      recorded.push_back({index,backend.run(images[index],false)});
    }
    const double elapsed=std::chrono::duration<double>(Clock::now()-begin).count();
    windows << w << ',' << recorded.size() << ',' << elapsed << ',' << recorded.size()/elapsed << '\n';
    for (size_t i=0;i<recorded.size();++i)
      frames << w << ',' << i << ',' << recorded[i].index << ','
             << recorded[i].result.raw_ready_ms << ',' << recorded[i].result.completed_ms << '\n';
    frames.flush(); windows.flush();
  }
}
int main(int argc,char** argv) {
  try {
    checkGuard();
    if (argc==2 && std::string(argv[1])=="--guard-selftest") {
      createInferBuilder_INTERNAL(nullptr,NV_TENSORRT_VERSION);
      throw std::runtime_error("Builder guard selftest did not terminate");
    }
    require(argc>=7,"Usage: ds_compare direct|deepstream SEALED_ENGINE NEW_OUTPUT WARMUP WINDOW_SECONDS IMAGE...");
    const std::string mode=argv[1],engine=argv[2];
    const std::string fd_prefix="/proc/self/fd/";
    require(engine.rfind(fd_prefix,0)==0,"Use run.py: only a sealed engine descriptor is accepted");
    const int engine_fd=std::stoi(engine.substr(fd_prefix.size()));
    const int seals=fcntl(engine_fd,F_GET_SEALS);
    const int required_seals=F_SEAL_SEAL|F_SEAL_SHRINK|F_SEAL_GROW|F_SEAL_WRITE;
    require(seals>=0 && (seals&required_seals)==required_seals,"Engine descriptor is not immutable");
    const fs::path output=argv[3];
    require((mode=="direct" || mode=="deepstream") && fs::create_directory(output),
            "Invalid backend or output already exists");
    const unsigned warmup=std::stoul(argv[4]);
    const double seconds=std::stod(argv[5]);
    require(warmup>0 && std::isfinite(seconds) && seconds>0 && seconds<=300,"Invalid measurement bounds");
    std::vector<cv::Mat> images;
    for (int i=6;i<argc;++i) {
      auto image=cv::imread(argv[i],cv::IMREAD_COLOR);
      require(!image.empty() && image.type()==CV_8UC3 && image.isContinuous(),"Cannot read packed BGR image");
      if (!images.empty()) require(images[0].size()==image.size(),"Mixed source dimensions unsupported");
      images.push_back(image);
    }
    const auto model=inspect(engine);
    writeModel(model,output/"model.json");
    if (mode=="direct") {
      Direct backend(engine);
      execute(backend,images,model,output,warmup,seconds);
    } else {
      gst_init(nullptr,nullptr);
      DeepStream backend(engine,model,images[0].size(),output);
      execute(backend,images,model,output,warmup,seconds);
    }
    auto attempts=reinterpret_cast<unsigned(*)()>(dlsym(RTLD_DEFAULT,"ds_compare_guard_attempts"));
    require(attempts && attempts()==0,"Unexpected engine builder attempt");
    std::ofstream(output/"completed.json") << "{\"status\":\"completed\",\"engine_build_attempts\":0,"
        "\"model_only_gpu_ms\":null,\"model_only_gpu_ms_reason\":\"requires separate GPU profiling; "
        "wall latency includes preprocessing, dispatch and host output readiness\"}\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "FAILED: " << error.what() << '\n';
    return 2;
  }
}
