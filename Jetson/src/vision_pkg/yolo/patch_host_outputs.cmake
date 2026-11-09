# 固定归档展开后的应用层补丁：输出按需回传、后端失败拒绝交付、资源事务及图捕获回退。
# 完整函数匹配及幂等检查拒绝未知上游。
function(_vision_replace_exact variable original revised description)
  set(_text "${${variable}}")
  string(FIND "${_text}" "${original}" _old)
  string(FIND "${_text}" "${revised}" _new)
  if(NOT _old EQUAL -1 AND _new EQUAL -1)
    string(REPLACE "${original}" "${revised}" _text "${_text}")
  elseif(_old EQUAL -1 AND NOT _new EQUAL -1)
    # 已完整应用。
  else()
    message(FATAL_ERROR "Upstream ${description} does not match the pinned stability patch")
  endif()
  set(${variable} "${_text}" PARENT_SCOPE)
endfunction()

set(_session_header "${_upstream}/include/yolos/core/trt_session_base.hpp")
file(READ "${_session_header}" _session_text)
# H13：推理前处理只走 NVMM 融合核（yolo/nvmm_preprocess.cu）。删除上游 cv::Mat 入口 inferGpu
# （pinned memcpy + H2D + BGR letterbox kernel）及只为它服务的 pinned/device 暂存缓冲。
# 旧构建目录里已打过旧 inferGpu 补丁的头文件不再兼容，需要清空构建目录重新展开归档。
set(_removed_infer_gpu [=[
    // ====================================================================
    // GPU-accelerated full pipeline: cv::Mat → detect
    // ====================================================================

    /// @brief Upload raw BGR image to GPU, run CUDA letterbox kernel that
    ///        writes directly into TRT input buffer, then run inference
    ///        and copy outputs to host. Entire preprocess is on GPU.
    ///
    /// @param image  Input BGR cv::Mat (any size)
    void inferGpu(const cv::Mat& image) {
        const int srcH = image.rows;
        const int srcW = image.cols;
        const size_t srcBytes = static_cast<size_t>(srcH) * srcW * 3;

        // Ensure staging buffers are big enough
        ensureSrcCapacity(srcBytes);

        // 1. Copy raw BGR pixels host→device via pinned staging (truly async)
        std::memcpy(pinnedSrc_.data(), image.data, srcBytes);
        CUDA_CHECK(cudaMemcpyAsync(
            deviceSrc_.data(), pinnedSrc_.data(), srcBytes,
            cudaMemcpyHostToDevice, stream_.get()));

        // 2. CUDA letterbox+BGR→RGB+normalize → directly into TRT input buffer
        float* trtInput = static_cast<float*>(inputTensors_[0].deviceBuf.data());
        cuda::letterboxPreprocess(
            static_cast<const uint8_t*>(deviceSrc_.data()),
            srcW, srcH,
            trtInput,
            inputShape_.width, inputShape_.height,
            stream_.get());

        // 3. Inference (CUDA graph if available, else enqueueV3)
        if (graphCaptured_ && graphExec_) {
            CUDA_CHECK(cudaGraphLaunch(graphExec_, stream_.get()));
        } else {
            context_->enqueueV3(stream_.get());
        }

        // 4. D2H copy of outputs
        for (auto& out : outputTensors_) {
            CUDA_CHECK(cudaMemcpyAsync(
                out.hostBuf.data(), out.deviceBuf.data(), out.bytes,
                cudaMemcpyDeviceToHost, stream_.get()));
        }

        // 5. Single sync point
        stream_.synchronize();

        // 6. Cache scale/pad for postprocessing (computed on host, negligible)
        computeScalePad(srcW, srcH);
    }
]=])
_vision_replace_exact(_session_text "${_removed_infer_gpu}" "    // vision: cv::Mat inferGpu removed\n" "inferGpu")
set(_removed_staging [=[
    // ── GPU Preprocessing Buffers ───────────────────────────────────────

    void allocGpuPreprocessBuffers() {
        // Pre-allocate for a common input size (1080p = 1920x1080x3 ≈ 6MB)
        constexpr size_t INITIAL_CAPACITY = 1920 * 1080 * 3;
        maxSrcBytes_ = INITIAL_CAPACITY;
        pinnedSrc_ = trt::PinnedBuffer(maxSrcBytes_);
        deviceSrc_ = trt::DeviceBuffer(maxSrcBytes_);
    }

    void ensureSrcCapacity(size_t bytes) {
        if (bytes > maxSrcBytes_) {
            maxSrcBytes_ = bytes + bytes / 4; // 25% headroom
            pinnedSrc_ = trt::PinnedBuffer(maxSrcBytes_);
            deviceSrc_ = trt::DeviceBuffer(maxSrcBytes_);
        }
    }
]=])
_vision_replace_exact(_session_text "${_removed_staging}" "    // vision: host staging buffers removed\n" "GPU preprocess buffers")
set(_removed_members [=[
    // ── GPU Preprocessing Staging ───────────────────────────────────────
    trt::PinnedBuffer pinnedSrc_;     // pinned host buffer for raw BGR image
    trt::DeviceBuffer deviceSrc_;     // device buffer for raw BGR image
    size_t maxSrcBytes_{0};           // current allocation capacity
]=])
_vision_replace_exact(_session_text "${_removed_members}" "    // vision: staging members removed\n" "staging members")
_vision_replace_exact(_session_text "        allocGpuPreprocessBuffers();\n" "        // vision: no host staging\n" "staging allocation")
set(_original [=[
    void infer(const float* blob, size_t count) {
        // Copy from pinned input host buffer for speed
        TensorInfo& inp = inputTensors_[0];
        const size_t bytes = count * sizeof(float);
        assert(bytes <= inp.bytes);

        // Copy blob into pinned buffer first, then async H2D
        std::memcpy(inp.hostBuf.data(), blob, bytes);
        CUDA_CHECK(cudaMemcpyAsync(
            inp.deviceBuf.data(), inp.hostBuf.data(), bytes,
            cudaMemcpyHostToDevice, stream_.get()));

        if (graphCaptured_ && graphExec_) {
            CUDA_CHECK(cudaGraphLaunch(graphExec_, stream_.get()));
        } else {
            context_->enqueueV3(stream_.get());
        }

        for (auto& out : outputTensors_) {
            CUDA_CHECK(cudaMemcpyAsync(
                out.hostBuf.data(), out.deviceBuf.data(), out.bytes,
                cudaMemcpyDeviceToHost, stream_.get()));
        }
        stream_.synchronize();
    }
]=])
set(_revised [=[
    void infer(const float* blob, size_t count) {
        // Copy from pinned input host buffer for speed
        TensorInfo& inp = inputTensors_[0];
        const size_t bytes = count * sizeof(float);
        assert(bytes <= inp.bytes);

        // Copy blob into pinned buffer first, then async H2D
        std::memcpy(inp.hostBuf.data(), blob, bytes);
        CUDA_CHECK(cudaMemcpyAsync(
            inp.deviceBuf.data(), inp.hostBuf.data(), bytes,
            cudaMemcpyHostToDevice, stream_.get()));

        if (graphCaptured_ && graphExec_) {
            CUDA_CHECK(cudaGraphLaunch(graphExec_, stream_.get()));
        } else {
            ::vision::requireBackendSubmission(context_->enqueueV3(stream_.get()), "inferCpuBlob");
        }

        for (auto& out : outputTensors_) {
            CUDA_CHECK(cudaMemcpyAsync(
                out.hostBuf.data(), out.deviceBuf.data(), out.bytes,
                cudaMemcpyDeviceToHost, stream_.get()));
        }
        stream_.synchronize();
    }
]=])
_vision_replace_exact(_session_text "${_original}" "${_revised}" "inferCpuBlob")
set(_original [=[
    void warmUp(int runs) {
        if (runs <= 0) return;
        std::cout << "[INFO] Warming up (" << runs << " iterations)..." << std::endl;

        CUDA_CHECK(cudaMemsetAsync(
            inputTensors_[0].deviceBuf.data(), 0,
            inputTensors_[0].bytes, stream_.get()));

        for (int i = 0; i < runs; ++i) {
            context_->enqueueV3(stream_.get());
        }
        for (auto& out : outputTensors_) {
            CUDA_CHECK(cudaMemcpyAsync(
                out.hostBuf.data(), out.deviceBuf.data(), out.bytes,
                cudaMemcpyDeviceToHost, stream_.get()));
        }
        stream_.synchronize();
        std::cout << "[INFO] Warm-up complete." << std::endl;
    }
]=])
set(_revised [=[
    void warmUp(int runs) {
        if (runs <= 0) return;
        std::cout << "[INFO] Warming up (" << runs << " iterations)..." << std::endl;

        CUDA_CHECK(cudaMemsetAsync(
            inputTensors_[0].deviceBuf.data(), 0,
            inputTensors_[0].bytes, stream_.get()));

        for (int i = 0; i < runs; ++i) {
            ::vision::requireBackendSubmission(context_->enqueueV3(stream_.get()), "warmUp");
        }
        for (auto& out : outputTensors_) {
            CUDA_CHECK(cudaMemcpyAsync(
                out.hostBuf.data(), out.deviceBuf.data(), out.bytes,
                cudaMemcpyDeviceToHost, stream_.get()));
        }
        stream_.synchronize();
        std::cout << "[INFO] Warm-up complete." << std::endl;
    }
]=])
_vision_replace_exact(_session_text "${_original}" "${_revised}" "warmUp")
set(_original [=[
    void captureInferenceGraph() {
        // Only capture for fixed-shape, non-dynamic models
        if (isDynamicInputShape_ || isDynamicBatchSize_) {
            std::cout << "[INFO] Dynamic shapes — CUDA graph disabled." << std::endl;
            return;
        }

        CUDA_CHECK(cudaStreamBeginCapture(stream_.get(), cudaStreamCaptureModeGlobal));
        bool ok = context_->enqueueV3(stream_.get());
        CUDA_CHECK(cudaStreamEndCapture(stream_.get(), &graph_));

        if (ok && graph_) {
            CUDA_CHECK(cudaGraphInstantiate(&graphExec_, graph_, 0));
            graphCaptured_ = true;
            std::cout << "[INFO] CUDA graph captured — inference dispatch optimised." << std::endl;
        } else {
            std::cout << "[WARN] CUDA graph capture failed — using enqueueV3 fallback." << std::endl;
            if (graph_) { cudaGraphDestroy(graph_); graph_ = nullptr; }
        }
    }
]=])
set(_revised [=[
protected:
    // 捕获只是一项优化。仅明确的捕获不兼容允许回退，且必须先证实普通推理可用。
    // submit 为应用层捕获边界，合同使用真实 CUDA 操作触发捕获失效，不替换驱动 API。
    template<class Submit>
    bool tryCaptureInferenceGraph(Submit&& submit) {
        if (graph_ || graphExec_ || graphCaptured_) {
            throw ::vision::BackendError("CUDA graph capture requires an empty graph state");
        }
        struct CaptureResources {
            cudaStream_t stream;
            cudaGraph_t graph{nullptr};
            cudaGraphExec_t executable{nullptr};
            bool active{false};
            ~CaptureResources() noexcept {
                // 异常退出仍结束捕获；清理不能覆盖原来的 BackendError。
                if (active) {cudaStreamEndCapture(stream, &graph);}
                if (executable) {cudaGraphExecDestroy(executable);}
                if (graph) {cudaGraphDestroy(graph);}
            }
            void release() {
                // 回退前释放两份资源，收齐清理错误后再检查，任何失败都禁止普通提交。
                const auto exec_error = executable ? cudaGraphExecDestroy(executable) : cudaSuccess;
                executable = nullptr;
                const auto exec_pending = cudaGetLastError();
                const auto graph_error = graph ? cudaGraphDestroy(graph) : cudaSuccess;
                graph = nullptr;
                const auto graph_pending = cudaGetLastError();
                CUDA_CHECK(exec_error);
                CUDA_CHECK(exec_pending);
                CUDA_CHECK(graph_error);
                CUDA_CHECK(graph_pending);
            }
        } capture{stream_.get()};
        const auto capture_ready = [](bool complete, std::initializer_list<cudaError_t> errors) {
            bool unsupported = false;
            auto failure = cudaSuccess;
            for (const auto error : errors) {
                if (error == cudaErrorStreamCaptureUnsupported ||
                    error == cudaErrorStreamCaptureInvalidated || error == cudaErrorNotSupported) {
                    unsupported = true;
                } else if (error != cudaSuccess && failure == cudaSuccess) {
                    failure = error;
                }
            }
            const auto decision = ::vision::graphCaptureDecision(
                complete, unsupported, failure != cudaSuccess);
            if (decision == ::vision::GraphCaptureDecision::failure) {
                CUDA_CHECK(failure);  // 优先报告真实错误，不能被同批捕获失效掩盖。
                throw ::vision::BackendError("CUDA graph capture missing submission or graph resource");
            }
            return decision == ::vision::GraphCaptureDecision::captured;
        };
        const auto fallback = [&]() {
            if (capture.active) {
                const auto ended = cudaStreamEndCapture(stream_.get(), &capture.graph);
                capture.active = false;
                const auto end_error = cudaGetLastError();
                capture_ready(true, {ended, end_error});
            }
            capture.release();
            ::vision::requireBackendSubmission(context_->enqueueV3(stream_.get()), "graph fallback");
            CUDA_CHECK(cudaGetLastError());
            stream_.synchronize();
            return false;
        };

        // 不把进入捕获前遗留的错误当作可恢复的不兼容。
        CUDA_CHECK(cudaGetLastError());
        const auto begin = cudaStreamBeginCapture(stream_.get(), cudaStreamCaptureModeGlobal);
        capture.active = begin == cudaSuccess;
        const auto begin_error = cudaGetLastError();
        if (!capture_ready(capture.active, {begin, begin_error})) {return fallback();}
        const bool submitted = submit();
        // EndCapture 可能覆盖线程错误槽，先保留提交错误，再结束捕获后统一检查。
        const auto submit_error = cudaGetLastError();
        const auto ended = cudaStreamEndCapture(stream_.get(), &capture.graph);
        capture.active = false;
        const auto end_error = cudaGetLastError();
        if (!capture_ready(submitted && capture.graph, {submit_error, ended, end_error})) {
            return fallback();
        }

        const auto instantiated = cudaGraphInstantiate(&capture.executable, capture.graph, 0);
        const auto instantiate_error = cudaGetLastError();
        if (!capture_ready(capture.executable != nullptr, {instantiated, instantiate_error})) {
            return fallback();
        }

        // 全部成功后才转交所有权；此前任何异常均由 CaptureResources 清理。
        graph_ = capture.graph;
        graphExec_ = capture.executable;
        capture.graph = nullptr;
        capture.executable = nullptr;
        graphCaptured_ = true;
        return true;
    }

private:
    void captureInferenceGraph() {
        if (isDynamicInputShape_ || isDynamicBatchSize_) {
            std::cout << "[INFO] Dynamic shapes — CUDA graph disabled." << std::endl;
            return;
        }
        if (tryCaptureInferenceGraph([&]() {return context_->enqueueV3(stream_.get());})) {
            std::cout << "[INFO] CUDA graph captured — inference dispatch optimised." << std::endl;
        } else {
            std::cout << "[WARN] CUDA graph capture unsupported — validated enqueueV3 fallback." << std::endl;
        }
    }
]=])
_vision_replace_exact(_session_text "${_original}" "${_revised}" "transactional CUDA graph capture")
_vision_replace_exact(_session_text "#include <iostream>\n#include <numeric>"
  "#include <iostream>\n#include <limits>\n#include <initializer_list>\n#include <numeric>" "limits/initializer_list include")

set(_utils_header "${_upstream}/include/yolos/core/trt_utils.hpp")
file(READ "${_utils_header}" _utils_text)
set(_original [=[
inline void cudaCheck(cudaError_t code, const char* file, int line) {
    if (code != cudaSuccess) {
        std::ostringstream ss;
        ss << "CUDA error at " << file << ":" << line
           << " — " << cudaGetErrorName(code)
           << ": " << cudaGetErrorString(code);
        throw std::runtime_error(ss.str());
    }
}
]=])
set(_revised [=[
inline void cudaCheck(cudaError_t code, const char* file, int line) {
    if (code != cudaSuccess) {
        std::ostringstream ss;
        ss << "CUDA error at " << file << ":" << line
           << " — " << cudaGetErrorName(code)
           << ": " << cudaGetErrorString(code);
        throw ::vision::BackendError(ss.str());
    }
}
]=])
_vision_replace_exact(_utils_text "${_original}" "${_revised}" "CUDA failure type")
set(_backend_class "#include <vector>\n#include <stdexcept>\n#include <string>\nnamespace vision {\nclass BackendError : public std::runtime_error {\npublic:\n  using std::runtime_error::runtime_error;\n};\ninline void requireBackendSubmission(bool submitted, const char * operation) {\n  if (!submitted) {throw BackendError(std::string(operation) + \": TensorRT enqueueV3 failed\");}\n}\n}\n\nnamespace yolos {")
_vision_replace_exact(_utils_text "#include <vector>\n\nnamespace yolos {" "${_backend_class}" "backend error include")
# 所有匹配成功后再写入，不把失败配置留下的半补丁当成成功升级。
file(WRITE "${_session_header}" "${_session_text}")
file(WRITE "${_utils_header}" "${_utils_text}")

# 上游分割任务的 cv::Mat 入口依赖已删除的 inferGpu，一并删除；本仓库只用 FastSegmentor::segmentFast。
set(_segmentation_header "${_upstream}/include/yolos/tasks/segmentation.hpp")
file(READ "${_segmentation_header}" _segmentation_text)
set(_removed_segment [=[
    /// @brief Run segmentation on an image
    std::vector<Segmentation> segment(const cv::Mat& image,
                                      float confThreshold = 0.4f,
                                      float iouThreshold = 0.45f) {
        // Full GPU pipeline: pinned H2D → CUDA letterbox → TRT infer → D2H
        inferGpu(image);
        return postprocess(image.size(), inputShape_, confThreshold, iouThreshold);
    }
]=])
_vision_replace_exact(_segmentation_text "${_removed_segment}" "    // vision: cv::Mat segment() removed\n" "segment")
file(WRITE "${_segmentation_header}" "${_segmentation_text}")
