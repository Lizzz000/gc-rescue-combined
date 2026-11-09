// ---- MP4 封装实现 ----
// 职责：编码、复用和关闭。
#include "ffmpeg/mp4_writer.hpp"

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/error.h>
#include <libavutil/imgutils.h>
#include <libavutil/opt.h>
#include <libswscale/swscale.h>
}

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <exception>
#include <limits>
#include <stdexcept>
#include <string>
#include <system_error>
#include <fcntl.h>
#include <sys/stat.h>
#ifdef _WIN32
#include <io.h>
#else
#include <unistd.h>
#endif

namespace record {
namespace {
constexpr AVRational microseconds{1, 1000000};

std::string ff_error(int error) {
  char message[AV_ERROR_MAX_STRING_SIZE]{};
  av_strerror(error, message, sizeof(message));
  return message;
}

void check(int result, const char* operation) {
  if (result < 0)
    throw std::runtime_error(std::string(operation) + ": " + ff_error(result));
}

// 只归类明确的 JPEG 解码拒绝；内存、状态和其他库错误原样作为致命错误。
void check_jpeg_decode(int result, const char* operation) {
  if (result == AVERROR_INVALIDDATA)
    throw BadJpeg(std::string(operation) + ": " + ff_error(result));
  check(result, operation);
}

struct Dictionary {
  AVDictionary* value = nullptr;
  ~Dictionary() { av_dict_free(&value); }
  void set(const char* key, const char* value_text) {
    check(av_dict_set(&value, key, value_text, 0), "set FFmpeg option");
  }
};

int exclusive_open(const std::filesystem::path& path) {
#ifdef _WIN32
  const int fd = _wopen(path.c_str(), _O_WRONLY | _O_CREAT | _O_EXCL |
                                     _O_BINARY | _O_NOINHERIT,
                       _S_IREAD | _S_IWRITE);
#else
  const int fd = ::open(path.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC, 0666);
#endif
  if (fd < 0) {
    const int error = errno;
    throw std::system_error(error, std::generic_category(),
                            "create MP4 exclusively: " + path.u8string());
  }
  return fd;
}

int close_descriptor(int& fd) noexcept {
  if (fd < 0) return 0;
  const int owned = fd;
  fd = -1;  // EINTR 后不重试关闭，避免误关已复用的描述符。
#ifdef _WIN32
  const int result = _close(owned);
#else
  const int result = ::close(owned);
#endif
  return result < 0 ? AVERROR(errno) : 0;
}

AVPixelFormat normalized_jpeg_format(AVPixelFormat format) {
  // 色彩范围单独设置，不依赖旧 YUVJ 格式名。
  switch (format) {
    case AV_PIX_FMT_YUVJ420P: return AV_PIX_FMT_YUV420P;
    case AV_PIX_FMT_YUVJ422P: return AV_PIX_FMT_YUV422P;
    case AV_PIX_FMT_YUVJ444P: return AV_PIX_FMT_YUV444P;
    case AV_PIX_FMT_YUVJ440P: return AV_PIX_FMT_YUV440P;
    case AV_PIX_FMT_YUVJ411P: return AV_PIX_FMT_YUV411P;
    default: return format;
  }
}

std::string exception_message(const std::exception_ptr& error) {
  if (!error) return {};
  try { std::rethrow_exception(error); }
  catch (const std::exception& e) { return e.what(); }
  catch (...) { return "unknown exception"; }
}
}  // 匿名命名空间

struct Mp4Writer::Impl {
  AVFormatContext* format = nullptr;
  AVCodecContext* decoder = nullptr;
  AVCodecContext* encoder = nullptr;
  AVStream* stream = nullptr;
  AVFrame* decoded = nullptr;
  AVFrame* extra = nullptr;
  AVFrame* yuv = nullptr;
  AVPacket* input = nullptr;
  AVPacket* output = nullptr;
  AVPacket* pending = nullptr;
  SwsContext* scaler = nullptr;
  AVPixelFormat scaler_format = AV_PIX_FMT_NONE;
  int scaler_full_range = -1;
  int fd = -1;
  int width = 0, height = 0, fps = 0;
  bool header_written = false, finished = false;
  std::uint64_t count = 0, first_ns = 0, last_ns = 0;
  std::int64_t last_pts = 0, last_key_pts = 0, last_duration = 0;
  std::string source, selected_encoder;
  std::exception_ptr failure;

  ~Impl() noexcept {
    if (header_written && !finished) {
      try { finish(); } catch (...) {}
    }
    if (format && format->pb) avio_closep(&format->pb);
    close_descriptor(fd);
    sws_freeContext(scaler);
    av_packet_free(&input);
    av_packet_free(&output);
    av_packet_free(&pending);
    av_frame_free(&decoded);
    av_frame_free(&extra);
    av_frame_free(&yuv);
    avcodec_free_context(&decoder);
    avcodec_free_context(&encoder);
    avformat_free_context(format);
  }

  // H.264 固定 libx264（CPU），不尝试硬件编码器、不回退；内存失败直接上抛。
  std::string open_encoder(const char* name) {
    const AVCodec* codec = avcodec_find_encoder_by_name(name);
    if (!codec) return std::string(name) + ": encoder is not available in this FFmpeg build";
    auto release = [](AVCodecContext* context) { avcodec_free_context(&context); };
    std::unique_ptr<AVCodecContext, decltype(release)> candidate(avcodec_alloc_context3(codec), release);
    if (!candidate) throw std::bad_alloc();
    auto* context = candidate.get();
    context->width = width; context->height = height;
    context->pix_fmt = AV_PIX_FMT_YUV420P;
    context->time_base = microseconds;
    context->framerate = AVRational{fps, 1};
    context->gop_size = fps;
    context->max_b_frames = 0;
    context->color_range = AVCOL_RANGE_MPEG;
    context->colorspace = AVCOL_SPC_SMPTE170M;
    context->sample_aspect_ratio = AVRational{1, 1};
    if (format->oformat->flags & AVFMT_GLOBALHEADER) context->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;
    Dictionary options;
    options.set("forced-idr", "1");
    options.set("crf", "18");
    options.set("preset", "veryfast");
    options.set("tune", "zerolatency");
    options.set("x264-params", "scenecut=0:open-gop=0");
    const int result = avcodec_open2(context, codec, &options.value);
    if (result == AVERROR(ENOMEM)) throw std::bad_alloc();
    if (result < 0) return std::string(name) + ": initialization failed: " + ff_error(result);
    if (options.value) return std::string(name) + ": required encoder option was not consumed";
    selected_encoder = name;
    encoder = candidate.release();
    return {};
  }

  void initialize(const std::filesystem::path& path, int w, int h, int rate,
                  const std::string& selection) {
    // encoder 枚举与宽高/fps 已在 CLI 与采集端入口校验，这里只保留库尺寸检查。
    check(av_image_check_size(w, h, 0, nullptr), "validate image size");
    width = w; height = h; fps = rate;
    last_duration = std::max<std::int64_t>(1, av_rescale_q(1, AVRational{1, fps}, microseconds));
    const AVCodec* mjpeg = avcodec_find_decoder(AV_CODEC_ID_MJPEG);
    if (!mjpeg) throw std::runtime_error("FFmpeg requires MJPEG decoder");
    decoder = avcodec_alloc_context3(mjpeg);
    decoded = av_frame_alloc(); extra = av_frame_alloc(); yuv = av_frame_alloc();
    input = av_packet_alloc(); output = av_packet_alloc(); pending = av_packet_alloc();
    if (!decoder || !decoded || !extra || !yuv || !input || !output || !pending)
      throw std::bad_alloc();
    decoder->thread_count = 1;
    decoder->max_pixels = static_cast<std::int64_t>(width) * height;
    decoder->err_recognition = AV_EF_BITSTREAM | AV_EF_BUFFER | AV_EF_EXPLODE;
    check(avcodec_open2(decoder, mjpeg, nullptr), "open MJPEG decoder");
    const std::string filename = path.u8string();
    check(avformat_alloc_output_context2(&format, nullptr, "mp4", filename.c_str()), "allocate MP4 muxer");
    if (!format) throw std::bad_alloc();
    (void)selection; // auto/nvenc/x264 均使用 libx264，入口已打印忽略说明。
    const std::string error = open_encoder("libx264");
    if (!error.empty()) throw std::runtime_error(error);
    stream = avformat_new_stream(format, nullptr);
    if (!stream) throw std::bad_alloc();
    stream->time_base = microseconds;
    stream->avg_frame_rate = encoder->framerate;
    check(avcodec_parameters_from_context(stream->codecpar, encoder), "copy H264 parameters");
    yuv->format = encoder->pix_fmt; yuv->width = width; yuv->height = height;
    yuv->color_range = encoder->color_range; yuv->colorspace = encoder->colorspace;
    yuv->sample_aspect_ratio = encoder->sample_aspect_ratio;
    check(av_frame_get_buffer(yuv, 32), "allocate YUV420 frame");

    // 独占创建后直接交给进程内 AVIO pipe 协议，避免按路径重开造成覆盖竞态。
    // 4.4 借用描述符，新版复制描述符；自有描述符统一在 avio_closep 后关闭。
    fd = exclusive_open(path);
    const std::string url = "pipe:" + std::to_string(fd);
    check(avio_open2(&format->pb, url.c_str(), AVIO_FLAG_WRITE, nullptr, nullptr), "open MP4 AVIO");
    Dictionary mux_options;
    mux_options.set("movflags", "empty_moov+frag_keyframe+default_base_moof");
    mux_options.set("video_track_timescale", "1000000");
    check(avformat_write_header(format, &mux_options.value), "write MP4 header");
    header_written = true;
    flush_io();
  }

  void flush_io() {
    avio_flush(format->pb);
    check(format->pb->error, "flush MP4 AVIO");
  }

  void decode_and_convert(const Frame& frame) {
    avcodec_flush_buffers(decoder);
    av_frame_unref(decoded); av_frame_unref(extra); av_packet_unref(input);
    check(av_new_packet(input, static_cast<int>(frame.jpeg.size())), "allocate JPEG packet");
    std::memcpy(input->data, frame.jpeg.data(), frame.jpeg.size());
    const int sent = avcodec_send_packet(decoder, input);
    av_packet_unref(input);
    check_jpeg_decode(sent, "send JPEG packet");
    const int received = avcodec_receive_frame(decoder, decoded);
    // 完整 JPEG 未产出帧可跳过；意外 EOF 属于解码器状态异常。
    if (received == AVERROR(EAGAIN)) throw BadJpeg("JPEG packet produced no frame");
    check_jpeg_decode(received, "decode JPEG");
    if (!decoded->data[0] || decoded->width != width || decoded->height != height ||
        decoded->decode_error_flags || (decoded->flags & AV_FRAME_FLAG_CORRUPT))
      throw BadJpeg("JPEG decoded to no pixels, wrong size or corrupt pixels");
    const auto src_format = static_cast<AVPixelFormat>(decoded->format);
    // 相机固定输出 8 位 BT.601 JPEG，不再逐帧检查像素格式与色彩矩阵。
    const int full_range = decoded->color_range == AVCOL_RANGE_MPEG ? 0 : 1;
    const auto normalized_format = normalized_jpeg_format(src_format);
    if (!scaler || scaler_format != normalized_format || scaler_full_range != full_range) {
      std::unique_ptr<SwsContext, decltype(&sws_freeContext)> next(sws_alloc_context(), &sws_freeContext);
      if (!next) throw std::bad_alloc();
      check(av_opt_set_int(next.get(), "srcw", width, 0), "set scaler input width");
      check(av_opt_set_int(next.get(), "srch", height, 0), "set scaler input height");
      check(av_opt_set_int(next.get(), "src_format", normalized_format, 0), "set scaler input format");
      check(av_opt_set_int(next.get(), "dstw", width, 0), "set scaler output width");
      check(av_opt_set_int(next.get(), "dsth", height, 0), "set scaler output height");
      check(av_opt_set_int(next.get(), "dst_format", AV_PIX_FMT_YUV420P, 0), "set scaler output format");
      check(av_opt_set_int(next.get(), "sws_flags", SWS_BILINEAR | SWS_ACCURATE_RND, 0), "set scaler flags");
      // 初始化前设置范围，避免同尺寸平面格式走直接复制而漏掉范围转换。
      check(av_opt_set_int(next.get(), "src_range", full_range, 0), "set scaler input range");
      check(av_opt_set_int(next.get(), "dst_range", 0, 0), "set scaler output range");
      check(sws_init_context(next.get(), nullptr, nullptr), "initialize JPEG color converter");
      const int* coefficients = sws_getCoefficients(SWS_CS_ITU601);
      check(sws_setColorspaceDetails(next.get(), coefficients, full_range, coefficients,
                                    0, 0, 1 << 16, 1 << 16), "set JPEG full/limited range conversion");
      sws_freeContext(scaler);
      scaler = next.release();
      scaler_format = normalized_format;
      scaler_full_range = full_range;
    }
    check(av_frame_make_writable(yuv), "make YUV420 frame writable");
    const int rows = sws_scale(scaler, decoded->data, decoded->linesize, 0, height,
                               yuv->data, yuv->linesize);
    if (rows != height) throw std::runtime_error("JPEG color conversion returned incomplete frame");
  }

  void mux_pending(std::int64_t duration) {
    if (!pending->size) return;
    pending->duration = duration;
    av_packet_rescale_ts(pending, encoder->time_base, stream->time_base);
    pending->stream_index = stream->index;
    check(av_interleaved_write_frame(format, pending), "mux H264 packet");
    av_packet_unref(pending);
    flush_io();  // 及时送出已完成片段；不承诺断电持久性。
  }

  void drain_encoder(bool flushing) {
    for (;;) {
      av_packet_unref(output);
      const int result = avcodec_receive_packet(encoder, output);
      if (result == AVERROR_EOF) {
        if (!flushing) throw std::runtime_error("H264 encoder ended unexpectedly");
        return;
      }
      if (result == AVERROR(EAGAIN)) {
        if (flushing) throw std::runtime_error("H264 encoder did not finish draining");
        return;
      }
      check(result, "receive H264 packet");
      // max_b_frames=0，时间戳不重排，不再逐包校验。
      if (pending->size) mux_pending(output->pts - pending->pts);
      av_packet_move_ref(pending, output);
    }
  }

  void write(const Frame& frame) {
    if (failure) std::rethrow_exception(failure);
    if (count && frame.source_id != source)
      throw std::invalid_argument("frame source changed; capture clocks cannot be mixed");
    if (count && frame.capture_ns <= last_ns)
      throw std::invalid_argument("capture_ns must be strictly increasing");
    const std::uint64_t elapsed_us = count ? (frame.capture_ns - first_ns) / 1000 : 0;
    // 先做无符号纳秒差再转微秒，避免绝对采集时间超过 INT64_MAX。
    const auto pts = static_cast<std::int64_t>(elapsed_us);
    decode_and_convert(frame);
    if (!count) source = frame.source_id;
    const bool key = !count || pts - last_key_pts >= microseconds.den;
    yuv->pts = pts;
    yuv->pict_type = key ? AV_PICTURE_TYPE_I : AV_PICTURE_TYPE_NONE;
    try {
      check(avcodec_send_frame(encoder, yuv), "send H264 frame");
      drain_encoder(false);
    } catch (...) {
      failure = std::current_exception();
      throw;
    }
    if (!count) first_ns = frame.capture_ns;
    else last_duration = pts - last_pts;
    last_ns = frame.capture_ns; last_pts = pts;
    if (key) last_key_pts = pts;
    ++count;
  }

  void finish() {
    if (finished) {
      if (failure) std::rethrow_exception(failure);
      return;
    }
    std::exception_ptr drain_failure;
    try {
      check(avcodec_send_frame(encoder, nullptr), "flush H264 encoder");
      drain_encoder(true);
      // 末帧沿用最后一次实际间隔；只有单帧录制才用标称帧率确定持续时间。
      mux_pending(last_duration);
    } catch (...) { drain_failure = std::current_exception(); }
    // 任一步失败仍尝试其余收尾；只报告首个错误，始终保留部分文件。
    const int trailer_error = av_write_trailer(format);
    avio_flush(format->pb);
    const int io_error = format->pb->error;
    const int avio_error = avio_closep(&format->pb);
    const int close_error = close_descriptor(fd);
    finished = true;
    if (failure || drain_failure || trailer_error < 0 || io_error < 0 || avio_error < 0 || close_error < 0) {
      try {
        std::string message = "MP4 finalization failed: ";
        if (failure) message += exception_message(failure);
        else if (drain_failure) message += exception_message(drain_failure);
        else message += ff_error(trailer_error < 0 ? trailer_error : io_error < 0 ? io_error
                                 : avio_error < 0 ? avio_error : close_error);
        throw std::runtime_error(message);
      } catch (...) {
        failure = std::current_exception();
        throw;
      }
    }
  }
};

Mp4Writer::Mp4Writer(const std::filesystem::path& path, int width, int height, int fps,
                     const std::string& encoder)
    : impl_(std::make_unique<Impl>()) {
  impl_->initialize(path, width, height, fps, encoder);
}
Mp4Writer::~Mp4Writer() noexcept = default;
void Mp4Writer::write(const Frame& frame) { impl_->write(frame); }
void Mp4Writer::finish() { impl_->finish(); }
std::uint64_t Mp4Writer::frames() const { return impl_->count; }
const std::string& Mp4Writer::encoder_name() const noexcept { return impl_->selected_encoder; }

}  // 命名空间 record
