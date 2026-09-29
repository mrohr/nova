#include "stream/deck_v4l2_decoder.h"

#include <Limelight.h>

extern "C" {
#include <libavutil/buffer.h>
#include <libavutil/frame.h>
#include <libavutil/hwcontext.h>
#include <libavutil/hwcontext_drm.h>
}

#include <libdrm/drm_fourcc.h>
#include <linux/videodev2.h>

#include <dirent.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <mutex>
#include <string_view>

namespace nova::deck::stream {

namespace {

constexpr auto outputType = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
constexpr auto captureType = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;

int xioctl(int fd, unsigned long request, void* argument) {
    int result;
    do {
        result = ioctl(fd, request, argument);
    } while (result < 0 && errno == EINTR);
    return result;
}

std::string systemError(std::string_view what) {
    return std::string(what) + ": " + std::strerror(errno);
}

std::uint32_t codecFourcc(int videoFormat) {
    if (videoFormat == VIDEO_FORMAT_H264) return V4L2_PIX_FMT_H264;
    if (videoFormat == VIDEO_FORMAT_H265 || videoFormat == VIDEO_FORMAT_H265_MAIN10) return V4L2_PIX_FMT_HEVC;
    return 0;
}

// Linear layouts only. Tiled and vendor-compressed layouts (Qualcomm QC08C,
// Mediatek MM21) need modifiers the presenters do not import.
bool linearCaptureFormat(std::uint32_t fourcc, bool tenBit) {
    return tenBit ? fourcc == V4L2_PIX_FMT_P010 : fourcc == V4L2_PIX_FMT_NV12 || fourcc == V4L2_PIX_FMT_NV12M;
}

int planesFor(std::uint32_t fourcc) {
    return fourcc == V4L2_PIX_FMT_NV12M ? 2 : 1;
}

bool offersFormat(int fd, v4l2_buf_type type, std::uint32_t fourcc) {
    v4l2_fmtdesc description{};
    description.type = type;
    for (; xioctl(fd, VIDIOC_ENUM_FMT, &description) == 0; ++description.index)
        if (description.pixelformat == fourcc) return true;
    return false;
}

bool offersLinearCapture(int fd, bool tenBit) {
    v4l2_fmtdesc description{};
    description.type = captureType;
    for (; xioctl(fd, VIDIOC_ENUM_FMT, &description) == 0; ++description.index)
        if (linearCaptureFormat(description.pixelformat, tenBit)) return true;
    return false;
}

bool setOutputFormat(int fd, std::uint32_t codec, int width, int height) {
    v4l2_format format{};
    format.type = outputType;
    format.fmt.pix_mp.pixelformat = codec;
    format.fmt.pix_mp.width = static_cast<std::uint32_t>(width);
    format.fmt.pix_mp.height = static_cast<std::uint32_t>(height);
    format.fmt.pix_mp.num_planes = 1;
    // Room for one IDR access unit; the driver may round this up.
    format.fmt.pix_mp.plane_fmt[0].sizeimage =
        std::max<std::uint32_t>(1u << 20, static_cast<std::uint32_t>(width) * static_cast<std::uint32_t>(height) * 3 / 2);
    return xioctl(fd, VIDIOC_S_FMT, &format) == 0 && format.fmt.pix_mp.pixelformat == codec;
}

// Reported limits only, never a guessed default: a driver that does not
// enumerate frame sizes is treated as unsupported.
DeckDecodeLimits frameSizeLimits(int fd, std::uint32_t codec) {
    DeckDecodeLimits limits;
    v4l2_frmsizeenum size{};
    size.pixel_format = codec;
    const auto constrain = [](int& limit, std::uint32_t value) {
        if (value > 0 && value <= 65536) limit = std::max(limit, static_cast<int>(value));
    };
    for (; xioctl(fd, VIDIOC_ENUM_FRAMESIZES, &size) == 0; ++size.index) {
        if (size.type == V4L2_FRMSIZE_TYPE_DISCRETE) {
            constrain(limits.maxWidth, size.discrete.width);
            constrain(limits.maxHeight, size.discrete.height);
        } else {
            constrain(limits.maxWidth, size.stepwise.max_width);
            constrain(limits.maxHeight, size.stepwise.max_height);
            break;
        }
    }
    return limits;
}

std::vector<std::string> videoNodes() {
    std::vector<std::pair<int, std::string>> nodes;
    if (DIR* directory = opendir("/dev")) {
        while (const dirent* entry = readdir(directory)) {
            const std::string_view name(entry->d_name);
            if (name.size() <= 5 || name.substr(0, 5) != "video"
                    || !std::all_of(name.begin() + 5, name.end(), [](char c) { return c >= '0' && c <= '9'; }))
                continue;
            nodes.emplace_back(std::stoi(std::string(name.substr(5))), "/dev/" + std::string(name));
        }
        closedir(directory);
    }
    std::sort(nodes.begin(), nodes.end());
    std::vector<std::string> paths;
    for (auto& node : nodes) paths.push_back(std::move(node.second));
    return paths;
}

// Opens a node that is a multi-planar memory-to-memory device with streaming
// I/O, which every stateful decoder in mainline Linux is.
int openDecoderNode(const std::string& path, std::string* card = nullptr) {
    const int fd = ::open(path.c_str(), O_RDWR | O_NONBLOCK | O_CLOEXEC);
    if (fd < 0) return -1;
    v4l2_capability capability{};
    if (xioctl(fd, VIDIOC_QUERYCAP, &capability) == 0) {
        const auto caps = (capability.capabilities & V4L2_CAP_DEVICE_CAPS) ? capability.device_caps : capability.capabilities;
        if ((caps & V4L2_CAP_VIDEO_M2M_MPLANE) && (caps & V4L2_CAP_STREAMING)) {
            if (card) *card = reinterpret_cast<const char*>(capability.card);
            return fd;
        }
    }
    ::close(fd);
    return -1;
}

bool nodeDecodes(int fd, std::uint32_t codec, bool tenBit) {
    return offersFormat(fd, outputType, codec) && setOutputFormat(fd, codec, 1920, 1080)
        && offersLinearCapture(fd, tenBit);
}

AVColorPrimaries primariesFor(std::uint32_t colorspace) {
    switch (colorspace) {
    case V4L2_COLORSPACE_REC709:
    case V4L2_COLORSPACE_SRGB: return AVCOL_PRI_BT709;
    case V4L2_COLORSPACE_SMPTE170M: return AVCOL_PRI_SMPTE170M;
    case V4L2_COLORSPACE_470_SYSTEM_BG: return AVCOL_PRI_BT470BG;
    case V4L2_COLORSPACE_BT2020: return AVCOL_PRI_BT2020;
    default: return AVCOL_PRI_UNSPECIFIED;
    }
}

AVColorSpace matrixFor(std::uint32_t colorspace, std::uint32_t encoding) {
    if (encoding == V4L2_YCBCR_ENC_DEFAULT) encoding = V4L2_MAP_YCBCR_ENC_DEFAULT(colorspace);
    switch (encoding) {
    case V4L2_YCBCR_ENC_601: return AVCOL_SPC_SMPTE170M;
    case V4L2_YCBCR_ENC_709: return AVCOL_SPC_BT709;
    case V4L2_YCBCR_ENC_BT2020: return AVCOL_SPC_BT2020_NCL;
    case V4L2_YCBCR_ENC_BT2020_CONST_LUM: return AVCOL_SPC_BT2020_CL;
    default: return AVCOL_SPC_UNSPECIFIED;
    }
}

AVColorTransferCharacteristic transferFor(std::uint32_t colorspace, std::uint32_t transfer) {
    if (transfer == V4L2_XFER_FUNC_DEFAULT) transfer = V4L2_MAP_XFER_FUNC_DEFAULT(colorspace);
    switch (transfer) {
    case V4L2_XFER_FUNC_709: return AVCOL_TRC_BT709;
    case V4L2_XFER_FUNC_SRGB: return AVCOL_TRC_IEC61966_2_1;
    case V4L2_XFER_FUNC_SMPTE2084: return AVCOL_TRC_SMPTE2084;
    default: return AVCOL_TRC_UNSPECIFIED;
    }
}

// Buffers the decoder has handed out are returned from whichever thread drops
// the last frame reference, so their queue state lives here, shared with the
// frames. It outlives the decoder object while any frame is still presented.
struct CaptureQueue {
    std::mutex mutex;
    int fd = -1;
    int planes = 1;
    unsigned generation = 0;
    bool streaming = false;

    ~CaptureQueue() {
        if (fd >= 0) ::close(fd);
    }

    bool queue(unsigned index) {
        v4l2_plane planeInfo[VIDEO_MAX_PLANES]{};
        v4l2_buffer buffer{};
        buffer.type = captureType;
        buffer.memory = V4L2_MEMORY_MMAP;
        buffer.index = index;
        buffer.m.planes = planeInfo;
        buffer.length = static_cast<std::uint32_t>(planes);
        return xioctl(fd, VIDIOC_QBUF, &buffer) == 0;
    }

    void requeue(unsigned bufferGeneration, unsigned index) {
        const std::lock_guard<std::mutex> lock(mutex);
        // A buffer from before a capture reconfiguration or teardown is gone
        // from the queue; its memory lives on only until its DMA-BUF closes.
        if (streaming && bufferGeneration == generation) queue(index);
    }
};

struct ExportedBuffer {
    int fds[2] = {-1, -1};
    std::size_t sizes[2] = {};
    std::size_t offsets[2] = {};

    ~ExportedBuffer() {
        for (const int fd : fds)
            if (fd >= 0) ::close(fd);
    }
};

// Held by each frame: the DMA-BUFs stay open while the GPU may read them, and
// dropping the last frame reference hands the buffer back to the decoder.
struct CaptureHold {
    std::shared_ptr<CaptureQueue> queue;
    std::shared_ptr<ExportedBuffer> exported;
    unsigned generation = 0;
    unsigned index = 0;

    CaptureHold(std::shared_ptr<CaptureQueue> captureQueue, std::shared_ptr<ExportedBuffer> buffer, unsigned bufferGeneration, unsigned bufferIndex)
        : queue(std::move(captureQueue)), exported(std::move(buffer)), generation(bufferGeneration), index(bufferIndex) {}
    CaptureHold(const CaptureHold&) = delete;
    CaptureHold& operator=(const CaptureHold&) = delete;
    ~CaptureHold() {
        queue->requeue(generation, index);
    }
};

struct DrmFrameOwner {
    AVDRMFrameDescriptor descriptor{};
    std::shared_ptr<void> owner;
};

struct OutputBuffer {
    void* data = MAP_FAILED;
    std::size_t length = 0;
    bool queued = false;
};

} // namespace

AVBufferRef* deckV4l2FramesContext(bool tenBit, int width, int height) {
    AVBufferRef* device = av_hwdevice_ctx_alloc(AV_HWDEVICE_TYPE_DRM);
    if (!device) return nullptr;
    // Frames carry their own DMA-BUFs. No DRM node is opened, and the device
    // must not close the descriptor zero it was allocated with.
    auto* deviceContext = reinterpret_cast<AVHWDeviceContext*>(device->data);
    static_cast<AVDRMDeviceContext*>(deviceContext->hwctx)->fd = -1;
    if (av_hwdevice_ctx_init(device) < 0) {
        av_buffer_unref(&device);
        return nullptr;
    }
    AVBufferRef* frames = av_hwframe_ctx_alloc(device);
    av_buffer_unref(&device);
    if (!frames) return nullptr;
    auto* framesContext = reinterpret_cast<AVHWFramesContext*>(frames->data);
    framesContext->format = AV_PIX_FMT_DRM_PRIME;
    framesContext->sw_format = tenBit ? AV_PIX_FMT_P010LE : AV_PIX_FMT_NV12;
    framesContext->width = width;
    framesContext->height = height;
    if (av_hwframe_ctx_init(frames) < 0) av_buffer_unref(&frames);
    return frames;
}

AVFrame* deckV4l2WrapDrmFrame(const DeckV4l2FrameLayout& layout, AVBufferRef* framesContext, std::shared_ptr<void> owner) {
    if (!framesContext || layout.width <= 0 || layout.height <= 0 || layout.objectCount < 1 || layout.objectCount > 2) return nullptr;
    for (int i = 0; i < layout.objectCount; ++i)
        if (layout.fds[i] < 0 || layout.sizes[i] == 0 || layout.offsets[i] >= layout.sizes[i]) return nullptr;
    const int chromaObject = layout.objectCount == 2 ? 1 : 0;
    const int chromaPitch = layout.pitches[chromaObject] > 0 ? layout.pitches[chromaObject] : layout.pitches[0];
    const std::size_t chromaOffset = layout.objectCount == 2
        ? layout.offsets[1]
        : layout.offsets[0] + static_cast<std::size_t>(layout.pitches[0]) * static_cast<std::size_t>(layout.lumaRows);
    if (layout.pitches[0] <= 0 || chromaPitch <= 0 || (layout.objectCount == 1 && layout.lumaRows < layout.height))
        return nullptr;
    // Both planes must lie inside their exported buffers.
    const auto rowBytes = static_cast<std::size_t>(layout.width) * (layout.tenBit ? 2 : 1);
    const auto chromaRows = static_cast<std::size_t>(layout.height + 1) / 2;
    if (static_cast<std::size_t>(layout.pitches[0]) < rowBytes || static_cast<std::size_t>(chromaPitch) < rowBytes
            || layout.offsets[0] + static_cast<std::size_t>(layout.pitches[0]) * static_cast<std::size_t>(layout.height) > layout.sizes[0]
            || chromaOffset + static_cast<std::size_t>(chromaPitch) * chromaRows > layout.sizes[chromaObject])
        return nullptr;

    auto* holder = new DrmFrameOwner{};
    holder->owner = std::move(owner);
    auto& drm = holder->descriptor;
    drm.nb_objects = layout.objectCount;
    for (int i = 0; i < layout.objectCount; ++i)
        drm.objects[i] = {layout.fds[i], layout.sizes[i], DRM_FORMAT_MOD_LINEAR};
    drm.nb_layers = 2;
    drm.layers[0].format = layout.tenBit ? DRM_FORMAT_R16 : DRM_FORMAT_R8;
    drm.layers[0].nb_planes = 1;
    drm.layers[0].planes[0] = {0, static_cast<ptrdiff_t>(layout.offsets[0]), layout.pitches[0]};
    drm.layers[1].format = layout.tenBit ? DRM_FORMAT_GR1616 : DRM_FORMAT_GR88;
    drm.layers[1].nb_planes = 1;
    drm.layers[1].planes[0] = {chromaObject, static_cast<ptrdiff_t>(chromaOffset), chromaPitch};

    AVFrame* frame = av_frame_alloc();
    if (frame) {
        frame->buf[0] = av_buffer_create(reinterpret_cast<std::uint8_t*>(&holder->descriptor), sizeof(holder->descriptor),
            [](void* opaque, std::uint8_t*) { delete static_cast<DrmFrameOwner*>(opaque); }, holder, AV_BUFFER_FLAG_READONLY);
    }
    if (!frame || !frame->buf[0]) {
        delete holder;
        av_frame_free(&frame);
        return nullptr;
    }
    frame->data[0] = frame->buf[0]->data;
    frame->hw_frames_ctx = av_buffer_ref(framesContext);
    if (!frame->hw_frames_ctx) {
        av_frame_free(&frame);
        return nullptr;
    }
    frame->format = AV_PIX_FMT_DRM_PRIME;
    frame->width = layout.width;
    frame->height = layout.height;
    frame->color_primaries = layout.primaries;
    frame->color_trc = layout.transfer;
    frame->colorspace = layout.matrix;
    frame->color_range = layout.range;
    return frame;
}

DeckVideoDecodeSupport probeV4l2DecodeSupport() {
    DeckVideoDecodeSupport support;
    for (const auto& path : videoNodes()) {
        const int fd = openDecoderNode(path);
        if (fd < 0) continue;
        if (!support.h264.maxWidth && nodeDecodes(fd, V4L2_PIX_FMT_H264, false))
            support.h264 = frameSizeLimits(fd, V4L2_PIX_FMT_H264);
        if (!support.hevc.maxWidth && nodeDecodes(fd, V4L2_PIX_FMT_HEVC, false))
            support.hevc = frameSizeLimits(fd, V4L2_PIX_FMT_HEVC);
        // Capture formats follow the coded format, so Main10 output is only
        // listed once the input is HEVC.
        if (!support.main10.maxWidth && nodeDecodes(fd, V4L2_PIX_FMT_HEVC, true))
            support.main10 = frameSizeLimits(fd, V4L2_PIX_FMT_HEVC);
        ::close(fd);
    }
    return support;
}

struct DeckV4l2Decoder::Impl {
    std::shared_ptr<CaptureQueue> capture = std::make_shared<CaptureQueue>();
    int fd = -1; // Owned by `capture`, which may outlive this object.
    std::vector<OutputBuffer> output;
    std::vector<std::shared_ptr<ExportedBuffer>> exported;
    AVBufferRef* framesContext = nullptr;
    DeckV4l2FrameLayout layout;
    unsigned generation = 0;
    std::uint32_t codec = 0;
    bool tenBit = false;
    bool failed = false;
    int streamWidth = 0, streamHeight = 0;
    std::uint64_t sequence = 0;
    std::string error, description;

    ~Impl() {
        stopCapture();
        if (fd >= 0) {
            int type = outputType;
            xioctl(fd, VIDIOC_STREAMOFF, &type);
        }
        for (auto& buffer : output)
            if (buffer.data != MAP_FAILED) munmap(buffer.data, buffer.length);
        av_buffer_unref(&framesContext);
    }

    bool fail(std::string message) {
        error = std::move(message);
        failed = true;
        return false;
    }

    // Presented frames keep their DMA-BUFs; only the queue is torn down here.
    void stopCapture() {
        if (fd < 0) return;
        {
            const std::lock_guard<std::mutex> lock(capture->mutex);
            if (capture->streaming) {
                int type = captureType;
                xioctl(fd, VIDIOC_STREAMOFF, &type);
            }
            capture->streaming = false;
            ++capture->generation;
            v4l2_requestbuffers request{};
            request.type = captureType;
            request.memory = V4L2_MEMORY_MMAP;
            xioctl(fd, VIDIOC_REQBUFS, &request);
        }
        exported.clear();
        av_buffer_unref(&framesContext);
    }

    bool open(int videoFormat, int width, int height) {
        codec = codecFourcc(videoFormat);
        tenBit = videoFormat == VIDEO_FORMAT_H265_MAIN10;
        streamWidth = width;
        streamHeight = height;
        if (!codec) return fail("unsupported video format for the V4L2 decoder");
        for (const auto& path : videoNodes()) {
            std::string card;
            const int candidate = openDecoderNode(path, &card);
            if (candidate < 0) continue;
            const auto limits = frameSizeLimits(candidate, codec);
            if (offersFormat(candidate, outputType, codec) && limits.supports(width, height)
                    && setOutputFormat(candidate, codec, width, height) && offersLinearCapture(candidate, tenBit)) {
                fd = capture->fd = candidate;
                description = card + " (" + path + ")";
                return configureOutput();
            }
            ::close(candidate);
        }
        return fail("no V4L2 stateful decoder offers this codec and stream size with linear output");
    }

    bool configureOutput() {
        // Streaming wants each picture as soon as it is decoded. Not every
        // driver has these controls, so a refusal is not an error.
        for (const auto& [id, value] : {std::pair<std::uint32_t, int>{V4L2_CID_MPEG_VIDEO_DEC_DISPLAY_DELAY_ENABLE, 1},
                                        std::pair<std::uint32_t, int>{V4L2_CID_MPEG_VIDEO_DEC_DISPLAY_DELAY, 0}}) {
            v4l2_control control{id, value};
            xioctl(fd, VIDIOC_S_CTRL, &control);
        }
        v4l2_event_subscription subscription{};
        subscription.type = V4L2_EVENT_SOURCE_CHANGE;
        if (xioctl(fd, VIDIOC_SUBSCRIBE_EVENT, &subscription) < 0) return fail(systemError("VIDIOC_SUBSCRIBE_EVENT"));

        v4l2_requestbuffers request{};
        request.type = outputType;
        request.memory = V4L2_MEMORY_MMAP;
        request.count = 6;
        if (xioctl(fd, VIDIOC_REQBUFS, &request) < 0 || request.count == 0) return fail(systemError("VIDIOC_REQBUFS(OUTPUT)"));
        output.resize(request.count);
        for (unsigned index = 0; index < request.count; ++index) {
            v4l2_plane planes[VIDEO_MAX_PLANES]{};
            v4l2_buffer buffer{};
            buffer.type = outputType;
            buffer.memory = V4L2_MEMORY_MMAP;
            buffer.index = index;
            buffer.m.planes = planes;
            buffer.length = VIDEO_MAX_PLANES;
            if (xioctl(fd, VIDIOC_QUERYBUF, &buffer) < 0) return fail(systemError("VIDIOC_QUERYBUF(OUTPUT)"));
            output[index].length = planes[0].length;
            output[index].data = mmap(nullptr, planes[0].length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, planes[0].m.mem_offset);
            if (output[index].data == MAP_FAILED) return fail(systemError("mmap(OUTPUT)"));
        }
        int type = outputType;
        if (xioctl(fd, VIDIOC_STREAMON, &type) < 0) return fail(systemError("VIDIOC_STREAMON(OUTPUT)"));
        return true;
    }

    // Runs when the decoder has parsed a sequence header (and again when the
    // stream changes size): pick a linear format, export and queue buffers.
    bool configureCapture() {
        stopCapture();
        v4l2_format format{};
        format.type = captureType;
        if (xioctl(fd, VIDIOC_G_FMT, &format) < 0) return fail(systemError("VIDIOC_G_FMT(CAPTURE)"));
        if (!linearCaptureFormat(format.fmt.pix_mp.pixelformat, tenBit)) {
            format.fmt.pix_mp.pixelformat = tenBit ? V4L2_PIX_FMT_P010 : V4L2_PIX_FMT_NV12;
            if (xioctl(fd, VIDIOC_S_FMT, &format) < 0 || !linearCaptureFormat(format.fmt.pix_mp.pixelformat, tenBit))
                return fail("the V4L2 decoder did not accept a linear " + std::string(tenBit ? "P010" : "NV12") + " output");
        }
        const auto& pix = format.fmt.pix_mp;
        const int planes = planesFor(pix.pixelformat);
        if (pix.num_planes != planes) return fail("the V4L2 decoder reported an unexpected plane count");

        int visibleWidth = static_cast<int>(pix.width), visibleHeight = static_cast<int>(pix.height);
        v4l2_selection selection{};
        selection.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        selection.target = V4L2_SEL_TGT_COMPOSE;
        if (xioctl(fd, VIDIOC_G_SELECTION, &selection) == 0 && selection.r.left == 0 && selection.r.top == 0
                && selection.r.width > 0 && selection.r.height > 0) {
            visibleWidth = std::min(visibleWidth, static_cast<int>(selection.r.width));
            visibleHeight = std::min(visibleHeight, static_cast<int>(selection.r.height));
        } else {
            // Without a compose rectangle, the aligned coded size would show
            // padding rows; the stream's own size is the visible picture.
            visibleWidth = std::min(visibleWidth, streamWidth);
            visibleHeight = std::min(visibleHeight, streamHeight);
        }

        layout = {};
        layout.width = visibleWidth;
        layout.height = visibleHeight;
        layout.tenBit = tenBit;
        layout.objectCount = planes;
        layout.lumaRows = static_cast<int>(pix.height);
        for (int i = 0; i < planes; ++i) layout.pitches[i] = static_cast<int>(pix.plane_fmt[i].bytesperline);
        layout.primaries = primariesFor(pix.colorspace);
        layout.matrix = matrixFor(pix.colorspace, pix.ycbcr_enc);
        layout.transfer = transferFor(pix.colorspace, pix.xfer_func);
        layout.range = pix.quantization == V4L2_QUANTIZATION_FULL_RANGE ? AVCOL_RANGE_JPEG
            : pix.quantization == V4L2_QUANTIZATION_LIM_RANGE || pix.colorspace != V4L2_COLORSPACE_DEFAULT ? AVCOL_RANGE_MPEG
            : AVCOL_RANGE_UNSPECIFIED;

        framesContext = deckV4l2FramesContext(tenBit, visibleWidth, visibleHeight);
        if (!framesContext) return fail("could not create a DRM PRIME frames context");

        v4l2_control minimum{V4L2_CID_MIN_BUFFERS_FOR_CAPTURE, 0};
        const int decoderBuffers = xioctl(fd, VIDIOC_G_CTRL, &minimum) == 0 ? std::max(minimum.value, 1) : 4;
        // Extra buffers cover the frames the pump and the renderer hold while
        // the GPU reads them; the decoder stalls if it runs out.
        v4l2_requestbuffers request{};
        request.type = captureType;
        request.memory = V4L2_MEMORY_MMAP;
        request.count = static_cast<std::uint32_t>(std::min(decoderBuffers + 6, 32));
        if (xioctl(fd, VIDIOC_REQBUFS, &request) < 0 || request.count == 0) return fail(systemError("VIDIOC_REQBUFS(CAPTURE)"));

        for (unsigned index = 0; index < request.count; ++index) {
            v4l2_plane planeInfo[VIDEO_MAX_PLANES]{};
            v4l2_buffer buffer{};
            buffer.type = captureType;
            buffer.memory = V4L2_MEMORY_MMAP;
            buffer.index = index;
            buffer.m.planes = planeInfo;
            buffer.length = VIDEO_MAX_PLANES;
            if (xioctl(fd, VIDIOC_QUERYBUF, &buffer) < 0) return fail(systemError("VIDIOC_QUERYBUF(CAPTURE)"));
            auto exportedBuffer = std::make_shared<ExportedBuffer>();
            for (int plane = 0; plane < planes; ++plane) {
                v4l2_exportbuffer exportRequest{};
                exportRequest.type = captureType;
                exportRequest.index = index;
                exportRequest.plane = static_cast<std::uint32_t>(plane);
                exportRequest.flags = O_RDONLY | O_CLOEXEC;
                if (xioctl(fd, VIDIOC_EXPBUF, &exportRequest) < 0) return fail(systemError("VIDIOC_EXPBUF"));
                exportedBuffer->fds[plane] = exportRequest.fd;
                exportedBuffer->sizes[plane] = planeInfo[plane].length;
                exportedBuffer->offsets[plane] = planeInfo[plane].data_offset;
            }
            exported.push_back(std::move(exportedBuffer));
        }

        const std::lock_guard<std::mutex> lock(capture->mutex);
        capture->planes = planes;
        for (unsigned index = 0; index < exported.size(); ++index)
            if (!capture->queue(index)) return fail(systemError("VIDIOC_QBUF(CAPTURE)"));
        int type = captureType;
        if (xioctl(fd, VIDIOC_STREAMON, &type) < 0) return fail(systemError("VIDIOC_STREAMON(CAPTURE)"));
        capture->streaming = true;
        generation = capture->generation;
        return true;
    }

    void reclaimOutput() {
        for (;;) {
            v4l2_plane planes[VIDEO_MAX_PLANES]{};
            v4l2_buffer buffer{};
            buffer.type = outputType;
            buffer.memory = V4L2_MEMORY_MMAP;
            buffer.m.planes = planes;
            buffer.length = VIDEO_MAX_PLANES;
            if (xioctl(fd, VIDIOC_DQBUF, &buffer) < 0) return;
            if (buffer.index < output.size()) output[buffer.index].queued = false;
        }
    }

    bool queueAccessUnit(const std::vector<std::uint8_t>& accessUnit, int waitMillis) {
        reclaimOutput();
        auto free = std::find_if(output.begin(), output.end(), [](const auto& buffer) { return !buffer.queued; });
        if (free == output.end()) {
            pollfd waiter{fd, POLLOUT, 0};
            if (poll(&waiter, 1, waitMillis) > 0) reclaimOutput();
            free = std::find_if(output.begin(), output.end(), [](const auto& buffer) { return !buffer.queued; });
            if (free == output.end()) return fail("the V4L2 decoder input queue is full");
        }
        if (accessUnit.size() > free->length) return fail("the access unit is larger than the V4L2 decoder input buffer");
        std::memcpy(free->data, accessUnit.data(), accessUnit.size());
        v4l2_plane planes[VIDEO_MAX_PLANES]{};
        planes[0].bytesused = static_cast<std::uint32_t>(accessUnit.size());
        v4l2_buffer buffer{};
        buffer.type = outputType;
        buffer.memory = V4L2_MEMORY_MMAP;
        buffer.index = static_cast<std::uint32_t>(free - output.begin());
        buffer.m.planes = planes;
        buffer.length = 1;
        // Only an identifier; decoders copy it to the matching CAPTURE buffer.
        ++sequence;
        buffer.timestamp.tv_sec = static_cast<time_t>(sequence / 1000000);
        buffer.timestamp.tv_usec = static_cast<suseconds_t>(sequence % 1000000);
        if (xioctl(fd, VIDIOC_QBUF, &buffer) < 0) return fail(systemError("VIDIOC_QBUF(OUTPUT)"));
        free->queued = true;
        return true;
    }

    bool handleEvents() {
        for (;;) {
            v4l2_event event{};
            if (xioctl(fd, VIDIOC_DQEVENT, &event) < 0) return true;
            if (event.type == V4L2_EVENT_SOURCE_CHANGE && (event.u.src_change.changes & V4L2_EVENT_SRC_CH_RESOLUTION)
                    && !configureCapture())
                return false;
        }
    }

    // Returns false on a decode error. The newest ready picture wins; older
    // ones go straight back to the decoder.
    bool dequeuePictures(AVFrame** newest) {
        bool corrupt = false;
        for (;;) {
            v4l2_plane planes[VIDEO_MAX_PLANES]{};
            v4l2_buffer buffer{};
            buffer.type = captureType;
            buffer.memory = V4L2_MEMORY_MMAP;
            buffer.m.planes = planes;
            buffer.length = VIDEO_MAX_PLANES;
            if (xioctl(fd, VIDIOC_DQBUF, &buffer) < 0) break;
            if (buffer.index >= exported.size()) continue;
            const bool empty = planes[0].bytesused == 0; // The LAST buffer of a drain carries no picture.
            corrupt = corrupt || (buffer.flags & V4L2_BUF_FLAG_ERROR);
            if (empty || (buffer.flags & V4L2_BUF_FLAG_ERROR)) {
                capture->requeue(generation, buffer.index);
                continue;
            }
            const auto& source = *exported[buffer.index];
            DeckV4l2FrameLayout picture = layout;
            for (int i = 0; i < layout.objectCount; ++i) {
                picture.fds[i] = source.fds[i];
                picture.sizes[i] = source.sizes[i];
                picture.offsets[i] = source.offsets[i];
            }
            auto hold = std::make_shared<CaptureHold>(capture, exported[buffer.index], generation, buffer.index);
            AVFrame* frame = deckV4l2WrapDrmFrame(picture, framesContext, std::move(hold));
            if (!frame) {
                corrupt = true;
                continue;
            }
            // Freeing an older picture requeues it; no lock is held here.
            av_frame_free(newest);
            *newest = frame;
        }
        if (corrupt) {
            av_frame_free(newest);
            error = "the V4L2 decoder reported a corrupt picture";
        }
        return !corrupt;
    }
};

DeckV4l2Decoder::DeckV4l2Decoder() : d(std::make_unique<Impl>()) {}
DeckV4l2Decoder::~DeckV4l2Decoder() = default;

bool DeckV4l2Decoder::open(int videoFormat, int width, int height) {
    return d->open(videoFormat, width, height);
}

DeckV4l2Decoder::Result DeckV4l2Decoder::decode(const std::vector<std::uint8_t>& accessUnit, int waitMillis, AVFrame** frame) {
    *frame = nullptr;
    if (d->failed || d->fd < 0) {
        if (d->error.empty()) d->error = "the V4L2 decoder is not open";
        return Result::Failed;
    }
    if (!d->queueAccessUnit(accessUnit, waitMillis)) return Result::Failed;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(std::max(waitMillis, 0));
    for (;;) {
        if (!d->handleEvents()) return Result::Failed;
        if (!d->dequeuePictures(frame)) return Result::Failed;
        if (*frame) return Result::Frame;
        const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - std::chrono::steady_clock::now()).count();
        if (remaining <= 0) return Result::Pending;
        pollfd waiter{d->fd, POLLIN | POLLPRI, 0};
        // Before the first sequence header the capture queue is idle, and a
        // decoder with nothing queued reports POLLERR rather than blocking.
        if (poll(&waiter, 1, static_cast<int>(remaining)) <= 0 || (waiter.revents & (POLLIN | POLLPRI)) == 0) {
            if (!d->handleEvents() || !d->dequeuePictures(frame)) return Result::Failed;
            return *frame ? Result::Frame : Result::Pending;
        }
    }
}

const std::string& DeckV4l2Decoder::error() const {
    return d->error;
}

const std::string& DeckV4l2Decoder::deviceDescription() const {
    return d->description;
}

} // namespace nova::deck::stream
