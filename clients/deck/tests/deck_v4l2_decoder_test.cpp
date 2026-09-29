#include "stream/deck_stream_media_adapters.h"
#include "stream/deck_v4l2_decoder.h"
#include <QCoreApplication>
extern "C" {
#include <libavutil/frame.h>
#include <libavutil/hwcontext.h>
#include <libavutil/hwcontext_drm.h>
}
#include <libdrm/drm_fourcc.h>
#include <sys/mman.h>
#include <unistd.h>
#include <cstdlib>
#include <iostream>

using namespace nova::deck::stream;
namespace { void require(bool ok, const char* message) { if (!ok) { std::cerr << message << '\n'; std::exit(1); } } }

namespace {
// Stands in for an exported CAPTURE buffer: iris pads NV12 rows to 128 bytes
// and luma to 32 rows, so 1080p chroma starts after 1088 rows.
struct FakeBuffer {
    int fd = -1;
    std::size_t size = 0;
    explicit FakeBuffer(std::size_t bytes) : fd(memfd_create("nova-v4l2-test", MFD_CLOEXEC)), size(bytes) {
        require(fd >= 0 && ftruncate(fd, static_cast<off_t>(bytes)) == 0, "memfd allocation failed");
    }
    ~FakeBuffer() { close(fd); }
};

DeckV4l2FrameLayout nv12Layout(const FakeBuffer& buffer) {
    DeckV4l2FrameLayout layout;
    layout.width = 1920;
    layout.height = 1080;
    layout.objectCount = 1;
    layout.fds[0] = buffer.fd;
    layout.sizes[0] = buffer.size;
    layout.pitches[0] = 2048;
    layout.lumaRows = 1088;
    layout.primaries = AVCOL_PRI_BT709;
    layout.transfer = AVCOL_TRC_BT709;
    layout.matrix = AVCOL_SPC_BT709;
    layout.range = AVCOL_RANGE_MPEG;
    return layout;
}
}

int main(int argc, char** argv) {
    QCoreApplication app(argc, argv);

    const FakeBuffer buffer(2048 * 1088 * 3 / 2);
    AVBufferRef* frames = deckV4l2FramesContext(false, 1920, 1080);
    require(frames, "DRM PRIME frames context was not created without a DRM node");
    auto owner = std::make_shared<int>(0);
    std::weak_ptr<int> ownerAlive = owner;
    AVFrame* frame = deckV4l2WrapDrmFrame(nv12Layout(buffer), frames, std::move(owner));
    require(frame && frame->format == AV_PIX_FMT_DRM_PRIME && frame->width == 1920 && frame->height == 1080,
        "V4L2 buffer was not wrapped as the visible DRM PRIME picture");
    const auto& drm = *reinterpret_cast<const AVDRMFrameDescriptor*>(frame->data[0]);
    require(drm.nb_objects == 1 && drm.objects[0].fd == buffer.fd && drm.objects[0].format_modifier == DRM_FORMAT_MOD_LINEAR,
        "exported buffer is not one linear object");
    require(drm.nb_layers == 2 && drm.layers[0].format == DRM_FORMAT_R8 && drm.layers[1].format == DRM_FORMAT_GR88,
        "NV12 did not split into the R8/GR88 layers both presenters import");
    require(drm.layers[1].planes[0].object_index == 0 && drm.layers[1].planes[0].offset == 2048 * 1088
            && drm.layers[1].planes[0].pitch == 2048,
        "chroma did not start after the padded luma rows");
    const auto color = DeckVideoColorInfo::fromFrame(*frame);
    require(color.bitDepth == 8 && color.yuv420 && color.matrix == AVCOL_SPC_BT709 && color.range == AVCOL_RANGE_MPEG,
        "decoder color tags or NV12 depth were lost");

    auto lease = DeckQrhiVaapiFrameLease::cloneHardwareFrame(*frame);
    av_frame_free(&frame);
    require(lease && lease->valid() && !ownerAlive.expired(), "lease did not retain the decoder buffer");
    const auto plan = DeckVaapiEglImagePresenter::validateDrmPrimeMetadata(lease->exportDrmPrimeDescriptor());
    require(plan.status == DeckQrhiVaapiImportStatus::DrmPrimeExported && plan.drmPrimeLayerCount == 2,
        "EGL presenter refused the V4L2 DRM PRIME layout");
    lease.reset();
    require(ownerAlive.expired(), "decoder buffer was not returned after the last frame reference");

    // Luma rows that overlap the chroma plane, or a chroma plane beyond the
    // exported object, must not reach the GPU import.
    auto broken = nv12Layout(buffer);
    broken.lumaRows = 1000;
    require(!deckV4l2WrapDrmFrame(broken, frames, {}), "chroma overlapping luma was accepted");
    broken = nv12Layout(buffer);
    broken.sizes[0] = 2048 * 1088;
    require(!deckV4l2WrapDrmFrame(broken, frames, {}), "chroma outside the buffer was accepted");
    broken = nv12Layout(buffer);
    broken.objectCount = 0;
    require(!deckV4l2WrapDrmFrame(broken, frames, {}), "a picture with no buffer was accepted");
    require(!deckV4l2WrapDrmFrame(nv12Layout(buffer), nullptr, {}), "a picture with no frames context was accepted");
    av_buffer_unref(&frames);

    // NV12M: each plane in its own buffer.
    const FakeBuffer chroma(2048 * 544);
    frames = deckV4l2FramesContext(false, 1920, 1080);
    auto split = nv12Layout(buffer);
    split.objectCount = 2;
    split.fds[1] = chroma.fd;
    split.sizes[1] = chroma.size;
    split.pitches[1] = 2048;
    frame = deckV4l2WrapDrmFrame(split, frames, {});
    require(frame, "two-plane NV12M was refused");
    const auto& splitDrm = *reinterpret_cast<const AVDRMFrameDescriptor*>(frame->data[0]);
    require(splitDrm.nb_objects == 2 && splitDrm.layers[1].planes[0].object_index == 1 && splitDrm.layers[1].planes[0].offset == 0,
        "NV12M chroma did not use its own buffer");
    av_frame_free(&frame);
    av_buffer_unref(&frames);

    // Main10 decodes to P010.
    const FakeBuffer tenBitBuffer(4096 * 1088 * 3 / 2);
    frames = deckV4l2FramesContext(true, 1920, 1080);
    auto p010 = nv12Layout(tenBitBuffer);
    p010.tenBit = true;
    p010.pitches[0] = 4096;
    frame = deckV4l2WrapDrmFrame(p010, frames, {});
    require(frame, "P010 was refused");
    const auto& tenBitDrm = *reinterpret_cast<const AVDRMFrameDescriptor*>(frame->data[0]);
    require(tenBitDrm.layers[0].format == DRM_FORMAT_R16 && tenBitDrm.layers[1].format == DRM_FORMAT_GR1616
            && DeckVideoColorInfo::fromFrame(*frame).bitDepth == 10,
        "P010 did not map to 10-bit R16/GR1616 layers");
    av_frame_free(&frame);
    av_buffer_unref(&frames);

    // VAAPI keeps a stream it can decode; V4L2 fills codecs VAAPI lacks.
    const DeckVideoDecodeSupport vaapi{.h264 = {4096, 4096}};
    const DeckVideoDecodeSupport v4l2{.h264 = {1920, 1088}, .hevc = {8192, 4352}};
    const auto merged = mergeDecodeSupport(vaapi, v4l2);
    require(merged.h264.maxWidth == 4096 && merged.hevc.maxWidth == 8192 && !merged.main10.supports(1280, 720),
        "decoder backends were merged incorrectly");

    // Hosts without a stateful decoder report nothing and never guess.
    const auto probed = probeV4l2DecodeSupport();
    for (const auto& limits : {probed.h264, probed.hevc, probed.main10})
        require((limits.maxWidth == 0) == (limits.maxHeight == 0), "V4L2 probe reported half a size limit");
    DeckV4l2Decoder unopened;
    AVFrame* none = nullptr;
    require(unopened.decode({0, 0, 0, 1}, 0, &none) == DeckV4l2Decoder::Result::Failed && !none,
        "an unopened decoder accepted video");
    DeckV4l2Decoder unsupported;
    require(!unsupported.open(VIDEO_FORMAT_AV1_MAIN8, 1280, 720) && !unsupported.error().empty(),
        "an unsupported codec opened a V4L2 decoder");
    return 0;
}
