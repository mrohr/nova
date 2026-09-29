#pragma once

#include "stream/deck_video_capabilities.h"

extern "C" {
#include <libavutil/pixfmt.h>
}

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

struct AVBufferRef;
struct AVFrame;

namespace nova::deck::stream {

// Hardware decode through a Linux V4L2 stateful memory-to-memory decoder, such
// as Qualcomm iris or venus on Snapdragon devices that have no VA-API driver.
//
// FFmpeg's h264_v4l2m2m/hevc_v4l2m2m wrappers only return frames in mapped
// system memory, which would put a CPU copy between the decoder and the GPU.
// This decoder drives the V4L2 queues directly, exports every CAPTURE buffer
// as a DMA-BUF and returns AV_PIX_FMT_DRM_PRIME frames with the same two-layer
// R8/GR88 (R16/GR1616 for 10-bit) shape Mesa's VA-API export has, so both
// presenters import them unchanged. A CAPTURE buffer returns to the decoder
// only when the last reference to its frame is released, after the GPU reads.

// Codec limits of the first stateful V4L2 decoder that reports each codec with
// a linear NV12 (or P010 for Main10) output. A node that cannot open, such as
// one hidden by a Flatpak sandbox or missing its firmware, reports nothing.
DeckVideoDecodeSupport probeV4l2DecodeSupport();

// One decoded CAPTURE buffer, described for DRM PRIME export.
struct DeckV4l2FrameLayout {
    int width = 0, height = 0; // Visible picture, anchored at the top left.
    bool tenBit = false;
    // One object holds both planes (NV12, P010), or each plane has its own
    // (NV12M). In the single-object case chroma starts after `lumaRows` rows.
    int objectCount = 1;
    int fds[2] = {-1, -1};
    std::size_t sizes[2] = {};
    std::size_t offsets[2] = {};
    int pitches[2] = {};
    int lumaRows = 0;
    AVColorPrimaries primaries = AVCOL_PRI_UNSPECIFIED;
    AVColorTransferCharacteristic transfer = AVCOL_TRC_UNSPECIFIED;
    AVColorSpace matrix = AVCOL_SPC_UNSPECIFIED;
    AVColorRange range = AVCOL_RANGE_UNSPECIFIED;
};

// A DRM PRIME frames context carrying the software layout (NV12 or P010) that
// the color checks and libplacebo read. It opens no DRM device.
AVBufferRef* deckV4l2FramesContext(bool tenBit, int width, int height);

// Wraps the layout as an AV_PIX_FMT_DRM_PRIME frame. `owner` is kept alive,
// and so are the DMA-BUFs it owns, until the frame's last reference is freed.
// Returns nullptr for a layout the presenters could not import.
AVFrame* deckV4l2WrapDrmFrame(const DeckV4l2FrameLayout& layout, AVBufferRef* framesContext, std::shared_ptr<void> owner);

class DeckV4l2Decoder final {
public:
    enum class Result { Frame, Pending, Failed };

    DeckV4l2Decoder();
    ~DeckV4l2Decoder();
    DeckV4l2Decoder(const DeckV4l2Decoder&) = delete;
    DeckV4l2Decoder& operator=(const DeckV4l2Decoder&) = delete;

    bool open(int videoFormat, int width, int height);
    // Queues one Annex-B access unit and waits up to `waitMillis` for a decoded
    // picture. Pending means the unit was accepted but no picture is ready yet,
    // as before the decoder has parsed the first sequence header. On Frame,
    // `frame` receives a new frame the caller frees.
    Result decode(const std::vector<std::uint8_t>& accessUnit, int waitMillis, AVFrame** frame);
    const std::string& error() const;
    const std::string& deviceDescription() const;

private:
    struct Impl;
    std::unique_ptr<Impl> d;
};

} // namespace nova::deck::stream
