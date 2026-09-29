#pragma once
#include <string_view>

struct AVBufferRef;

namespace nova::deck::stream {

struct DeckDecodeLimits {
    int maxWidth = 0, maxHeight = 0;
    bool supports(int width, int height) const {
        return width > 0 && height > 0 && width <= maxWidth && height <= maxHeight;
    }
};

// Decoder support only. Main10 never implies an HDR display/presenter.
struct DeckVideoDecodeSupport {
    DeckDecodeLimits h264, hevc, main10, pyrowave;
    bool supports(int videoFormat, int width, int height) const;
};

// Query VAAPI codec limits and, in enabled builds, the independent PyroWave
// Vulkan decoder. A missing VAAPI device does not disable Vulkan decoding.
// Missing profiles, decode entrypoints, surface formats or size limits remain unsupported.
DeckVideoDecodeSupport probeVideoDecodeSupport(AVBufferRef* device);
// VAAPI, PyroWave and any V4L2 stateful decoder (see deck_v4l2_decoder.h).
DeckVideoDecodeSupport detectVideoDecodeSupport();
// Per codec, keep whichever backend reports the larger picture. A stream is
// decoded by VAAPI when VAAPI supports it, otherwise by V4L2.
DeckVideoDecodeSupport mergeDecodeSupport(DeckVideoDecodeSupport vaapi, const DeckVideoDecodeSupport& v4l2);

// Auto prefers HEVC only when both endpoints support this stream size.
int selectSdrVideoFormat(std::string_view preference, bool hostH264, bool hostHevc,
    const DeckVideoDecodeSupport& decoder, int width, int height, bool hostPyrowave = false);

} // namespace nova::deck::stream
