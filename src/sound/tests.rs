#[test]
fn max_ring_period_policy() {
    // stereo S32, device rate == graph rate: the 2048-frame default
    assert_eq!(super::max_ring_period_bytes(8, 48000, 48000), 16384);
    // a 96k device under a 48k graph needs twice the device frames per cycle
    assert_eq!(super::max_ring_period_bytes(8, 96000, 48000), 32768);
    // fat stride: the kernel ring cap binds (ring/4, frame-aligned)
    assert_eq!(super::max_ring_period_bytes(40, 48000, 48000), 819 * 40);
    // unknown graph rate falls back to device frames
    assert_eq!(super::max_ring_period_bytes(8, 48000, 0), 16384);
}

#[test]
fn advertised_quantum_cap() {
    // stride 8 @48k: ring/4 = 4096 device frames >= the 2048 default - no cap
    assert_eq!(super::advertised_quantum_cap_frames(8, 48000), None);
    // 192k device: 4096 device frames is only 1024 frames at a 48k graph
    assert_eq!(super::advertised_quantum_cap_frames(8, 192000), Some(4096));
    // 96k device: on the 42.7ms boundary - published for a 44.1k clock.rate
    // (inert at the 48k default, where the cap equals the max quantum)
    assert_eq!(super::advertised_quantum_cap_frames(8, 96000), Some(4096));
    // fat stride @48k: 819 device frames < 2048 - the original case
    assert_eq!(super::advertised_quantum_cap_frames(40, 48000), Some(819));
    // 44.1k stereo: 4096 device frames is ~4458 graph frames - no cap
    assert_eq!(super::advertised_quantum_cap_frames(8, 44100), None);
}

#[test]
fn pcm_format_widths_cover_u8_float_and_three_byte_24() {
    const STEREO: u32 = 2 << 20;

    assert_eq!(super::afmt_frame_bytes(super::AFMT_U8), 1);
    assert_eq!(super::afmt_frame_bytes(super::AFMT_U8 | STEREO), 2);
    assert_eq!(super::afmt_frame_bytes(super::AFMT_S24_LE), 3);
    assert_eq!(super::afmt_frame_bytes(super::AFMT_S24_BE | STEREO), 6);
    assert_eq!(super::afmt_frame_bytes(super::AFMT_F32_LE), 4);
    assert_eq!(super::afmt_frame_bytes(super::AFMT_F32_BE | STEREO), 8);
}

#[test]
fn fallback_formats_cover_the_supported_surface() {
    let mapped = crate::utils::FORMAT_MAP
        .iter()
        .fold(0, |formats, (oss, _, _)| formats | oss);
    assert_eq!(super::DspCaps::fallback().formats, mapped);
}

#[test]
fn drain_quantum_probe() {
    for unit in [0u32, 1, 6] {
        let node = format!("/dev/dsp{unit}"); // the production string shape
        println!(
            "{}: play {} ns, rec {} ns",
            node,
            super::drain_quantum_ns(&node, true),
            super::drain_quantum_ns(&node, false)
        );
    }
}

use super::test_util::{drain, pattern, pipe_pair};

#[test]
fn write_zeroes_floors_to_frames() {
    let (r, w) = pipe_pair(true, true);
    let mut dsp = super::DspWriter::test_on_fd(w, 8);
    dsp.write_zeroes(2047); // odelay through a vchan can produce counts like this
    let got = drain(r);
    assert_eq!(got.len(), 2040);
    assert!(got.iter().all(|&b| b == 0));
    unsafe { libc::close(r) };
}

// The 2026-07-17 noise bug: a short write that splits a frame must never
// leave the device byte stream mid-sample - every sample after an
// unaligned boundary is stitched from two neighbors (white noise with the
// audio faintly underneath).
#[test]
fn short_write_keeps_stream_frame_aligned() {
    let (r, w) = pipe_pair(true, true);
    let mut dsp = super::DspWriter::test_on_fd(w, 8);

    // fill the pipe to capacity, then free a mid-frame hole: the next write
    // is forced short at an unaligned count, like a full OSS ring
    let total_fill = super::test_util::fill_pipe(w);
    super::test_util::free_space(r, 2046);

    // 2046 = 255 frames + 6 bytes: the kernel takes all of it, the 2-byte
    // frame tail can't fit, and the split is recorded rather than dropped
    let a = pattern(4096, 1);
    let ret = dsp.write(&a);
    assert_eq!(ret, 2046);
    assert_eq!(dsp.frame_off, 6);

    let queued = drain(r); // remaining filler, then the accepted head
    assert_eq!(queued.len(), total_fill); // the 2046-byte hole was exactly refilled
    assert_eq!(&queued[queued.len() - 2046..], &a[..2046]);

    // with space available again, the next write closes the split frame
    // with zeros before any new data: the stream returns to a frame
    // boundary instead of shifting every later sample
    let b = pattern(4096, 2);
    let ret = dsp.write(&b);
    assert_eq!(ret, 4096);
    assert_eq!(dsp.frame_off, 0);
    let tail = drain(r);
    assert_eq!(&tail[..2], &[0, 0]);
    assert_eq!(&tail[2..], &b[..]);
    assert_eq!((2046 + tail.len()) % 8, 0); // the stream is whole frames again
    unsafe { libc::close(r) };
}

// capture mirror image: a read that lands mid-frame must hide the torn
// frame's head and discard its tail, so every returned buffer starts on a
// frame boundary
#[test]
fn read_hides_torn_frame_and_realigns() {
    let (r, w) = pipe_pair(false, false);
    let mut dsp = super::Dsp::test_on_fd(r, 8);
    let s = pattern(2056, 3);
    assert_eq!(unsafe { libc::write(w, s.as_ptr().cast(), 2046) }, 2046);

    // 2046 available < 4096 requested: the pipe returns it all, mid-frame
    let mut buf = vec![0u8; 4096];
    let n = dsp.read(&mut buf[..4096]);
    assert_eq!(n, 2040);
    assert_eq!(&buf[..2040], &s[..2040]);
    assert_eq!(dsp.skip, 2);

    // the stream continues; the torn frame's tail is skipped and the next
    // buffer starts exactly on the following frame boundary
    assert_eq!(
        unsafe { libc::write(w, s.as_ptr().add(2046).cast(), 10) },
        10
    );
    let n = dsp.read(&mut buf[..8]);
    assert_eq!(n, 8);
    assert_eq!(&buf[..8], &s[2048..2056]);
    assert_eq!(dsp.skip, 0);
    unsafe { libc::close(w) };
}
