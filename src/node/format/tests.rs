use super::enum_format_widths;

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
fn fallback_formats_cover_the_supported_surface() {
    let mapped = super::FORMAT_MAP
        .iter()
        .fold(0, |formats, (oss, _, _)| formats | oss);
    assert_eq!(crate::oss::DspCaps::fallback().formats, mapped);
}

fn caps(
    formats: u32,
    channels: (u32, u32),
    rate_range: (u32, u32),
    rates: &[u32],
    preferred_rate: Option<u32>,
    convertless: bool,
) -> crate::oss::DspCaps {
    crate::oss::DspCaps {
        formats,
        min_channels: channels.0,
        max_channels: channels.1,
        min_rate: rate_range.0,
        max_rate: rate_range.1,
        preferred_rate,
        rates: rates.to_vec(),
        convertless,
    }
}

// Parse-back semantics: run representative pods that WirePlumber and the
// audio adapter parse off the wire back through the same libspa
// PodDeserializer those consumers use, and pin what they depend on -
// property keys and order, values, choice types, array contents and
// per-property flags. (A byte-identical old-vs-new comparison proved the
// serializer migration once; the lasting invariant is the parsed
// meaning, not the encoding.)
#[test]
fn enum_format_parses_back_with_expected_choices() {
    use crate::oss::{
        AFMT_F32_BE, AFMT_F32_LE, AFMT_S16_BE, AFMT_S16_LE, AFMT_S24_BE, AFMT_S24_LE, AFMT_S32_BE,
        AFMT_S32_LE, AFMT_U8,
    };
    use libspa::pod::{ChoiceValue, Object, Value, ValueArray};
    use libspa::sys::*;
    use libspa::utils::{Choice, ChoiceEnum, ChoiceFlags, Id};

    // a multi-format device with a rate range: choice-enum of formats,
    // choice-range of rates, ids, ints and the id-array of positions
    let caps = caps(
        AFMT_U8
            | AFMT_S16_LE
            | AFMT_S16_BE
            | AFMT_S24_LE
            | AFMT_S24_BE
            | AFMT_S32_LE
            | AFMT_S32_BE
            | AFMT_F32_LE
            | AFMT_F32_BE,
        (1, 2),
        (8000, 192000),
        &[],
        None,
        false,
    );
    let pod = super::build_enum_format_info(&caps, 0).unwrap();
    assert_eq!(
        crate::spa::parse_back(&pod),
        Value::Object(Object {
            type_: SPA_TYPE_OBJECT_Format,
            id: SPA_PARAM_EnumFormat,
            properties: vec![
                super::pod_prop(SPA_FORMAT_mediaType, Value::Id(Id(SPA_MEDIA_TYPE_audio))),
                super::pod_prop(
                    SPA_FORMAT_mediaSubtype,
                    Value::Id(Id(SPA_MEDIA_SUBTYPE_raw)),
                ),
                // the best (widest) native format is the enum default
                super::pod_prop(
                    SPA_FORMAT_AUDIO_format,
                    Value::Choice(ChoiceValue::Id(Choice(
                        ChoiceFlags::empty(),
                        ChoiceEnum::Enum {
                            default: Id(SPA_AUDIO_FORMAT_S32_LE),
                            alternatives: vec![
                                Id(SPA_AUDIO_FORMAT_S32_BE),
                                Id(SPA_AUDIO_FORMAT_F32_LE),
                                Id(SPA_AUDIO_FORMAT_F32_BE),
                                Id(SPA_AUDIO_FORMAT_S24_LE),
                                Id(SPA_AUDIO_FORMAT_S24_BE),
                                Id(SPA_AUDIO_FORMAT_S16_LE),
                                Id(SPA_AUDIO_FORMAT_S16_BE),
                                Id(SPA_AUDIO_FORMAT_U8),
                            ],
                        },
                    ))),
                ),
                // a rate range defaults to the host reference 48000
                super::pod_prop(
                    SPA_FORMAT_AUDIO_rate,
                    Value::Choice(ChoiceValue::Int(Choice(
                        ChoiceFlags::empty(),
                        ChoiceEnum::Range {
                            default: 48000,
                            min: 8000,
                            max: 192000,
                        },
                    ))),
                ),
                // stereo first (the 9875023 invariant), FL/FR positions
                super::pod_prop(SPA_FORMAT_AUDIO_channels, Value::Int(2)),
                super::pod_prop(
                    SPA_FORMAT_AUDIO_position,
                    Value::ValueArray(ValueArray::Id(vec![
                        Id(SPA_AUDIO_CHANNEL_FL),
                        Id(SPA_AUDIO_CHANNEL_FR),
                    ])),
                ),
            ],
        })
    );
}

fn raw_format(format: u32) -> libspa::sys::spa_audio_info_raw {
    let mut raw: libspa::sys::spa_audio_info_raw = unsafe { std::mem::zeroed() };
    raw.format = format;
    raw.rate = 48000;
    raw.channels = 2;
    raw.position[0] = libspa::sys::SPA_AUDIO_CHANNEL_FL;
    raw.position[1] = libspa::sys::SPA_AUDIO_CHANNEL_FR;
    raw
}

#[test]
fn format_snap_preserves_and_selects_float_three_byte_24_and_u8() {
    use crate::oss::{AFMT_F32_LE, AFMT_S16_LE, AFMT_S24_BE, AFMT_U8};
    use libspa::sys::{
        SPA_AUDIO_FORMAT_F32_LE, SPA_AUDIO_FORMAT_S16_LE, SPA_AUDIO_FORMAT_S24_BE,
        SPA_AUDIO_FORMAT_U8,
    };

    let float_caps = caps(
        AFMT_F32_LE | AFMT_S16_LE,
        (1, 2),
        (8000, 192000),
        &[],
        None,
        false,
    );
    let mut native_float = raw_format(SPA_AUDIO_FORMAT_F32_LE);
    assert!(!super::snap_raw_to_caps(&float_caps, &mut native_float));
    assert_eq!(native_float.format, SPA_AUDIO_FORMAT_F32_LE);

    let mut snapped_float = raw_format(SPA_AUDIO_FORMAT_S24_BE);
    assert!(super::snap_raw_to_caps(&float_caps, &mut snapped_float));
    assert_eq!(snapped_float.format, SPA_AUDIO_FORMAT_F32_LE);

    let three_byte_caps = caps(AFMT_S24_BE, (1, 2), (8000, 192000), &[], None, false);
    let mut snapped_three_byte = raw_format(SPA_AUDIO_FORMAT_F32_LE);
    assert!(super::snap_raw_to_caps(
        &three_byte_caps,
        &mut snapped_three_byte
    ));
    assert_eq!(snapped_three_byte.format, SPA_AUDIO_FORMAT_S24_BE);

    let u8_caps = caps(AFMT_U8, (1, 2), (8000, 192000), &[], None, false);
    let mut snapped_u8 = raw_format(SPA_AUDIO_FORMAT_S16_LE);
    assert!(super::snap_raw_to_caps(&u8_caps, &mut snapped_u8));
    assert_eq!(snapped_u8.format, SPA_AUDIO_FORMAT_U8);

    assert_eq!(
        super::oss_format_info(SPA_AUDIO_FORMAT_F32_LE),
        Some((AFMT_F32_LE, 4))
    );
    assert_eq!(
        super::oss_format_info(SPA_AUDIO_FORMAT_S24_BE),
        Some((AFMT_S24_BE, 3))
    );
    assert_eq!(
        super::oss_format_info(SPA_AUDIO_FORMAT_S16_LE),
        Some((AFMT_S16_LE, 2))
    );
    assert_eq!(
        super::oss_format_info(SPA_AUDIO_FORMAT_U8),
        Some((AFMT_U8, 1))
    );
}

#[test]
fn convertless_format_offers_are_native_only() {
    use crate::oss::{AFMT_F32_LE, AFMT_S24_BE, AFMT_U8};
    use libspa::sys::{SPA_AUDIO_FORMAT_F32_LE, SPA_AUDIO_FORMAT_S24_BE, SPA_AUDIO_FORMAT_U8};

    let native = caps(
        AFMT_F32_LE | AFMT_S24_BE | AFMT_U8,
        (1, 2),
        (8000, 192000),
        &[],
        None,
        true,
    );
    assert_eq!(
        super::offered_formats(&native),
        [
            SPA_AUDIO_FORMAT_F32_LE,
            SPA_AUDIO_FORMAT_S24_BE,
            SPA_AUDIO_FORMAT_U8,
        ]
    );

    let unmatched = caps(0, (1, 2), (8000, 192000), &[], None, true);
    assert!(super::offered_formats(&unmatched).is_empty());

    let feeder = caps(0, (1, 2), (8000, 192000), &[], None, false);
    assert_eq!(
        super::offered_formats(&feeder),
        super::FORMAT_MAP
            .iter()
            .map(|(_, spa, _)| *spa)
            .collect::<Vec<_>>()
    );
}

// the 9875023 invariant: pulse falls back to the LAST EnumFormat map when
// Format is gone, and HW route volume is always 2ch - so whenever the
// device can do stereo, the list must open with it (host default) and end
// with it (fallback), or cvolume.channels flips and pops the volume OSD
#[test]
fn stereo_pins_both_ends_of_enum_format() {
    for (min, max) in [(1u32, 2u32), (2, 2), (1, 8), (2, 8), (1, 10), (2, 32)] {
        let widths = enum_format_widths(min, max);
        assert_eq!(
            *widths.first().unwrap(),
            2,
            "min {min} max {max}: {widths:?}"
        );
        assert_eq!(
            *widths.last().unwrap(),
            2,
            "min {min} max {max}: {widths:?}"
        );
        assert!(
            widths.contains(&max),
            "native width lost: min {min} max {max}: {widths:?}"
        );
        assert!(
            widths.iter().all(|w| *w >= min && *w <= max),
            "width out of range: min {min} max {max}: {widths:?}"
        );
        assert_eq!(
            widths.iter().filter(|w| **w == 2).count().min(2),
            if widths.len() == 1 { 1 } else { 2 }
        );
    }
    // concrete shapes: this machine's HDMI 8ch and a 10ch USB mixer
    assert_eq!(enum_format_widths(2, 8), [2, 4, 6, 8, 2]);
    assert_eq!(enum_format_widths(1, 10), [2, 4, 6, 8, 1, 10, 2]);
    assert_eq!(enum_format_widths(2, 2), [2]);
    // no stereo support: no pinning, the native width still leads/closes
    assert_eq!(enum_format_widths(1, 1), [1]);
    assert_eq!(enum_format_widths(4, 8), [4, 6, 8]);
    assert_eq!(enum_format_widths(3, 3), [3]);
}
