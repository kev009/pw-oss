use super::{device_period_bytes, enum_format_widths, ns_to_bytes, ns_to_frame_bytes};

fn caps(
    formats: u32,
    channels: (u32, u32),
    rate_range: (u32, u32),
    rates: &[u32],
    preferred_rate: Option<u32>,
    convertless: bool,
) -> crate::sound::DspCaps {
    crate::sound::DspCaps {
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
    use crate::sound::{AFMT_S16_LE, AFMT_S32_LE};
    use libspa::pod::{ChoiceValue, Object, Value, ValueArray};
    use libspa::sys::*;
    use libspa::utils::{Choice, ChoiceEnum, ChoiceFlags, Id};

    // a multi-format device with a rate range: choice-enum of formats,
    // choice-range of rates, ids, ints and the id-array of positions
    let caps = caps(
        AFMT_S16_LE | AFMT_S32_LE,
        (1, 2),
        (8000, 192000),
        &[],
        None,
        false,
    );
    let pod = super::build_enum_format_info(&caps, 0).unwrap();
    assert_eq!(
        super::parse_back(&pod),
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
                            alternatives: vec![Id(SPA_AUDIO_FORMAT_S16_LE)],
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

#[test]
fn latency_offset_props_parse_back_with_params_struct() {
    use libspa::pod::{Object, Value};
    use libspa::sys::*;

    // the sink's Props shape: a long, plus the params struct of
    // string/int pairs
    let pod =
        super::build_latency_offset_props(50_000_000, &[("oss.delay", 4), ("oss.fragment", 0)]);
    assert_eq!(
        super::parse_back(&pod),
        Value::Object(Object {
            type_: SPA_TYPE_OBJECT_Props,
            id: SPA_PARAM_Props,
            properties: vec![
                super::pod_prop(SPA_PROP_latencyOffsetNsec, Value::Long(50_000_000)),
                super::pod_prop(
                    SPA_PROP_params,
                    Value::Struct(vec![
                        Value::String("oss.delay".to_string()),
                        Value::Int(4),
                        Value::String("oss.fragment".to_string()),
                        Value::Int(0),
                    ]),
                ),
            ],
        })
    );
}

#[test]
fn frame_bytes_round_up_and_stay_saturated() {
    // the floored hardware quantum comes back up to a whole frame
    assert_eq!(ns_to_frame_bytes(5_333_333, 48000, 8), 2048);
    assert_eq!(ns_to_frame_bytes(0, 48000, 8), 0);
    // a saturated conversion must not overflow the round-up (debug builds
    // would abort the data loop; release would wrap the chunk to zero) -
    // it stays pinned at the largest frame multiple
    assert_eq!(
        ns_to_frame_bytes(u64::MAX, u32::MAX, 8),
        u32::MAX - u32::MAX % 8
    );
    assert_eq!(ns_to_frame_bytes(u64::MAX, u32::MAX, 1), u32::MAX);
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

#[test]
fn ns_to_bytes_floors() {
    // the production hw-quantum shape: 256 frames @ 48k S32 stereo is
    // 5333333 ns, which floors to 2047 - call sites round back up to the
    // stride and rely on this direction; a rounding change here silently
    // shifts every geometry derived from it
    assert_eq!(ns_to_bytes(5_333_333, 48000, 8), 2047);
    assert_eq!(ns_to_bytes(5_333_334, 48000, 8), 2048);
    assert_eq!(ns_to_bytes(0, 48000, 8), 0);
    assert_eq!(ns_to_bytes(1_000_000_000, 48000, 8), 384_000);
    // saturates instead of wrapping
    assert_eq!(ns_to_bytes(u64::MAX, u32::MAX, u32::MAX), u32::MAX);
}

#[test]
fn device_period_scales_with_rate_ratio() {
    assert_eq!(device_period_bytes(2048, 48000, 48000, 8), 16384);
    assert_eq!(device_period_bytes(2048, 96000, 48000, 8), 32768);
    assert_eq!(device_period_bytes(2048, 44100, 48000, 8), 15048); // floors
    assert_eq!(device_period_bytes(2048, 48000, 0, 8), 0);
    assert_eq!(
        device_period_bytes(u64::MAX, u32::MAX, 1, u32::MAX),
        u32::MAX
    );
}
