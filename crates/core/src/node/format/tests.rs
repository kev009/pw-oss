use crate::backend;
use crate::spa::parse_back;

fn caps(
    formats: &[u32],
    channels: (u32, u32),
    rate_range: (u32, u32),
    rates: &[u32],
    preferred_rate: Option<u32>,
    convertless: bool,
) -> backend::StreamCaps {
    let mut layouts = (channels.0..=channels.1)
        .map(|channels| backend::ChannelLayout {
            channels,
            positions: match channels {
                1 => Some(vec![libspa::sys::SPA_AUDIO_CHANNEL_MONO]),
                2 => Some(vec![
                    libspa::sys::SPA_AUDIO_CHANNEL_FL,
                    libspa::sys::SPA_AUDIO_CHANNEL_FR,
                ]),
                _ => None,
            },
        })
        .collect::<Vec<_>>();
    if channels.0 <= 2 && channels.1 >= 2 {
        let stereo = layouts
            .iter()
            .find(|layout| layout.channels == 2)
            .cloned()
            .unwrap();
        layouts.retain(|layout| layout.channels != 2);
        layouts.insert(0, stereo.clone());
        if layouts.last() != Some(&stereo) {
            layouts.push(stereo);
        }
    }
    backend::StreamCaps {
        configurations: vec![backend::StreamConfiguration {
            formats: formats.to_vec(),
            channels: backend::ChannelSet::Discrete(layouts),
            rates: if rates.is_empty() {
                backend::RateSet::Range {
                    min: rate_range.0,
                    max: rate_range.1,
                }
            } else {
                backend::RateSet::Discrete(rates.to_vec())
            },
            preferred_rate,
            rate_tolerance: 50,
            conversion: if convertless {
                backend::ConversionPath::None
            } else {
                backend::ConversionPath::Kernel
            },
            flags: if convertless {
                backend::ConfigurationFlags::default()
            } else {
                backend::ConfigurationFlags::with_layout_reorder()
            },
        }],
        preferred: 0,
    }
}

// Parse-back semantics: run representative pods that WirePlumber and the
// audio adapter parse off the wire back through the same libspa
// PodDeserializer those consumers use, and pin what they depend on -
// property keys and order, values, choice types, array contents and
// per-property flags. The invariant is the parsed meaning, not the encoding.
#[test]
fn enum_format_parses_back_with_expected_choices() {
    use libspa::pod::{ChoiceValue, Object, Value, ValueArray};
    use libspa::sys::*;
    use libspa::utils::{Choice, ChoiceEnum, ChoiceFlags, Id};

    // a multi-format device with a rate range: choice-enum of formats,
    // choice-range of rates, ids, ints and the id-array of positions
    let caps = caps(
        &[
            SPA_AUDIO_FORMAT_S32_LE,
            SPA_AUDIO_FORMAT_S32_BE,
            SPA_AUDIO_FORMAT_F32_LE,
            SPA_AUDIO_FORMAT_F32_BE,
            SPA_AUDIO_FORMAT_S24_LE,
            SPA_AUDIO_FORMAT_S24_BE,
            SPA_AUDIO_FORMAT_S16_LE,
            SPA_AUDIO_FORMAT_S16_BE,
            SPA_AUDIO_FORMAT_U8,
            SPA_AUDIO_FORMAT_S8,
            SPA_AUDIO_FORMAT_U16_LE,
            SPA_AUDIO_FORMAT_U16_BE,
            SPA_AUDIO_FORMAT_U24_LE,
            SPA_AUDIO_FORMAT_U24_BE,
            SPA_AUDIO_FORMAT_U32_LE,
            SPA_AUDIO_FORMAT_U32_BE,
            SPA_AUDIO_FORMAT_ULAW,
            SPA_AUDIO_FORMAT_ALAW,
        ],
        (1, 2),
        (8000, 192000),
        &[],
        None,
        false,
    );
    let pod = super::build_enum_format_info(&caps, 0).unwrap();
    assert_eq!(
        parse_back(&pod),
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
                                Id(SPA_AUDIO_FORMAT_S8),
                                Id(SPA_AUDIO_FORMAT_U16_LE),
                                Id(SPA_AUDIO_FORMAT_U16_BE),
                                Id(SPA_AUDIO_FORMAT_U24_LE),
                                Id(SPA_AUDIO_FORMAT_U24_BE),
                                Id(SPA_AUDIO_FORMAT_U32_LE),
                                Id(SPA_AUDIO_FORMAT_U32_BE),
                                Id(SPA_AUDIO_FORMAT_ULAW),
                                Id(SPA_AUDIO_FORMAT_ALAW),
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

fn enum_format_summary(caps: &backend::StreamCaps, index: u32) -> (u32, i32, i32) {
    use libspa::pod::Value;
    use libspa::sys::{SPA_FORMAT_AUDIO_channels, SPA_FORMAT_AUDIO_format, SPA_FORMAT_AUDIO_rate};

    let Value::Object(object) = parse_back(
        &super::build_enum_format_info(caps, index).expect("configuration should enumerate"),
    ) else {
        panic!("EnumFormat did not parse as an object");
    };
    let mut summary = (0, 0, 0);
    for property in object.properties {
        #[expect(non_upper_case_globals)]
        match (property.key, property.value) {
            (SPA_FORMAT_AUDIO_format, Value::Id(format)) => summary.0 = format.0,
            (SPA_FORMAT_AUDIO_rate, Value::Int(rate)) => summary.1 = rate,
            (SPA_FORMAT_AUDIO_channels, Value::Int(channels)) => summary.2 = channels,
            _ => (),
        }
    }
    summary
}

#[test]
fn configurations_enumerate_preferred_first_without_crossing_constraints() {
    use crate::backend::{
        ChannelLayout, ChannelSet, ConversionPath, RateSet, StreamCaps, StreamConfiguration,
    };
    use libspa::sys::{SPA_AUDIO_FORMAT_S16_LE, SPA_AUDIO_FORMAT_U8};

    let caps = StreamCaps {
        configurations: vec![
            StreamConfiguration {
                formats: vec![SPA_AUDIO_FORMAT_U8],
                channels: ChannelSet::Discrete(vec![ChannelLayout {
                    channels: 1,
                    positions: None,
                }]),
                rates: RateSet::Discrete(vec![44100]),
                preferred_rate: None,
                rate_tolerance: 0,
                conversion: ConversionPath::None,
                flags: backend::ConfigurationFlags::default(),
            },
            StreamConfiguration {
                formats: vec![SPA_AUDIO_FORMAT_S16_LE],
                channels: ChannelSet::Discrete(vec![ChannelLayout {
                    channels: 2,
                    positions: None,
                }]),
                rates: RateSet::Discrete(vec![48000]),
                preferred_rate: None,
                rate_tolerance: 0,
                conversion: ConversionPath::None,
                flags: backend::ConfigurationFlags::default(),
            },
        ],
        preferred: 1,
    };

    assert_eq!(
        enum_format_summary(&caps, 0),
        (SPA_AUDIO_FORMAT_S16_LE, 48000, 2)
    );
    assert_eq!(
        enum_format_summary(&caps, 1),
        (SPA_AUDIO_FORMAT_U8, 44100, 1)
    );
    assert!(super::build_enum_format_info(&caps, 2).is_none());
    assert!(!caps.admits(SPA_AUDIO_FORMAT_U8, 2, None, 48000));
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
    use libspa::sys::{
        SPA_AUDIO_FORMAT_F32_LE, SPA_AUDIO_FORMAT_S16_LE, SPA_AUDIO_FORMAT_S24_BE,
        SPA_AUDIO_FORMAT_U8,
    };

    let float_caps = caps(
        &[SPA_AUDIO_FORMAT_F32_LE, SPA_AUDIO_FORMAT_S16_LE],
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

    let three_byte_caps = caps(
        &[SPA_AUDIO_FORMAT_S24_BE],
        (1, 2),
        (8000, 192000),
        &[],
        None,
        false,
    );
    let mut snapped_three_byte = raw_format(SPA_AUDIO_FORMAT_F32_LE);
    assert!(super::snap_raw_to_caps(
        &three_byte_caps,
        &mut snapped_three_byte
    ));
    assert_eq!(snapped_three_byte.format, SPA_AUDIO_FORMAT_S24_BE);

    let u8_caps = caps(
        &[SPA_AUDIO_FORMAT_U8],
        (1, 2),
        (8000, 192000),
        &[],
        None,
        false,
    );
    let mut snapped_u8 = raw_format(SPA_AUDIO_FORMAT_S16_LE);
    assert!(super::snap_raw_to_caps(&u8_caps, &mut snapped_u8));
    assert_eq!(snapped_u8.format, SPA_AUDIO_FORMAT_U8);
}

#[test]
fn convertless_format_offers_are_native_only() {
    use libspa::sys::{SPA_AUDIO_FORMAT_F32_LE, SPA_AUDIO_FORMAT_S24_BE, SPA_AUDIO_FORMAT_U8};

    let native = caps(
        &[
            SPA_AUDIO_FORMAT_F32_LE,
            SPA_AUDIO_FORMAT_S24_BE,
            SPA_AUDIO_FORMAT_U8,
        ],
        (1, 2),
        (8000, 192000),
        &[],
        None,
        true,
    );
    assert_eq!(
        backend::offered_formats(native.preferred_configuration().unwrap()),
        [
            SPA_AUDIO_FORMAT_F32_LE,
            SPA_AUDIO_FORMAT_S24_BE,
            SPA_AUDIO_FORMAT_U8,
        ]
    );

    let unmatched = caps(&[], (1, 2), (8000, 192000), &[], None, true);
    assert!(backend::offered_formats(unmatched.preferred_configuration().unwrap()).is_empty());

    let converted_formats = [
        SPA_AUDIO_FORMAT_F32_LE,
        SPA_AUDIO_FORMAT_S24_BE,
        SPA_AUDIO_FORMAT_U8,
    ];
    let feeder = caps(&converted_formats, (1, 2), (8000, 192000), &[], None, false);
    assert_eq!(
        backend::offered_formats(feeder.preferred_configuration().unwrap()),
        converted_formats
    );
}
