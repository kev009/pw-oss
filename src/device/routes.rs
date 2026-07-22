use super::*;
use crate::platform;

// One hardware route per (pcm device, direction) that has a usable mixer
// control - except capture with a multi-source RECMASK, which gets one
// selectable route per source (the acp port model). The shadow fields mirror
// the kernel mixer state; the poll timer and set_param keep them in sync so
// re-emissions never report placeholders.
pub(super) struct RouteState {
    pub(super) node_id: u32, // our node object id (index * 2 + rec)
    pub(super) rec: bool,
    pub(super) name: String, // stable, never localized: WirePlumber's persistence key
    pub(super) description: String,
    pub(super) priority: i32,
    pub(super) mixer: usize,            // index into State::mixers
    pub(super) control: Option<c_uint>, // mixer level control; None = no volume props
    pub(super) follows_recsrc: bool,    // control derives from RECSRC; re-resolve on change
    pub(super) source: Option<c_uint>,  // the RECSRC bit this route selects (multi-source)
    pub(super) active: bool, // currently routed to its node; only active routes emit Route pods
    pub(super) levels: (u32, u32), // shadow OSS levels, 0-100 each
    pub(super) mute: bool,
    pub(super) save: bool, // echoed back in the Route pod, never interpreted
}

pub(super) struct MixerHandle {
    pub(super) mixer: platform::Mixer,
    pub(super) counter: c_int, // modify_counter baseline for external-change detection
    pub(super) recsrc: u32,    // RECSRC shadow; polled by value (the counter never ticks for it)
}

// OSS levels are a 0-100 slider scale, so map them through the cubic curve
// like ALSA devices without a dB scale (acp channel_map.c); a 1:1 linear map
// would make the volume keys feel wrong at the bottom of the range.
pub(super) fn linear_to_mixer(v: f32) -> u32 {
    if v.is_nan() || v <= 0.0 {
        // hostile pods included
        return 0;
    }
    (v.min(1.0).cbrt() * 100.0).round() as u32
}

// report the quantized readback, never the request, so the session manager
// converges on values the hardware can actually hold
pub(super) fn mixer_to_linear(l: u32) -> f32 {
    let x = l.min(100) as f32 / 100.0;
    x * x * x
}

// the mixer is stereo everywhere (STEREODEVS is the devmask, mixer.c:1094),
// so routes carry fixed FL/FR maps whatever width the node negotiates
pub(super) const ROUTE_CHANNELS: u32 = 2;
pub(super) const ROUTE_MAP: [u32; ROUTE_CHANNELS as usize] =
    [SPA_AUDIO_CHANNEL_FL, SPA_AUDIO_CHANNEL_FR];

pub(super) fn capitalize(s: &str) -> String {
    let mut chars = s.chars();
    match chars.next() {
        Some(first) => first.to_ascii_uppercase().to_string() + chars.as_str(),
        None => String::new(),
    }
}

// Discover the usable hardware controls and read their ACTUAL state before
// anything is emitted: reporting 1.0 placeholders and correcting later is a
// classic volume-jump source.
pub(super) fn probe_routes(
    pcm_devices: &[platform::AudioDevice],
) -> (Vec<RouteState>, Vec<MixerHandle>) {
    let mut routes: Vec<RouteState> = vec![];
    let mut mixers: Vec<MixerHandle> = vec![];
    let mut n_out = 0;
    let mut n_in = 0;
    let device_count = pcm_devices.len();

    for device in pcm_devices {
        let Some(mixer) = platform::Mixer::open(device.index) else {
            continue; // no mixer device: the node keeps its softvol
        };

        // one read shared by the route active flags and the poll shadow: a
        // RECSRC change between two reads would mismark the active route
        let probe_recsrc = mixer.recsrc().unwrap_or(0);

        let mixer_index = mixers.len();
        let mut used = false;

        for (rec, enabled) in [(false, device.play), (true, device.rec)] {
            if !enabled {
                continue;
            }

            // A multi-source RECMASK becomes one selectable route per source (the
            // acp port model). Single-source and no-recmask devices keep the v1
            // single route below - its name is WirePlumber's persistence key and
            // must not churn.
            if rec && mixer.recmask().count_ones() >= 2 {
                let recmask = mixer.recmask();
                let recsrc = probe_recsrc & recmask;
                // multiple set bits: the lowest wins, matching the v1 convention
                let current = if recsrc != 0 {
                    recsrc.trailing_zeros()
                } else {
                    recmask.trailing_zeros()
                };
                for dev_bit in 0..platform::MIXER_SOURCE_COUNT {
                    if recmask & (1 << dev_bit) == 0 {
                        continue;
                    }
                    let control = mixer.source_volume_control(dev_bit);
                    let levels = control.and_then(|c| mixer.level(c));
                    let control = control.filter(|_| levels.is_some());
                    let mute = control.and_then(|c| mixer.muted(c)).unwrap_or(false);
                    let src = platform::MIXER_SOURCE_NAMES[dev_bit as usize];
                    let (name, description) = if device_count == 1 {
                        (format!("oss-input-{src}"), capitalize(src))
                    } else {
                        (
                            format!("oss-input-pcm{}-{}", device.index, src),
                            format!("{} (pcm{})", capitalize(src), device.index),
                        )
                    };
                    routes.push(RouteState {
                        node_id: device.index * 2 + 1,
                        rec: true,
                        name,
                        description,
                        priority: platform::mixer_source_priority(dev_bit),
                        mixer: mixer_index,
                        control,
                        follows_recsrc: false,
                        source: Some(dev_bit),
                        active: dev_bit == current,
                        levels: levels.unwrap_or((100, 100)), // soft shadow starts at unity
                        mute,
                        save: false,
                    });
                    used = true;
                }
                continue;
            }

            let picked = if rec {
                mixer.input_control()
            } else {
                mixer.output_control().map(|c| (c, false))
            };
            let Some((control, follows_recsrc)) = picked else {
                continue; // no usable control for this direction
            };
            let Some(levels) = mixer.level(control) else {
                continue;
            };
            let mute = mixer.muted(control).unwrap_or(false);

            // Names are the session manager's persistence key: stable, no locale.
            // Derived from the pcm unit, not an ordinal - an ordinal shifts every
            // sibling when one unit's mixer fails to probe (attach-order race) or
            // the unit set changes, restoring saved volumes onto the wrong output.
            let (name, description) = if rec {
                n_in += 1;
                if n_in == 1 && device_count == 1 {
                    ("oss-input".to_string(), "Input".to_string())
                } else {
                    (
                        format!("oss-input-pcm{}", device.index),
                        format!("Input (pcm{})", device.index),
                    )
                }
            } else {
                n_out += 1;
                if n_out == 1 && device_count == 1 {
                    ("oss-output".to_string(), "Output".to_string())
                } else {
                    (
                        format!("oss-output-pcm{}", device.index),
                        format!("Output (pcm{})", device.index),
                    )
                }
            };

            routes.push(RouteState {
                node_id: device.index * 2 + rec as u32,
                rec,
                name,
                description,
                priority: 100,
                mixer: mixer_index,
                control: Some(control),
                follows_recsrc,
                source: None,
                active: true,
                levels,
                mute,
                save: false,
            });
            used = true;
        }

        if used {
            let counter = mixer.modify_counter().unwrap_or(0);
            mixers.push(MixerHandle {
                mixer,
                counter,
                recsrc: probe_recsrc,
            });
        }
    }

    (routes, mixers)
}

// the init-dict device properties: the parent device name (for
// SPA_KEY_DEVICE_NAME) and the pcm unit indexes this device aggregates
pub(super) fn common_description(pcm_devices: &[platform::AudioDevice]) -> String {
    let mut common_desc = pcm_devices[0].desc.clone();
    for pcm_device in &pcm_devices[1..] {
        let count = common_desc
            .chars()
            .zip(pcm_device.desc.chars())
            .take_while(|(a, b)| a == b)
            .map(|(c, _)| c.len_utf8())
            .sum();
        common_desc.truncate(count);
    }

    while common_desc.ends_with(' ') || common_desc.ends_with('(') {
        common_desc.truncate(common_desc.len() - 1);
    }

    common_desc
}

#[cfg(test)]
mod tests {
    use super::*;

    fn pcm(desc: &str) -> platform::AudioDevice {
        platform::AudioDevice {
            index: 0,
            desc: desc.to_string(),
            location: String::new(),
            play: true,
            rec: false,
        }
    }

    #[test]
    fn common_description_stops_at_a_utf8_character_boundary() {
        assert_eq!(
            common_description(&[pcm("Beyoncé DAC"), pcm("Beyoncê ADC")]),
            "Beyonc"
        );
    }
}
