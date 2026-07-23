> A fork of [shkhln/pw-oss](https://github.com/shkhln/pw-oss), maintained for
> FreeBSD ports packaging (`audio/pipewire-spa-oss-ng`).

This repo contains a very basic FreeBSD sound input/output plugin for PipeWire.
No other operating systems are supported.

## Limitations

1. The plugin is only sufficiently complete to be used with the
`node.features.audio.no-dsp=false` Wireplumber setting (which is the default).

## Bitperfect / exclusive devices

Devices in bitperfect mode (`sysctl dev.pcm.X.bitperfect=1`, usually together
with `dev.pcm.X.play.vchans=0` or `hw.snd.vchans_enable=0`) are fully
supported: the plugin probes and advertises the device's native formats,
rates and channel widths (including >8-channel interfaces), and handles the
single exclusive channel across renegotiation. Supported interleaved audio
formats are G.711 mu-law and A-law; signed and unsigned 8- and 16-bit, packed
three-byte 24-bit, and four-byte 32-bit-container integer PCM; and 32-bit
float, with both byte orders for multibyte samples. Note that the kernel is
only half of the story - for the samples to remain untouched end to end, the
PipeWire graph must also run at the device's native rate and avoid volume
scaling. Pin the graph rate in `pipewire.conf.d`:

```
context.properties = {
    default.clock.rate        = 96000       # the device's native rate
    default.clock.allowed-rates = [ 96000 ]
}
```

and keep the *software* volumes (streams and the node) at 100%; the route
(hardware) volume is free - it applies after the bit-exact sample path, so
adjusting it loses nothing. On bitperfect devices it is in fact the only
volume that does anything: the kernel's bitperfect feeder chain carries no
volume feeder, so software attenuation via the `pcm` control is silently
inert there. A bitperfect device without a usable mixer control exposes no
route volume at all; the session manager's node softvol is then the
(un-bitperfect) fallback, and purists simply pin it at 100%. Exclusive
devices allow one open per direction: a second client gets EBUSY and stays
silent by design.

## Tunables

Per-node properties can be set from WirePlumber rules
(`wireplumber.conf.d`), matched against the properties emitted for each OSS
node:

```
monitor.oss.rules = [
  {
    matches = [ { node.name = "pcm0.play" } ]
    actions = {
      update-props = {
        oss.delay = 16
        api.freebsd-oss.force-timer = true
      }
    }
  }
]
```

`oss.delay` sets the sink's fill target in eighths of a graph period
(default 10 = 1.25 periods; higher absorbs more scheduling jitter at the
cost of latency). It can also be changed live on a node via
`pw-cli set-param <node> Props '{ params: [ "oss.delay", 16 ] }'`.

`oss.fragment` sets the device fragment size in bytes for both playback and
capture (default 0 = automatic, which sizes 1 KiB fragments). Values are
rounded down to a power of two and clamped to 64..16384; the Props readback
reports the effective value. Smaller fragments help latency-critical small
quanta (finer DMA delivery granularity at the price of more interrupts);
larger fragments mean fewer interrupts when latency doesn't matter. The
device may still grant a different size (some drivers force a fixed period);
the plugin reads the granted size back - and the device's real hardware
cadence from sndstat(4), which can be coarser than any fragment (USB
transfer chunks, vchan parents) - and the rate servo's measurement
granularity and noise model follow the real quantum automatically. Live:
`pw-cli set-param <node> Props '{ params: [ "oss.fragment", 4096 ] }'`.

`api.freebsd-oss.force-timer = true` forces a node to use the portable SPA
timer wakeup path instead of enriched OSS kqueue events, including on kernels
that support them. This is a creation-time setting for `monitor.oss.rules`;
restart or recreate the node after changing it. It accepts `true`, `yes`, `on`
or `1` (and the corresponding `false`, `no`, `off` or `0` forms), and is not
available through the runtime Props parameter.

### Routes / hardware volume

Each pcm device with a usable mixer control (`vol`, else `pcm` for playback;
`rec`, else the current recording source, else `igain` for capture) exposes a
PipeWire route, so desktop volume keys, `wpctl` and `pactl` drive the OSS
mixer on `/dev/mixerN` directly instead of attenuating in software; the node's
software volume stays at 100%. Volumes map through the same cubic curve ALSA
devices without a dB scale use, quantized to the mixer's 0-100 steps. Changes
made outside PipeWire (e.g. `mixer(8)`) are picked up by a ~1 Hz poll.
Devices without a usable control get no route and keep the WirePlumber node
softvol. WirePlumber persists route volumes under the route name and restores
them on login (`device.restore-routes`, default true).

A capture device whose mixer offers more than one recording source (`mic`,
`line`, ...) exposes one selectable route per source, so the input can be
switched from `pavucontrol`/`wpctl` and the choice is persisted by
WirePlumber like any other port. Selecting a route writes the OSS recording
source; changes made with `mixer(8) recsrc` are reflected within a second.
Each source uses its own level control when it has one, the shared `rec`
(RECLEV) control otherwise - sources sharing `rec` share their volume - and
sources with no hardware control get a software route volume (applied in
the graph, still selectable and persisted like any other route).

Log verbosity follows the `spa.oss.{device,sink,source,monitor}` topics;
the quoted glob covers them all, e.g.
`PIPEWIRE_DEBUG='spa.oss.*:3' pipewire`.

## Usage

To build and run the project locally:
1. `sudo pkg install rust`
1. `git clone <this repo>`
1. `cd pw-oss`
1. `cargo build`
1. Start PipeWire with`./pipewire.sh`.
1. Start client apps with run.sh, e.g. `./run.sh pw-play whatever.wav`.

## Installation

The recommended way is the FreeBSD `audio/pipewire-spa-oss-ng` port/package.  It
conflicts with `audio/pipewire-spa-oss` as they currently use the same soname.

To install manually after `cargo build --release`, copy the build outputs to the
system PipeWire/WirePlumber locations (`PREFIX` is `/usr/local`):

| File | Destination |
|------|-------------|
| `target/release/libspa_freebsd_oss.so` | `${PREFIX}/lib/spa-0.2/` |
| `conf/pipewire/pipewire.conf.d/oss.conf` | `${PREFIX}/share/pipewire/pipewire.conf.d/` |
| `conf/wireplumber/wireplumber.conf.d/oss.conf` | `${PREFIX}/share/wireplumber/wireplumber.conf.d/` |
| `share/wireplumber/scripts/monitors/oss.lua` | `${PREFIX}/share/wireplumber/scripts/monitors/` |

`conf/pipewire/pipewire.conf.d/exec.conf` is only for the local dev workflow
(see Usage) and is not installed system-wide.

Restart PipeWire and WirePlumber afterward.

## License

This code is *by necessity* derived and closely follows PipeWire's SPA
plugin code, which is covered by the MIT license.

There is no way to actually implement plugins independently
(in the copyright terms), while attributing each line would be
completely obnoxious, so hopefully this notice is enough.

Anything original is also subject to the MIT license.
