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
single exclusive channel across renegotiation. Note that the kernel is only
half of the story - for the samples to remain untouched end to end, the
PipeWire graph must also run at the device's native rate and avoid volume
scaling. Pin the graph rate in `pipewire.conf.d`:

```
context.properties = {
    default.clock.rate        = 96000       # the device's native rate
    default.clock.allowed-rates = [ 96000 ]
}
```

and keep the stream/device volume at 100% (or use hardware volume once
available). Exclusive devices allow one open per direction: a second client
gets EBUSY and stays silent by design.

## Tunables

Per-device properties can be set from WirePlumber rules
(`wireplumber.conf.d`), matched against the monitor's device objects:

```
monitor.oss.rules = [
  {
    matches = [ { device.name = "~oss_card.*" } ]
    actions = { update-props = { oss.delay = 16 } }
  }
]
```

`oss.delay` sets the sink's fill target in eighths of a graph period
(default 10 = 1.25 periods; higher absorbs more scheduling jitter at the
cost of latency). It can also be changed live on a node via
`pw-cli set-param <node> Props '{ params: [ "oss.delay", 16 ] }'`.

Log verbosity follows the `spa.oss` topic, e.g.
`PIPEWIRE_DEBUG='spa.oss:3' pipewire`.

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
