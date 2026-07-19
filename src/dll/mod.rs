// This code was borrowed from PipeWire's spa/include/spa/utils/dll.h that has the following header:

/* Simple DLL */
/* SPDX-FileCopyrightText: Copyright © 2019 Wim Taymans */
/* SPDX-License-Identifier: MIT */

pub(crate) const SPA_DLL_BW_MAX: f64 = 0.128;
pub(crate) const SPA_DLL_BW_MIN: f64 = 0.016;

#[derive(Default)]
pub(crate) struct SpaDLL {
    bw: f64,
    z1: f64,
    z2: f64,
    z3: f64,
    w0: f64,
    w1: f64,
    w2: f64,
}

impl SpaDLL {
    #[inline(always)]
    pub(crate) fn init(&mut self) {
        self.bw = 0.0;
        self.z1 = 0.0;
        self.z2 = 0.0;
        self.z3 = 0.0;
        // also the gains (upstream keeps them): update() before the adaptive
        // cold-start must be a true no-op, not one sample at stale gains
        self.w0 = 0.0;
        self.w1 = 0.0;
        self.w2 = 0.0;
    }

    #[inline(always)]
    pub(crate) fn set_bw(&mut self, bw: f64, period: u32, rate: u32) {
        let w = 2.0 * std::f64::consts::PI * bw * period as f64 / rate as f64;
        self.w0 = 1.0 - (-20.0 * w).exp();
        self.w1 = w * 1.5 / period as f64;
        self.w2 = w / 1.5;
        self.bw = bw;
    }

    #[inline(always)]
    pub(crate) fn bw(&self) -> f64 {
        self.bw
    }

    #[inline(always)]
    pub(crate) fn update(&mut self, err: f64) -> f64 {
        self.z1 += self.w0 * (self.w1 * err - self.z1);
        self.z2 += self.w0 * (self.z1 - self.z2);
        self.z3 += self.w2 * self.z2;
        1.0 - (self.z2 + self.z3)
    }
}

// ALSA's bandwidth floor for the adaptive scheme (alsa-pcm.c); well below the
// classic SPA_DLL_BW_MIN - a quiet servo may relax that far
pub(crate) const SPA_ALSA_DLL_BW_MIN: f64 = 0.001;

const BW_PERIOD_NSEC: u64 = 3_000_000_000;

// alsa-pcm.c update_time's bandwidth adaptation: an EWMA of the servo error's
// mean and variance over ~1 s of cycles; every 3 s window the bandwidth is
// re-tuned to (|avg| + sqrt(var)) / 1000 in frames. Two OSS adaptations: the
// known fragment-quantization variance (step^2/12) is subtracted so idle
// granularity jitter reads as locked (OSS delay/queue readings move in whole
// fragments, unlike ALSA's pointer-accurate delays), and the bandwidth is
// capped by measurement granularity - a fragment wider than the period can't
// support the full loop gain without wobbling the steered clock.
#[derive(Default, Clone)]
pub(crate) struct BwAdapt {
    err_avg: f64,
    err_var: f64,
    base_time: u64,
    // the committed servo geometry, latched by configure(): the device
    // fragment (noise), byte-domain period/rate and the frame stride only
    // change when the geometry is re-committed, not per cycle
    stride: u32,
    noise: u32,
    period: u32,
    rate: u32,
}

impl BwAdapt {
    pub(crate) fn reset(&mut self) {
        // the latched geometry survives: a relock (dll.init + reset) must
        // cold-start at the same committed geometry
        self.err_avg = 0.0;
        self.err_var = 0.0;
        self.base_time = 0;
    }

    // latch the committed geometry; update() no-ops until this ran (rate 0)
    pub(crate) fn configure(&mut self, stride: u32, noise: u32, period: u32, rate: u32) {
        self.stride = stride;
        self.noise = noise;
        self.period = period;
        self.rate = rate;
    }

    fn bw_cap(period: u32, noise: u32) -> f64 {
        if noise == 0 || period == 0 {
            return SPA_DLL_BW_MAX;
        }
        (SPA_DLL_BW_MAX * period as f64 / noise as f64).clamp(SPA_DLL_BW_MIN, SPA_DLL_BW_MAX)
    }

    // `err` is the clamped servo error as fed to the DLL; the geometry
    // (stride cancels everywhere except the /1000 heuristic) comes latched
    // from configure(). Cold-starts the DLL at the granularity cap when
    // bw == 0 (i.e. after init()), making dll.init() + reset() the whole
    // relock idiom.
    pub(crate) fn update(&mut self, dll: &mut SpaDLL, err: f64, now: u64) {
        let (stride, noise, period, rate) = (self.stride, self.noise, self.period, self.rate);
        if rate == 0 {
            return; // unconfigured; nothing to steer safely
        }
        if dll.bw() == 0.0 {
            dll.set_bw(Self::bw_cap(period, noise), period, rate);
            self.err_avg = 0.0;
            self.err_var = 0.0;
            self.base_time = now;
            return; // the gains were zero this cycle; track from the next one
        }
        if self.base_time == 0 {
            self.base_time = now;
        }
        let stride = stride.max(1) as f64;
        let err = err / stride;
        let wdw = rate as f64 / period as f64; // cycles per second
        let avg = (self.err_avg * wdw + (err - self.err_avg)) / (wdw + 1.0);
        self.err_var = (self.err_var * wdw + (err - self.err_avg) * (err - avg)) / (wdw + 1.0);
        self.err_avg = avg;
        if now.saturating_sub(self.base_time) > BW_PERIOD_NSEC {
            self.base_time = now;
            // half the uniform-quantization floor (step^2/12): a locked loop
            // regulates the quantized reading, so the sampling phase correlates
            // with the fragment sawtooth and full subtraction could mask genuine
            // fragment-sized disturbance. (On vchans the parent's mix block is the
            // real granularity and `noise` understates it - which only errs toward
            // higher bandwidth, the safe direction.)
            let step = noise as f64 / stride;
            let var = (self.err_var.abs() - step * step / 24.0).max(0.0);
            let bw = (self.err_avg.abs() + var.sqrt()) / 1000.0;
            dll.set_bw(
                bw.clamp(SPA_ALSA_DLL_BW_MIN, Self::bw_cap(period, noise)),
                period,
                rate,
            );
        }
    }
}

#[cfg(test)]
mod tests;
