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
mod tests {
    use super::*;

    #[test]
    fn init_makes_update_a_true_no_op() {
        // documented on init(): update() before the adaptive cold-start must be
        // a no-op at zeroed gains, not one sample at stale gains
        let mut dll = SpaDLL::default();
        dll.set_bw(SPA_DLL_BW_MAX, 1024, 48000);
        dll.update(500.0);
        dll.init();
        assert_eq!(dll.update(1e6), 1.0);
        assert_eq!(dll.bw(), 0.0);
    }

    #[test]
    fn servo_converges_on_a_constant_offset() {
        // closed loop against an offset plant: corr is a rate multiplier, so
        // the fill error moves by the per-cycle correction. A positive error
        // (fill above target) must pull corr under 1.0 and drive the error to
        // zero without oscillating out or going non-finite.
        let period = 1024u32;
        let mut dll = SpaDLL::default();
        dll.init();
        dll.set_bw(SPA_DLL_BW_MAX, period, 48000);
        let mut err: f64 = 256.0;
        let mut first_corr = None;
        for _ in 0..5000 {
            let corr = dll.update(err);
            assert!(corr.is_finite());
            assert!(err.abs() < 1e5, "servo diverged: err {}", err);
            first_corr.get_or_insert(corr);
            err -= (1.0 - corr) * period as f64;
        }
        assert!(first_corr.unwrap() < 1.0);
        assert!(err.abs() < 1.0, "servo failed to converge: err {}", err);
    }

    #[test]
    fn bw_cap_binds_only_when_fragments_outsize_the_period() {
        assert_eq!(BwAdapt::bw_cap(16384, 0), SPA_DLL_BW_MAX);
        assert_eq!(BwAdapt::bw_cap(0, 2048), SPA_DLL_BW_MAX);
        assert_eq!(BwAdapt::bw_cap(16384, 16384), SPA_DLL_BW_MAX);
        let capped = BwAdapt::bw_cap(16384, 32768);
        assert!((SPA_DLL_BW_MIN..SPA_DLL_BW_MAX).contains(&capped));
        assert_eq!(BwAdapt::bw_cap(1024, u32::MAX), SPA_DLL_BW_MIN);
    }

    #[test]
    fn adaptive_bw_cold_starts_at_the_granularity_cap() {
        let mut dll = SpaDLL::default();
        dll.init();
        let mut bw = BwAdapt::default();
        bw.configure(8, 2048, 16384, 48000 * 8);
        bw.update(&mut dll, 0.0, 1_000);
        assert_eq!(dll.bw(), BwAdapt::bw_cap(16384, 2048));
        assert_eq!(dll.bw(), SPA_DLL_BW_MAX); // fragment under the period: uncapped
    }

    #[test]
    fn quiet_servo_relaxes_to_the_alsa_floor() {
        let mut dll = SpaDLL::default();
        dll.init();
        let mut bw = BwAdapt::default();
        bw.configure(8, 2048, 16384, 48000 * 8);
        let mut now = 1u64;
        // cold start, then over 3 s of dead-quiet errors: the retune window
        // sees zero mean and zero variance and relaxes to the ALSA floor
        for _ in 0..50 {
            bw.update(&mut dll, 0.0, now);
            now += 100_000_000;
        }
        assert_eq!(dll.bw(), SPA_ALSA_DLL_BW_MIN);
    }

    #[test]
    fn noisy_servo_holds_bandwidth_up() {
        let mut dll = SpaDLL::default();
        dll.init();
        let mut bw = BwAdapt::default();
        bw.configure(8, 2048, 16384, 48000 * 8);
        let mut now = 1u64;
        let mut sign = 1.0;
        // errors well above the fragment-quantization floor: the variance term
        // must keep the loop gain off the ALSA floor
        for _ in 0..50 {
            bw.update(&mut dll, sign * 4.0 * 2048.0, now);
            sign = -sign;
            now += 100_000_000;
        }
        assert!(dll.bw() > SPA_ALSA_DLL_BW_MIN);
    }
}
