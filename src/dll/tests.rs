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
        assert!(err.abs() < 1e5, "servo diverged: err {err}");
        first_corr.get_or_insert(corr);
        err -= (1.0 - corr) * period as f64;
    }
    assert!(first_corr.unwrap() < 1.0);
    assert!(err.abs() < 1.0, "servo failed to converge: err {err}");
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
