use crate::sensor::imu::GRAVITY_M_S2;

/// Single-axis complementary filter that fuses IMU-integrated altitude
/// with an absolute reference (GPS or barometer).
///
/// ## Rationale over a Kalman filter
/// A Kalman filter requires a tuned process-noise covariance matrix Q and a
/// measurement-noise covariance R; mis-tuning either produces divergence or
/// sluggish tracking. The complementary filter replaces that with a single
/// scalar `alpha` that has an immediately intuitive physical meaning: "how
/// much do I trust the slow but stable absolute sensor versus the fast but
/// drift-prone IMU?" The tradeoff is deterministic, requires no matrix
/// inversion, and runs in O(1) with zero allocations — ideal for embedded
/// real-time loops.
///
/// ## Signal flow
/// ```text
///  accel_z ──► integrate ──► Δz_imu ──► × (1 − α) ──┐
///                                                      ▼
///                                              altitude_estimate
///                                                      ▲
/// baro/GPS ──────────────────────────────── × α ──────┘
/// ```
///
/// ## Alpha tuning guide
/// | Platform                    | Recommended α |
/// |-----------------------------|---------------|
/// | Slow-moving (balloon, buoy) | 0.98          |
/// | General-purpose default     | 0.85          |
/// | Fast-maneuvering (drone)    | 0.60          |
pub struct ComplementaryFilter {
    /// Blend coefficient [0.0, 1.0].
    alpha: f32,
    /// Current altitude estimate in metres MSL.
    altitude_estimate: f32,
    /// Vertical velocity derived from IMU integration (m/s).
    vertical_velocity: f32,
    /// Timestamp of the last IMU update (µs), used to compute Δt.
    last_imu_timestamp_us: u64,
    /// Running count of IMU updates — exposed for diagnostics.
    imu_update_count: u64,
    /// Running count of absolute updates — exposed for diagnostics.
    abs_update_count: u64,
}

impl ComplementaryFilter {
    /// Construct a new filter with the given `alpha` and a zeroed initial state.
    pub fn new(alpha: f32) -> Self {
        assert!(
            (0.0..=1.0).contains(&alpha),
            "alpha must be in [0.0, 1.0], got {alpha}"
        );
        Self {
            alpha,
            altitude_estimate: 0.0,
            vertical_velocity: 0.0,
            last_imu_timestamp_us: 0,
            imu_update_count: 0,
            abs_update_count: 0,
        }
    }

    /// Seed the filter with a known starting altitude (e.g. the first GPS fix).
    /// Call this before the first `update_imu` to avoid an initial transient.
    pub fn seed_altitude(&mut self, altitude_m: f32) {
        self.altitude_estimate = altitude_m;
    }

    /// Ingest an IMU measurement and advance the filter by one integration step.
    ///
    /// `raw_accel_z` is the body-frame vertical acceleration **including gravity**
    /// (typically ~−9.81 m/s² at rest). Gravity is subtracted internally so the
    /// integration tracks true linear acceleration.
    ///
    /// The first call initialises the timestamp reference and returns without
    /// updating the estimate (Δt would be undefined).
    #[inline]
    pub fn update_imu(&mut self, raw_accel_z: f32, timestamp_us: u64) {
        if self.last_imu_timestamp_us == 0 {
            self.last_imu_timestamp_us = timestamp_us;
            return;
        }

        let delta_us = timestamp_us.saturating_sub(self.last_imu_timestamp_us);
        let dt_s = delta_us as f32 / 1_000_000.0;

        // Remove gravitational component; positive = upward acceleration.
        let linear_accel_z = raw_accel_z + GRAVITY_M_S2;

        // Euler integration: velocity then position.
        self.vertical_velocity += linear_accel_z * dt_s;
        let imu_altitude_delta = self.vertical_velocity * dt_s;

        // High-pass contribution from IMU (captures rapid dynamics).
        self.altitude_estimate += (1.0 - self.alpha) * imu_altitude_delta;
        self.last_imu_timestamp_us = timestamp_us;
        self.imu_update_count += 1;
    }

    /// Ingest an absolute altitude reference (GPS or barometer, in metres MSL).
    ///
    /// Low-pass contribution: gently pulls the estimate toward the stable
    /// absolute reference, correcting accumulated IMU drift.
    #[inline]
    pub fn update_absolute(&mut self, absolute_altitude_m: f32) {
        self.altitude_estimate = self.alpha * absolute_altitude_m
            + (1.0 - self.alpha) * self.altitude_estimate;
        self.abs_update_count += 1;
    }

    /// Returns the current fused altitude estimate in metres MSL.
    #[inline]
    pub fn altitude(&self) -> f32 {
        self.altitude_estimate
    }

    /// Returns the vertical velocity estimated by IMU integration (m/s).
    #[inline]
    pub fn vertical_velocity(&self) -> f32 {
        self.vertical_velocity
    }

    /// Returns the active alpha coefficient.
    #[inline]
    pub fn alpha(&self) -> f32 {
        self.alpha
    }

    /// Update alpha at runtime (useful for adaptive tuning or testing).
    #[inline]
    pub fn set_alpha(&mut self, new_alpha: f32) {
        assert!((0.0..=1.0).contains(&new_alpha));
        self.alpha = new_alpha;
    }

    /// Diagnostic snapshot: (imu_updates, abs_updates).
    #[inline]
    pub fn update_counts(&self) -> (u64, u64) {
        (self.imu_update_count, self.abs_update_count)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn stationary_imu_holds_altitude() {
        let mut f = ComplementaryFilter::new(0.85);
        f.seed_altitude(100.0);

        // Simulate 1 second at rest: accel_z = −g, no net vertical motion.
        let base_ts: u64 = 1_000_000;
        let dt_us: u64 = 5_000; // 200 Hz
        for i in 0..200u64 {
            f.update_imu(-GRAVITY_M_S2, base_ts + i * dt_us);
            f.update_absolute(100.0);
        }
        // Altitude should remain close to 100 m.
        assert!(
            (f.altitude() - 100.0).abs() < 1.0,
            "drift = {:.3} m",
            (f.altitude() - 100.0).abs()
        );
    }

    #[test]
    fn alpha_one_tracks_absolute_instantly() {
        let mut f = ComplementaryFilter::new(1.0);
        f.seed_altitude(0.0);
        f.update_absolute(250.0);
        assert_eq!(f.altitude(), 250.0);
    }

    #[test]
    fn alpha_zero_ignores_absolute() {
        let mut f = ComplementaryFilter::new(0.0);
        f.seed_altitude(50.0);
        f.update_absolute(999.0); // should be completely ignored
        assert_eq!(f.altitude(), 50.0);
    }
}
