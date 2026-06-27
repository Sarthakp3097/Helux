use nalgebra::{UnitQuaternion, Vector3};

/// Fused navigation state produced by the fusion thread after every sensor message.
///
/// All spatial quantities use the **East-North-Up (ENU)** local-tangent-plane frame
/// with the first valid GPS fix as the coordinate origin.
#[derive(Debug, Clone, Copy)]
pub struct StateEstimate {
    /// Wall-clock timestamp of the most-recent sensor packet that triggered this update (µs).
    pub timestamp_us: u64,

    /// Position in metres relative to the ENU origin.
    pub position: Vector3<f32>,

    /// Velocity in m/s (ENU).
    pub velocity: Vector3<f32>,

    /// Body orientation expressed as a unit quaternion (scalar-first convention).
    /// Initialised to identity; updated by gyro integration.
    pub orientation: UnitQuaternion<f32>,

    /// Complementary-filter altitude output (metres MSL).
    pub altitude_fused: f32,

    /// The alpha coefficient that was active when this estimate was computed.
    pub alpha: f32,
}

impl StateEstimate {
    /// Zero-initialised estimate at t = 0. Used as the fusion thread's seed value.
    pub fn zeroed() -> Self {
        Self {
            timestamp_us: 0,
            position: Vector3::zeros(),
            velocity: Vector3::zeros(),
            orientation: UnitQuaternion::identity(),
            altitude_fused: 0.0,
            alpha: 0.85,
        }
    }
}

impl std::fmt::Display for StateEstimate {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let (roll, pitch, yaw) = self.orientation.euler_angles();
        write!(
            f,
            "t={:.3}s pos=({:.2},{:.2},{:.2})m vel=({:.2},{:.2},{:.2})m/s \
             alt_fused={:.2}m rpy=({:.1}°,{:.1}°,{:.1}°) α={:.2}",
            self.timestamp_us as f64 / 1_000_000.0,
            self.position.x,
            self.position.y,
            self.position.z,
            self.velocity.x,
            self.velocity.y,
            self.velocity.z,
            self.altitude_fused,
            roll.to_degrees(),
            pitch.to_degrees(),
            yaw.to_degrees(),
            self.alpha,
        )
    }
}
