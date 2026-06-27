/// IMU packet received at ~200 Hz over UDP port 5002.
///
/// Wire layout: little-endian bincode — exactly 36 bytes.
/// Field order matches the Python simulator's `struct.pack('<Qfffffff')`.
#[derive(Debug, Clone, Copy, serde::Serialize, serde::Deserialize)]
#[repr(C)]
pub struct ImuPacket {
    /// Microseconds since Unix epoch.
    pub timestamp_us: u64,
    /// Linear acceleration along body X-axis (m/s²).
    pub accel_x: f32,
    /// Linear acceleration along body Y-axis (m/s²).
    pub accel_y: f32,
    /// Linear acceleration along body Z-axis (m/s²) — includes gravity (~−9.81 at rest).
    pub accel_z: f32,
    /// Angular rate around body X-axis (rad/s).
    pub gyro_x: f32,
    /// Angular rate around body Y-axis (rad/s).
    pub gyro_y: f32,
    /// Angular rate around body Z-axis (rad/s).
    pub gyro_z: f32,
}

/// Expected on-wire size in bytes.
pub const IMU_WIRE_BYTES: usize = 36;

/// Gravity constant used to strip the gravitational component before
/// integrating vertical acceleration.
pub const GRAVITY_M_S2: f32 = 9.80665;
