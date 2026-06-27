pub mod barometer;
pub mod gps;
pub mod imu;

pub use barometer::BaroPacket;
pub use gps::GpsPacket;
pub use imu::ImuPacket;

/// Discriminated union of all sensor packets flowing through the shared channel.
///
/// Kept as a flat enum so the crossbeam channel holds a single concrete type
/// (no `dyn Trait` boxing, no extra heap allocation).
#[derive(Debug, Clone, Copy)]
pub enum SensorMessage {
    Gps(GpsPacket),
    Imu(ImuPacket),
    Baro(BaroPacket),
}

impl SensorMessage {
    /// Extract the sensor-local timestamp regardless of variant.
    #[inline]
    pub fn timestamp_us(&self) -> u64 {
        match self {
            SensorMessage::Gps(p) => p.timestamp_us,
            SensorMessage::Imu(p) => p.timestamp_us,
            SensorMessage::Baro(p) => p.timestamp_us,
        }
    }
}
