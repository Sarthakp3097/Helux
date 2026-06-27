/// GPS fix packet received at ~10 Hz over UDP port 5001.
///
/// Wire layout is little-endian bincode — exactly 28 bytes.
/// `#[repr(C)]` guarantees field ordering matches the Python simulator's
/// `struct.pack('<Qdff')` format string.
#[derive(Debug, Clone, Copy, serde::Serialize, serde::Deserialize)]
#[repr(C)]
pub struct GpsPacket {
    /// Microseconds since Unix epoch.
    pub timestamp_us: u64,
    /// WGS-84 latitude in decimal degrees.
    pub latitude: f64,
    /// WGS-84 longitude in decimal degrees.
    pub longitude: f64,
    /// Altitude above mean sea level in metres.
    pub altitude_m: f32,
    /// Horizontal dilution of precision (lower = better, < 2.0 is good).
    pub hdop: f32,
}

/// Expected on-wire size in bytes (validated at compile time by the bench suite).
pub const GPS_WIRE_BYTES: usize = 28;

/// Equirectangular projection of a GPS fix into a local ENU offset (metres).
///
/// `origin_lat` / `origin_lon` are the reference-point degrees set on the first
/// valid fix. Returns `(east_m, north_m)`.
pub fn latlon_to_enu_xy(
    lat_deg: f64,
    lon_deg: f64,
    origin_lat: f64,
    origin_lon: f64,
) -> (f32, f32) {
    const EARTH_RADIUS_M: f64 = 6_371_000.0;
    let dlat = (lat_deg - origin_lat).to_radians();
    let dlon = (lon_deg - origin_lon).to_radians();
    let mid_lat = ((lat_deg + origin_lat) / 2.0).to_radians();
    let north_m = EARTH_RADIUS_M * dlat;
    let east_m = EARTH_RADIUS_M * dlon * mid_lat.cos();
    (east_m as f32, north_m as f32)
}
