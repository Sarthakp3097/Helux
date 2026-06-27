mod config;
mod fusion;
mod sensor;
mod state;

use anyhow::Result;
use crossbeam_channel::bounded;
use nalgebra::Vector3;
use sensor::{BaroPacket, GpsPacket, ImuPacket, SensorMessage};
use state::StateEstimate;
use tokio::net::UdpSocket;
use tracing::{error, info, warn};

use config::EngineConfig;
use fusion::{integrate_gyro, ComplementaryFilter};
use sensor::barometer::barometric_altitude;
use sensor::gps::latlon_to_enu_xy;

// ──────────────────────────────────────────────────────────────────────────────
// Entry point
// ──────────────────────────────────────────────────────────────────────────────

#[tokio::main]
async fn main() -> Result<()> {
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| tracing_subscriber::EnvFilter::new("info")),
        )
        .init();

    let cfg = EngineConfig::from_env();
    info!(
        gps_port = cfg.gps_port,
        imu_port = cfg.imu_port,
        baro_port = cfg.baro_port,
        alpha = cfg.filter_alpha,
        channel_depth = cfg.channel_depth,
        "Helix fusion engine starting"
    );

    // Bounded channel — all three sensor producers share one queue.
    // If the fusion thread falls behind the backpressure blocks producers
    // rather than letting the heap grow without bound.
    let (tx, rx) = bounded::<SensorMessage>(cfg.channel_depth);

    let gps_tx = tx.clone();
    let imu_tx = tx.clone();
    let baro_tx = tx.clone();

    let gps_port = cfg.gps_port;
    let imu_port = cfg.imu_port;
    let baro_port = cfg.baro_port;

    // Each sensor stream lives in its own async Tokio task.
    tokio::spawn(async move {
        if let Err(e) = gps_listener(gps_tx, gps_port).await {
            error!("GPS listener terminated: {e}");
        }
    });
    tokio::spawn(async move {
        if let Err(e) = imu_listener(imu_tx, imu_port).await {
            error!("IMU listener terminated: {e}");
        }
    });
    tokio::spawn(async move {
        if let Err(e) = baro_listener(baro_tx, baro_port).await {
            error!("Baro listener terminated: {e}");
        }
    });

    // Fusion runs on a dedicated OS thread — not a Tokio task.
    // This keeps the hot loop off the async executor and avoids cooperative
    // scheduling jitter in the critical path.
    let alpha = cfg.filter_alpha;
    let log_throttle = cfg.log_throttle_us;
    std::thread::Builder::new()
        .name("helix-fusion".into())
        .spawn(move || fusion_loop(rx, alpha, log_throttle))?;

    info!("All tasks spawned — waiting for Ctrl-C");
    tokio::signal::ctrl_c().await?;
    info!("Shutting down");
    Ok(())
}

// ──────────────────────────────────────────────────────────────────────────────
// Async UDP listeners — one per sensor, identical shape
// ──────────────────────────────────────────────────────────────────────────────

async fn gps_listener(
    tx: crossbeam_channel::Sender<SensorMessage>,
    port: u16,
) -> Result<()> {
    let addr = format!("0.0.0.0:{port}");
    let socket = UdpSocket::bind(&addr).await?;
    info!("GPS listener bound to {addr}");

    // Stack buffer — larger than the 28-byte GPS packet, never heap-allocated.
    let mut wire_buf = [0u8; 64];
    loop {
        let (nbytes, _origin) = socket.recv_from(&mut wire_buf).await?;
        match bincode::deserialize::<GpsPacket>(&wire_buf[..nbytes]) {
            Ok(pkt) => {
                if tx.send(SensorMessage::Gps(pkt)).is_err() {
                    warn!("GPS: channel closed, dropping packet");
                    break;
                }
            }
            Err(e) => warn!("GPS: bad packet ({nbytes} bytes): {e}"),
        }
    }
    Ok(())
}

async fn imu_listener(
    tx: crossbeam_channel::Sender<SensorMessage>,
    port: u16,
) -> Result<()> {
    let addr = format!("0.0.0.0:{port}");
    let socket = UdpSocket::bind(&addr).await?;
    info!("IMU listener bound to {addr}");

    // Stack buffer — larger than the 36-byte IMU packet.
    let mut wire_buf = [0u8; 64];
    loop {
        let (nbytes, _origin) = socket.recv_from(&mut wire_buf).await?;
        match bincode::deserialize::<ImuPacket>(&wire_buf[..nbytes]) {
            Ok(pkt) => {
                if tx.send(SensorMessage::Imu(pkt)).is_err() {
                    warn!("IMU: channel closed, dropping packet");
                    break;
                }
            }
            Err(e) => warn!("IMU: bad packet ({nbytes} bytes): {e}"),
        }
    }
    Ok(())
}

async fn baro_listener(
    tx: crossbeam_channel::Sender<SensorMessage>,
    port: u16,
) -> Result<()> {
    let addr = format!("0.0.0.0:{port}");
    let socket = UdpSocket::bind(&addr).await?;
    info!("Baro listener bound to {addr}");

    // Stack buffer — larger than the 16-byte baro packet.
    let mut wire_buf = [0u8; 64];
    loop {
        let (nbytes, _origin) = socket.recv_from(&mut wire_buf).await?;
        match bincode::deserialize::<BaroPacket>(&wire_buf[..nbytes]) {
            Ok(pkt) => {
                if tx.send(SensorMessage::Baro(pkt)).is_err() {
                    warn!("Baro: channel closed, dropping packet");
                    break;
                }
            }
            Err(e) => warn!("Baro: bad packet ({nbytes} bytes): {e}"),
        }
    }
    Ok(())
}

// ──────────────────────────────────────────────────────────────────────────────
// Fusion thread — pure synchronous hot loop
// ──────────────────────────────────────────────────────────────────────────────

fn fusion_loop(
    rx: crossbeam_channel::Receiver<SensorMessage>,
    alpha: f32,
    log_throttle_us: u64,
) {
    let mut filt = ComplementaryFilter::new(alpha);
    let mut orientation = nalgebra::UnitQuaternion::<f32>::identity();
    let mut position = Vector3::<f32>::zeros();
    let mut velocity = Vector3::<f32>::zeros();

    // ENU origin — set on the first valid GPS fix.
    let mut enu_origin: Option<(f64, f64, f32)> = None;

    let mut latest_ts_us: u64 = 0;
    let mut last_log_ts_us: u64 = 0;
    let mut last_imu_ts_us: u64 = 0;

    // Cycle counter for diagnostics.
    let mut cycle: u64 = 0;

    loop {
        let msg = match rx.recv() {
            Ok(m) => m,
            Err(_) => {
                info!("Fusion channel closed after {cycle} cycles — exiting");
                break;
            }
        };

        latest_ts_us = msg.timestamp_us().max(latest_ts_us);

        match msg {
            // ── IMU (200 Hz) ─────────────────────────────────────────────────
            SensorMessage::Imu(pkt) => {
                // Gyro integration — update body orientation.
                if last_imu_ts_us > 0 {
                    let dt_s = pkt
                        .timestamp_us
                        .saturating_sub(last_imu_ts_us) as f32
                        / 1_000_000.0;
                    let gyro_vec = Vector3::new(pkt.gyro_x, pkt.gyro_y, pkt.gyro_z);
                    integrate_gyro(gyro_vec, dt_s, &mut orientation);

                    // Horizontal velocity integration (body → ENU via orientation).
                    let accel_body = Vector3::new(pkt.accel_x, pkt.accel_y, pkt.accel_z);
                    let accel_enu = orientation * accel_body;
                    // Remove gravity (ENU up = +Z).
                    let accel_enu_debiased = accel_enu
                        - Vector3::new(0.0, 0.0, sensor::imu::GRAVITY_M_S2);
                    velocity += accel_enu_debiased * dt_s;
                    position += velocity * dt_s;
                }
                last_imu_ts_us = pkt.timestamp_us;

                // Altitude channel — complementary filter high-pass path.
                filt.update_imu(pkt.accel_z, pkt.timestamp_us);
            }

            // ── Barometer (50 Hz) ─────────────────────────────────────────────
            SensorMessage::Baro(pkt) => {
                let alt_m = barometric_altitude(pkt.pressure_pa, pkt.temp_c);
                filt.update_absolute(alt_m);
                position.z = filt.altitude();
            }

            // ── GPS (10 Hz) ───────────────────────────────────────────────────
            SensorMessage::Gps(pkt) => {
                // Seed ENU origin on the first fix.
                let (origin_lat, origin_lon, origin_alt) = *enu_origin.get_or_insert((
                    pkt.latitude,
                    pkt.longitude,
                    pkt.altitude_m,
                ));

                let (east_m, north_m) =
                    latlon_to_enu_xy(pkt.latitude, pkt.longitude, origin_lat, origin_lon);

                // Hard-correct horizontal position — GPS is the ground truth here.
                position.x = east_m;
                position.y = north_m;

                // Fuse altitude — both GPS and baro feed the absolute path.
                let gps_alt_above_origin = pkt.altitude_m - origin_alt;
                filt.update_absolute(gps_alt_above_origin);
                position.z = filt.altitude();
            }
        }

        let estimate = StateEstimate {
            timestamp_us: latest_ts_us,
            position,
            velocity,
            orientation,
            altitude_fused: filt.altitude(),
            alpha,
        };

        cycle += 1;

        // Throttled console output — avoids I/O dominating the hot path.
        if latest_ts_us.saturating_sub(last_log_ts_us) >= log_throttle_us {
            info!(cycle, "{estimate}");
            last_log_ts_us = latest_ts_us;
        }
    }
}
