use anyhow::Result;
use clap::Clap;
use hamilton::{
    driver::{BodyConfig, HamiltonLssDriver},
    holonomic_controller::HolonomicWheelCommand,
};
use nalgebra as na;
use remote_controller::start_remote_controller_server;
use remote_controller::CanvasTouch;
use std::net::SocketAddrV4;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::Duration;
use tokio::sync::Mutex;
use tokio::time::{interval, sleep};
use tokio::{self, spawn};
use tracing::*;
use tracing_subscriber::filter::LevelFilter;

#[derive(Clap)]
#[clap()]
struct Args {
    #[clap(about = "Serial port to use")]
    port: String,
    #[clap(long = "config", about = "Config path")]
    config: Option<String>,
    #[clap(short, long, default_value = "239.0.0.22:7070")]
    address: SocketAddrV4,
}

#[tokio::main]
async fn main() -> Result<()> {
    let args = Args::parse();
    tracing_subscriber::fmt()
        .pretty()
        .with_env_filter("hamilton=info")
        .with_max_level(LevelFilter::INFO)
        .init();
    let body_config = if let Some(path) = args.config {
        info!("Loading configuration from {}", path);
        BodyConfig::load(&path)?
    } else {
        info!("Loading default configuration");
        BodyConfig::load_from_default()?
    };

    let lss_driver = Arc::new(Mutex::new(lss_driver::LSSDriver::new(&args.port)?));
    let hamilton_driver = HamiltonLssDriver::new(lss_driver, body_config).await?;
    let shared_driver = Arc::new(Mutex::new(hamilton_driver));

    let cloned_driver = Arc::clone(&shared_driver);
    let controller_state = start_remote_controller_server(([0, 0, 0, 0], 8080));

    spawn(async move {
        let mut reading_rate = interval(Duration::from_secs(1));
        loop {
            reading_rate.tick().await;
            let mut driver = shared_driver.lock().await;
            if let Ok(voltage) = driver.read_voltage().await {
                info!("Current voltage is {}", voltage);
                let color = if voltage < 3.0 * 3.6 {
                    lss_driver::LedColor::Red
                } else {
                    lss_driver::LedColor::Magenta
                };
                if driver.set_color(color).await.is_err() {
                    error!("Failed to set color");
                }
            } else {
                error!("Failed to read voltage");
            }
        }
    });

    let running = Arc::new(AtomicBool::new(true));
    let running_handle = running.clone();

    ctrlc::set_handler(move || {
        running_handle.store(false, Ordering::Release);
        println!("Caught interrupt\nExiting...");
    })?;

    let mut localization_rx =
        hamilton::localiser::create_localization_subscriber(args.address).await?;

    let mut desired_position = None;

    while let Some(message) = localization_rx.recv().await {
        if !running.load(Ordering::Acquire) {
            break;
        }
        if let Some((position, yaw)) = message.get_tracker_pose() {
            // set start position
            if desired_position.is_none() {
                desired_position = Some(position)
            }
            let state = controller_state.get_last_gamepad_command();

            if let Some(canvas_touch) = controller_state.get_latest_canvas_touch() {
                let target = from_canvas_to_position(canvas_touch);
                desired_position = Some(target);
            }

            let command_yaw = state.left_y.atan2(state.left_x);
            let translation = position - desired_position.unwrap();
            let gain_vector = na::Rotation2::new(-yaw) * translation;

            let mut forward_gain = -(gain_vector.x * 10.0).clamp(-0.5, 0.5);
            if forward_gain.abs() < 0.1 {
                forward_gain = 0.0;
            }
            let mut strafe_gain = -(gain_vector.y * 10.0).clamp(-0.5, 0.5);
            if strafe_gain.abs() < 0.1 {
                strafe_gain = 0.0;
            }

            let limit = 0.5;
            let mut yaw_gain = -(yaw - command_yaw).clamp(-limit, limit);
            if yaw_gain.abs() < 0.1 {
                yaw_gain = 0.0;
            }

            let move_command =
                HolonomicWheelCommand::from_move(forward_gain, strafe_gain, yaw_gain);
            cloned_driver.lock().await.send(move_command).await?;
        } else {
            let move_command = HolonomicWheelCommand::from_move(0.0, 0.0, 0.0);
            cloned_driver.lock().await.send(move_command).await?;
            error!("No tracker pose");
        }
    }
    let move_command = HolonomicWheelCommand::from_move(0.0, 0.0, 0.0);
    cloned_driver.lock().await.send(move_command).await?;
    sleep(Duration::from_secs(1)).await;
    Ok(())
}

fn from_canvas_to_position(touch_event: CanvasTouch) -> na::Point2<f32> {
    let front = na::Point2::new(0.42_f32, 0.15_f32);
    let rear = na::Point2::new(-0.75_f32, -1.24_f32);
    let y = linear_map(touch_event.down_x, 0.0, touch_event.width, front.y, rear.y);
    let x = linear_map(touch_event.down_y, 0.0, touch_event.height, front.x, rear.x);
    na::Point2::new(x, y)
}

fn linear_map(value: f32, in_min: f32, in_max: f32, out_min: f32, out_max: f32) -> f32 {
    (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
}
