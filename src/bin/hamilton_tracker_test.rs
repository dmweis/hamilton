use anyhow::Result;
use clap::Clap;
use nalgebra as na;
use remote_controller::start_remote_controller_server;
use remote_controller::CanvasTouch;
use std::{
    net::SocketAddrV4,
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc,
    },
};
use tracing::*;
use tracing_subscriber::filter::LevelFilter;

#[derive(Clap)]
#[clap()]
struct Args {
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

    let controller_state = start_remote_controller_server(([0, 0, 0, 0], 8080));

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

            println!(
                "forward {:.2} strafe {:.2} yaw {:.2}",
                forward_gain, strafe_gain, yaw_gain
            );
        } else {
            error!("No tracker pose");
        }
    }
    println!("Exiting");
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
