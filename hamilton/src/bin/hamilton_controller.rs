use anyhow::Result;
use clap::Clap;
use hamilton::{
    driver::{hamilton_driver_from_config, BodyConfig},
    holonomic_controller::MoveCommand,
    lidar::Lidar,
    localisation::{LocalisationManager, LocaliserType},
    map::Map,
    navigation::{NavigationController, Pose2d},
    rviz_client::RvizClient,
};
use nalgebra as na;
use remote_controller::{start_remote_controller_server_with_map, Action, ActionList, AreaSize};
use std::{
    net::SocketAddrV4,
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc,
    },
    time::Duration,
};
use tracing::*;
use tracing_subscriber::filter::LevelFilter;

#[derive(Clap)]
#[clap()]
struct Args {
    #[clap(
        about = "Serial port for motors",
        default_value = "/dev/hamilton_dc_motors"
    )]
    motor_port: String,
    #[clap(about = "Serial port for lidar", default_value = "/dev/rplidar")]
    lidar_port: String,
    #[clap(long = "body_config", about = "Config path")]
    config: Option<String>,
    #[clap(long, default_value = "239.0.0.22:7071")]
    localisation_address: SocketAddrV4,
    #[clap(long, default_value = "239.0.0.22:7072")]
    pose_pub_address: SocketAddrV4,
    #[clap(long, default_value = "239.0.0.22:7075")]
    point_cloud_addr: SocketAddrV4,
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

    let mut lidar_driver = Lidar::open(args.lidar_port, args.point_cloud_addr)?;
    lidar_driver.stop_motor();

    let driver = hamilton_driver_from_config(&args.motor_port, body_config).await?;
    let localiser =
        LocalisationManager::new(args.localisation_address, LocaliserType::IrMarker).await?;
    let rviz_client = RvizClient::new(args.pose_pub_address)?;

    let mut navigation_controller = NavigationController::new(driver, localiser, rviz_client);

    let map = Map::new(na::Vector2::new(1., 1.), na::Vector2::new(0., -0.15));
    let (area_height, area_width) = map.get_size();

    let actions = ActionList::new(vec![
        Action::new("start_spin", "Start spinning lidar"),
        Action::new("stop_spin", "Stop spinning lidar"),
    ]);

    let controller_state = start_remote_controller_server_with_map(
        ([0, 0, 0, 0], 8080),
        AreaSize::new(area_width, area_height),
        actions,
    );

    let running = Arc::new(AtomicBool::new(true));
    let running_handle = running.clone();

    ctrlc::set_handler(move || {
        running_handle.store(false, Ordering::Release);
        println!("Caught interrupt\nExiting...");
    })?;

    while running.load(Ordering::Acquire) {
        tokio::time::sleep(Duration::from_millis(100)).await;
        if let Some(canvas_touch) = controller_state.try_receive_canvas_touch().await? {
            let (target, heading) = map.canvas_touch_to_pose(canvas_touch);
            let target_pose = Pose2d::from_na(target, heading);
            navigation_controller.set_target(target_pose);
        }

        if let Some((gamepad_command, time)) = controller_state.get_last_input_with_time().await {
            let move_command = MoveCommand::new(
                gamepad_command.left_x,
                gamepad_command.left_y,
                gamepad_command.right_y,
            );
            navigation_controller.set_user_command(move_command, time);
        }

        if let Some(action) = controller_state.try_receive_action().await? {
            match action.id() {
                "start_spin" => lidar_driver.start_motor(),
                "stop_spin" => lidar_driver.stop_motor(),
                _ => error!("Unknown action {:?}", action),
            }
        }

        navigation_controller.tick().await?;
    }
    Ok(())
}
