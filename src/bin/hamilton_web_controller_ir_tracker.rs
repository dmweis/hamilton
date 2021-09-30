use anyhow::Result;
use clap::Clap;
use hamilton::{
    driver::{BodyConfig, DriverType, HamiltonDcDriver, HamiltonDriver, HamiltonLssDriver},
    holonomic_controller::HolonomicWheelCommand,
    map::Map,
    navigation::{NavigationController, Pose},
};
use nalgebra as na;
use pose_publisher::{pose::Color, PoseClientUpdate, PosePublisher};
use remote_controller::{start_remote_controller_server_with_map, ActionList, AreaSize};
use std::{
    net::SocketAddrV4,
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc,
    },
    time::Duration,
};
use tokio::{
    self, spawn,
    sync::Mutex,
    time::{interval, sleep, timeout},
};
use tracing::*;
use tracing_subscriber::filter::LevelFilter;

#[derive(Clap)]
#[clap()]
struct Args {
    #[clap(about = "Serial port to use")]
    port: String,
    #[clap(long = "config", about = "Config path")]
    config: Option<String>,
    #[clap(short, long, default_value = "239.0.0.22:7071")]
    address: SocketAddrV4,
    #[clap(short, long, default_value = "239.0.0.22:7072")]
    pose_pub_address: SocketAddrV4,
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

    let map = Map::new(na::Vector2::new(1., 1.), na::Vector2::new(0., 0.));
    let mut navigation_controller = NavigationController::default();

    let pose_publisher = PosePublisher::new(args.pose_pub_address)?;

    let hamilton_driver: Box<dyn HamiltonDriver> =
        if let DriverType::LSS = body_config.driver_type() {
            let lss_driver = Arc::new(Mutex::new(lss_driver::LSSDriver::new(&args.port)?));
            Box::new(HamiltonLssDriver::new(lss_driver, body_config).await?)
        } else {
            Box::new(HamiltonDcDriver::new(&args.port, body_config)?)
        };
    let shared_driver = Arc::new(Mutex::new(hamilton_driver));

    let cloned_driver = Arc::clone(&shared_driver);
    let (area_height, area_width) = map.get_size();
    let controller_state = start_remote_controller_server_with_map(
        ([0, 0, 0, 0], 8080),
        AreaSize::new(area_width, area_height),
        ActionList::default(),
    );

    spawn(async move {
        let mut reading_rate = interval(Duration::from_secs(1));
        loop {
            reading_rate.tick().await;
            let mut driver = shared_driver.lock().await;
            if let Ok(Some(voltage)) = driver.read_voltage().await {
                info!("Current voltage is {}", voltage);
                let color = if voltage < 3.0 * 3.6 {
                    lss_driver::LedColor::Red
                } else {
                    lss_driver::LedColor::Magenta
                };
                if driver.set_color(color).await.is_err() {
                    error!("Failed to set color");
                }
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
        hamilton::ir_tracker_localiser::create_localization_subscriber(args.address).await?;

    // this loop is getting super convoluted
    // refactor
    // pattern match here would decrease the indentation
    // but I don't think it would make this cleaner
    // this is complicated logic so it looks dense
    // refactor to structure
    loop {
        if !running.load(Ordering::Acquire) {
            break;
        }
        if let Ok(message) = timeout(Duration::from_millis(500), localization_rx.recv()).await {
            if let Some(message) = message {
                if let Some(pose) = message.find_tracker_pose() {
                    let mut update = PoseClientUpdate::new();

                    let robot_target_vector = pose.rotation() * na::Vector2::new(0.1, 0.);
                    let robot_rotation =
                        na::UnitQuaternion::from_euler_angles(0., 0., pose.rotation().angle())
                            .coords;
                    update
                        .add("robot", (pose.position().x, pose.position().y, 0.1))
                        .with_rotation((
                            robot_rotation[0],
                            robot_rotation[1],
                            robot_rotation[2],
                            robot_rotation[3],
                        ));
                    update
                        .add(
                            "robot direction",
                            (
                                pose.position().x + robot_target_vector.x,
                                pose.position().y + robot_target_vector.y,
                                0.1,
                            ),
                        )
                        .with_color(Color::Green);
                    // set start position
                    navigation_controller.update_current_pose(pose.clone());

                    if let Some(canvas_touch) = controller_state.get_latest_canvas_touch() {
                        let (target, heading) = map.canvas_touch_to_pose(canvas_touch);

                        update.add("target", (target.x, target.y, 0.1));
                        let target_vector = heading * na::Vector2::new(0.1, 0.);
                        update
                            .add(
                                "target direction",
                                (target.x + target_vector.x, target.y + target_vector.y, 0.1),
                            )
                            .with_color(Color::Green);

                        navigation_controller.update_target_pose(Pose::from_na(target, heading));
                    }

                    if let Some(move_command) = navigation_controller.calculate_drive() {
                        cloned_driver.lock().await.send(move_command).await?;
                    } else {
                        cloned_driver
                            .lock()
                            .await
                            .send(HolonomicWheelCommand::stopped())
                            .await?;
                    }
                    pose_publisher.publish(update)?;
                } else {
                    let move_command = HolonomicWheelCommand::stopped();
                    cloned_driver.lock().await.send(move_command).await?;
                    error!("No tracker pose found");
                }
            } else {
                error!("UDP channel closed");
                break;
            }
        } else {
            let move_command = HolonomicWheelCommand::stopped();
            cloned_driver.lock().await.send(move_command).await?;
            error!("No udp messages");
        }
    }

    let move_command = HolonomicWheelCommand::stopped();
    cloned_driver.lock().await.send(move_command).await?;
    sleep(Duration::from_secs(1)).await;
    Ok(())
}
