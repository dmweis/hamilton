use anyhow::Result;
use clap::Clap;
use hamilton::{
    map::Map,
    navigation::{OldNavigationController, Pose2d},
};
use nalgebra as na;
use pose_publisher::{pose::Color, PoseClientUpdate, PosePublisher};
use remote_controller::{start_remote_controller_server_with_map, ActionList, AreaSize};
use std::net::SocketAddrV4;

#[derive(Clap)]
#[clap(version = "0.0.1", author = "David M. W. <dweis7@gmail.com>")]
struct Args {
    #[clap(short, long, default_value = "239.0.0.22:7071")]
    address: SocketAddrV4,
    #[clap(short, long, default_value = "239.0.0.22:7072")]
    pose_pub_address: SocketAddrV4,
}

#[tokio::main]
async fn main() -> Result<()> {
    let args: Args = Args::parse();

    let map = Map::new(na::Vector2::new(1., 1.), na::Vector2::new(0., 0.));
    let (area_height, area_width) = map.get_size();
    let controller_state = start_remote_controller_server_with_map(
        ([0, 0, 0, 0], 8080),
        AreaSize::new(area_width, area_height),
        ActionList::default(),
    );
    let pose_publisher = PosePublisher::new(args.pose_pub_address)?;
    let mut navigation_controller = OldNavigationController::default();

    let mut localization_rx =
        hamilton::localisation::ir_tracker_localiser::create_localization_subscriber(args.address)
            .await?;
    while let Some(message) = localization_rx.recv().await {
        let mut update = PoseClientUpdate::new();

        if let Some(pose) = message.find_tracker_pose() {
            update
                .add("robot", (pose.position().x, pose.position().y, 0.1))
                .with_color(Color::Blue);
            let robot_target_vector = pose.rotation() * na::Vector2::new(0.1, 0.);
            update
                .add(
                    "robot direction",
                    (
                        pose.position().x + robot_target_vector.x,
                        pose.position().y + robot_target_vector.y,
                        0.1,
                    ),
                )
                .with_color(Color::Red);
            navigation_controller.update_current_pose(pose);
        } else {
            println!("Failed to find points");
        }

        if let Some(canvas_touch) = controller_state.get_latest_canvas_touch() {
            let (target, heading) = map.canvas_touch_to_pose(canvas_touch);
            update
                .add("target", (target.x, target.y, 0.1))
                .with_color(Color::Blue);

            let target_vector = heading * na::Vector2::new(0.1, 0.);
            update
                .add(
                    "target direction",
                    (target.x + target_vector.x, target.y + target_vector.y, 0.1),
                )
                .with_color(Color::Red);

            navigation_controller.update_target_pose(Pose2d::from_na(target, heading));
        }
        pose_publisher.publish(update)?;

        if let Some(command) = navigation_controller.calculate_gains() {
            println!("Navigation command: {:?}", command);
        }
    }
    Ok(())
}
