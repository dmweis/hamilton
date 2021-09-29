use anyhow::Result;
use clap::Clap;
use nalgebra as na;
use pose_publisher::{pose::Color, PoseClientUpdate, PosePublisher};
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

    let pose_publisher = PosePublisher::new(args.pose_pub_address)?;

    let mut localization_rx =
        hamilton::ir_tracker_localiser::create_localization_subscriber(args.address).await?;
    while let Some(message) = localization_rx.recv().await {
        if let Some(pose) = message.find_tracker_pose() {
            let mut update = PoseClientUpdate::new();
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
            pose_publisher.publish(update)?;

            println!("Center pose {}", pose);
        } else {
            println!("Failed to find points");
        }
    }
    Ok(())
}
