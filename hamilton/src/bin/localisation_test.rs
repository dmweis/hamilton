use anyhow::Result;
use clap::Clap;
use hamilton::{
    localisation::{LocalisationManager, LocaliserType},
    rviz_client::RvizClient,
};
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
    #[clap(long, default_value = "239.0.0.22:7071")]
    localisation_address: SocketAddrV4,
    #[clap(long, default_value = "239.0.0.22:7072")]
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

    let mut localiser =
        LocalisationManager::new(args.localisation_address, LocaliserType::IrMarker).await?;
    let mut rviz_client = RvizClient::new(args.pose_pub_address)?;
    rviz_client.set_robot_name("fake_robot");
    rviz_client.set_target_name("fake_target");

    let running = Arc::new(AtomicBool::new(true));
    let running_handle = running.clone();

    ctrlc::set_handler(move || {
        running_handle.store(false, Ordering::Release);
        println!("Caught interrupt\nExiting...");
    })?;

    while running.load(Ordering::Acquire) {
        tokio::time::sleep(Duration::from_millis(100)).await;

        if let Some(pose) = localiser.get_latest_pose().await? {
            info!(
                "x {:?} y {:?} angle {:?}",
                pose.position().x,
                pose.position().y,
                pose.rotation().angle().to_degrees()
            );
            rviz_client.set_robot_pose(pose);

            rviz_client.publish()?;
        } else {
            warn!("Not localised");
        }
    }
    Ok(())
}
