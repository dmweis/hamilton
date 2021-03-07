use anyhow::Result;
use clap::Clap;
use std::net::SocketAddrV4;

#[derive(Clap)]
#[clap(version = "0.0.1", author = "David M. W. <dweis7@gmail.com>")]
struct Args {
    #[clap(short, long, default_value = "239.0.0.22:7070")]
    address: SocketAddrV4,
}

#[tokio::main]
async fn main() -> Result<()> {
    let args: Args = Args::parse();

    let mut localization_rx =
        hamilton::localiser::create_localization_subscriber(args.address).await?;
    while let Some(message) = localization_rx.recv().await {
        if let Some((position, yaw)) = message.get_tracker_pose() {
            println!(
                "x: {:.2} y {:.2} yaw {:.2}",
                position.x,
                position.y,
                yaw.to_degrees()
            );
        } else {
            println!("Device not visible");
        }
    }
    Ok(())
}
