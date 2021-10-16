use anyhow::Result;
use clap::Clap;
use pose_publisher::{pose::Color, PointCloud2, PointCloudPublisher};
use rplidar_driver::{utils::sort_scan, RplidarDevice, RposError, ScanOptions};
use std::net::SocketAddrV4;

fn wait_for_enter() {
    use std::io::Read;
    println!("Press Enter to continue...");
    let _ = std::io::stdin().read(&mut [0]).unwrap();
}

#[derive(Clap)]
#[clap(version = "0.0.1", author = "David M. W. <dweis7@gmail.com>")]
struct Args {
    #[clap(long)]
    lidar_on: bool,
    #[clap(short, long, default_value = "239.0.0.22:7075")]
    address: SocketAddrV4,
    #[clap(long)]
    port: String,
}

fn main() -> Result<()> {
    let args: Args = Args::parse();

    let mut lidar = RplidarDevice::open_port(&args.port)?;
    if args.lidar_on {
        let point_cloud_publisher = PointCloudPublisher::new(args.address)?;

        let scan_options = ScanOptions::with_mode(2);

        let _ = lidar.start_scan_with_options(&scan_options)?;
        loop {
            match lidar.grab_scan() {
                Ok(mut scan) => {
                    sort_scan(&mut scan)?;

                    let scan = scan
                        .into_iter()
                        .filter(|scan| scan.is_valid())
                        .map(|scan_point| {
                            let x = scan_point.distance() * (-scan_point.angle()).cos();
                            let y = scan_point.distance() * (-scan_point.angle()).sin();
                            (x, y)
                        })
                        .collect::<Vec<_>>();

                    let point_cloud = PointCloud2::from_points("example cloud", scan)
                        .with_color(Color::Red)
                        .with_parent_frame_id("robot");
                    point_cloud_publisher.publish(&point_cloud)?;
                }
                Err(err) => match err {
                    RposError::OperationTimeout => continue,
                    _ => println!("Error: {:?}", err),
                },
            }
        }
    } else {
        lidar.stop_motor()?;
        println!("Lidar paused.");
        wait_for_enter();
    }
    Ok(())
}
