use anyhow::Result;
use clap::Clap;
use hamilton::driver::{
    BodyConfig, DriverType, HamiltonDcDriver, HamiltonDriver, HamiltonLssDriver,
};
use hamilton::holonomic_controller;
use holonomic_controller::HolonomicWheelCommand;
use std::sync::Arc;
use std::time::Duration;
use tokio::sync::Mutex;
use tokio::time::sleep;
use tokio::{self};
use tonic::Status;
use tracing::*;
use tracing_subscriber::filter::LevelFilter;

#[derive(Clap)]
#[clap()]
struct Args {
    #[clap(about = "Serial port to use")]
    port: String,
    #[clap(long = "test", about = "test wheels")]
    test: bool,
    #[clap(long = "move_test", about = "test wheels")]
    move_test: bool,
    #[clap(long = "config", about = "Config path")]
    config: Option<String>,
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

    let mut hamilton_driver: Box<dyn HamiltonDriver> =
        if let DriverType::LSS = body_config.driver_type() {
            let lss_driver = Arc::new(Mutex::new(lss_driver::LSSDriver::new(&args.port)?));
            Box::new(HamiltonLssDriver::new(lss_driver, body_config).await?)
        } else {
            Box::new(HamiltonDcDriver::new(&args.port, body_config)?)
        };

    if args.test {
        return wheels_test(&mut hamilton_driver).await;
    }
    if args.move_test {
        return move_test(&mut hamilton_driver).await;
    }
    Ok(())
}

async fn wheels_test(driver: &mut Box<dyn HamiltonDriver>) -> Result<()> {
    async fn wait() {
        sleep(Duration::from_secs_f32(2.)).await;
    }
    let move_size = 1.0;
    info!("Left front");
    driver
        .send(HolonomicWheelCommand::new(move_size, 0.0, 0.0, 0.0))
        .await?;
    wait().await;
    driver
        .send(HolonomicWheelCommand::new(-move_size, 0.0, 0.0, 0.0))
        .await?;
    wait().await;
    info!("Right front");
    driver
        .send(HolonomicWheelCommand::new(0.0, move_size, 0.0, 0.0))
        .await?;
    wait().await;
    driver
        .send(HolonomicWheelCommand::new(0.0, -move_size, 0.0, 0.0))
        .await?;
    wait().await;
    info!("Left rear");
    driver
        .send(HolonomicWheelCommand::new(0.0, 0.0, move_size, 0.0))
        .await?;
    wait().await;
    driver
        .send(HolonomicWheelCommand::new(0.0, 0.0, -move_size, 0.0))
        .await?;
    wait().await;
    info!("Right rear");
    driver
        .send(HolonomicWheelCommand::new(0.0, 0.0, 0.0, move_size))
        .await?;
    wait().await;
    driver
        .send(HolonomicWheelCommand::new(0.0, 0.0, 0.0, -move_size))
        .await?;
    wait().await;
    info!("Stopping");
    driver
        .send(HolonomicWheelCommand::new(0.0, 0.0, 0.0, 0.0))
        .await?;
    wait().await;

    Ok(())
}

async fn move_test(driver: &mut Box<dyn HamiltonDriver>) -> Result<()> {
    driver
        .send(
            MoveCommand {
                x: 0.25,
                y: 0.75,
                yaw: 0.0,
            }
            .into(),
        )
        .await
        .map_err(|_| Status::internal("Failed to send message over serial port"))?;
    sleep(Duration::from_secs_f32(1.5)).await;
    driver
        .send(
            MoveCommand {
                x: -0.25,
                y: -0.75,
                yaw: 0.0,
            }
            .into(),
        )
        .await
        .map_err(|_| Status::internal("Failed to send message over serial port"))?;
    sleep(Duration::from_secs_f32(1.5)).await;
    let command = holonomic_controller::HolonomicWheelCommand::new(0.0, 0.0, 0.0, 0.0);
    driver.send(command).await?;
    sleep(Duration::from_secs_f32(1.)).await;
    Ok(())
}

struct MoveCommand {
    x: f32,
    y: f32,
    yaw: f32,
}

impl From<MoveCommand> for HolonomicWheelCommand {
    fn from(move_command: MoveCommand) -> Self {
        let forward = move_command.x;
        let strafe = move_command.y;
        let rotation = move_command.yaw;
        HolonomicWheelCommand::new(
            forward - rotation - strafe,
            forward + rotation + strafe,
            forward - rotation + strafe,
            forward + rotation - strafe,
        )
    }
}
