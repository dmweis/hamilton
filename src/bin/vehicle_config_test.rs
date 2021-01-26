use anyhow::Result;
use clap::Clap;
use hamilton::driver::{BodyConfig, HamiltonLssDriver};
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
    let lss_driver = Arc::new(Mutex::new(lss_driver::LSSDriver::new(&args.port)?));
    let mut hamilton_driver = HamiltonLssDriver::new(lss_driver, body_config).await?;

    if args.test {
        return wheels_test(&mut hamilton_driver).await;
    }
    if args.move_test {
        return move_test(&mut hamilton_driver).await;
    }
    Ok(())
}

async fn wheels_test(driver: &mut HamiltonLssDriver) -> Result<()> {
    info!("Left front");
    let command = holonomic_controller::HolonomicWheelCommand::new(1.0, 0.0, 0.0, 0.0);
    driver.send(command).await?;
    sleep(Duration::from_secs_f32(2.)).await;
    info!("Right front");
    let command = holonomic_controller::HolonomicWheelCommand::new(0.0, 1.0, 0.0, 0.0);
    driver.send(command).await?;
    sleep(Duration::from_secs_f32(2.)).await;
    info!("Left rear");
    let command = holonomic_controller::HolonomicWheelCommand::new(0.0, 0.0, 1.0, 0.0);
    driver.send(command).await?;
    sleep(Duration::from_secs_f32(2.)).await;
    info!("Right rear");
    let command = holonomic_controller::HolonomicWheelCommand::new(0.0, 0.0, 0.0, 1.0);
    driver.send(command).await?;
    sleep(Duration::from_secs_f32(2.)).await;
    info!("Stopping");
    let command = holonomic_controller::HolonomicWheelCommand::new(0.0, 0.0, 0.0, 0.0);
    driver.send(command).await?;
    sleep(Duration::from_secs_f32(1.)).await;
    return Ok(());
}

async fn move_test(driver: &mut HamiltonLssDriver) -> Result<()> {
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
    return Ok(());
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
