use anyhow::Result;
use clap::Parser;
use hamilton::driver::{DriverType, HamiltonDcDriver, HamiltonDriver, HamiltonLssDriver};
use hamilton::{configuration, holonomic_controller};
use holonomic_controller::HolonomicWheelCommand;
use std::path::PathBuf;
use std::sync::Arc;
use std::time::Duration;
use tokio::sync::Mutex;
use tokio::time::sleep;
use tokio::{self};
use tracing::*;

#[derive(Parser, Debug)]
#[command(
    version,
    author = "David M. Weis <dweis7@gmail.com>",
    about = "Hamilton"
)]
struct Args {
    /// Serial port to use
    #[arg()]
    port: String,
    /// test wheels
    #[arg(long = "test")]
    test: bool,
    /// test wheels
    #[arg(long = "move_test")]
    move_test: bool,
    /// Config path
    #[arg(short, long)]
    config: Option<PathBuf>,
}

#[tokio::main]
async fn main() -> Result<()> {
    let args = Args::parse();

    let app_config = configuration::AppConfig::load_config(&args.config)?;

    let body_config = app_config.body;

    let mut hamilton_driver: Box<dyn HamiltonDriver> =
        if let DriverType::LSS = body_config.driver_type() {
            let lss_driver = Arc::new(Mutex::new(lss_driver::LSSDriver::new(&args.port)?));
            Box::new(HamiltonLssDriver::new(lss_driver, body_config).await?)
        } else {
            Box::new(HamiltonDcDriver::new(body_config)?)
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
        sleep(Duration::from_secs_f32(0.1)).await;
    }
    sleep(Duration::from_secs_f32(2.0)).await;
    let move_size = 1.0;
    info!("Left front");
    info!("forwards");
    for _ in 0..20 {
        driver
            .send(HolonomicWheelCommand::new(move_size, 0.0, 0.0, 0.0))
            .await?;
        wait().await;
    }
    info!("backwards");
    for _ in 0..20 {
        driver
            .send(HolonomicWheelCommand::new(-move_size, 0.0, 0.0, 0.0))
            .await?;
        wait().await;
    }

    info!("Right front");
    info!("forwards");
    for _ in 0..20 {
        driver
            .send(HolonomicWheelCommand::new(0.0, move_size, 0.0, 0.0))
            .await?;
        wait().await;
    }
    info!("backwards");
    for _ in 0..20 {
        driver
            .send(HolonomicWheelCommand::new(0.0, -move_size, 0.0, 0.0))
            .await?;
        wait().await;
    }
    info!("Left rear");
    info!("forwards");
    for _ in 0..20 {
        driver
            .send(HolonomicWheelCommand::new(0.0, 0.0, move_size, 0.0))
            .await?;
        wait().await;
    }
    info!("backwards");
    for _ in 0..20 {
        driver
            .send(HolonomicWheelCommand::new(0.0, 0.0, -move_size, 0.0))
            .await?;
        wait().await;
    }
    info!("Right rear");
    info!("forwards");
    for _ in 0..20 {
        driver
            .send(HolonomicWheelCommand::new(0.0, 0.0, 0.0, move_size))
            .await?;
        wait().await;
    }
    info!("backwards");
    for _ in 0..20 {
        driver
            .send(HolonomicWheelCommand::new(0.0, 0.0, 0.0, -move_size))
            .await?;
        wait().await;
    }
    info!("Stopping");
    for _ in 0..20 {
        driver
            .send(HolonomicWheelCommand::new(0.0, 0.0, 0.0, 0.0))
            .await?;
        wait().await;
    }
    Ok(())
}

async fn move_test(driver: &mut Box<dyn HamiltonDriver>) -> Result<()> {
    for _ in 0..15 {
        driver
            .send(
                MoveCommand {
                    x: 0.25,
                    y: 0.75,
                    yaw: 0.0,
                }
                .into(),
            )
            .await?;
        sleep(Duration::from_secs_f32(0.1)).await;
    }
    for _ in 0..15 {
        driver
            .send(
                MoveCommand {
                    x: -0.25,
                    y: -0.75,
                    yaw: 0.0,
                }
                .into(),
            )
            .await?;
        sleep(Duration::from_secs_f32(0.1)).await;
    }
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
