mod driver;
mod holonomic_controller;
use anyhow::Result;
use clap::Clap;
use driver::{BodyConfig, HamiltonLssDriver};
use hamilton::hamilton_remote_server::{HamiltonRemote, HamiltonRemoteServer};
use std::sync::Arc;
use std::time::Duration;
use tokio::sync::Mutex;
use tokio::time::{interval, sleep};
use tokio::{self, spawn};
use tonic::{transport::Server, Request, Response, Status, Streaming};
use tracing::*;
use tracing_subscriber::filter::LevelFilter;

pub mod hamilton {
    tonic::include_proto!("hamilton");
}

struct HamiltonRemoteController {
    driver: Arc<Mutex<HamiltonLssDriver>>,
}

#[tonic::async_trait]
impl HamiltonRemote for HamiltonRemoteController {
    async fn move_stream(
        &self,
        commands: Request<Streaming<hamilton::MoveRequest>>,
    ) -> std::result::Result<Response<hamilton::MoveResponse>, tonic::Status> {
        let mut command_stream = commands.into_inner();
        while let Some(message) = command_stream.message().await? {
            let move_command = message.command.unwrap();
            let mut driver = self.driver.lock().await;
            driver
                .send(move_command.into())
                .await
                .map_err(|_| Status::internal("Failed to send message over serial port"))?;
        }
        Ok(Response::new(hamilton::MoveResponse {}))
    }
}

impl HamiltonRemoteController {
    fn new(driver: Arc<Mutex<HamiltonLssDriver>>) -> Self {
        HamiltonRemoteController { driver }
    }
}

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
    let mut driver = HamiltonLssDriver::new(&args.port, body_config).await?;

    if args.test {
        return wheels_test(&mut driver).await;
    }
    if args.move_test {
        return move_test(&mut driver).await;
    }

    let address = "0.0.0.0:5001".parse()?;

    let shared_driver = Arc::new(Mutex::new(driver));

    let hamilton_remote = HamiltonRemoteController::new(Arc::clone(&shared_driver));
    spawn(async move {
        let mut reading_rate = interval(Duration::from_secs(1));
        loop {
            reading_rate.tick().await;
            let mut driver = shared_driver.lock().await;
            if let Ok(voltage) = driver.read_voltage().await {
                info!("Current voltage is {}", voltage);
                let color = if voltage < 3.0 * 3.6 {
                    lss_driver::LedColor::Red
                } else {
                    lss_driver::LedColor::Magenta
                };
                if driver.set_color(color).await.is_err() {
                    error!("Failed to set color");
                }
            } else {
                error!("Failed to read voltage");
            }
        }
    });
    info!("Hamilton remote started at {}", address);

    Server::builder()
        .add_service(HamiltonRemoteServer::new(hamilton_remote))
        .serve(address)
        .await?;

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
            hamilton::MoveCommand {
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
            hamilton::MoveCommand {
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
