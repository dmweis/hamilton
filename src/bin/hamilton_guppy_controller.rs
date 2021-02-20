use anyhow::Result;
use clap::Clap;
use guppy::arm_config::ArmConfig;
use guppy::arm_controller::LssArmController;
use guppy::arm_driver::SharedSerialArmDriver;
use guppy::grpc_controller::{
    ControllerWrapper, GuppyConfigHandler, GuppyConfigureServer, GuppyControllerHandler,
    GuppyControllerServer,
};
use hamilton::{
    driver::{BodyConfig, HamiltonLssDriver},
    holonomic_controller::HolonomicWheelCommand,
};
#[allow(unused_imports)]
use hamilton_service::hamilton_remote_server::{HamiltonRemote, HamiltonRemoteServer};
use lss_driver::LSSDriver;
use remote_controller::start_remote_controller_server;
use std::sync::Arc;
use std::time::Duration;
use tokio::sync::Mutex;
use tokio::time::interval;
use tokio::{self, spawn};
use tonic::{transport::Server, Request, Response, Status, Streaming};
use tracing::*;
use tracing_subscriber::filter::LevelFilter;

pub mod hamilton_service {
    tonic::include_proto!("hamilton_service");
}

struct HamiltonRemoteController {
    driver: Arc<Mutex<HamiltonLssDriver>>,
}

#[tonic::async_trait]
impl HamiltonRemote for HamiltonRemoteController {
    async fn move_stream(
        &self,
        commands: Request<Streaming<hamilton_service::MoveRequest>>,
    ) -> std::result::Result<Response<hamilton_service::MoveResponse>, tonic::Status> {
        let mut command_stream = commands.into_inner();
        while let Some(message) = command_stream.message().await? {
            let move_command = message.command.unwrap();
            let mut driver = self.driver.lock().await;
            driver
                .send(move_command.into())
                .await
                .map_err(|_| Status::internal("Failed to send message over serial port"))?;
        }
        Ok(Response::new(hamilton_service::MoveResponse {}))
    }
}

impl HamiltonRemoteController {
    #[allow(dead_code)]
    fn new(driver: Arc<Mutex<HamiltonLssDriver>>) -> Self {
        HamiltonRemoteController { driver }
    }
}

async fn connect_guppy(driver: Arc<Mutex<LSSDriver>>) -> Result<ControllerWrapper> {
    let config = ArmConfig::included();
    let driver = SharedSerialArmDriver::new(driver, config.clone()).await?;
    let controller = LssArmController::new(driver, config);
    Ok(Arc::new(Mutex::new(controller)))
}

#[derive(Clap)]
#[clap()]
struct Args {
    #[clap(about = "Serial port to use")]
    port: String,
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
    let guppy_controller = connect_guppy(Arc::clone(&lss_driver)).await?;
    let hamilton_driver = HamiltonLssDriver::new(lss_driver, body_config).await?;

    let address = "0.0.0.0:5001".parse()?;

    let shared_driver = Arc::new(Mutex::new(hamilton_driver));

    let cloned_driver = Arc::clone(&shared_driver);
    let controller_state = start_remote_controller_server(([0, 0, 0, 0], 8080));
    spawn(async move {
        let mut reading_rate = interval(Duration::from_millis(50));
        loop {
            reading_rate.tick().await;
            let state = controller_state.lock().unwrap().get_latest();
            let move_command =
                HolonomicWheelCommand::from_move(state.left_x, state.left_y, state.right_y);
            cloned_driver.lock().await.send(move_command).await.unwrap();
        }
    });

    let config_handler = GuppyConfigHandler::new(Arc::clone(&guppy_controller));
    let controller_handler = GuppyControllerHandler::new(guppy_controller);
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
        .add_service(GuppyConfigureServer::new(config_handler))
        .add_service(GuppyControllerServer::new(controller_handler))
        .serve(address)
        .await?;

    Ok(())
}

impl From<hamilton_service::MoveCommand> for HolonomicWheelCommand {
    fn from(move_command: hamilton_service::MoveCommand) -> Self {
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
