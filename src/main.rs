mod driver;
mod holonomic_controller;
use anyhow::Result;
use clap::Clap;
use hamilton::hamilton_remote_server::{HamiltonRemote, HamiltonRemoteServer};
use std::sync::Arc;
use tokio::sync::Mutex;
use tonic::{transport::Server, Request, Response, Status, Streaming};

pub mod hamilton {
    tonic::include_proto!("hamilton");
}

struct HamiltonRemoteController {
    driver: Arc<Mutex<driver::HamiltonDriver>>,
    motor_mapping: holonomic_controller::MotorMapping,
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
            let mapped_move_command = self
                .motor_mapping
                .apply_commands_by_mapping(&move_command.into());
            let mut driver = self.driver.lock().await;
            driver
                .send(mapped_move_command)
                .await
                .map_err(|_| Status::internal("Failed to send message over serial port"))?;
        }
        Ok(Response::new(hamilton::MoveResponse {}))
    }
}

impl HamiltonRemoteController {
    fn new(
        driver: driver::HamiltonDriver,
        controller_config: holonomic_controller::MotorMapping,
    ) -> Self {
        HamiltonRemoteController {
            driver: Arc::new(Mutex::new(driver)),
            motor_mapping: controller_config,
        }
    }
}

#[derive(Clap)]
#[clap()]
struct Args {
    #[clap(about = "Serial port to use")]
    port: String,
}

#[tokio::main]
async fn main() -> Result<()> {
    let address = "127.0.0.1:5001".parse()?;
    let args = Args::parse();
    let driver = driver::HamiltonDriver::new(&args.port)?;
    let hamilton_remote = HamiltonRemoteController::new(
        driver,
        holonomic_controller::MotorMapping::load_from_default()?,
    );

    println!("Hamilton remote started at {}", address);

    Server::builder()
        .add_service(HamiltonRemoteServer::new(hamilton_remote))
        .serve(address)
        .await?;

    Ok(())
}
