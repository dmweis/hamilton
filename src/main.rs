mod driver;
mod holonomic_controller;
use anyhow::Result;
use clap::Clap;
use driver::HamiltonDriver;
use hamilton::hamilton_remote_server::{HamiltonRemote, HamiltonRemoteServer};
use holonomic_controller::MotorMapping;
use std::sync::Arc;
use std::time::Duration;
use tokio::sync::Mutex;
use tokio::time::{interval, sleep};
use tokio::{self, spawn};
use tonic::{transport::Server, Request, Response, Status, Streaming};

pub mod hamilton {
    tonic::include_proto!("hamilton");
}

struct HamiltonRemoteController {
    driver: Arc<Mutex<Box<dyn driver::HamiltonDriver>>>,
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
        driver: Arc<Mutex<Box<dyn driver::HamiltonDriver>>>,
        controller_config: holonomic_controller::MotorMapping,
    ) -> Self {
        HamiltonRemoteController {
            driver,
            motor_mapping: controller_config,
        }
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
    let mut driver: Box<dyn HamiltonDriver> =
        Box::new(driver::hamilton_lss_driver::HamiltonLssDriver::new(&args.port).await?);

    let mapping = if let Some(path) = args.config {
        holonomic_controller::MotorMapping::load(&path)?
    } else {
        holonomic_controller::MotorMapping::load_from_default()?
    };
    if args.test {
        return wheels_test(&mut driver, &mapping).await;
    }
    if args.move_test {
        return move_test(&mut driver, &mapping).await;
    }

    let address = "0.0.0.0:5001".parse()?;

    let shared_driver = Arc::new(Mutex::new(driver));

    let hamilton_remote = HamiltonRemoteController::new(Arc::clone(&shared_driver), mapping);
    spawn(async move {
        let mut reading_rate = interval(Duration::from_secs(1));
        loop {
            reading_rate.tick().await;
            let mut driver = shared_driver.lock().await;
            if let Ok(Some(voltage)) = driver.read_voltage().await {
                let color = if voltage < 3.0 * 3.6 {
                    lss_driver::LedColor::Red
                } else {
                    lss_driver::LedColor::Magenta
                };
                if driver.set_color(color).await.is_err() {
                    eprintln!("Failed to set color");
                }
            } else {
                eprintln!("Failed to read voltage");
            }
        }
    });
    println!("Hamilton remote started at {}", address);

    Server::builder()
        .add_service(HamiltonRemoteServer::new(hamilton_remote))
        .serve(address)
        .await?;

    Ok(())
}

async fn wheels_test(driver: &mut Box<dyn HamiltonDriver>, mapping: &MotorMapping) -> Result<()> {
    let command = holonomic_controller::HolonomicWheelCommand::new(1.0, 0.0, 0.0, 0.0);
    driver
        .send(mapping.apply_commands_by_mapping(&command))
        .await?;
    sleep(Duration::from_secs_f32(2.)).await;
    let command = holonomic_controller::HolonomicWheelCommand::new(0.0, 1.0, 0.0, 0.0);
    driver
        .send(mapping.apply_commands_by_mapping(&command))
        .await?;
    sleep(Duration::from_secs_f32(2.)).await;
    let command = holonomic_controller::HolonomicWheelCommand::new(0.0, 0.0, 1.0, 0.0);
    driver
        .send(mapping.apply_commands_by_mapping(&command))
        .await?;
    sleep(Duration::from_secs_f32(2.)).await;
    let command = holonomic_controller::HolonomicWheelCommand::new(0.0, 0.0, 0.0, 1.0);
    driver
        .send(mapping.apply_commands_by_mapping(&command))
        .await?;
    sleep(Duration::from_secs_f32(2.)).await;
    let command = holonomic_controller::HolonomicWheelCommand::new(0.0, 0.0, 0.0, 0.0);
    driver
        .send(mapping.apply_commands_by_mapping(&command))
        .await?;
    sleep(Duration::from_secs_f32(1.)).await;
    return Ok(());
}

async fn move_test(driver: &mut Box<dyn HamiltonDriver>, mapping: &MotorMapping) -> Result<()> {
    let mapped_move_command = mapping.apply_commands_by_mapping(
        &hamilton::MoveCommand {
            x: 0.25,
            y: 0.75,
            yaw: 0.0,
        }
        .into(),
    );
    driver
        .send(mapped_move_command)
        .await
        .map_err(|_| Status::internal("Failed to send message over serial port"))?;
    sleep(Duration::from_secs_f32(1.5)).await;
    let mapped_move_command = mapping.apply_commands_by_mapping(
        &hamilton::MoveCommand {
            x: -0.25,
            y: -0.75,
            yaw: 0.0,
        }
        .into(),
    );
    driver
        .send(mapped_move_command)
        .await
        .map_err(|_| Status::internal("Failed to send message over serial port"))?;
    sleep(Duration::from_secs_f32(1.5)).await;
    let command = holonomic_controller::HolonomicWheelCommand::new(0.0, 0.0, 0.0, 0.0);
    driver
        .send(mapping.apply_commands_by_mapping(&command))
        .await?;
    sleep(Duration::from_secs_f32(1.)).await;
    return Ok(());
}
