use anyhow::Result;
use clap::Clap;
use hamilton::{
    driver::{BodyConfig, HamiltonLssDriver},
    holonomic_controller::HolonomicWheelCommand,
};
use remote_controller::start_remote_controller_server;
use std::{
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc,
    },
    time::Duration,
};
use tokio::{
    self, spawn,
    sync::Mutex,
    time::{interval, sleep},
};
use tracing::*;
use tracing_subscriber::filter::LevelFilter;

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
    let hamilton_driver = HamiltonLssDriver::new(lss_driver, body_config).await?;
    let shared_driver = Arc::new(Mutex::new(hamilton_driver));

    let cloned_driver = Arc::clone(&shared_driver);
    let controller_state = start_remote_controller_server(([0, 0, 0, 0], 8080));

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

    let running = Arc::new(AtomicBool::new(true));
    let running_handle = running.clone();

    ctrlc::set_handler(move || {
        running_handle.store(false, Ordering::Release);
        println!("Caught interrupt\nExiting...");
    })?;

    while running.load(Ordering::Acquire) {
        let gamepad_command = controller_state.get_last_gamepad_command();
        let move_command = HolonomicWheelCommand::from_move(
            gamepad_command.left_x,
            gamepad_command.left_y,
            gamepad_command.right_y,
        );
        cloned_driver.lock().await.send(move_command).await?;
        tokio::time::sleep(Duration::from_millis(20)).await;
    }

    let move_command = HolonomicWheelCommand::stopped();
    cloned_driver.lock().await.send(move_command).await?;
    sleep(Duration::from_secs(1)).await;
    Ok(())
}
