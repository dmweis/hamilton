mod messages;

use std::sync::Arc;

use anyhow::Result;
use tracing::error;
use zenoh::{prelude::r#async::*, subscriber::FlumeSubscriber, Session, SessionDeclarations};

use crate::{
    driver::HamiltonDriver, error::ErrorWrapper, holonomic_controller::HolonomicWheelCommand,
};
use messages::InputMessage;

pub async fn start_gamepad_loop(
    zenoh_session: Arc<Session>,
    mut driver: Box<dyn HamiltonDriver>,
) -> Result<()> {
    let mut gamepad_subscriber = zenoh_session
        .declare_subscriber("remote-control/gamepad")
        .res()
        .await
        .map_err(ErrorWrapper::ZenohError)?;

    tokio::spawn({
        let zenoh_session = zenoh_session.clone();
        async move {
            while let Err(err) =
                run_gamepad_listener(&mut gamepad_subscriber, &mut *driver, zenoh_session.clone())
                    .await
            {
                error!("Gamepad listener failed with {:?}", err);
            }
        }
    });
    Ok(())
}

async fn run_gamepad_listener(
    subscriber: &mut FlumeSubscriber<'_>,
    driver: &mut dyn HamiltonDriver,
    zenoh_session: Arc<Session>,
) -> anyhow::Result<()> {
    loop {
        let sample = subscriber.recv_async().await?;
        let message: String = sample.value.try_into()?;
        let message: InputMessage = serde_json::from_str(&message)?;

        // tracing::info!(?message, "Received gamepad message");
        if let Some(gamepad_message) = message.get_first() {
            let x = apply_deadzone(
                gamepad_message
                    .axis_state
                    .get(&messages::Axis::LeftStickY)
                    .cloned()
                    .unwrap_or_default(),
            );
            let y = apply_deadzone(
                gamepad_message
                    .axis_state
                    .get(&messages::Axis::LeftStickX)
                    .cloned()
                    .unwrap_or_default(),
            );
            let yaw = apply_deadzone(
                gamepad_message
                    .axis_state
                    .get(&messages::Axis::RightStickX)
                    .cloned()
                    .unwrap_or_default(),
            );

            if gamepad_message
                .button_down
                .get(&messages::Button::DPadDown)
                .cloned()
                .unwrap_or_default()
            {
                zenoh_session
                    .put("rplidar/state", "off")
                    .res_async()
                    .await
                    .map_err(ErrorWrapper::ZenohError)?;
            }

            if gamepad_message
                .button_down
                .get(&messages::Button::DPadUp)
                .cloned()
                .unwrap_or_default()
            {
                zenoh_session
                    .put("rplidar/state", "on")
                    .res_async()
                    .await
                    .map_err(ErrorWrapper::ZenohError)?;
            }

            let command = HolonomicWheelCommand::from_move(x, -y, -yaw);
            driver.send(command).await?;
        }
    }
}

fn apply_deadzone(value: f32) -> f32 {
    const DEADZONE: f32 = 0.07;
    if value.abs() < DEADZONE {
        0.0
    } else {
        value
    }
}
