use anyhow::Result;
use remote_controller::{Action, ActionList, AreaSize};
use std::time::Duration;
use tokio::time::sleep;
use tracing_subscriber::filter::LevelFilter;

#[tokio::main]
async fn main() -> Result<()> {
    tracing_subscriber::fmt()
        .pretty()
        .with_max_level(LevelFilter::INFO)
        .init();

    let action_list = ActionList::new(vec![
        Action::new(String::from("save"), String::from("Save current position")),
        Action::new(String::from("load"), String::from("Load current position")),
    ]);

    let controller_handle = remote_controller::start_remote_controller_server_with_map(
        ([0, 0, 0, 0], 8080),
        AreaSize::new(1.0, 2.0),
        action_list,
    );

    loop {
        sleep(Duration::from_millis(20)).await;
        if let Some(gamepad) = controller_handle.get_last_input().await {
            println!("gamepad: {:?}", gamepad);
        }
        if let Some(touch) = controller_handle.try_receive_canvas_touch().await? {
            println!("touch: {:?}", touch);
        }
        if let Some(action) = controller_handle.try_receive_action().await? {
            println!("Action received: {:?}", action);
        }
    }
}
