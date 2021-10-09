use anyhow::Result;
use serde::{Deserialize, Serialize};
use std::{
    sync::Arc,
    time::{Duration, Instant},
};
use tokio::sync::{mpsc, Mutex};

static GAMEPAD_TIMEOUT: Duration = Duration::from_millis(500);

pub struct ControllerState {
    last_gamepad_input: Arc<Mutex<(GamepadCommand, Instant)>>,
    canvas_touch_receiver: Mutex<mpsc::Receiver<CanvasTouch>>,
    canvas_touch_sender: mpsc::Sender<CanvasTouch>,
    action_receiver: Mutex<mpsc::Receiver<ActionIdWrapper>>,
    action_sender: mpsc::Sender<ActionIdWrapper>,
}

impl Default for ControllerState {
    fn default() -> Self {
        let (canvas_touch_sender, canvas_touch_receiver) = mpsc::channel(10);
        let (action_sender, action_receiver) = mpsc::channel(10);
        Self {
            last_gamepad_input: Arc::new(Mutex::new((
                GamepadCommand::default(),
                Instant::now() - GAMEPAD_TIMEOUT,
            ))),
            canvas_touch_receiver: Mutex::new(canvas_touch_receiver),
            canvas_touch_sender,
            action_receiver: Mutex::new(action_receiver),
            action_sender,
        }
    }
}

impl ControllerState {
    pub(crate) fn send_action(&self, action: ActionIdWrapper) -> Result<()> {
        match self.action_sender.try_send(action) {
            Err(mpsc::error::TrySendError::Closed(_)) => Err(anyhow::anyhow!("Channel closed")),
            _ => Ok(()),
        }
    }

    pub async fn try_receive_action(&self) -> Result<Option<ActionIdWrapper>> {
        match self.action_receiver.lock().await.try_recv() {
            Ok(action) => Ok(Some(action)),
            Err(mpsc::error::TryRecvError::Empty) => Ok(None),
            Err(mpsc::error::TryRecvError::Disconnected) => Err(anyhow::anyhow!("Channel closed")),
        }
    }

    pub(crate) fn send_canvas_touch(&self, touch: CanvasTouch) -> Result<()> {
        match self.canvas_touch_sender.try_send(touch) {
            Err(mpsc::error::TrySendError::Closed(_)) => Err(anyhow::anyhow!("Channel closed")),
            _ => Ok(()),
        }
    }

    pub async fn try_receive_canvas_touch(&self) -> Result<Option<CanvasTouch>> {
        match self.canvas_touch_receiver.lock().await.try_recv() {
            Ok(action) => Ok(Some(action)),
            Err(mpsc::error::TryRecvError::Empty) => Ok(None),
            Err(mpsc::error::TryRecvError::Disconnected) => Err(anyhow::anyhow!("Channel closed")),
        }
    }

    pub(crate) async fn set_gamepad(&self, input: GamepadCommand) {
        *self.last_gamepad_input.lock().await = (input, Instant::now());
    }

    pub async fn get_last_input(&self) -> Option<GamepadCommand> {
        let (command, time) = self.last_gamepad_input.lock().await.clone();
        if time.elapsed() > GAMEPAD_TIMEOUT {
            None
        } else {
            Some(command)
        }
    }
}

#[derive(Debug, Serialize, Clone)]
pub struct Action {
    id: String,
    description: String,
}

impl Action {
    pub fn new(id: String, description: String) -> Self {
        Self { id, description }
    }
}

#[derive(Debug, Serialize, Default, Clone)]
pub struct ActionList {
    actions: Vec<Action>,
}

impl ActionList {
    pub fn new(actions: Vec<Action>) -> Self {
        Self { actions }
    }
}

#[derive(Debug, Deserialize)]
pub struct ActionIdWrapper {
    id: String,
}

#[derive(Debug, Deserialize, Default, Clone)]
pub struct CanvasTouch {
    pub width: f32,
    pub height: f32,
    pub down_x: f32,
    pub down_y: f32,
    pub up_x: f32,
    pub up_y: f32,
}

#[derive(Debug, Deserialize, Default, Clone)]
pub struct GamepadCommand {
    #[serde(rename = "lx")]
    pub left_x: f32,
    #[serde(rename = "ly")]
    pub left_y: f32,
    #[serde(rename = "rx")]
    pub right_x: f32,
    #[serde(rename = "ry")]
    pub right_y: f32,
}
