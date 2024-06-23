use chrono::prelude::{DateTime, Utc};
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;
use std::hash::Hash;

#[derive(Debug, Deserialize, Serialize)]
pub struct InputMessage {
    pub gamepads: BTreeMap<usize, GamepadMessage>,
    pub time: DateTime<Utc>,
}

impl InputMessage {
    pub fn get_first(&self) -> Option<GamepadMessage> {
        self.gamepads
            .first_key_value()
            .map(|(_id, gamepad)| gamepad.clone())
    }
}

#[derive(Debug, Deserialize, Serialize, Default, Clone)]
pub struct GamepadMessage {
    pub name: String,
    pub connected: bool,
    pub last_event_time: DateTime<Utc>,
    pub button_down_event_counter: BTreeMap<Button, usize>,
    pub button_up_event_counter: BTreeMap<Button, usize>,
    pub button_down: BTreeMap<Button, bool>,
    pub axis_state: BTreeMap<Axis, f32>,
}

#[derive(Debug, Deserialize, Serialize, PartialEq, Eq, Hash, PartialOrd, Ord, Clone, Copy)]
pub enum Button {
    South,
    East,
    North,
    West,
    C,
    Z,
    LeftTrigger,
    LeftTrigger2,
    RightTrigger,
    RightTrigger2,
    Select,
    Start,
    Mode,
    LeftThumb,
    RightThumb,
    DPadUp,
    DPadDown,
    DPadLeft,
    DPadRight,
    Unknown,
}

#[derive(Debug, Deserialize, Serialize, PartialEq, Eq, Hash, PartialOrd, Ord, Clone, Copy)]
pub enum Axis {
    LeftStickX,
    LeftStickY,
    LeftZ,
    RightStickX,
    RightStickY,
    RightZ,
    DPadX,
    DPadY,
    Unknown,
}

impl Axis {
    #[allow(unused)]
    pub fn all_axes() -> &'static [Axis] {
        &[
            Axis::LeftStickX,
            Axis::LeftStickY,
            Axis::LeftZ,
            Axis::RightStickX,
            Axis::RightStickY,
            Axis::RightZ,
            Axis::DPadX,
            Axis::DPadY,
        ]
    }
}
