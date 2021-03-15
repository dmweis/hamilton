#[derive(Debug)]
pub struct HolonomicWheelCommand {
    left_front: f32,
    right_front: f32,
    left_rear: f32,
    right_rear: f32,
}

impl HolonomicWheelCommand {
    pub fn new(
        left_front: f32,
        right_front: f32,
        left_rear: f32,
        right_rear: f32,
    ) -> HolonomicWheelCommand {
        HolonomicWheelCommand {
            left_front,
            right_front,
            left_rear,
            right_rear,
        }
    }

    pub fn stopped() -> Self {
        Self {
            left_front: 0.0,
            right_front: 0.0,
            left_rear: 0.0,
            right_rear: 0.0,
        }
    }

    pub fn from_move(forward: f32, strafe: f32, yaw: f32) -> HolonomicWheelCommand {
        HolonomicWheelCommand::new(
            forward - yaw - strafe,
            forward + yaw + strafe,
            forward - yaw + strafe,
            forward + yaw - strafe,
        )
    }

    pub fn left_front(&self) -> f32 {
        self.left_front
    }
    pub fn right_front(&self) -> f32 {
        self.right_front
    }
    pub fn left_rear(&self) -> f32 {
        self.left_rear
    }
    pub fn right_rear(&self) -> f32 {
        self.right_rear
    }
}
