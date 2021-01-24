use crate::hamilton::MoveCommand;

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

impl From<MoveCommand> for HolonomicWheelCommand {
    fn from(move_command: MoveCommand) -> Self {
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
