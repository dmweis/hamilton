use crate::holonomic_controller::HolonomicWheelCommand;
use nalgebra as na;

#[derive(Debug)]
pub struct Pose {
    position: na::Point2<f32>,
    rotation: na::Rotation2<f32>,
}

impl Pose {
    pub fn from_na(position: na::Point2<f32>, rotation: na::Rotation2<f32>) -> Self {
        Self { position, rotation }
    }

    pub fn new(position: (f32, f32), rotation: f32) -> Self {
        let (x, y) = position;
        Self {
            position: na::Point2::new(x, y),
            rotation: na::Rotation2::new(rotation),
        }
    }
}

#[derive(Debug, Default)]
pub struct NavigationController {
    current_pose: Option<Pose>,
    target_pose: Option<Pose>,
}

impl NavigationController {
    pub fn localized(&self) -> bool {
        self.current_pose.is_some()
    }

    pub fn update_current_pose(&mut self, pose: Pose) {
        self.current_pose = Some(pose);
    }

    pub fn clear_current_pose(&mut self) {
        self.current_pose = None;
    }

    pub fn update_target_pose(&mut self, pose: Pose) {
        self.target_pose = Some(pose);
    }

    pub fn clear_target(&mut self) {
        self.target_pose = None;
    }

    pub fn calculate_drive(&self) -> Option<HolonomicWheelCommand> {
        // maybe this should be done on `update_current_pose`
        match (&self.current_pose, &self.target_pose) {
            (Some(current), Some(target)) => Some(calculate_drive_gains(current, target)),
            _ => None,
        }
    }
}

const TRANSLATION_GAIN: f32 = 10.0;
const CLAMP: f32 = 0.5;
const DEAD_BAND: f32 = 0.1;

fn calculate_drive_gains(current: &Pose, target: &Pose) -> HolonomicWheelCommand {
    let translation = current.position - target.position;
    let gain_vector = current.rotation.inverse() * translation;

    let mut forward_gain = -(gain_vector.x * TRANSLATION_GAIN).clamp(-CLAMP, CLAMP);
    if forward_gain.abs() < DEAD_BAND {
        forward_gain = 0.0;
    }
    let mut strafe_gain = -(gain_vector.y * TRANSLATION_GAIN).clamp(-CLAMP, CLAMP);
    if strafe_gain.abs() < DEAD_BAND {
        strafe_gain = 0.0;
    }
    let mut yaw_gain = current
        .rotation
        .angle_to(&target.rotation)
        .clamp(-CLAMP, CLAMP);
    if yaw_gain.abs() < DEAD_BAND {
        yaw_gain = 0.0;
    }
    HolonomicWheelCommand::from_move(forward_gain, strafe_gain, yaw_gain)
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn test_creation_with_into() {
        let _pose = Pose::new((10.0, 10.0), 10.0);
    }

    /// these tests are kind of testing nalgebra.
    /// I just used them to check that this behaves the way I expect
    #[test]
    fn rotation_angle_to() {
        let pose = na::Rotation2::new(0_f32.to_radians());
        let target = na::Rotation2::new(90_f32.to_radians());
        let angle_to = pose.angle_to(&target);
        assert_relative_eq!(angle_to, 90_f32.to_radians());
    }

    #[test]
    fn rotation_angle_to_inverted() {
        let pose = na::Rotation2::new(0_f32.to_radians());
        let target = na::Rotation2::new(-90_f32.to_radians());
        let angle_to = pose.angle_to(&target);
        assert_relative_eq!(angle_to, -90_f32.to_radians());
    }

    #[test]
    fn rotation_angle_to_wrap() {
        let pose = na::Rotation2::new(-170_f32.to_radians());
        let target = na::Rotation2::new(170_f32.to_radians());
        let angle_to = pose.angle_to(&target);
        assert_relative_eq!(angle_to, -20_f32.to_radians(), max_relative = 0.00001);
    }
}
