use crate::driver::HamiltonDriver;
use crate::holonomic_controller::{HolonomicWheelCommand, MoveCommand};
use crate::lidar::Lidar;
use crate::localisation::LocalisationManager;
use crate::rviz_client::RvizClient;
use crate::simple_collision_detector::SimpleCollisionDetector;
use anyhow::Result;
use nalgebra as na;
use std::fmt;
use std::time::{Duration, Instant};
use tracing::warn;

#[derive(Debug, Clone)]
pub struct Pose2d {
    position: na::Point2<f32>,
    rotation: na::Rotation2<f32>,
}

impl Pose2d {
    pub fn from_na(position: na::Point2<f32>, rotation: na::Rotation2<f32>) -> Self {
        Self { position, rotation }
    }

    pub fn new((x, y): (f32, f32), rotation: f32) -> Self {
        Self {
            position: na::Point2::new(x, y),
            rotation: na::Rotation2::new(rotation),
        }
    }

    pub fn position(&self) -> &na::Point2<f32> {
        &self.position
    }

    pub fn rotation(&self) -> &na::Rotation2<f32> {
        &self.rotation
    }
}

impl fmt::Display for Pose2d {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(
            f,
            "[{}, {}] -> {}",
            self.position.x,
            self.position.y,
            self.rotation.angle().to_degrees()
        )
    }
}

static USER_COMMAND_TIMEOUT: Duration = Duration::from_secs(1);

pub struct NavigationController {
    driver: Box<dyn HamiltonDriver>,
    localiser: LocalisationManager,
    target: Option<Pose2d>,
    last_user_command: MoveCommand,
    last_user_command_time: Instant,
    rviz_client: RvizClient,
    collision_detector: SimpleCollisionDetector,
}

impl NavigationController {
    pub fn new(
        driver: Box<dyn HamiltonDriver>,
        localiser: LocalisationManager,
        rviz_client: RvizClient,
        lidar: Lidar,
    ) -> Self {
        Self {
            driver,
            localiser,
            target: None,
            last_user_command: MoveCommand::new(0., 0., 0.),
            last_user_command_time: Instant::now(),
            rviz_client,
            collision_detector: SimpleCollisionDetector::new(lidar),
        }
    }

    pub fn set_target(&mut self, target: Pose2d) {
        self.target = Some(target);
    }

    pub fn clear_target(&mut self) {
        self.target = None;
    }

    pub fn issue_user_command(&mut self, command: MoveCommand) {
        self.last_user_command = command;
        self.last_user_command_time = Instant::now();
    }

    pub fn set_user_command(&mut self, command: MoveCommand, time: Instant) {
        self.last_user_command = command;
        self.last_user_command_time = time;
    }

    pub async fn tick(&mut self) -> Result<()> {
        if self.last_user_command_time.elapsed() < USER_COMMAND_TIMEOUT {
            self.clear_target();

            if self
                .collision_detector
                .check_move_safe(&self.last_user_command)
            {
                self.driver
                    .send(HolonomicWheelCommand::from_move_command(
                        &self.last_user_command,
                    ))
                    .await?;
            } else {
                self.driver
                    .send(HolonomicWheelCommand::from_move_command(
                        &self.last_user_command.with_rotation_only(),
                    ))
                    .await?;
            }

            // try to publish robot pose even when not using localisation
            if let Some(pose) = self.localiser.get_latest_pose().await? {
                self.rviz_client.set_robot_pose(pose);
                self.rviz_client.publish()?;
            }
            return Ok(());
        }
        if let Some(pose) = self.localiser.get_latest_pose().await? {
            self.rviz_client.set_robot_pose(pose.clone());
            if let Some(target) = &self.target {
                self.rviz_client.set_target_pose(target.clone());
                let command = calculate_drive_gains(&pose, target);
                if self.collision_detector.check_move_safe(&command) {
                    self.driver
                        .send(HolonomicWheelCommand::from_move_command(&command))
                        .await?;
                } else {
                    self.driver.send(HolonomicWheelCommand::stopped()).await?;
                }
            }
            self.rviz_client.publish()?;
            return Ok(());
        } else {
            warn!("Not localised");
        }
        self.driver.send(HolonomicWheelCommand::stopped()).await?;
        Ok(())
    }

    pub fn start_lidar(&mut self) {
        self.collision_detector.start_lidar();
    }

    pub fn stop_lidar(&mut self) {
        self.collision_detector.stop_lidar()
    }
}

const TRANSLATION_GAIN: f32 = 10.0;
const CLAMP: f32 = 0.5;
const DEAD_BAND: f32 = 0.15;

fn calculate_drive_gains(current: &Pose2d, target: &Pose2d) -> MoveCommand {
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
    MoveCommand::new(forward_gain, strafe_gain, yaw_gain)
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn test_creation_with_into() {
        let _pose = Pose2d::new((10.0, 10.0), 10.0);
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
