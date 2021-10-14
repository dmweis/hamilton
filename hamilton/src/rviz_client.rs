use crate::navigation::Pose2d;
use anyhow::Result;
use nalgebra as na;
use pose_publisher::{
    pose::{Color, Shape},
    PoseClientUpdate, PosePublisher,
};
use std::net::SocketAddrV4;

const DEFAULT_ROBOT_NAME: &str = "Robot";
const DEFAULT_TARGET_NAME: &str = "Robot";

pub struct RvizClient {
    pose_publisher: PosePublisher,
    last_robot_pose: Option<Pose2d>,
    last_target_pose: Option<Pose2d>,
    robot_name: String,
    target_name: String,
}

impl RvizClient {
    pub fn new(broadcast_address: SocketAddrV4) -> Result<Self> {
        Ok(Self {
            pose_publisher: PosePublisher::new(broadcast_address)?,
            last_robot_pose: None,
            last_target_pose: None,
            robot_name: DEFAULT_ROBOT_NAME.to_owned(),
            target_name: DEFAULT_TARGET_NAME.to_owned(),
        })
    }

    pub fn set_robot_name(&mut self, name: &str) {
        self.robot_name = name.to_owned();
    }

    pub fn set_target_name(&mut self, name: &str) {
        self.target_name = name.to_owned();
    }

    pub fn set_robot_pose(&mut self, robot_pose: Pose2d) {
        self.last_robot_pose = Some(robot_pose);
    }

    pub fn set_target_pose(&mut self, target_pose: Pose2d) {
        self.last_target_pose = Some(target_pose);
    }

    pub fn publish(&mut self) -> Result<()> {
        let mut update = PoseClientUpdate::new();
        if let Some(robot_pose) = self.last_robot_pose.take() {
            let robot_target_vector = robot_pose.rotation() * na::Vector2::new(0.1, 0.);
            // TODO (David): Why am I doing this? I am pretty sure this could be made a lot simpler
            let robot_rotation =
                na::UnitQuaternion::from_euler_angles(0., 0., robot_pose.rotation().angle()).coords;
            update
                .add(
                    &self.robot_name,
                    (robot_pose.position().x, robot_pose.position().y, 0.1),
                )
                .with_rotation((
                    robot_rotation[0],
                    robot_rotation[1],
                    robot_rotation[2],
                    robot_rotation[3],
                ));
            update
                .add(
                    &format!("{} direction", &self.robot_name),
                    (
                        robot_pose.position().x + robot_target_vector.x,
                        robot_pose.position().y + robot_target_vector.y,
                        0.1,
                    ),
                )
                .with_shape(Shape::Sphere(0.02))
                .with_color(Color::Green);
        }
        if let Some(target_pose) = self.last_target_pose.take() {
            // Same as above TODO
            update.add(
                &self.target_name,
                (target_pose.position().x, target_pose.position().y, 0.1),
            );
            let target_vector = target_pose.rotation() * na::Vector2::new(0.1, 0.);
            update
                .add(
                    &format!("{} direction", &self.target_name),
                    (
                        target_pose.position().x + target_vector.x,
                        target_pose.position().y + target_vector.y,
                        0.1,
                    ),
                )
                .with_shape(Shape::Sphere(0.02))
                .with_color(Color::Green);
        }
        self.pose_publisher.publish(update)?;
        Ok(())
    }
}
