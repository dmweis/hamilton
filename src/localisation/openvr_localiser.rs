use nalgebra as na;
use serde::Deserialize;
use std::str;

#[derive(Debug, Deserialize)]
pub struct TrackedObjects {
    pub ts: u128,
    pub trackers: Vec<VrDevice>,
}

impl TrackedObjects {
    fn get_tracker_pose_in_openvr(&self) -> Option<(na::Point3<f32>, na::UnitQuaternion<f32>)> {
        for object in &self.trackers {
            if let VrDeviceClass::Tracker = object.class {
                if object.seen && object.tracked {
                    return Some((object.position, object.rotation));
                }
            }
        }
        None
    }

    pub fn get_tracker_pose(&self) -> Option<(na::Point2<f32>, na::Rotation2<f32>)> {
        self.get_tracker_pose_in_openvr().map(tracker_pose_to_plane)
    }

    fn get_any_controller_pose_in_openvr(
        &self,
    ) -> Option<(na::Point3<f32>, na::UnitQuaternion<f32>)> {
        for object in &self.trackers {
            match object.class {
                VrDeviceClass::LeftController
                | VrDeviceClass::RightController
                | VrDeviceClass::Controller => {
                    if object.seen && object.tracked {
                        return Some((object.position, object.rotation));
                    }
                }
                _ => (),
            }
        }
        None
    }

    pub fn get_any_controller_pose(&self) -> Option<na::Point2<f32>> {
        self.get_any_controller_pose_in_openvr()
            .map(|(position, _)| position.zx())
    }

    pub fn get_any_controller_trigger(&self) -> Option<f32> {
        for object in &self.trackers {
            match object.class {
                VrDeviceClass::LeftController
                | VrDeviceClass::RightController
                | VrDeviceClass::Controller => {
                    if object.seen && object.tracked {
                        if let Some(inputs) = &object.inputs {
                            if let Some(trigger) = &inputs.trigger {
                                return Some(*trigger);
                            }
                        }
                    }
                }
                _ => (),
            }
        }
        None
    }
}

pub fn tracker_pose_to_plane(
    pose: (na::Point3<f32>, na::UnitQuaternion<f32>),
) -> (na::Point2<f32>, na::Rotation2<f32>) {
    let (position_openvr_space, rotation) = pose;
    let projection = rotation * na::Vector3::y_axis();
    let yaw = -projection.x.atan2(-projection.z);
    let position = position_openvr_space.zx();
    let position_centered = position + na::Rotation2::new(yaw) * na::Vector2::new(0.14, 0.0);
    (position_centered, na::Rotation2::new(yaw))
}

#[derive(Debug, Eq, PartialEq, Copy, Clone, Deserialize)]
pub enum VrDeviceClass {
    Controller,
    LeftController,
    RightController,
    Tracker,
    HMD,
    Sensor,
    Other,
}

#[derive(Debug, Clone, Deserialize)]
pub struct VrDevice {
    pub id: usize,
    pub tracked: bool,
    pub seen: bool,
    pub position: na::Point3<f32>,
    pub rotation: na::UnitQuaternion<f32>,
    pub class: VrDeviceClass,
    pub inputs: Option<InputsState>,
}

#[derive(Debug, Clone, Deserialize)]
pub struct InputsState {
    pub trigger: Option<f32>,
}
