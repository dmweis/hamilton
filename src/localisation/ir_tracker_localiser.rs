use crate::navigation::Pose2d;
use na::distance;
use nalgebra as na;
use serde::{Deserialize, Serialize};
use std::str;
use tracing::*;

#[derive(Default, Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct IrTrackers {
    frame_time: f32,
    point_count: usize,
    height: f32,
    width: f32,
    channels: i32,
    #[serde(rename = "useing_otsu_thresholding")]
    using_otsu_thresholding: bool,
    #[serde(rename = "binarization_threshold")]
    binarization_threshold: i32,
    points: Vec<na::Point2<f32>>,
}

fn linear_map(value: f32, in_min: f32, in_max: f32, out_min: f32, out_max: f32) -> f32 {
    (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
}

const TRIANGLE_REJECTION_DISTANCE: f32 = 0.02;

impl IrTrackers {
    fn points_in_screen_space(&self) -> Vec<na::Point2<f32>> {
        let mut new_points = Vec::with_capacity(self.point_count);
        for point in &self.points {
            if self.width > self.height {
                let longer_side = self.width / self.height;
                let new_x = linear_map(point.x, 0., self.width, 0., longer_side);
                let new_y = linear_map(point.y, 0., self.height, 0., 1.);
                new_points.push(na::Point2::new(new_x, new_y));
            } else {
                let longer_side = self.height / self.width;
                let new_x = linear_map(point.x, 0., self.width, 0., 1.);
                let new_y = linear_map(point.y, 0., self.height, 0., longer_side);
                new_points.push(na::Point2::new(new_x, new_y));
            }
        }
        new_points
    }

    pub fn find_tracker_pose(&self) -> Option<Pose2d> {
        if self.point_count < 4 {
            warn!("Less than 4 points visible");
            return None;
        }
        let local_points = self.points_in_screen_space();
        for current_point in &local_points {
            let mut points_copy = local_points.clone();
            if points_copy.len() < 4 {
                error!("Somehow ended up with less than 3 points");
                return None;
            }
            points_copy.sort_by(|a, b| {
                distance(current_point, a)
                    .partial_cmp(&distance(current_point, b))
                    .unwrap()
            });

            let likely_current = points_copy.first().unwrap();
            if current_point != likely_current {
                error!("Closest point is not current point");
                return None;
            }
            let first = points_copy.get(1).unwrap();
            let second = points_copy.get(2).unwrap();
            let direction_point = points_copy.get(3).unwrap();
            let direction_point_distance = distance(current_point, direction_point);
            let distance_first = distance(current_point, first);
            let distance_second = distance(current_point, second);
            if distance_first > TRIANGLE_REJECTION_DISTANCE
                || distance_second > TRIANGLE_REJECTION_DISTANCE
            {
                // reject triangle point
                continue;
            }
            if direction_point_distance < distance_first
                || direction_point_distance < distance_second
            {
                warn!("Rejected point because direction point distance was smaller");
                continue;
            }
            // we are in triangle now

            let triangle_center_x = (current_point.x + first.x + second.x) / 3.;
            let triangle_center_y = (current_point.y + first.y + second.y) / 3.;
            let triangle_center = na::Point2::new(triangle_center_x, triangle_center_y);

            let translation = direction_point.coords - triangle_center.coords;
            let normalized_translation = translation.normalize();
            let rotation =
                na::Rotation2::new(normalized_translation.x.atan2(normalized_translation.y));

            let triangle_to_robot_translation = na::Vector2::new(0.058, 0.0);
            // use hardcoded value because this causes jitter
            // Or maybe that's an over eager controller?
            // let triangle_to_robot_translation =
            //     na::Vector2::new(translation.magnitude() * 1.5, 0.0);

            let robot_pose = triangle_center + rotation * triangle_to_robot_translation;
            // TODO (David): This is now incorrect because these are not 0.0 <-> 1.0
            // but it works in a limited space
            let pose = Pose2d::from_na(
                na::Point2::new(1.0 - robot_pose.y, 1.0 - robot_pose.x),
                rotation,
            );
            return Some(pose);
        }
        None
    }

    pub fn pretty_points(&self) -> String {
        let mut buffer = String::new();
        for point in &self.points_in_screen_space() {
            buffer.push_str(&format!("[{}, {}] ", point.x, point.y));
        }
        buffer
    }
}

#[cfg(test)]
mod test {
    use super::*;
    use approx::*;

    fn create_ir_points(points: Vec<na::Point2<f32>>, height: f32, width: f32) -> IrTrackers {
        IrTrackers {
            frame_time: 0.,
            point_count: points.len(),
            height,
            width,
            channels: 0,
            using_otsu_thresholding: false,
            binarization_threshold: 0,
            points,
        }
    }

    #[test]
    fn ir_tracker_orientation() {
        // width is actually X here
        // images are x right, y down
        let points = vec![
            na::Point2::new(500.0, 496.0),
            na::Point2::new(500.0, 504.0),
            na::Point2::new(506.928, 500.0),
            na::Point2::new(502.309, 520.0),
        ];
        let points = create_ir_points(points, 1000.0, 1000.0);
        let pose = points.find_tracker_pose();
        assert!(pose.is_some());
        let pose = pose.unwrap();
        // These aren't particularly string asserts
        // but they are good enough to get the orientation and pose correctly
        assert_abs_diff_eq!(pose.rotation().angle().to_degrees(), 0., epsilon = 0.01);
        assert_abs_diff_eq!(
            pose.position(),
            &na::Point2::new(0.5, 0.439),
            epsilon = 0.001
        );
    }
}
