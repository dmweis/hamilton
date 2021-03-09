use anyhow::Result;
use nalgebra as na;
use remote_controller::CanvasTouch;
use serde::{Deserialize, Serialize};
use std::fs::File;
use std::include_str;

#[derive(Debug, Serialize, Deserialize)]
pub struct Map {
    front_left: na::Vector2<f32>,
    rear_right: na::Vector2<f32>,
}

impl Map {
    pub fn get_size(&self) -> (f32, f32) {
        (
            (self.front_left.x - self.rear_right.x).abs(),
            (self.front_left.y - self.rear_right.y).abs(),
        )
    }

    pub fn canvas_touch_to_pose(&self, touch_event: CanvasTouch) -> (na::Point2<f32>, f32) {
        let y = linear_map(
            touch_event.down_x,
            0.0,
            touch_event.width,
            self.front_left.y,
            self.rear_right.y,
        );
        let x = linear_map(
            touch_event.down_y,
            0.0,
            touch_event.height,
            self.front_left.x,
            self.rear_right.x,
        );
        let relative_x = touch_event.down_x - touch_event.up_x;
        let relative_y = touch_event.down_y - touch_event.up_y;
        let heading = relative_x.atan2(relative_y);
        (na::Point2::new(x, y), heading)
    }

    pub fn save_json(&self, path: &str) -> Result<()> {
        let file = File::create(path)?;
        serde_json::to_writer_pretty(file, self)?;
        Ok(())
    }

    pub fn load_json(path: &str) -> Result<Self> {
        let file = File::create(path)?;
        Ok(serde_json::from_reader(file)?)
    }

    pub fn included_room() -> Self {
        serde_json::from_str(include_str!("../config/map.json")).unwrap()
    }
}

fn linear_map(value: f32, in_min: f32, in_max: f32, out_min: f32, out_max: f32) -> f32 {
    (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_map_included() {
        Map::included_room();
    }
}
