pub mod hamilton_lss_driver;

pub use hamilton_lss_driver::{BodyConfig, HamiltonLssDriver};

#[derive(Debug)]
pub struct MotorCommand {
    id: u8,
    speed: f32,
}

impl MotorCommand {
    pub fn new(id: u8, speed: f32) -> Self {
        Self { id, speed }
    }

    pub fn id(&self) -> u8 {
        self.id
    }

    pub fn speed(&self) -> f32 {
        self.speed
    }
}
