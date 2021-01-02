pub mod lss_driver;
pub mod stepper_driver;

use anyhow::Result;
use async_trait::async_trait;

#[derive(Default, Debug)]
pub struct WireMoveCommand {
    pub wheel_a: f32,
    pub wheel_b: f32,
    pub wheel_c: f32,
    pub wheel_d: f32,
}

impl WireMoveCommand {
    fn encode(&self) -> Vec<u8> {
        let mut buffer = vec![];
        // push wheels in the weird format that I chose for some reason
        buffer.push((self.wheel_a > 0.0) as u8);
        buffer.push(self.wheel_a.abs() as u8);
        buffer.push((self.wheel_b > 0.0) as u8);
        buffer.push(self.wheel_b.abs() as u8);
        buffer.push((self.wheel_c > 0.0) as u8);
        buffer.push(self.wheel_c.abs() as u8);
        buffer.push((self.wheel_d > 0.0) as u8);
        buffer.push(self.wheel_d.abs() as u8);

        let mut encoded = postcard_cobs::encode_vec(&buffer);
        encoded.push(0);
        encoded
    }
}

#[async_trait]
pub trait HamiltonDriver: Send + Sync {
    async fn send(&mut self, command: WireMoveCommand) -> Result<()>;
}
