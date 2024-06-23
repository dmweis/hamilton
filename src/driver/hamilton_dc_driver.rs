use crate::driver::MotorConfig;
use crate::holonomic_controller::HolonomicWheelCommand;

use super::BodyConfig;
use super::Clampable;
use super::HamiltonDriver;
use anyhow::Error;
use anyhow::Result;
use async_trait::async_trait;
use bytes::{BufMut, BytesMut};
use futures::SinkExt;
use lss_driver::LedColor;
use tokio_serial::SerialPortBuilderExt;
use tokio_util::codec::{Decoder, Encoder};

#[derive(thiserror::Error, Debug)]
#[non_exhaustive]
pub enum HamiltonError {
    #[error("communication with motor driver failed")]
    CommError,
    #[error("failed opening serial port")]
    FailedOpeningSerialPort,
}

#[derive(Default, Debug)]
pub struct WireMoveCommand {
    pub wheel_a: f32,
    pub wheel_b: f32,
    pub wheel_c: f32,
    pub wheel_d: f32,
}

impl WireMoveCommand {
    fn new(wheel_a: f32, wheel_b: f32, wheel_c: f32, wheel_d: f32) -> Self {
        Self {
            wheel_a,
            wheel_b,
            wheel_c,
            wheel_d,
        }
    }

    fn encode(&self) -> Vec<u8> {
        // push wheels in the weird format that I chose for some reason
        let buffer = vec![
            (self.wheel_a > 0.0) as u8,
            self.wheel_a.abs() as u8,
            (self.wheel_b > 0.0) as u8,
            self.wheel_b.abs() as u8,
            (self.wheel_c > 0.0) as u8,
            self.wheel_c.abs() as u8,
            (self.wheel_d > 0.0) as u8,
            self.wheel_d.abs() as u8,
        ];

        let mut encoded = postcard_cobs::encode_vec(&buffer);
        encoded.push(0);
        encoded
    }
}

trait ConfigMappable {
    fn apply_commands_by_mapping(&self, command: &HolonomicWheelCommand) -> WireMoveCommand;
}

impl ConfigMappable for BodyConfig {
    fn apply_commands_by_mapping(&self, command: &HolonomicWheelCommand) -> WireMoveCommand {
        fn create_motor_data(mapping: &MotorConfig, value: f32, multiplier: f32) -> f32 {
            let inversion_mul = if mapping.inverted { -1.0 } else { 1.0 };
            (value * multiplier).clamp_num(-multiplier, multiplier) * inversion_mul
        }

        let mut data = [0.0; 4];

        data[self.left_front_controller.id as usize] = create_motor_data(
            &self.left_front_controller,
            command.left_front(),
            self.multiplier,
        );
        data[self.right_front_controller.id as usize] = create_motor_data(
            &self.right_front_controller,
            command.right_front(),
            self.multiplier,
        );
        data[self.left_rear_controller.id as usize] = create_motor_data(
            &self.left_rear_controller,
            command.left_rear(),
            self.multiplier,
        );
        data[self.right_rear_controller.id as usize] = create_motor_data(
            &self.right_rear_controller,
            command.right_rear(),
            self.multiplier,
        );
        WireMoveCommand::new(data[0], data[1], data[2], data[3])
    }
}

pub struct HamiltonProtocol;

impl Decoder for HamiltonProtocol {
    type Item = ();
    type Error = Error;

    fn decode(&mut self, _: &mut BytesMut) -> Result<Option<Self::Item>, Self::Error> {
        Ok(None)
    }
}

impl Encoder<WireMoveCommand> for HamiltonProtocol {
    type Error = Error;

    fn encode(&mut self, data: WireMoveCommand, buf: &mut BytesMut) -> Result<(), Error> {
        let encoded_data = data.encode();
        buf.reserve(encoded_data.len());
        buf.put_slice(&encoded_data);
        Ok(())
    }
}

pub struct HamiltonDcDriver {
    framed_port: tokio_util::codec::Framed<tokio_serial::SerialStream, HamiltonProtocol>,
    config: BodyConfig,
}

const BAUD_RATE: u32 = 115200;

impl HamiltonDcDriver {
    pub fn new(config: BodyConfig) -> Result<Self> {
        let serial_port = tokio_serial::new(&config.port, BAUD_RATE)
            .open_native_async()
            .map_err(|_| HamiltonError::FailedOpeningSerialPort)?;
        Ok(Self {
            framed_port: HamiltonProtocol.framed(serial_port),
            config,
        })
    }
}

#[async_trait]
impl HamiltonDriver for HamiltonDcDriver {
    async fn send(&mut self, command: HolonomicWheelCommand) -> Result<()> {
        let wire_command = self.config.apply_commands_by_mapping(&command);
        self.framed_port
            .send(wire_command)
            .await
            .map_err(|_| HamiltonError::CommError)?;
        Ok(())
    }

    async fn read_voltage(&mut self) -> Result<Option<f32>> {
        Ok(None)
    }

    async fn set_color(&mut self, _color: LedColor) -> Result<Option<()>> {
        Ok(None)
    }

    fn set_halt_mode(&mut self, _on: bool) {}

    fn halt_mode(&self) -> bool {
        false
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn encoding_adds_trailing_zero() {
        let move_command = WireMoveCommand::default();
        let encoded = move_command.encode();
        assert_eq!(*encoded.last().unwrap(), 0_u8);
    }

    #[test]
    fn left_front_positive() {
        let move_command = WireMoveCommand {
            wheel_a: 255.0,
            ..Default::default()
        };
        let encoded = move_command.encode();
        let mut iter = encoded.iter();
        assert_eq!(*iter.next().unwrap(), 3_u8);
        assert_eq!(*iter.next().unwrap(), 1_u8);
        assert_eq!(*iter.next().unwrap(), 255_u8);
    }

    #[test]
    fn left_front_negative() {
        let move_command = WireMoveCommand {
            wheel_a: -255.0,
            ..Default::default()
        };
        let encoded = move_command.encode();
        let mut iter = encoded.iter();
        assert_eq!(*iter.next().unwrap(), 1_u8);
        assert_eq!(*iter.next().unwrap(), 2_u8);
        assert_eq!(*iter.next().unwrap(), 255_u8);
    }
}
