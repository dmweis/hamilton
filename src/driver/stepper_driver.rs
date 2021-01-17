use anyhow::Error;
use anyhow::Result;
use async_trait::async_trait;
use bytes::{BufMut, BytesMut};
use futures::SinkExt;
use std::str;
use tokio_util::codec::{Decoder, Encoder};

use super::{HamiltonDriver, WireMoveCommand};

#[derive(thiserror::Error, Debug)]
#[non_exhaustive]
pub enum HamiltonError {
    #[error("communication with motor driver failed")]
    CommError,
    #[error("failed opening serial port")]
    FailedOpeningSerialPort,
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

pub struct HamiltonStepperDriver {
    framed_port: tokio_util::codec::Framed<tokio_serial::Serial, HamiltonProtocol>,
}

impl HamiltonStepperDriver {
    pub fn new(port: &str) -> Result<Self> {
        let settings = tokio_serial::SerialPortSettings {
            baud_rate: 115200,
            ..Default::default()
        };
        let serial_port = tokio_serial::Serial::from_path(port, &settings)
            .map_err(|_| HamiltonError::FailedOpeningSerialPort)?;
        Ok(Self {
            framed_port: HamiltonProtocol.framed(serial_port),
        })
    }
}

#[async_trait]
impl HamiltonDriver for HamiltonStepperDriver {
    async fn send(&mut self, command: WireMoveCommand) -> Result<()> {
        self.framed_port
            .send(command)
            .await
            .map_err(|_| HamiltonError::CommError)?;
        Ok(())
    }

    async fn read_voltage(&mut self) -> Option<f32> {
        Ok(None)
    }

    async fn set_color(&mut self, _color: LedColor) -> Result<()> {
        Ok(())
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
