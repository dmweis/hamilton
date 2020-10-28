use anyhow::Error;
use anyhow::Result;
use bytes::{BufMut, BytesMut};
use futures::SinkExt;
use std::str;
use tokio_util::codec::{Decoder, Encoder};

#[derive(thiserror::Error, Debug)]
#[non_exhaustive]
pub enum HamiltonError {
    #[error("communication with motor driver failed")]
    CommError,
    #[error("failed opening serial port")]
    FailedOpeningSerialPort,
}

#[derive(Default)]
pub struct WireMoveCommand {
    pub wheel_a: i32,
    pub wheel_b: i32,
    pub wheel_c: i32,
    pub wheel_d: i32,
}

impl WireMoveCommand {
    fn encode(&self) -> Vec<u8> {
        let mut buffer = vec![];
        // push wheels in the weird format that I chose for some reason
        buffer.push((self.wheel_a > 0) as u8);
        buffer.push(self.wheel_a.abs() as u8);
        buffer.push((self.wheel_b > 0) as u8);
        buffer.push(self.wheel_b.abs() as u8);
        buffer.push((self.wheel_c > 0) as u8);
        buffer.push(self.wheel_c.abs() as u8);
        buffer.push((self.wheel_d > 0) as u8);
        buffer.push(self.wheel_d.abs() as u8);

        let mut encoded = postcard_cobs::encode_vec(&buffer);
        encoded.push(0);
        encoded
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

pub struct HamiltonDriver {
    framed_port: tokio_util::codec::Framed<tokio_serial::Serial, HamiltonProtocol>,
}

impl HamiltonDriver {
    pub fn new(port: &str) -> Result<HamiltonDriver> {
        let mut settings = tokio_serial::SerialPortSettings::default();
        settings.baud_rate = 115200;
        let serial_port = tokio_serial::Serial::from_path(port, &settings)
            .map_err(|_| HamiltonError::FailedOpeningSerialPort)?;
        Ok(HamiltonDriver {
            framed_port: HamiltonProtocol.framed(serial_port),
        })
    }

    pub async fn send(&mut self, command: WireMoveCommand) -> Result<()> {
        self.framed_port
            .send(command)
            .await
            .map_err(|_| HamiltonError::CommError)?;
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
        let mut move_command = WireMoveCommand::default();
        move_command.wheel_a = 255;
        let encoded = move_command.encode();
        let mut iter = encoded.iter();
        assert_eq!(*iter.next().unwrap(), 3_u8);
        assert_eq!(*iter.next().unwrap(), 1_u8);
        assert_eq!(*iter.next().unwrap(), 255_u8);
    }

    #[test]
    fn left_front_negative() {
        let mut move_command = WireMoveCommand::default();
        move_command.wheel_a = -255;
        let encoded = move_command.encode();
        let mut iter = encoded.iter();
        assert_eq!(*iter.next().unwrap(), 1_u8);
        assert_eq!(*iter.next().unwrap(), 2_u8);
        assert_eq!(*iter.next().unwrap(), 255_u8);
    }
}
