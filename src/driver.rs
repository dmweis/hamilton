use anyhow::Error;
use anyhow::Result;
use bytes::{BufMut, BytesMut};
use futures::{SinkExt, StreamExt};
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
        let mut buffer = vec![];
        // push wheels in the weird format that I chose for some reason
        buffer.push((data.wheel_a > 0) as u8);
        buffer.push(data.wheel_a as u8);
        buffer.push((data.wheel_b > 0) as u8);
        buffer.push(data.wheel_b as u8);
        buffer.push((data.wheel_c > 0) as u8);
        buffer.push(data.wheel_c as u8);
        buffer.push((data.wheel_d > 0) as u8);
        buffer.push(data.wheel_d as u8);

        let encoded_data = postcard_cobs::encode_vec(&buffer);
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
