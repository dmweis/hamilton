mod driver;
mod holonomic_controller;

use serde::{Deserialize, Serialize};

use anyhow::Result;
use std::sync::Arc;
use tokio::sync::Mutex;

use tonic::{transport::Server, Request, Response, Status};

use hamilton::hamilton_remote_server::{HamiltonRemote, HamiltonRemoteServer};

pub mod hamilton {
    tonic::include_proto!("hamilton");
}

struct HamiltonRemoteController {
    driver: Arc<Mutex<driver::HamiltonDriver>>,
}

#[tonic::async_trait]
impl HamiltonRemote for HamiltonRemoteController {
    async fn move_stream(
        &self,
        commands: tonic::Request<tonic::Streaming<hamilton::MoveRequest>>,
    ) -> std::result::Result<tonic::Response<hamilton::MoveResponse>, tonic::Status> {
        let mut command_stream = commands.into_inner();
        while let Some(message) = command_stream.message().await? {
            let mut driver = self.driver.lock().await;
        }
        Ok(tonic::Response::new(hamilton::MoveResponse {}))
    }
}

fn main() {
    println!("Hello, world!");
}
