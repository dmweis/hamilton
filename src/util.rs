use anyhow::{anyhow, Result};
use std::sync::{
    atomic::{AtomicBool, Ordering},
    Arc, Mutex,
};
use tokio::sync::Notify;

pub fn latest_value_channel<T>() -> (LatestSender<T>, LatestReceiver<T>) {
    let value = Arc::new(Mutex::new(None));
    let notify = Arc::new(Notify::new());
    let both_alive = Arc::new(AtomicBool::new(true));

    let sender = LatestSender {
        value: Arc::clone(&value),
        notify: Arc::clone(&notify),
        both_alive: Arc::clone(&both_alive),
    };
    let receiver = LatestReceiver {
        value,
        notify,
        both_alive,
    };
    (sender, receiver)
}

pub struct LatestSender<T> {
    value: Arc<Mutex<Option<T>>>,
    notify: Arc<Notify>,
    both_alive: Arc<AtomicBool>,
}

impl<T> LatestSender<T> {
    pub fn send(&self, value: T) -> Result<()> {
        if !self.both_alive.load(Ordering::SeqCst) {
            Err(anyhow!("Other end died"))
        } else {
            *self.value.lock().unwrap() = Some(value);
            self.notify.notify_one();
            Ok(())
        }
    }
}

impl<T> Drop for LatestSender<T> {
    fn drop(&mut self) {
        self.both_alive.store(false, Ordering::SeqCst);
        self.notify.notify_waiters()
    }
}

pub struct LatestReceiver<T> {
    value: Arc<Mutex<Option<T>>>,
    notify: Arc<Notify>,
    both_alive: Arc<AtomicBool>,
}

impl<T> LatestReceiver<T> {
    pub async fn recv(&self) -> Result<T> {
        if !self.both_alive.load(Ordering::SeqCst) {
            return Err(anyhow!("Sender died"));
        }
        self.notify.notified().await;
        if !self.both_alive.load(Ordering::SeqCst) {
            return Err(anyhow!("Sender died"));
        }
        Ok(self
            .value
            .lock()
            .expect("Lock poisoned")
            .take()
            .expect("No value inside. This is bad"))
    }
}

impl<T> Drop for LatestReceiver<T> {
    fn drop(&mut self) {
        self.both_alive.store(false, Ordering::SeqCst);
    }
}
