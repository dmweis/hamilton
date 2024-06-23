use std::{
    any::{Any, TypeId},
    collections::HashMap,
    sync::{Arc, Mutex, OnceLock},
};
use thiserror::Error;
use tracing::error;

#[derive(Error, Debug)]
pub enum IocContainerError {
    #[error("service not available in container: {0:?}")]
    ServiceNotAvailable(&'static str),
    #[error("failed to downcast service: {0:?}")]
    FailedToDowncast(&'static str),
}

type Handle = Arc<dyn Any + Send + Sync>;

#[derive(Debug, Clone)]
pub struct IocContainer {
    map: Arc<Mutex<HashMap<TypeId, Handle>>>,
}

impl IocContainer {
    fn new() -> Self {
        Self {
            map: Arc::new(Mutex::new(HashMap::new())),
        }
    }

    pub fn register<T: Any + Send + Sync>(&self, object: T) {
        let type_id = object.type_id();
        self.map.lock().unwrap().insert(type_id, Arc::new(object));
    }

    pub fn register_arc<T: Any + Send + Sync>(&self, object: Arc<T>) {
        let type_id = (*object).type_id();
        self.map.lock().unwrap().insert(type_id, object);
    }

    pub fn service<T: Any + Send + Sync>(&self) -> Result<Arc<T>, IocContainerError> {
        let type_id = TypeId::of::<T>();
        let type_name = std::any::type_name::<T>();
        if let Some(object) = self.map.lock().unwrap().get(&type_id) {
            let handle = object.clone();
            match handle.downcast::<T>() {
                Ok(service) => Ok(service),
                Err(err) => {
                    error!(
                        "Failed to downcast type {:?} with error {:?}",
                        type_name, err
                    );
                    Err(IocContainerError::FailedToDowncast(type_name))
                }
            }
        } else {
            error!("Service not available {:?}", type_name);
            Err(IocContainerError::ServiceNotAvailable(type_name))
        }
    }

    #[cfg(test)]
    fn delete<T: Any + Send + Sync>(&self) {
        let type_id = TypeId::of::<T>();
        self.map.lock().unwrap().remove(&type_id);
    }

    pub fn global_instance() -> &'static IocContainer {
        static INSTANCE: OnceLock<IocContainer> = OnceLock::new();
        INSTANCE.get_or_init(IocContainer::new)
    }
}

#[macro_export]
macro_rules! ioc_register {
    ( $instance:expr ) => {{
        IocContainer::global_instance().register($instance);
    }};
}

#[macro_export]
macro_rules! ioc_register_arc {
    ( $instance:expr ) => {{
        IocContainer::global_instance().register_arc($instance);
    }};
}

#[macro_export]
macro_rules! ioc_service {
    ( $service_type:ty ) => {{
        IocContainer::global_instance().service::<$service_type>()
    }};
}

#[cfg(test)]
mod tests {
    use super::*;

    struct A;

    #[test]
    fn simple_ioc() {
        let container = IocContainer::new();
        container.register(A);
        let a = container.service::<A>();
        assert!(a.is_ok());
    }

    #[test]
    fn simple_ioc_with_arc() {
        let container = IocContainer::new();
        let a_arc = Arc::new(A);
        container.register_arc(a_arc);
        let a = container.service::<A>();
        assert!(a.is_ok());
    }

    #[test]
    fn fail_on_unregistered_type() {
        let container = IocContainer::new();
        let not_a = container.service::<A>();
        assert!(not_a.is_err())
    }

    /// Lock around this to make sure tests that use the global ioc store are not running in parallel
    static GLOBAL_IOC_TEST_MUTEX: Mutex<()> = Mutex::new(());

    #[test]
    fn register_service_macro() {
        let _guard = GLOBAL_IOC_TEST_MUTEX.lock().unwrap();

        IocContainer::global_instance().delete::<A>();
        assert!(IocContainer::global_instance().service::<A>().is_err());
        ioc_register!(A);
        let global = IocContainer::global_instance();
        let _a: Arc<A> = global.service::<A>().unwrap();
    }

    #[test]
    fn register_service_arc_macro() {
        let _guard = GLOBAL_IOC_TEST_MUTEX.lock().unwrap();

        IocContainer::global_instance().delete::<Arc<A>>();
        IocContainer::global_instance().delete::<A>();
        assert!(IocContainer::global_instance().service::<A>().is_err());
        assert!(IocContainer::global_instance().service::<Arc<A>>().is_err());
        let instance = Arc::new(A);
        ioc_register_arc!(instance);
        let global = IocContainer::global_instance();
        // type is in there
        let _a: Arc<A> = global.service::<A>().unwrap();
        // arc wrapped type is not
        assert!(IocContainer::global_instance().service::<Arc<A>>().is_err());
    }

    #[test]
    fn get_service_macro() {
        let _guard = GLOBAL_IOC_TEST_MUTEX.lock().unwrap();

        let global = IocContainer::global_instance();
        global.register(A);
        let _a: Arc<A> = ioc_service!(A).unwrap();
    }
}
