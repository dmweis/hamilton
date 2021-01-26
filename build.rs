fn main() {
    tonic_build::compile_protos("proto/hamilton/hamilton_service.proto").unwrap();
}
