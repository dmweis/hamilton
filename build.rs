fn main() {
    tonic_build::compile_protos("proto/hamilton/hamilton.proto").unwrap();
}
