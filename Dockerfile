FROM balenalib/raspberrypi4-64-debian AS chef

WORKDIR /app

# Install dependancies
RUN apt-get update && apt-get install -y lld clang autoconf libtool pkg-config libssl-dev build-essential

# Install rust
RUN curl https://sh.rustup.rs -sSf | bash -s -- -y
ENV PATH="/root/.cargo/bin:${PATH}"

RUN cargo install cargo-chef

# rust layer caching
FROM chef AS planner
COPY . .
RUN cargo chef prepare --recipe-path recipe.json

# rebuild dependencies if changed
FROM chef AS builder
# install deb because it doesn't chage often
RUN cargo install cargo-deb

# dependancies rebuild
COPY --from=planner /app/recipe.json recipe.json
RUN cargo chef cook --release --recipe-path recipe.json

# Now copy code
COPY . .

# Build
RUN cargo build --release
RUN cargo deb --no-build

# Copy to exporter
FROM scratch AS export
COPY --from=builder /app/target/debian/hamilton*.deb /
COPY --from=builder /app/target/debian/hamilton*.deb /hamilton.deb
