# Hamilton

[![codecov](https://codecov.io/gh/dmweis/hamilton/branch/main/graph/badge.svg)](https://codecov.io/gh/dmweis/hamilton)
[![Rust](https://github.com/dmweis/hamilton/workflows/Rust/badge.svg)](https://github.com/dmweis/hamilton/actions)
[![Rust-windows](https://github.com/dmweis/hamilton/workflows/Rust-windows/badge.svg)](https://github.com/dmweis/hamilton/actions)
[![Private docs](https://github.com/dmweis/hamilton/workflows/Deploy%20Docs%20to%20GitHub%20Pages/badge.svg)](https://davidweis.dev/hamilton/hamilton/index.html)

Mecanum wheeled robot

[Read more here](https://davidweis.dev/hamilton)

## Installing

Udev rules need to be added to `/etc/udev/rules.d/40-hamilton_motors.rules`

When using arduino leonardo add following rule:

```shell
KERNEL=="ttyACM*", ATTRS{idVendor}=="2341", ATTRS{idProduct}=="8036", MODE:="0666", SYMLINK+="hamilton_motors"
```

When using LLS motors with the USB to TTL UART adaptor use:

```shell
KERNEL=="ttyUSB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE:="0666", SYMLINK+="hamilton_motors"
```

## Cross compilation

To cross compile for raspberry pi you need to add `armv7-unknown-linux-musleabihf` target to rustup

You will also need an arm compatible compiler. On debian that is `gcc-arm-linux-gnueabihf`

To set this up run:

```shell
rustup target add armv7-unknown-linux-musleabihf
sudo apt update
sudo apt install gcc-arm-linux-gnueabihf
```

Then compile for arm with `cargo build --no-default-features --target=armv7-unknown-linux-musleabihf`

The [deploy script](./deploy) will try to build and copy the binaries to the right place. But they both take assumptions about hostnames and paths because I am lazy.
