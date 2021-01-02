# Hamilton

[![codecov](https://codecov.io/gh/dmweis/hamilton/branch/main/graph/badge.svg)](https://codecov.io/gh/dmweis/hamilton)
[![Rust](https://github.com/dmweis/hamilton/workflows/Rust/badge.svg)](https://github.com/dmweis/hamilton/actions)
[![Security audit](https://github.com/dmweis/hamilton/workflows/Security%20audit/badge.svg)](https://github.com/dmweis/hamilton/actions)
[![Private docs](https://github.com/dmweis/hamilton/workflows/Deploy%20Docs%20to%20GitHub%20Pages/badge.svg)](https://davidweis.dev/hamilton/hamilton/index.html)

Mecanum wheeled robot

[Read more here](https://davidweis.dev/hamilton)

When using arduino leonardo add following rule:  
`KERNEL=="ttyACM*", ATTRS{idVendor}=="2341", ATTRS{idProduct}=="8036", MODE:="0666", SYMLINK+="hamilton_motors"`  
to:  
`/etc/udev/rules.d/40-hamilton_motors.rules`
