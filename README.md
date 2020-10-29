# Hamilton

Mecanum wheeled robot

[Read more here](https://davidweis.dev/hamilton)

When using arduino leonardo add following rule:  
`KERNEL=="ttyACM*", ATTRS{idVendor}=="2341", ATTRS{idProduct}=="8036", MODE:="0666", SYMLINK+="hamilton_motors"`  
to:  
`/etc/udev/rules.d/40-hamilton_motors.rules`
