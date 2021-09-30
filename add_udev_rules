readonly udev_path="/etc/udev/rules.d/40-hamilton.rules"

echo "writing udev rules to $udev_path"

cat <<EOT | sudo tee $udev_path > /dev/null

# rplidar
KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0777", SYMLINK+="rplidar"

# sensors
KERNEL=="ttyUSB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE:="0777", SYMLINK+="hamilton_dc_motors"

EOT

sudo udevadm control --reload-rules && udevadm trigger
echo "Done"