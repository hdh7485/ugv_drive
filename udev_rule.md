1. Rule file path: `/etc/udev/rules.d`
2. Check udev info `$(udevadm info -q path -n /dev/ttyUSB1)`
3. Make a new rule: `80-px4.rules`
4. Rule: 
ACTIO=="add", SUBSYSTEM=="usb", ATTRS{idVendor}=="26ac", ATTRS{idProduct}=="0012", SYMLINK+="ttyPX4"

