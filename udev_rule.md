1. Rule file path: `/etc/udev/rules.d`
2. Check udev info `$(udevadm info -q path -n /dev/ttyUSB1)`
3. Check udev info `lsusb`
4. Make a new rule: `80-ugv.rules`
5. Rule: 
px4
`ACTION=="add", SUBSYSTEM=="tty", SUBSYSTEMS=="usb", ATTRS{idVendor}=="26ac", ATTRS{idProduct}=="0012", OWNER="ugv", GROUP="dialout", SYMLINK+="ttyPX4"`

U2D2(dynamixel driver)
`ACTION=="add", SUBSYSTEM=="tty", SUBSYSTEMS=="usb", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6014", OWNER="ugv", GROUP="dialout", SYMLINK+="ttyDynamixel"`
