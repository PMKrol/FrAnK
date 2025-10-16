# About
Toolset for acquiring data from ESP32: current weight, IR frame, motor position.

FrAnK reads data output from ESP ie.: sudo ./frank /dev/ttyUSB0 board_name
and saves it as txt file with zipped frames from camera.

FrAnCs translates txt file to three csv files and [todo] zip file (photos) to data.

fakeRS sends sample data (mimics serial port with ESP32 connected).

# TODO
esp soft with explanation.


# Permissions
On Ubuntu 24.04 change usb camera permissions by creating /lib/udev/rules.d/20-webcam.rules

#Bus 002 Device 003: ID 090c:37b6 Silicon Motion, Inc. - Taiwan (formerly Feiya Technology Corp.) USB 2.0 PC Cam
#Disable laptops build in camera
SUBSYSTEM=="usb", ATTRS{idVendor}=="090c", ATTRS{idProduct}=="37b6", ATTR{authorized}="0"

#No root access for this device
SUBSYSTEM=="usb", ATTR{idVendor}=="1224", ATTR{idProduct}=="2a25", MODE="0666"

