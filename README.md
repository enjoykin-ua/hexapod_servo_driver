# Hexapod servo driver
Status: in development  

This software receives commands via USB UART to control the following: servos, LEDs, reading switches, etc.

# Build it:
go to the path of the project. Project structure:

pico
├── pico-sdk
├── pimoroni-pico
└── hexapod_servo_driver

# if cmake changed:
rm -rf build

# for the first time or if changed cmake/structure
mkdir build
cd build
cmake ..

# just compile
make -j$(nproc)

# put the image to the servo2040 (unconnect usb, press user boot + connect usb connector)
sudo picotool load Hexapod_servo_driver.uf2
sudo picotool reboot