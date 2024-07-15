### Rescue Rollers microROS control code for ESP32-S3

# Requirements
https://github.com/espressif/esp-idf
https://github.com/micro-ROS/docker/tree/iron

# Getting Started

git clone https://github.com/snibo13/rescue-rollers or gh repo clone snibo13/rescue-rollers with GitHub CLI
git submodule init
git submodule update

# Building
mkdir build && cd build
source /path/to/idf/install/esp/esp-idf/export.sh
idf.py set-target esp32s3
idf.py menuconfig

microROS Settings 
- Set your micro-ROS Agent IP
microROS Settings > Wifi Configuration
- Set your WiFi SSID and Password

idf.py build
idf.py flash
