# kuroko_setup
Summarize the work required to set up the robot


### Environment
- Desktop PC
  - Ubuntu20.04 kernel 5.15.0-67
    - ```sudo apt install -y linux-headers-5.15.0-67-generic linux-hwe-5.15-headers-5.15.0-67 linux-image-5.15.0-67-generic linux-modules-5.15.0-67-generic linux-modules-extra-5.15.0-67-generic```
  - linux-modules-extra-5.15.0-67-generic
    - ```curl -1sLf 'https://dl.cloudsmith.io/public/balena/etcher/setup.deb.sh'| sudo -E bash```
    - ```sudo apt update;sudo apt install -y balena-etcher-electron```
  - Download jetpack 5.0 for jetson xavier nx
    - https://developer.nvidia.com/jetpack-sdk-50dp/sd-card-image

- Robot PC

