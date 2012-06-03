sudo rmmod vcan
sudo modprobe vcan
sudo ip link add type vcan
sudo ifconfig vcan0 up
