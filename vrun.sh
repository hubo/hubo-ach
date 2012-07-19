sudo rmmod vcan
sudo modprobe vcan
sudo ip link add type vcan
sudo ifconfig vcan0 up
sudo ip link add type vcan
sudo ifconfig vcan1 up
sudo ip link add type vcan
sudo ifconfig vcan2 up
sudo ip link add type vcan
sudo ifconfig vcan3 up
sudo ach -U hubo
sudo ach -C hubo
sudo ./hubo-default
sudo ./hubo-main -v
