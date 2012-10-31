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
sudo ach -U hubo-ref
sudo ach -C hubo-ref -m 10 -n 3000
sudo ach -U hubo-state
sudo ach -C hubo-state -m 10 -n 3000
sudo ach -U hubo-init-cmd
sudo ach -C hubo-init-cmd -m 10 -n 3000
sudo ach -U hubo-param
sudo ach -C hubo-param -m 10 -n 3000
sudo ./hubo-default
sudo ./hubo-main 
