sudo ifconfig can0 down
sudo ifconfig can1 down
sudo ifconfig can2 down
sudo ifconfig can3 down
sudo echo "i 0x0014 e" > /dev/pcan0
sudo echo "i 0x0014 e" > /dev/pcan1
sudo echo "i 0x0014 e" > /dev/pcan2
sudo echo "i 0x0014 e" > /dev/pcan3
sudo ifconfig can0 up
sudo ifconfig can1 up
sudo ifconfig can2 up
sudo ifconfig can3 up
sudo chmod 666 /dev/shm/achshm-hubo-ref
sudo ./hubo-default
sudo ./hubo-main 
