kill -9 $(pidof achpipe.bin)
sudo /etc/init.d/openbsd-inetd stop
sudo rm /dev/shm/achshm-hubo-*
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
sudo ach -U hubo-ref-filter
sudo ach -C hubo-ref-filter -m 10 -n 30000
sudo ach -U hubo-ref
sudo ach -C hubo-ref -m 10 -n 30000
sudo ach -U hubo-state
sudo ach -C hubo-state -m 10 -n 30000
sudo ach -U hubo-init-cmd
sudo ach -C hubo-init-cmd -m 10 -n 30000
sudo ach -U hubo-param
sudo ach -C hubo-param -m 10 -n 3000
sudo ./hubo-default
sudo ./hubo-main 
