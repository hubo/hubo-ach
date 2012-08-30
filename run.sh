sudo ifconfig can0 down

sudo ifconfig can1 down
sudo ifconfig can2 down
sudo ifconfig can3 down
sudo ip link set can0 type can tq 125 prop-seg 1 phase-seg1 2 phase-seg2 4 sjw 1
sudo ip link set can1 type can tq 125 prop-seg 1 phase-seg1 2 phase-seg2 4 sjw 1
sudo ip link set can2 type can tq 125 prop-seg 1 phase-seg1 2 phase-seg2 4 sjw 1
sudo ip link set can3 type can tq 125 prop-seg 1 phase-seg1 2 phase-seg2 4 sjw 1
sudo ifconfig can0 up
sudo ifconfig can1 up
sudo ifconfig can2 up
sudo ifconfig can3 up
sudo ach -U hubo-ref
sudo ach -C hubo-ref -m 10 -n 3000
sudo ach -U hubo-state
sudo ach -C hubo-state -m 10 -n 3000
sudo ach -U hubo-init-cmd
sudo ach -C hubo-init-cmd -m 10 -n 3000
sudo ach -U hubo-param
sudo ach -C hubo-param -m 10 -n 3000
sudo ./hubo-default
sudo ./hubo-main -v
