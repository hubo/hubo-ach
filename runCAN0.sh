sudo ifconfig can0 down
sudo ip link set can0 type can tq 125 prop-seg 1 phase-seg1 2 phase-seg2 4 sjw 1
sudo ifconfig can0 up
./can_write_100hz
