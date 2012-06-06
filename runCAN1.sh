ifconfig can1 down
ip link set can1 type can tq 125 prop-seg 1 phase-seg1 2 phase-seg2 4 sjw 1
ifconfig can1 up
./hubo-achCAN
