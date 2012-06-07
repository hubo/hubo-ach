ifconfig can1 down
ip link set can1 type can bitrate 1000000
ifconfig can1 up
./hubo-achCAN
