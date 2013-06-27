hubo-ach killall
hubo-ach clean
sudo mkdir /var/log/hubo/
sudo mkdir /var/lock/hubo/
./configure
sudo make clean
make
sudo make install
sudo updatedb
