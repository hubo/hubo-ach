hubo-ach killall
sudo mkdir /var/log/hubo/
sudo mkdir /var/lock/hubo/
./configure
sudo make clean
make
sudo make install
sudo updatedb
