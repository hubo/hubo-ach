hubo-ach killall
./hubo-ach-safe-clean.sh
sudo mkdir /var/log/hubo/
sudo mkdir /var/lock/hubo/
./configure
sudo make clean
make
sudo make install
sudo updatedb
