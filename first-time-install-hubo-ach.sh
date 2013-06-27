sudo apt-get install linux-headers-$(uname -r)
cd ~/
rm tmp.hubo-ach
rm sources.list.hubo-ach
sudo cp /etc/apt/sources.list /etc/apt/sources.list.bk-pre-ach
echo 'deb http://code.golems.org/debian precise golems.org' >> tmp.hubo-ach
cat /etc/apt/sources.list | cat - tmp.hubo-ach >> sources.list.hubo-ach
sudo mv sources.list.hubo-ach /etc/apt/sources.list
rm tmp.hubo-ach
rm sources.list.hubo-ach
sudo apt-get update
sudo apt-get install build-essential
sudo apt-get install git-core
sudo apt-get install libach1 libach-dev
sudo apt-get install ach-utils
sudo apt-get install autoconf automake libtool autoconf-archive
sudo apt-get install libreadline-dev
sudo apt-get install gcc
sudo apt-get install g++
sudo apt-get install libpopt-dev
git clone https://github.com/hubo/hubo-ach.git
cd hubo-ach/drivers
tar -xzf peak-linux-driver-7.7.tar.gz
cd peak-linux-driver-7.7
cat /lib/modules/$(uname -r)/build/include/linux/version.h
sudo make clean
sudo make
sudo make install
sudo /sbin/modprobe pcan
cd ~/hubo-ach
autoreconf -i
./hubo-ach-install.sh
# Install ACH python bindings
sudo apt-get install python-pip
sudo pip install http://code.golems.org/src/ach/py_ach-latest.tar.gz
