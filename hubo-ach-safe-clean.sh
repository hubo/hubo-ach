hubo-ach killall
sudo apt-get --yes --force-yes remove hubo-ach hubo-ach-dev
sudo apt-get --yes --force-yes purge hubo-ach hubo-ach-dev
sudo updatedb
sudo rm $(locate hubo-daemon | grep /bin)
sudo rm $(locate hubo-console | grep /bin)
sudo rm $(locate hubo-read | grep /bin)
sudo rm $(locate hubo-ach | grep /bin)
sudo rm /etc/hubo-ach/joint.table
sudo rm /etc/hubo-ach/sensor.table
sudo rm /etc/hubo-ach/drc-hubo.joint.table  
sudo rm /etc/hubo-ach/drc-hubo.sensor.table  
sudo rm /etc/hubo-ach/huboplus.joint.table  
sudo rm /etc/hubo-ach/huboplus.sensor.table
sudo rm /etc/hubo-ach/virtualHubo.py
sudo rm /etc/hubo-ach/hubo-ach-safe-clean.sh
sudo rm -R /etc/hubo-ach
sudo rm $(locate libhuboparams.so | grep /usr)
sudo rm $(locate hubo_ach.pyc | grep /usr)
sudo rm $(locate hubo_ach.pyo | grep /usr)
sudo rm $(locate hubo_ach.py | grep /usr)
sudo rm $(locate hubo.h | grep /usr)
sudo rm $(locate hubo-daemonID.h | grep /usr)
sudo rm $(locate hubo-jointparams.h | grep /usr)
