sudo apt-get remove hubo-ach hubo-ach-dev
sudo apt-get purge hubo-ach hubo-ach-dev
sudo rm /usr/bin/hubo*
sudo rm /usr/local/bin/hubo*
sudo rm /usr/include/hubo*
sudo rm /usr/local/include/hubo*
sudo rm -R /etc/hubo-ach
sudo rm /usr/lib/libhuboparams.la
sudo rm /usr/local/lib/libhuboparams.la
sudo rm $(locate hubo_ach.py | grep /usr/)
