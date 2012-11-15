sudo ach -U hubo-ref
sudo ach -C hubo-ref -m 10 -n 3000
sudo ach -U hubo-state
sudo ach -C hubo-state -m 10 -n 3000
sudo ach -U hubo-init-cmd
sudo ach -C hubo-init-cmd -m 10 -n 3000
sudo ach -U hubo-param
sudo ach -C hubo-param -m 10 -n 3000
sudo chmod 666 /dev/shm/achshm-hubo-ref
