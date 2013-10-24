#!/bin/bash
if [[ $# != 1 ]]; then
	echo 'Usage: ./record.sh JOINT_ACRONYM > file-to-save.txt'
	exit
fi
sudo hubo-read | grep $1 | awk '{print $11, $14}'
