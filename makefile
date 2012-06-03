default: hubo-ach hubo-achCAN hubo-achCAN-Read
hubo-ach: hubo-ach.c
	gcc -g hubo-ach.c -o hubo-ach -lach
hubo-achCAN: hubo-achCAN.c
	gcc -g hubo-achCAN.c -o hubo-achCAN -lach
hubo-achCAN-Read: hubo-achCAN-Read.c
	gcc -g hubo-achCAN-Read.c -o hubo-achCAN-Read -lach
