default: hubo-main

hubo-main: hubo-main.c
	gcc -o $@ $< -lach

clean: 
	rm hubo-main
