default: hubo-main hubo-default

hubo-main: hubo-main.c
	gcc -o $@ $< -lach -lrt

hubo-default: hubo-default.c
	gcc -o $@ $< -lach -lrt

clean: 
	rm -f hubo-main hubo-default
