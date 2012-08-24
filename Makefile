default: hubo-loop

hubo-loop: hubo-loop.c
	gcc -o $@ $< -lach -lrt

clean: 
	rm hubo-loop
