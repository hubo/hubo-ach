default: hubo-main make-ach

hubo-main: hubo-main.c
	gcc -o $@ $< -lach -lrt

make-ach: make-ach.c

	gcc -o $@ $< -lach -lrt

clean: 
	rm hubo-main
	rm make-ach
