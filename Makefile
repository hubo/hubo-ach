default: hubo-main make-ach

hubo-main: hubo-main.c
	gcc -o $@ $< -lach

make-ach: make-ach.c

	gcc -o $@ $< -lach

clean: 
	rm hubo-main
	rm make-ach
