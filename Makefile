default: hubo-console

hubo-console: hubo-console.cpp
	g++ -Wno-write-strings -o $@ $< -lach -lrt -lreadline -lcurses

clean: 
	rm hubo-console
