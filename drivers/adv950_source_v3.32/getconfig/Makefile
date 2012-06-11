CC = gcc
CFLAGS = -O
GETCONFIG = getconfig

all: $(GETCONFIG)

$(GETCONFIG): $(GETCONFIG).c
	$(CC) $(CFLAGS) -o $@ $<

clean:
	@rm -f *.o $(GETCONFIG)
