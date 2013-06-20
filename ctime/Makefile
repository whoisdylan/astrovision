CC = g++
CFLAGS = -Wall -Wextra -Werror -O2
OBJS = astrovision.o

all: astrovision

astrovision: objects/$(OBJS)
	$(CC) $(CFLAGS) -o astrovision objects/$(OBJS)

objects/astrovision.o: astrovision.cpp
	$(CC) $(CFLAGS) -c astrovision.cpp -o objects/astrovision.o

clean:
	rm -f *~ objects/*.o astrovision
