CC = g++
CFLAGS = -Wall -Wextra -Werror -O2
OBJS = main.o

all: astrovision

astrovision: $(OBJS)
	$(CC) $(CFLAGS) -o astrovision $(OBJS)

main.o: main.cpp
	$(CC) $(CFLAGS) -c main.cpp

clean:
	rm -f *~ *.o astrovision
