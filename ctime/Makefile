CC = g++
# CFLAGS = -Wall -Wextra -Werror -O2
CFLAGS = -Wall -Wextra -O2 `pkg-config --cflags opencv` 
OBJS = astrovision.o
LIBS = `pkg-config --libs opencv`

all: astrovision

astrovision: objects/$(OBJS)
	$(CC) $(CFLAGS) -o astrovision objects/$(OBJS) $(LIBS)

objects/astrovision.o: main.cpp
	$(CC) $(CFLAGS) -c main.cpp -o objects/astrovision.o

clean:
	rm -f *~ objects/*.o astrovision
