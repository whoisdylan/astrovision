CC = g++
# CFLAGS = -Wall -Wextra -Werror -O2
CFLAGS = -g -Wall -Wextra -O2 `pkg-config --cflags opencv` 
CFLAGSFAST = -Wall -Wextra -O4 -ffast-math `pkg-config --cflags opencv` 
OBJS = astrovision.o
POBJS = persistent.o
TOBJS = threeway.o
LIBS = `pkg-config --libs opencv`

all: astrovision

astrovision: objects/$(OBJS)
	$(CC) $(CFLAGS) -o astrovision objects/$(OBJS) $(LIBS)

objects/astrovision.o: main.cpp
	$(CC) $(CFLAGS) -c main.cpp -o objects/astrovision.o

three: objects/$(TOBJS)
	$(CC) $(CFLAGS) -o three objects/$(TOBJS) $(LIBS)

bvision: objects/$(POBJS)
	$(CC) $(CFLAGS) -o bvision objects/$(POBJS) $(LIBS)

objects/threeway.o: threeway.cpp
	$(CC) $(CFLAGS) -c threeway.cpp -o objects/threeway.o

objects/persistent.o: persistent.cpp
	$(CC) $(CFLAGS) -c persistent.cpp -o objects/persistent.o


clean:
	rm -f *~ objects/*.o astrovision
