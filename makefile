CC = clang++

OPT = 3

CFLAGS = -Wall -Wextra -pedantic -g -O$(OPT) -std=c++1y -stdlib=libc++ -Wno-deprecated-register

INCLUDES = -I/usr/include/eigen3/

LFLAGS = -stdlib=libc++ -O$(OPT)

LIBS = -lpthread -lX11

SRCS = main.cpp

HEADERS = hen.h stdcomp/shaders.h utils.h stdcomp/samplers.h

OBJS = $(SRCS:.cpp=.o)

MAIN = hen


all:    $(MAIN)

$(MAIN): $(OBJS) 
		$(CC) -o $(MAIN) $(OBJS) $(LFLAGS) $(LIBS)

main.o: main.cpp $(HEADERS)
		$(CC) $(CFLAGS) $(INCLUDES) -c $<  -o $@

clean:
		$(RM) *.o *~ $(MAIN)
