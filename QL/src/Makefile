TARGET		= libQL.a
OBJS		= ql.o
SRCS		= $(OBJS:.o=.cpp)
LDFLAGS= -std=c++11 -Wall -shared -fPIC -O2 -o

CC			= g++
AR			= ar

all:
	$(CC) $(LDFLAGS) ../lib/$(TARGET) $(SRCS)

