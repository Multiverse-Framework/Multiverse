TARGET := cmd_vel_sub

CYCLONEDDS_HOME ?= /usr/local

CXX := g++
CC := gcc
IDLC := $(CYCLONEDDS_HOME)/bin/idlc

CPPFLAGS := -I. -I$(CYCLONEDDS_HOME)/include
CXXFLAGS := -O2 -Wall -Wextra -std=c++17
CFLAGS := -O2 -Wall -Wextra -std=c11

LDFLAGS := -L$(CYCLONEDDS_HOME)/lib
LDLIBS := -lddsc

OBJ_CPP := main.o
OBJ_C := Vector3.o Twist.o

all: check gen $(TARGET)

check:
	@test -x "$(IDLC)" || (echo "Error: idlc not found at $(IDLC)"; exit 1)
	@test -f "$(CYCLONEDDS_HOME)/include/dds/dds.h" || (echo "Error: dds/dds.h not found"; exit 1)
	@test -f "$(CYCLONEDDS_HOME)/lib/libddsc.so" -o -f "$(CYCLONEDDS_HOME)/lib/libddsc.so.0" || (echo "Error: libddsc.so not found"; exit 1)

gen: Vector3.h Vector3.c Twist.h Twist.c

Vector3.h Vector3.c: Vector3.idl
	$(IDLC) Vector3.idl

Twist.h Twist.c: Twist.idl Vector3.h Vector3.c
	$(IDLC) Twist.idl

main.o: main.cpp Twist.h Vector3.h
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -c -o $@ main.cpp

Vector3.o: Vector3.c Vector3.h
	$(CC) $(CPPFLAGS) $(CFLAGS) -c -o $@ Vector3.c

Twist.o: Twist.c Twist.h Vector3.h
	$(CC) $(CPPFLAGS) $(CFLAGS) -c -o $@ Twist.c

$(TARGET): $(OBJ_CPP) $(OBJ_C)
	$(CXX) -o $@ $(OBJ_CPP) $(OBJ_C) $(LDFLAGS) $(LDLIBS)

clean:
	rm -f $(TARGET) *.o Vector3.c Vector3.h Twist.c Twist.h

.PHONY: all check gen clean