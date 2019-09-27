CC = g++

SOURCE_ROOT = $(shell pwd)

CFLAGS		:= -Wall -O2 -fno-buildin
CPPFLAGS	:=
INCLUDE 	:= -I ../include
LIBVAR		:= libcodec_utils.so
TARGET      := JpegDecoder

all: $(TARGET)
SRCS += JpegDecoder.cpp

OBJS = $(patsubst %.cpp, %.o, $(SRCS))

#OBJ_C = $(foreach file, $(OBJS), $(file))

$(OBJS): %.o:%.cpp
	$(CC) $(INCLUDE) $(CPPFLAGS) -c $(SRCS) -o $(OBJS)

$(TARGET): $(OBJS)
	$(CC) -o $(TARGET) $(LIBVAR) $(OBJS)

clean:
	@rm -f *.o $(TARGET)

.PHONY: clean
