CC = g++

SOURCE_ROOT = $(shell pwd)

CFLAGS		:= -Wall -O2 -fno-buildin
CPPFLAGS 	:= -D_FILE_OFFSET_BITS=64 -D_LARGE_FILE
INCLUDE 	:= ../include
TARGET 		:= libcodec_utils.so
LIBPATH 	:= ../libs

all: $(TARGET)
SRCS += AString.cpp ABitReader.cpp DataSource.cpp FileSource.cpp types_def.cpp

OBJS = $(patsubst %.cpp, %.o, $(SRCS))

#OBJ_C = $(foreach file, $(OBJS), $(file))

$(OBJS): %.o:%.cpp
	$(CC) -I $(INCLUDE) $(CPPFLAGS) -c $(SRCS)

$(TARGET): $(OBJS)
	$(CC) -shared -o $(TARGET) $(OBJS)
	mkdir -p $(LIBPATH)
	mv $(TARGET) $(LIBPATH)

clean:
	@rm -rf *.o $(LIBPATH)

.PHONY: clean
