CXX = g++

#SOURCE_ROOT = $(shell pwd)

CFLAGS      := -Wall -O2 -fno-buildin
CPPFLAGS    :=
INCLUDE     := -I ../include
#linux platform
#LD_LIBS     := -L../libs -lcodec_utils
#windows platform
LD_LIBS     := -L../libs libcodec_utils.so

TARGET_BIN  := JpegDecoder

SRCS    := JpegDecoder.cpp
OBJS    := $(SRCS:%.cpp=%.o)
#OBJS    := $(patsubst %.cpp, %.o, $(SRCS))

all:$(TARGET_BIN)

$(TARGET_BIN): $(OBJS)
	$(CXX) -o $@ $^ $(LD_LIBS)

$(OBJS): $(SRCS)
	$(CXX) $(INCLUDE) $(CPPFLAGS) -fPIC -c $^ -o $@

clean:
	@rm -f *.o $(TARGET_BIN)

.PHONY: clean
