CXX = g++

#SOURCE_ROOT = $(shell pwd)

CFLAGS     := -Wall -O2 -fno-buildin
CPPFLAGS   := -D_FILE_OFFSET_BITS=64 -D_LARGE_FILE
INCLUDE    := ../include
TARGET_LIB := libcodec_utils.so
DST_PATH   := ../libs

#SRCS = AString.cpp ABitReader.cpp DataSource.cpp FileSource.cpp types_def.cpp
#OBJS = $(patsubst %.cpp, %.o, $(SRCS))

SRCS := $(wildcard *.cpp)
OBJS := $(SRCS:%.cpp=%.o)

$(TARGET_LIB): $(OBJS)
	$(CXX) $(CPPFLAGS) -shared -o $@ $^

$(OBJS): $(SRCS)
	$(CXX) -I $(INCLUDE) $(CPPFLAGS) -fPIC -c $^

clean:
	@rm -rf *.o $(TARGET_LIB)

install:
	@mkdir -p $(DST_PATH)
	@cp $(TARGET_LIB) $(DST_PATH)

.PHONY: clean
