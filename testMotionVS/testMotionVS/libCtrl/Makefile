include ../mk.env

LIBS = -lpthread -lm -lstdc++

#include head file
INC = -I. -I./include 

TARGET = lib/libCtrl.a

DIRS:=$(shell find . -maxdepth 3 -type d)	#查找三级目录，根据需要修改
FILE_C = $(foreach dir,$(DIRS),$(wildcard $(dir)/*.c))
FILE_CPP = $(foreach dir,$(DIRS),$(wildcard $(dir)/*.cpp))

OBJS = $(patsubst %.c,%.o, $(FILE_C)) $(patsubst %.cpp,%.opp, $(FILE_CPP))

all:$(TARGET)

%.o:%.c
	$(CC) -c  $(CFLAGS) $(INC) $< -o $@
%.opp:%.cpp
	$(CC) -c $(CFLAGS) $(INC) $< -o $@

$(TARGET):$(OBJS)
	-mkdir lib
	$(AR) $(TARGET) $(OBJS)  

clean:
	-$(RM) $(TARGET)
	-$(RM) $(OBJS)

