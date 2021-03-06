## This is a project made within Seoul National University of Science and Technology
## Embedded Systems Laboratory 2018 - Raimarius Tolentino Delgado

#######################################################################################################
CUR_DIR = ./
CFLAGS_SWITCHES = -Wall -Wno-unused-but-set-variable -Wno-unused-variable 

INC_EMBD = $(CUR_DIR)/libs/embedded
INC_DIRS = -I$(CUR_DIR) -I$(INC_EMBD) 

RT_DOMAIN ?= rt_preempt
ifeq ($(RT_DOMAIN),xenomai)
XENOMAI_PATH=/usr/xenomai
INC_XENO = $(shell $(XENOMAI_PATH)/bin/xeno-config --skin alchemy --cflags) 
LIB_XENO = $(shell $(XENOMAI_PATH)/bin/xeno-config --skin native --ldflags) 
endif

CFLAGS   = $(CFLAGS_OPTIONS) $(INC_DIRS) $(INC_XENO)    
LDFLAGS	 = -lm -lrt -lpthread $(LIB_XENO)  

SOURCES	+= main.c 			
SOURCES	+= $(INC_EMBD)/src/rt_tasks.c
SOURCES	+= $(INC_EMBD)/src/rt_itc.c
ifneq ($(RT_DOMAIN),xenomai)
SOURCES	+= $(INC_EMBD)/src/rt_posix_task.c
SOURCES	+= $(INC_EMBD)/src/rt_posix_mutex.c
endif

OBJ_DIR = obj
OUT_DIR = bin
EXEC_TARGET	= test_perf
START	= start

ifeq ($(wildcard main.cpp),)
CC = $(CROSS_COMPILE)gcc
else
CC = $(CROSS_COMPILE)g++
endif

LD = $(CROSS_COMPILE)ld
AS = $(CROSS_COMPILE)as

CHMOD	= /bin/chmod
MKDIR	= /bin/mkdir
ECHO	= echo
RM	= /bin/rm
#######################################################################################################
OBJECTS = $(addprefix $(OBJ_DIR)/, $(notdir $(patsubst %.c, %.o, $(patsubst %.cpp, %.o, $(SOURCES)))))
#######################################################################################################
vpath %.c  $(CUR_DIR) $(INC_SERVO) $(INC_EMBD)/src
vpath %.cpp $(CUR_DIR) $(INC_SERVO) $(INC_EMBD)/src
#######################################################################################################

ifeq ($(wildcard $(START).sh),)
all: 	$(OUT_DIR)/$(EXEC_TARGET) $(START)
	@$(ECHO) BUILD DONE.
	@$(CHMOD) +x $(START).sh
else
all: 
	@$(ECHO) BUILD ERROR: Run make clean first!
endif

$(START): 
	@printf "## This is a project made within Seoul National University of Science and Technology \n" > $(START).sh
	@printf "## Embedded Systems Laboratory 2018 - Raimarius Tolentino Delgado \n\n" >> $(START).sh
	@printf "## Start-up for dynamically linked executable file \n\n\n\n" >> $(START).sh
ifeq ($(RT_DOMAIN),xenomai)
	@printf "export LD_LIBRARY_PATH=$(XENOMAI_PATH)/lib \n" >> $(START).sh
endif
	@printf "./$(OUT_DIR)/$(EXEC_TARGET) \$$1 \$$2 \$$3 \$$4 \$$5 \$$6\n" >> $(START).sh


# 	@printf "./$(OUT_DIR)/$(EXEC_TARGET) \$$1 \$$2 \$$3 \$$4 \$$5 \$$6\n" >> $(START).sh


$(OUT_DIR)/$(EXEC_TARGET): $(OBJECTS)
	@$(MKDIR) -p $(OUT_DIR); pwd > /dev/null
	$(CC) -o $(OUT_DIR)/$(EXEC_TARGET) $(OBJECTS) $(LDFLAGS)

$(OBJ_DIR)/%.o : %.cpp
	@$(MKDIR) -p $(OBJ_DIR); pwd > /dev/null
	$(CC) -MD $(CFLAGS) -c -o $@ $<

$(OBJ_DIR)/%.o : %.c
	@$(MKDIR) -p $(OBJ_DIR); pwd > /dev/null
	$(CC) -MD $(CFLAGS) -c -o $@ $<

clean:
	$(RM) -rf \
		$(OBJ_DIR)/* \
		$(OBJ_DIR)   \
			$(OUT_DIR)/* \
			$(OUT_DIR)   \
			*.dat	     \
		$(START)*
re:
	@touch ./* $(INC_EMBD)/src/* 
	make clean
	make 


.PHONY: all clean 
#######################################################################################################
# Include header file dependencies generated by -MD option:
-include $(OBJ_DIR_CUR)/*.d


