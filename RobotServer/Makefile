NAME = RobotServer
CONF  = ../Makefile.conf
RULES = ../Makefile.rules

##
## System config variables
##

include $(CONF)

##
## Variables
##

DEPENDENCIES += *.c

##
## local build variables
##

OBJS := $(OBJ_DIR)/RobotServer_query_example.o 
OBJS2 := $(OBJ_DIR)/RobotServer.o

#CFLAGS += -DTOTAL_debug

LIBS   += -ltcx $(OS_LIBS)

##
## Compulsory rules
##

all:
	$(MAKE) export $(DEPEND)
	$(MAKE) build

build: $(OBJ_DIR) \
	$(BIN_DIR) \
	$(LIB_DIR) \
	$(BIN_DIR)/RobotServer_query_example \
	$(BIN_DIR)/RobotServer

export:
	$(ECHO) "    ---- Installing header files"
	$(SILENT) $(MKDIR) $(INC_DIR)
	$(INSTALL_H) `pwd`/RobotServer-messages.h $(INC_DIR)

clean:
	$(RM) *.o *.a *~ core a.out obj obj_sun obj_solaris $(CLEAN_FILES)

distclean: clean
	$(RM) $(DEPEND) Makefile.depend* \
		$(NAME)-src.tgz $(NAME)-bin.tgz $(DISTCLEAN_FILES)


##
## local rules
##

$(OBJ_DIR) $(LIB_DIR) $(BIN_DIR):
	$(SILENT) $(MKDIR) $@

$(BIN_DIR)/RobotServer: $(OBJS2)
	$(ECHO) "    ---- Creating $@"
	$(SILENT) $(CC) -O3 -W -Wall -pedantic $(INCLUDES)  \
	$(CFLAGS) -o $@ $(OBJS2) $(LIB_DIRS) $(LIBS)

RobotServer.o:RobotServer.c
	$(ECHO) "    ---- Compiling $@"
	$(SILENT) $(CC) -O3 -W -Wall -pedantic $(INCLUDES) -c RobotServer.c \
	$(CFLAGS)  -o $@ $(LIB_DIRS) $(LIBS) \

$(BIN_DIR)/RobotServer_query_example: $(OBJS)
	$(ECHO) "    ---- Creating $@"
	$(SILENT) $(CC) -O3 -W -Wall -pedantic $(INCLUDES)  \
	$(CFLAGS) -o $@ $(OBJS) $(LIB_DIRS) $(LIBS)

RobotServer_query_example.o:RobotServer_query_example.c
	$(ECHO) "    ---- Compiling $@"
	$(SILENT) $(CC) -O3 -W -Wall -pedantic $(INCLUDES) -c RobotServer_query_example.c \
	$(CFLAGS)  -o $@ $(LIB_DIRS) $(LIBS) \
	  

##
## Standard suffix, version, packaging and dep rules
##

include $(RULES)
