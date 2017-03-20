NAME = head
CONF  = ../Makefile.conf
RULES = ../Makefile.rules

##
## System config variables
##

include $(CONF)
$BIN_DIR := .
##
## Variables
##

DEPENDENCIES += *.c

#CFLAGS += -DB18

#CFLAGS += -DGNUPLOT


##
## local build variables
##

OBJS1 := $(OBJ_DIR)/headServer.o $(OBJ_DIR)/sock.o $(OBJ_DIR)/serial.o $(OBJ_DIR)/face.o
OBJS2 := $(OBJ_DIR)/headClient.o $(OBJ_DIR)/sock.o
OBJS3 := $(OBJ_DIR)/headTest.o $(OBJ_DIR)/headClient.o $(OBJ_DIR)/sock.o


HEAD_LIBS = -lpthread -lm


##
## Compulsory rules
##

all:
	$(MAKE) export $(DEPEND)
	$(MAKE) build

build: $(OBJ_DIR) \
	$(BIN_DIR) \
	$(LIB_DIR) \
	$(BIN_DIR)/headServer \
	$(BIN_DIR)/headTest \
	$(LIB_DIR)/libheadClient.a 


export:
	$(ECHO) "    ---- Installing header files"
	$(SILENT) $(MKDIR) $(INC_DIR)
	$(INSTALL_H) `pwd`/headClient.h $(INC_DIR)



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

$(BIN_DIR)/headServer: $(OBJS1)
	$(ECHO) "    ---- Creating $@"
	$(SILENT) $(CC) $(INCLUDES) $(LIB_DIRS) $(CFLAGS) -o $@ \
		$(OBJS1) $(HEAD_LIBS)

$(BIN_DIR)/headTest: $(OBJS3)
	$(ECHO) "    ---- Creating $@"
	$(SILENT) $(CC) $(INCLUDES) $(LIB_DIRS) $(CFLAGS) -o $@ \
		$(OBJS3) $(HEAD_LIBS)

$(LIB_DIR)/libheadClient.a: $(OBJS2)
	$(ECHO) "    ---- Creating $@"
	$(SILENT) $(AR) $@ $(OBJS2)
	$(SILENT) $(RANLIB) $@


##
## Standard suffix, version, packaging and dep rules
##

include $(RULES)


