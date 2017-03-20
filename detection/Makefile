NAME = detection
CONF  = ../Makefile.conf
RULES = ../Makefile.rules

##
## System config variables
##

include $(CONF)
OBJ_DIR = .

##
## Variables
##

DEPENDENCIES += *.c
#CFLAGS    += -Wall -pedantic -g
CFLAGS    +=  -Wall -g


##
## local build variables
##

OBJS := $(OBJ_DIR)/laser.o $(OBJ_DIR)/detection.o graphics.o pantilt.o \
	$(OBJ_DIR)/function.o $(OBJ_DIR)/inline.o $(OBJ_DIR)/allocate.o \
	$(OBJ_DIR)/localTcx.o $(OBJ_DIR)/stuck.o $(OBJ_DIR)/human.o

DETECTION_LIBS = -lrobot -lezx -lX11 -ltcx -llocalize $(OS_LIBS) -lbUtils -lm -lcorr


##
## Compulsory rules
##

all:
	$(MAKE) export $(DEPEND)
	$(MAKE) build

build: $(OBJ_DIR) \
	$(BIN_DIR) \
	$(BIN_DIR)/DETECTION 

export:
	$(ECHO) "    ---- Installing header files"
	$(SILENT) $(MKDIR) $(INC_DIR)
	$(INSTALL_H) `pwd`/DETECTION-messages.h $(INC_DIR)


#install:
#	$(ECHO) "    ---- Installing bins"
#	$(SILENT) $(MKDIR) $(BIN_DIR)
#	$(INSTALL_EXEC) $(OBJ_DIR)/DETECTION $(BIN_DIR)

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


$(BIN_DIR)/DETECTION: $(OBJ_DIR) $(BIN_DIR) $(OBJS) $(OBJ_DIR)/DETECTION.o
	$(ECHO) "    ---- Creating $@"
	$(SILENT) $(CC) $(INCLUDES) $(LIB_DIRS) $(CFLAGS) -o $@ \
		$(OBJS) $(OBJ_DIR)/DETECTION.o $(DETECTION_LIBS)



##
## Standard suffix, version, packaging and dep rules
##

include $(RULES)













