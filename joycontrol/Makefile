NAME = joycontrol
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

OBJS := $(OBJ_DIR)/joycontrol.o $(OBJ_DIR)/joystick.o

JOYCONTROL_LIBS = -lm -L../../lib -lbee -ltcx -lpthread


##
## Compulsory rules
##

all:
	$(MAKE) export $(DEPEND)
	$(MAKE) build

build: $(OBJ_DIR) \
	$(BIN_DIR) \
	$(BIN_DIR)/joycontrol

export:
	$(ECHO) "    ---- Installing header files"
	$(ECHO) "    ---- Installing parameter files"


clean:
	$(RM) *.o *.a *~ core a.out obj obj_sun obj_solaris $(CLEAN_FILES)

distclean: clean
	$(RM) $(DEPEND) Makefile.depend* joycontrol \
		$(NAME)-src.tgz $(NAME)-bin.tgz $(DISTCLEAN_FILES)

##
## local rules
##

$(OBJ_DIR) $(LIB_DIR) $(BIN_DIR):
	$(SILENT) $(MKDIR) $@

$(BIN_DIR)/joycontrol: $(OBJ_DIR) $(BIN_DIR) $(OBJS)
	$(ECHO) "    ---- Creating $@"
	$(SILENT) $(CC) $(INCLUDES) $(LIB_DIRS) $(CFLAGS) -o $@ \
		$(OBJS) $(JOYCONTROL_LIBS)
	ln -fs $(BIN_DIR)/joycontrol joycontrol

##
## Standard suffix, version, packaging and dep rules
##

include $(RULES)
