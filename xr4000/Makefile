NAME = xr4000
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

OBJS := $(OBJ_DIR)/xr4000.o $(OBJ_DIR)/bee_interface.o $(OBJ_DIR)/robot_arg.o

XR4000_LIBS = -lezx -lX11 -lrobot -ltcx $(OS_LIBS) -lbUtils -lm 


##
## Compulsory rules
##

all:
	$(MAKE) export $(DEPEND)
	$(MAKE) build

build: $(OBJ_DIR) \
	$(BIN_DIR) \
	$(BIN_DIR)/xr4000

export:


clean:
	$(RM) *.o *~ core a.out obj obj_sun obj_solaris $(CLEAN_FILES)

distclean: clean
	$(RM) $(DEPEND) Makefile.depend* xr4000 \
		$(NAME)-src.tgz $(NAME)-bin.tgz $(DISTCLEAN_FILES)

##
## local rules
##

$(OBJ_DIR) $(LIB_DIR) $(BIN_DIR):
	$(SILENT) $(MKDIR) $@

$(BIN_DIR)/xr4000: $(OBJS)
	$(ECHO) "    ---- Creating $@"
	$(SILENT) $(CC) $(INCLUDES) $(LIB_DIRS) $(CFLAGS) -o $@ \
		$(OBJS) Nhost_client.a $(XR4000_LIBS)
	ln -fs $(BIN_DIR)/xr4000 xr4000

##
## Standard suffix, version, packaging and dep rules
##

include $(RULES)
