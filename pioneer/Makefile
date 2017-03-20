NAME = pioneer
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
#CFLAGS += -DTOTAL_debug

##
## local build variables
##

OBJS := $(OBJ_DIR)/pioneer.o $(OBJ_DIR)/bee_interface.o 

PIONEER_LIBS = -lezx -lX11 -lrobot -ltcx $(OS_LIBS) -lbUtils -lm 


##
## Compulsory rules
##

all:
	$(MAKE) export $(DEPEND)
	$(MAKE) build

build: $(OBJ_DIR) \
	$(BIN_DIR) \
	$(BIN_DIR)/pioneer

export:


clean:
	$(RM) *.o *.a *~ core a.out $(OBJ_DIR) $(CLEAN_FILES)

distclean: clean
	$(RM) $(DEPEND) pioneer \
		$(NAME)-src.tgz $(NAME)-bin.tgz $(DISTCLEAN_FILES)

##
## local rules
##

$(OBJ_DIR) $(LIB_DIR) $(BIN_DIR):
	$(SILENT) $(MKDIR) $@

$(BIN_DIR)/pioneer: $(OBJS)
	$(ECHO) "    ---- Creating $@"
	$(SILENT) $(CC) $(INCLUDES) $(LIB_DIRS) $(CFLAGS) -o $@ \
		$(OBJS) $(PIONEER_LIBS)

##
## Standard suffix, version, packaging and dep rules
##

include $(RULES)
