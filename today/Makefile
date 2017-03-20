NAME = today
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

CFLAGS      += -Wall		# for development purposes

##
## local build variables
##

OBJS := $(OBJ_DIR)/today.o 


TODAY_LIBS = -lheadClient -lrobot -lbUtils -ltcx -lezx -lX11 $(OS_LIBS) -lm 





##
## Compulsory rules
##

all:
	$(MAKE) export $(DEPEND)
	$(MAKE) build

build:  $(OBJ_DIR) \
	$(BIN_DIR) \
	$(BIN_DIR)/today 


export:

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

$(BIN_DIR)/today: $(OBJS) $(LIB_DIR)/librobot.a $(LIB_DIR)/libbUtils.a\
	$(LIB_DIR)/libtcx.a $(LIB_DIR)/libezx.a
	$(ECHO) "    ->>- Creating $@"
	$(SILENT) $(CC) $(INCLUDES) $(LIB_DIRS) $(CFLAGS) -o $@ \
		$(OBJS) $(TODAY_LIBS)
	ln -fs $(BIN_DIR)/today today


##
## Standard suffix, version, packaging and dep rules
##

include $(RULES)
