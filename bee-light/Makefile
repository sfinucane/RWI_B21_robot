NAME = BEESOFTAPI
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

CPP = gcc
##
## local build variables
##

##
## Compulsory rules
##

CFLAGS += -DOS_MINOR_VER=$(OS_MINOR_VER) -DOS_VER=$(OS_VER)

all:
	$(MAKE) export $(DEPEND)
	$(MAKE) build

build: $(OBJ_DIR) $(LIB_DIR) \
	$(LIB_DIR)/libbee.a

export:
	$(ECHO) "    ---- Installing header files"
	$(SILENT) $(MKDIR) $(INC_DIR)
	$(INSTALL_H) `pwd`/bee.h $(INC_DIR)


#install:
#	$(ECHO) "    ---- Installing libs"
#	$(SILENT) $(MKDIR) $(LIB_DIR)
#	$(INSTALL_LIB) $(OBJ_DIR)/librobot.a $(LIB_DIR)

clean:
	$(RM) *.o *.a *~ core a.out obj obj_sun obj_solaris $(CLEAN_FILES)

distclean: clean
	$(RM) $(DEPEND) Makefile.depend* \
		$(NAME)-src.tgz $(NAME)-bin.tgz $(DISTCLEAN_FILES)

##
## local rules
##

bee.o: bee.c
	$(ECHO) "    ---- Creating $@"
	$(SILENT) $(CPP) $(INCLUDES) -c bee.c
	$(SILENT) mv bee.o obj

$(OBJ_DIR) $(LIB_DIR) $(BIN_DIR):
	$(SILENT) $(MKDIR) $@

$(LIB_DIR)/libbee.a: bee.o
	$(ECHO) "    ---- Creating $@"
	$(SILENT) $(AR) $@ obj/bee.o
	$(SILENT) $(RANLIB) $@


##
## Standard suffix, version, packaging and dep rules
##

include $(RULES)
