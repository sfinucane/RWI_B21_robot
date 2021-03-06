NAME = laserint
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

OBJS := $(OBJ_DIR)/LASERINT.o $(OBJ_DIR)/file.o $(OBJ_DIR)/pos.o \
	$(OBJ_DIR)/graphics.o $(OBJ_DIR)/handlers.o $(OBJ_DIR)/init.o \
	$(OBJ_DIR)/objects.o $(OBJ_DIR)/image.o

LASERINT_LIBS = -lezx -lX11 -ltcx -lrobot -lcorr $(OS_LIBS) -lbUtils -lgd -lm


##
## Compulsory rules
##

all:
	$(MAKE) export $(DEPEND)
	$(MAKE) build

build: $(OBJ_DIR) \
	$(BIN_DIR) \
	$(BIN_DIR)/laserint

export:
	$(ECHO) "    ---- Installing header files"
	$(ECHO) "    ---- Installing parameter files"
	$(SILENT) $(MKDIR) $(ETC_DIR)
	$(INSTALL_ETC) `pwd`/laserint.ini $(ETC_DIR)


clean:
	$(RM) *.o *.a *~ core a.out obj obj_sun obj_solaris $(CLEAN_FILES)

distclean: clean
	$(RM) $(DEPEND) Makefile.depend* laserint \
		$(NAME)-src.tgz $(NAME)-bin.tgz $(DISTCLEAN_FILES)

##
## local rules
##

$(OBJ_DIR) $(LIB_DIR) $(BIN_DIR):
	$(SILENT) $(MKDIR) $@

$(BIN_DIR)/laserint: $(OBJ_DIR) $(BIN_DIR) $(OBJS)
	$(ECHO) "    ---- Creating $@"
	$(SILENT) $(CC) $(INCLUDES) $(LIB_DIRS) $(CFLAGS) -o $@ \
		$(OBJS) $(LASERINT_LIBS)
	ln -fs $(BIN_DIR)/laserint laserint

##
## Standard suffix, version, packaging and dep rules
##

include $(RULES)
