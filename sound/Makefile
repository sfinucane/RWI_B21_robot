NAME = sound
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

OBJS = $(OBJ_DIR)/wavplay.o \
	$(OBJ_DIR)/cdhardware.o \
	$(OBJ_DIR)/speech.o \
	$(OBJ_DIR)/mixer.o \
	$(OBJ_DIR)/file.o \
	$(OBJ_DIR)/tcx.o \
	$(OBJ_DIR)/handler.o \
	$(OBJ_DIR)/main.o

ifeq ($(OS_NAME),SunOS)
SOUND_LIBS = -ltcx -lm $(OS_LIBS)
else
SOUND_LIBS = -ltcx -lwav -lm $(OS_LIBS)
endif

CLEAN_FILES += soundtest

##
## Compulsory rules
##

all:
	$(MAKE) export $(DEPEND)
	$(MAKE) build

build: $(OBJ_DIR) \
	$(LIB_DIR) \
	$(LIB_DIR)/libwav.a \
	$(BIN_DIR) \
	$(BIN_DIR)/SOUND

export:
	$(ECHO) "    ---- Installing header files"
	$(SILENT) $(MKDIR) $(INC_DIR)
	$(INSTALL_H) `pwd`/wavlib.h $(INC_DIR)
	$(INSTALL_H) `pwd`/SOUND-messages.h $(INC_DIR)
	$(SILENT) $(MKDIR) $(ETC_DIR)
	$(ECHO) "    ---- Installing parameter files"
	$(INSTALL_H) `pwd`/sounds.dat $(ETC_DIR)

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

$(LIB_DIR)/libwav.a: $(OBJ_DIR) $(LIB_DIR) $(OBJ_DIR)/wavlib.o
	$(ECHO) "    ---- Creating $@"
	$(SILENT) $(AR) $@ $(OBJ_DIR)/wavlib.o
	$(SILENT) $(RANLIB) $@

$(BIN_DIR)/SOUND: $(OBJ_DIR) $(BIN_DIR) $(OBJS)
	$(ECHO) "    ---- Creating $@"
	$(SILENT) $(CC) $(INCLUDES) $(LIB_DIRS) $(CFLAGS) -o $@ \
		$(OBJS) $(SOUND_LIBS)

soundtest: $(OBJ_DIR) $(BIN_DIR) $(OBJ_DIR)/soundtest.o
	$(ECHO) "    ---- Creating $@"
	$(SILENT) $(CC) $(INCLUDES) $(LIB_DIRS) $(CFLAGS) -o $@ \
		$(OBJ_DIR)/soundtest.o $(SOUND_LIBS)

cdfind: $(OBJ_DIR) $(BIN_DIR) $(OBJ_DIR)/cdfind.o
	$(ECHO) "    ---- Creating $@"
	$(SILENT) $(CC) $(INCLUDES) $(LIB_DIRS) $(CFLAGS) -o $@ \
		$(OBJ_DIR)/cdfind.o
##
## Standard suffix, version, packaging and dep rules
##

include $(RULES)
