NAME = learn
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

OBJS := $(OBJ_DIR)/main.o $(OBJ_DIR)/vars.o $(OBJ_DIR)/fa.o $(OBJ_DIR)/appl.o

LIB_OBJS1 := $(OBJ_DIR)/dat.o 
LIB_OBJS2 := $(OBJ_DIR)/mem.o $(OBJ_DIR)/graphics.o $(OBJ_DIR)/io.o \
	$(OBJ_DIR)/global.o 



LEARN_LIBS = -ldat -llearn -lezx -lX11 -lrobot $(OS_LIBS) -lbaseClient -lcameraClient -lbuttonClient -lpantiltClient -lraiClient -lrai  -ltcx -lbUtils -lm



##
## Compulsory rules
##

all:
	$(MAKE) export $(DEPEND)
	$(MAKE) build

build: $(OBJ_DIR) \
	$(BIN_DIR) \
	$(LIB_DIR)/libdat.a \
	$(LIB_DIR)/liblearn.a \
	$(BIN_DIR)/learn

export:
	$(ECHO) "    ---- Installing header files"
	$(SILENT) $(MKDIR) $(INC_DIR)
	$(INSTALL_H) `pwd`/dat.h $(INC_DIR)
	$(ECHO) "    ---- Installing parameter files"


clean:
	$(RM) *.o *.a *~ core a.out obj obj_sun obj_solaris $(CLEAN_FILES)

distclean: clean
	$(RM) $(DEPEND) Makefile.depend* learn $(LIB_DIR)/libdat.a \
	$(LIB_DIR)/liblearn.a \
	$(NAME)-src.tgz $(NAME)-bin.tgz $(DISTCLEAN_FILES)

##
## local rules
##

$(OBJ_DIR) $(LIB_DIR) $(BIN_DIR):
	$(SILENT) $(MKDIR) $@


$(LIB_DIR)/libdat.a: $(LIB_DIR) $(LIB_OBJS1)
	$(ECHO) "    ---- Creating $@"
	$(SILENT) $(AR) $@ $(LIB_OBJS1)
	$(SILENT) $(RANLIB) $@


$(LIB_DIR)/liblearn.a: $(LIB_DIR) $(LIB_OBJS2)
	$(ECHO) "    ---- Creating $@"
	$(SILENT) $(AR) $@ $(LIB_OBJS2)
	$(SILENT) $(RANLIB) $@


$(BIN_DIR)/learn: $(OBJ_DIR) $(BIN_DIR) $(OBJS)
	$(ECHO) "    ---- Creating $@"
	$(SILENT) $(CC) $(INCLUDES) $(LIB_DIRS) $(CFLAGS) -o $@ \
		$(OBJS) $(LEARN_LIBS) 
	ln -fs $(BIN_DIR)/learn learn


##
## Standard suffix, version, packaging and dep rules
##

include $(RULES)
