NAME = maploc
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

OBJS1 := $(OBJ_DIR)/main.o 
OBJS3 := $(OBJ_DIR)/main3d.o $(OBJ_DIR)/data.o
OBJS4 := $(OBJ_DIR)/main4d.o $(OBJ_DIR)/data.o



MAPLOC_LIBS = -ldat -lezx -lX11 -lrobot $(OS_LIBS) -lbaseClient -lcameraClient -lbuttonClient -lpantiltClient -lraiClient -lrai  -ltcx -lbUtils -lm



##
## Compulsory rules
##

all:
	$(MAKE) export $(DEPEND)
	$(MAKE) build

build: $(OBJ_DIR) \
	$(BIN_DIR) \
	$(BIN_DIR)/maploc \
	$(BIN_DIR)/maploc3d \
	$(BIN_DIR)/maploc4d

export:
	$(ECHO) "    ---- Installing header files"
	$(ECHO) "    ---- Installing parameter files"



clean:
	$(RM) *.o *.a *~ core a.out obj obj_sun obj_solaris $(CLEAN_FILES)

distclean: clean
	$(RM) $(DEPEND) Makefile.depend* maploc \
		$(NAME)-src.tgz $(NAME)-bin.tgz $(DISTCLEAN_FILES)





##
## local rules
##

$(OBJ_DIR) $(LIB_DIR) $(BIN_DIR):
	$(SILENT) $(MKDIR) $@

$(BIN_DIR)/maploc: $(OBJS1)
	$(ECHO) "    ---- Creating $@"
	$(SILENT) $(CC) $(INCLUDES) $(LIB_DIRS) $(CFLAGS) -o $@ \
		$(OBJS1) $(MAPLOC_LIBS) 
	ln -fs $(BIN_DIR)/maploc maploc


$(BIN_DIR)/maploc3d: $(OBJS3)
	$(ECHO) "    ---- Creating $@"
	$(SILENT) $(CC) $(INCLUDES) $(LIB_DIRS) $(CFLAGS) -o $@ \
		$(OBJS3) $(MAPLOC_LIBS) 
	ln -fs $(BIN_DIR)/maploc3d maploc3d

$(BIN_DIR)/maploc4d: $(OBJS4)
	$(ECHO) "    ---- Creating $@"
	$(SILENT) $(CC) $(INCLUDES) $(LIB_DIRS) $(CFLAGS) -o $@ \
		$(OBJS4) $(MAPLOC_LIBS) 
	ln -fs $(BIN_DIR)/maploc4d maploc4d


##
## Standard suffix, version, packaging and dep rules
##

include $(RULES)
