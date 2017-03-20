NAME = param2map
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

PARAM2MAP_OBJS := $(OBJ_DIR)/PARAM2MAP.o
SIM2MAP_OBJS := $(OBJ_DIR)/SIM2MAP.o
SIM2FIG_OBJS := $(OBJ_DIR)/SIM2FIG.o
MAP2FIG_OBJS := $(OBJ_DIR)/MAP2FIG.o
MAP2SIM_OBJS := $(OBJ_DIR)/MAP2SIM.o
SAMPLES2FIG_OBJS := $(OBJ_DIR)/SAMPLES2FIG.o
SIM2PNM_OBJS := $(OBJ_DIR)/SIM2PNM.o
SIM2GIF_OBJS := $(OBJ_DIR)/SIM2GIF.o
SIM2OBSTSIM_OBJS := $(OBJ_DIR)/SIM2OBSTSIM.o
DISPLAYMAP_OBJS := $(OBJ_DIR)/DISPLAYMAP.o
MAP2GIF_OBJS := $(OBJ_DIR)/MAP2GIF.o
SCAN2MAP_OBJS := $(OBJ_DIR)/SCAN2MAP.o
SCRIPT2NEW_OBJS := $(OBJ_DIR)/SCRIPT2NEW.o
SOF2SIM_OBJS := $(OBJ_DIR)/SOF2SIM.o
PIC2MAP_OBJS := $(OBJ_DIR)/PIC2MAP.o

PARAM2MAP_LIBS = -lezx -lX11 -ltcx -lrobot $(OS_LIBS) -lm
SIM2MAP_LIBS = -lezx -lX11 -ltcx -lrobot $(OS_LIBS) -lm
SIM2FIG_LIBS = $(OS_LIBS) -lm
MAP2FIG_LIBS = $(OS_LIBS) -lm
MAP2SIM_LIBS = $(OS_LIBS) -lm
SAMPLES2FIG_LIBS = $(OS_LIBS) -lm
SIM2PNM_LIBS = $(OS_LIBS) -lm
SIM2GIF_LIBS = $(OS_LIBS) -lgd -lm
SIM2OBSTSIM_LIBS = $(OS_LIBS) -lm
DISPLAYMAP_LIBS = -lezx -lX11 -ltcx -lrobot $(OS_LIBS) -lm
MAP2GIF_LIBS = $(OS_LIBS) -lgd -lm
SCAN2MAP_LIBS = $(OS_LIBS) -lm
SCRIPT2NEW_LIBS = $(OS_LIBS) -lm
SOF2SIM_LIBS = $(OS_LIBS) -lsof -lm
PIC2MAP_LIBS = $(OS_LIBS) -lX11 -lXext -ljpeg -lpng -ltiff -lz -lgif -lm -lImlib

##
## Compulsory rules
##

all:
	$(MAKE) export $(DEPEND)
	$(MAKE) build

build: $(OBJ_DIR) \
	$(BIN_DIR) \
	$(BIN_DIR)/MAP2SIM \
	$(BIN_DIR)/PARAM2MAP \
	$(BIN_DIR)/SIM2MAP \
	$(BIN_DIR)/SIM2FIG \
	$(BIN_DIR)/MAP2FIG \
	$(BIN_DIR)/SAMPLES2FIG \
	$(BIN_DIR)/SIM2PNM \
	$(BIN_DIR)/SIM2GIF \
	$(BIN_DIR)/SIM2OBSTSIM \
	$(BIN_DIR)/DISPLAYMAP \
	$(BIN_DIR)/MAP2GIF \
        $(BIN_DIR)/SCAN2MAP \
        $(BIN_DIR)/SCRIPT2NEW \
        $(BIN_DIR)/SOF2SIM

pic2map: $(BIN_DIR)/PIC2MAP

export:
	$(ECHO) "    ---- Installing header files"

#install:
#	$(ECHO) "    ---- Installing bins"
#	$(SILENT) $(MKDIR) $(BIN_DIR)
#	$(INSTALL_EXEC) $(OBJ_DIR)/PARAM2MAP $(BIN_DIR)

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

$(BIN_DIR)/MAP2SIM: $(OBJ_DIR) $(BIN_DIR) $(MAP2SIM_OBJS)
	$(ECHO) "    ---- Creating $@"
	$(SILENT) $(CC) $(INCLUDES) $(LIB_DIRS) $(CFLAGS) -o $@ \
		$(MAP2SIM_OBJS) $(MAP2SIM_LIBS)
$(BIN_DIR)/PARAM2MAP: $(OBJ_DIR) $(BIN_DIR) $(PARAM2MAP_OBJS)
	$(ECHO) "    ---- Creating $@"
	$(SILENT) $(CC) $(INCLUDES) $(LIB_DIRS) $(CFLAGS) -o $@ \
		$(PARAM2MAP_OBJS) $(PARAM2MAP_LIBS)
$(BIN_DIR)/SIM2MAP: $(OBJ_DIR) $(BIN_DIR) $(SIM2MAP_OBJS)
	$(ECHO) "    ---- Creating $@"
	$(SILENT) $(CC) $(INCLUDES) $(LIB_DIRS) $(CFLAGS) -o $@ \
		$(SIM2MAP_OBJS) $(SIM2MAP_LIBS)
$(BIN_DIR)/SIM2FIG: $(OBJ_DIR) $(BIN_DIR) $(SIM2FIG_OBJS)
	$(ECHO) "    ---- Creating $@"
	$(SILENT) $(CC) $(INCLUDES) $(LIB_DIRS) $(CFLAGS) -lm -o $@ \
		$(SIM2FIG_OBJS) $(SIM2FIG_LIBS)
$(BIN_DIR)/MAP2FIG: $(OBJ_DIR) $(BIN_DIR) $(MAP2FIG_OBJS)
	$(ECHO) "    ---- Creating $@"
	$(SILENT) $(CC) $(INCLUDES) $(LIB_DIRS) $(CFLAGS) -lm -o $@ \
		$(MAP2FIG_OBJS) $(MAP2FIG_LIBS)
$(BIN_DIR)/SAMPLES2FIG: $(OBJ_DIR) $(BIN_DIR) $(SAMPLES2FIG_OBJS)
	$(ECHO) "    ---- Creating $@"
	$(SILENT) $(CC) $(INCLUDES) $(LIB_DIRS) $(CFLAGS) -lm -o $@ \
		$(SAMPLES2FIG_OBJS) $(SAMPLES2FIG_LIBS)
$(BIN_DIR)/SIM2PNM: $(OBJ_DIR) $(BIN_DIR) $(SIM2PNM_OBJS)
	$(ECHO) "    ---- Creating $@"
	$(SILENT) $(CC) $(INCLUDES) $(LIB_DIRS) $(CFLAGS) -o $@ \
		$(SIM2PNM_OBJS) $(SIM2PNM_LIBS)
$(BIN_DIR)/SIM2GIF: $(OBJ_DIR) $(BIN_DIR) $(SIM2GIF_OBJS)
	$(ECHO) "    ---- Creating $@"
	$(SILENT) $(CC) $(INCLUDES) $(LIB_DIRS) $(CFLAGS) -o $@ \
		$(SIM2GIF_OBJS) $(SIM2GIF_LIBS)
$(BIN_DIR)/SIM2OBSTSIM: $(OBJ_DIR) $(BIN_DIR) $(SIM2OBSTSIM_OBJS)
	$(ECHO) "    ---- Creating $@"
	$(SILENT) $(CC) $(INCLUDES) $(LIB_DIRS) $(CFLAGS) -o $@ \
		$(SIM2OBSTSIM_OBJS) $(SIM2OBSTSIM_LIBS)
$(BIN_DIR)/DISPLAYMAP: $(OBJ_DIR) $(BIN_DIR) $(DISPLAYMAP_OBJS)
	$(ECHO) "    ---- Creating $@"
	$(SILENT) $(CC) $(INCLUDES) $(LIB_DIRS) $(CFLAGS) -o $@ \
		$(DISPLAYMAP_OBJS) $(DISPLAYMAP_LIBS)
$(BIN_DIR)/MAP2GIF: $(OBJ_DIR) $(BIN_DIR) $(MAP2GIF_OBJS)
	$(ECHO) "    ---- Creating $@"
	$(SILENT) $(CC) $(INCLUDES) $(LIB_DIRS) $(CFLAGS) -o $@ \
		$(MAP2GIF_OBJS) $(MAP2GIF_LIBS)
$(BIN_DIR)/SCAN2MAP: $(OBJ_DIR) $(BIN_DIR) $(SCAN2MAP_OBJS)
	$(ECHO) "    ---- Creating $@"
	$(SILENT) $(CC) $(INCLUDES) $(LIB_DIRS) $(CFLAGS) -o $@ \
		$(SCAN2MAP_OBJS) $(SCAN2MAP_LIBS)
$(BIN_DIR)/SCRIPT2NEW: $(OBJ_DIR) $(BIN_DIR) $(SCRIPT2NEW_OBJS)
	$(ECHO) "    ---- Creating $@"
	$(SILENT) $(CC) $(INCLUDES) $(LIB_DIRS) $(CFLAGS) -o $@ \
		$(SCRIPT2NEW_OBJS) $(SRCIPT2NEW_LIBS)
$(BIN_DIR)/PIC2MAP: $(OBJ_DIR) $(BIN_DIR) $(PIC2MAP_OBJS)
	$(ECHO) "    ---- Creating $@"
	$(SILENT) $(CC) $(INCLUDES) $(LIB_DIRS) $(CFLAGS) -o $@ \
		$(PIC2MAP_OBJS) $(PIC2MAP_LIBS)
$(BIN_DIR)/SOF2SIM: $(OBJ_DIR) $(BIN_DIR) $(SOF2SIM_OBJS)
	$(ECHO) "    ---- Creating $@"
	$(SILENT) $(CC) $(INCLUDES) $(LIB_DIRS) $(CFLAGS) -o $@ \
		$(SOF2SIM_OBJS) $(SOF2SIM_LIBS)


##
## Standard suffix, version, packaging and dep rules
##

include $(RULES)





