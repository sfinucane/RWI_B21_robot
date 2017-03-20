NAME = laserServer2
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

CFLAGS      += -Wall -O3		# for development purposes

#CFLAGS      += -DBEE_INI

#CFLAGS      += -DVERBOSE

#CFLAGS      += -DTCX_DEBUG -DLASER_DEBUG

##
## local build variables
##

OBJS := $(OBJ_DIR)/mainlaser.o \
	$(OBJ_DIR)/io.o \
	$(OBJ_DIR)/laserHandlers.o 

DISPLAYLASER_OBJS := $(OBJ_DIR)/displayLaser.o

LASER_LIBS = -ltcx -lrobot -lbUtils $(OS_LIBS) -lm 

DISPLAYLASER_LIBS = -lezx -lX11 -ltcx -lrobot $(OS_LIBS) -lm

##
## Compulsory rules
##

all:
	$(MAKE) export $(DEPEND)
	$(MAKE) build

build: $(OBJ_DIR) \
	$(BIN_DIR) \
	$(LIB_DIR) \
	$(BIN_DIR)/laserServer2 \
	$(BIN_DIR)/displayLaser

export:
	$(INSTALL_H) `pwd`/laserClient.h $(INC_DIR)
	$(INSTALL_H) `pwd`/LASER_SERVER-messages.h $(INC_DIR)

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

$(BIN_DIR)/laserServer2: $(OBJS) $(LIB_DIR)/librobot.a $(LIB_DIR)/libbUtils.a\
	$(LIB_DIR)/libtcx.a $(LIB_DIR)/libezx.a
	$(ECHO) "    ->>- Creating $@"
	$(SILENT) $(CC) $(INCLUDES) $(LIB_DIRS) $(CFLAGS) -o $@ \
		$(OBJS) $(LASER_LIBS)

$(BIN_DIR)/displayLaser: $(OBJ_DIR) $(BIN_DIR) $(DISPLAYLASER_OBJS)
	$(ECHO) "    ---- Creating $@"
	$(SILENT) $(CC) $(INCLUDES) $(LIB_DIRS) $(CFLAGS) -o $@ \
		$(DISPLAYLASER_OBJS) $(DISPLAYLASER_LIBS)
##
## Standard suffix, version, packaging and dep rules
##

include $(RULES)
