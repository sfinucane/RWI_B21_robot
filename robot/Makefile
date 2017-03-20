NAME = robot
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

OBJS := $(OBJ_DIR)/LIB-math.o $(OBJ_DIR)/block3.o $(OBJ_DIR)/gettimeofday.o \
	$(OBJ_DIR)/ualarm.o $(OBJ_DIR)/Learn.o $(OBJ_DIR)/devUtils.o \
	$(OBJ_DIR)/handlers.o $(OBJ_DIR)/sample.o $(OBJ_DIR)/librobot.o

OBJS2 := $(OBJ_DIR)/corr.o 

OBJS3 := $(OBJ_DIR)/libsof.o 

##
## Compulsory rules
##

CFLAGS += -DOS_MINOR_VER=$(OS_MINOR_VER) -DOS_VER=$(OS_VER)

all:
	echo $(OS_NAME)
	echo $(OS_MINOR_VER)
	$(MAKE) export $(DEPEND)
	$(MAKE) build

build: $(OBJ_DIR) $(LIB_DIR) \
	$(LIB_DIR)/librobot.a \
	$(LIB_DIR)/libcorr.a \
	$(LIB_DIR)/libsof.a

export:
	$(ECHO) "    ---- Installing header files"
	$(SILENT) $(MKDIR) $(INC_DIR)
	$(INSTALL_H) `pwd`/corr.h $(INC_DIR)
	$(INSTALL_H) `pwd`/Common.h $(INC_DIR)
	$(INSTALL_H) `pwd`/LIB-math.h $(INC_DIR)
	$(INSTALL_H) `pwd`/devNames.h $(INC_DIR)
	$(INSTALL_H) `pwd`/libc.h $(INC_DIR)
	$(INSTALL_H) `pwd`/robot_specifications.h $(INC_DIR)
	$(INSTALL_H) `pwd`/vectors.h $(INC_DIR)
	$(INSTALL_H) `pwd`/Net.h $(INC_DIR)
	$(INSTALL_H) `pwd`/Application.h $(INC_DIR)
	$(INSTALL_H) `pwd`/devUtils.h $(INC_DIR)
	$(INSTALL_H) `pwd`/handlers.h $(INC_DIR)
	$(INSTALL_H) `pwd`/struct.h $(INC_DIR)
	$(INSTALL_H) `pwd`/workbone.h $(INC_DIR)
	$(INSTALL_H) `pwd`/librobot.h $(INC_DIR)
	$(INSTALL_H) `pwd`/libsof.h $(INC_DIR)


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

$(OBJ_DIR) $(LIB_DIR) $(BIN_DIR):
	$(SILENT) $(MKDIR) $@

$(LIB_DIR)/librobot.a: $(OBJS)
	$(ECHO) "    ---- Creating $@"
	$(SILENT) $(AR) $@ $(OBJS)
	$(SILENT) $(RANLIB) $@


$(LIB_DIR)/libcorr.a: $(OBJS2)
	$(ECHO) "    ---- Creating $@"
	$(SILENT) $(AR) $@ $(OBJS2)
	$(SILENT) $(RANLIB) $@

$(LIB_DIR)/libsof.a: $(OBJS3)
	$(ECHO) "    ---- Creating $@"
	$(SILENT) $(AR) $@ $(OBJS3)
	$(SILENT) $(RANLIB) $@

##
## Standard suffix, version, packaging and dep rules
##

include $(RULES)
