NAME = distanceServer
CONF  = ../Makefile.conf
RULES = ../Makefile.rules

USE_PTHREADS = FALSE
USE_EGCS = FALSE

include $(CONF)

##
## Variables
##

DEPENDENCIES += *.c *.cc

ifeq ($(OS_NAME),SunOS)

ifeq ($(CC),cc)

## Flags for SUN compiler

CFLAGS += -fast -xtarget=ultra
CXXFLAGS += -fast -xtarget=ultra -features=rtti
#CFLAGS += -g
#CXXFLAGS += -g -features=rtti
USER_DEFINES +=

ifeq ($(USE_PTHREADS),TRUE)
CXXFLAGS += -mt
endif

LIB_DIRS += -L/opt/VisualWS/SUNWspro/lib
INCLUDES += -I../../include/STL/SUNpro
# SUN CC vs. GCC 
else

## assume GNU compiler
CFLAGS += -g
CXXFLAGS += -g  -fpermissive -frtti
USER_DEFINES +=
ifeq ($(USE_EGCS), TRUE)
INCLUDES +=
CXXFLAGS += -fguiding-decls
else
INCLUDES += -I../../include/STL/gnu
endif
OS_LIBS += -lstdc++
endif

INCLUDES += -I/usr/openwin/include

ifeq ($(USE_PTHREADS),TRUE)
OS_LIBS += -lpthread -lthread -lposix4
endif

# assume LINUX 	
else

CFLAGS += -g
CXXFLAGS += -g -frtti -fguiding-decls
OS_LIBS += -lstdc++
ifeq ($(USE_EGCS), TRUE)
INCLUDES +=
CXXFLAGS += -fguiding-decls
else
INCLUDES += -I../../include/STL/gnu
endif


USER_DEFINES += -DFUNCPROTO=15
USER_DEFINES += -DNARROWPROTO
USER_DEFINES += -DLINUX

ifeq ($(USE_PTHREADS),TRUE)
OS_LIBS += -lpthread
endif

endif

# for all Operation systems

USER_DEFINES += -D_GNU_SOURCE -DX_LOCALE -DB21

LIB_DIRS +=  -L$(LIB_DIR) -L$(HOME)/bee/$(BEE_LIB)

##
## local build variables
##

ifeq ($(USE_PTHREADS), TRUE)
USER_DEFINES += -DUSE_PTHREADS -D_REENTRANT
endif

CFLAGS += $(USER_DEFINES)
CXXFLAGS += $(USER_DEFINES)

STORE_OBJS  = $(OBJ_DIR)/surface.o \
	        $(OBJ_DIR)/vector.o \
		$(OBJ_DIR)/linalg.o \
		$(OBJ_DIR)/obstacles.o \
		$(OBJ_DIR)/store.o \
		$(OBJ_DIR)/map_reader.o \
		$(OBJ_DIR)/human.o \
		$(OBJ_DIR)/circle.o \
		$(OBJ_DIR)/door.o \
		$(OBJ_DIR)/rectangle.o \
	        $(OBJ_DIR)/camera.o \
		$(OBJ_DIR)/obstacleset.o \
	        $(OBJ_DIR)/options.o \
		$(OBJ_DIR)/sim_rtp.o \
		$(OBJ_DIR)/md5.o \
		$(OBJ_DIR)/netlink.o \
		$(OBJ_DIR)/typeInfo.o

ifeq ($(USE_PTHREADS),TRUE)
STORE_OBJS += $(OBJ_DIR)/pthreads_schedule.o
else
STORE_OBJS += $(OBJ_DIR)/schedule.o
endif

LIB_STORE = $(LIB_DIR)/libstore.a

FAKE_OBJS = $(OBJ_DIR)/fakeSensors.o


##
## Compulsory rules
##

all:
	$(ECHO) "    ---- EXPORT ---"
	$(MAKE) export $(DEPEND)
	$(ECHO) "    ---- BUILD ---"
	$(MAKE) build

build:
	$(MAKE) $(OBJ_DIR)
	$(MAKE) $(LIB_DIR)/libstore.a
	$(MAKE) GetDistanceTest

export:
	$(ECHO) "    ---- Installing header files"
	$(SILENT) $(MKDIR) $(ETC_DIR)
	$(SILENT) $(MKDIR) $(INC_DIR)
	$(SILENT) $(MKDIR) $(INC_DIR)/RTL
	$(INSTALL_H) `pwd`/store.hh $(INC_DIR)
	$(INSTALL_H) `pwd`/store.hh $(INC_DIR)/RTL
	$(INSTALL_H) `pwd`/fakeSensors.hh $(INC_DIR)
	$(INSTALL_H) `pwd`/camera.hh $(INC_DIR)/RTL
	$(INSTALL_H) `pwd`/circle.hh $(INC_DIR)/RTL
	$(INSTALL_H) `pwd`/rectangle.hh $(INC_DIR)/RTL
	$(INSTALL_H) `pwd`/door.hh $(INC_DIR)/RTL
	$(INSTALL_H) `pwd`/human.hh $(INC_DIR)/RTL
	$(INSTALL_H) `pwd`/interface.hh $(INC_DIR)
	$(INSTALL_H) `pwd`/interface.hh $(INC_DIR)/RTL
	$(INSTALL_H) `pwd`/mapreader.hh $(INC_DIR)
	$(INSTALL_H) `pwd`/mapreader.hh $(INC_DIR)/RTL
	$(INSTALL_H) `pwd`/netlink.hh $(INC_DIR)
	$(INSTALL_H) `pwd`/netlink.hh $(INC_DIR)/RTL
	$(INSTALL_H) `pwd`/obstacles.hh $(INC_DIR)
	$(INSTALL_H) `pwd`/obstacles.hh $(INC_DIR)/RTL
	$(INSTALL_H) `pwd`/schedule.hh $(INC_DIR)
	$(INSTALL_H) `pwd`/schedule.hh $(INC_DIR)/RTL
	$(INSTALL_H) `pwd`/pthreads_schedule.hh $(INC_DIR)/RTL
	$(INSTALL_H) `pwd`/locking.hh $(INC_DIR)/RTL
	$(INSTALL_H) `pwd`/surface.hh $(INC_DIR)
	$(INSTALL_H) `pwd`/surface.hh $(INC_DIR)/RTL
	$(INSTALL_H) `pwd`/vector.hh $(INC_DIR)
	$(INSTALL_H) `pwd`/vector.hh $(INC_DIR)/RTL
	$(INSTALL_H) `pwd`/simrtp.hh $(INC_DIR)
	$(INSTALL_H) `pwd`/simrtp.hh $(INC_DIR)/RTL
	$(INSTALL_H) `pwd`/rtp.h $(INC_DIR)
	$(INSTALL_H) `pwd`/rtp.h $(INC_DIR)/RTL
	$(INSTALL_H) `pwd`/typeInfo.hh $(INC_DIR)
	$(INSTALL_H) `pwd`/typeInfo.hh $(INC_DIR)/RTL
	$(INSTALL_H) `pwd`/options.hh $(INC_DIR)/RTL
	$(INSTALL_H) `pwd`/polygons.hh $(INC_DIR)/RTL
	$(INSTALL_H) `pwd`/frameBuffer.hh $(INC_DIR)/RTL

#
# local rules
#

$(OBJ_DIR) $(BIN_DIR) $(LIB_DIR):
	$(ECHO) "making dir $@"
	$(MKDIR) $@

$(LIB_STORE): $(OBJ_DIR) $(STORE_OBJS)
	$(ECHO) "    ---- Creating $@"
	$(SILENT) $(AR) $@ $(STORE_OBJS)
	$(SILENT) $(RANLIB) $@

$(LIB_DIR)/libGetDistance.a: $(OBJ_DIR) $(LIB_DIR) $(LIB_STORE) $(FAKE_OBJS)
	$(ECHO) "    ---- Creating $@"
	$(SILENT) $(CP) $(LIB_STORE) $@ 
	$(SILENT) $(AR) $@ $(FAKE_OBJS)
	$(SILENT) $(RANLIB) $@

GetDistanceTest: $(OBJ_DIR) $(LIB_DIR) \
	         $(LIB_DIR)/libGetDistance.a $(OBJ_DIR)/getdisttest.o
	$(SILENT) $(CXX) $(INCLUDES) $(LIB_DIRS) -o $@ \
		        $(OBJ_DIR)/getdisttest.o  -lGetDistance $(OS_LIBS) -lm

clean:
	$(RM) Templates.DB *.o *.a *~ core a.out obj obj_sun4 obj_solaris $(CLEAN_FILES)

distclean: clean
	$(RM) $(DEPEND) Makefile.depend* GetDistanceTest \
		$(NAME)-src.tgz $(NAME)-bin.tgz $(DISTCLEAN_FILES)

##
## Standard suffix, version, packaging and dep rules
##

include $(RULES)
