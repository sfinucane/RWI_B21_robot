NAME = robotDetection
CONF  = ../Makefile.conf
RULES = ../Makefile.rules

##
## System config variables
##

include $(CONF)

##
## Variables
##

CFLAGS      += -DTCX_debug
CFLAGS      += -DDEBUG_CAMERA	# debugging info for the frame grabbers

CFLAGS      += -DG_DEBUG_PERFORMANCE # define, if you want performance output

CFLAGS      += -Wall		# print all possible errors during compiling

CFLAGS      += -DDO_DA_BEEP	# define, if the PC should beep for each saved frame

CFLAGS      += -DDO_DA_COUNTER	# define, if you want framenumber while saving

CFLAGS      += -DLINUX # for color quickcam library stuff.

CFLAGS      += -DHAVE_MATROX_METEOR
CFLAGS      += -DHAVE_COLOR_QUICKCAM

DEPENDENCIES += *.c *.cc

CXX = g++
CXXFLAGS = $(CFLAGS)

SILENT = 
OPT_FLAGS   = -g -O2


##
##
ROBOTDETECT_LIBS     = -lcameraClient -lraiClient -llaserClient \
		  -lrai -lbUtils -ltcx $(OS_LIBS) -lezx -lm -lX11
ROBOTDETECT_OBJS     = $(OBJ_DIR)/display.o \
		  $(OBJ_DIR)/functions.o \
		  $(OBJ_DIR)/colorFilter.o \
                  $(OBJ_DIR)/robotDetect.o


FILTERDEMO_OBJS = $(OBJ_DIR)/colorFilter.o


##
## Compulsory rules
##

all:
	$(MAKE) export $(DEPEND)
	$(MAKE) build


build:  $(OBJ_DIR) \
	$(BIN_DIR) \
	$(LIB_DIR) 


#	$(BIN_DIR)/filterDemo


export:
	$(ECHO) "    ---- Installing header files"
	$(SILENT) $(MKDIR) $(INC_DIR)
	$(INSTALL_H) `pwd`/ROBOT_DETECTION-messages.h $(INC_DIR)

clean:
	$(RM) *.o *.a *~ core a.out obj obj_sun obj_solaris $(CLEAN_FILES)

# make clean is intended to leave a smaller but working system.
# use 'make distclean' to delete all generated files.
# 'make distclean' deletes the lib, bin, etc. dirs completely.
#


distclean: clean
	$(RM) $(DEPEND) Makefile.depend* \
		$(NAME)-src.tgz $(NAME)-bin.tgz $(DISTCLEAN_FILES)

##
## local rules
##

$(OBJ_DIR) $(LIB_DIR) $(BIN_DIR):
	$(SILENT) $(MKDIR) $@


$(BIN_DIR)/robotDetect: $(ROBOTDETECT_OBJS) $(LIB_DIR)/libcameraClient.a \
	$(LIB_DIR)/libraiClient.a $(LIB_DIR)/librai.a $(LIB_DIR)/libbUtils.a \
	$(LIB_DIR)/libtcx.a $(LIB_DIR)/libezx.a
	$(ECHO) "    ->>- Creating $@"
	$(SILENT) $(CC) -o $@ $(ROBOTDETECT_OBJS) $(LIB_DIRS) $(ROBOTDETECT_LIBS) $(INCLUDES) 


#$(BIN_DIR)/filterDemo: $(FILTERDEMO_OBJS)
#	$(ECHO) "    ->>- Creating $@"
#	$(SILENT) $(CC) -o $@ $(FILTERDEMO_OBJS) $(LIB_DIRS) $(INCLUDES) 
#
##
## Standard suffix, version, packaging and dep rules
##

include $(RULES)
