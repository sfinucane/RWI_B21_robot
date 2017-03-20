NAME = face
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

OBJS := $(OBJ_DIR)/face_interface.o $(OBJ_DIR)/FACE.o
OBJS2 := $(OBJ_DIR)/testface.o

FACE_LIBS = -lezx -lX11 -lm -lFaceClient -lraiClient -lrai -lrobot $(OS_LIBS) -lbUtils -ltcx 

##
## Compulsory rules
##

all:
	$(MAKE) export $(DEPEND)
	$(MAKE) build

build: $(OBJ_DIR) \
	$(BIN_DIR) \
	$(LIB_DIR) \
	$(LIB_DIR)/libFaceClient.a \
	$(BIN_DIR)/faceServer \
	testface

export:
	$(ECHO) "    ---- Installing header files"
	$(SILENT) $(MKDIR) $(INC_DIR)
	$(INSTALL_H) `pwd`/FACE-messages.h $(INC_DIR)
	$(INSTALL_H) `pwd`/faceClient.h $(INC_DIR)

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

$(BIN_DIR)/faceServer: $(OBJS)
	$(ECHO) "    ---- Creating $@"
	$(SILENT) $(CC) $(INCLUDES) $(LIB_DIRS) $(CFLAGS) -o $@ \
		$(OBJS) $(FACE_LIBS)

testface: $(OBJS2)
	$(ECHO) "    ---- Creating $@"
	$(SILENT) $(CC) $(INCLUDES) $(LIB_DIRS) $(CFLAGS) -o $@ \
		$(OBJS2) $(FACE_LIBS)

$(LIB_DIR)/libFaceClient.a: $(OBJ_DIR)/faceClient.o
	$(ECHO) "    ---- Creating $@"
	$(SILENT) $(AR) $@ $(OBJ_DIR)/faceClient.o
	$(SILENT) $(RANLIB) $@

##
## Standard suffix, version, packaging and dep rules
##

include $(RULES)
