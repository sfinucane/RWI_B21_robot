NAME = collgraph
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

OBJS := $(OBJ_DIR)/COLLGRAPH.o

COLLGRAPH_LIBS = -lezx -lX11 -ltcx -lrobot $(OS_LIBS) -lbUtils -lm


##
## Compulsory rules
##

all:
	$(MAKE) export $(DEPEND)
	$(MAKE) build

build: $(OBJ_DIR) \
	$(BIN_DIR) \
	$(BIN_DIR)/COLLGRAPH

export:
	$(ECHO) "    ---- Installing header files"

#install:
#	$(ECHO) "    ---- Installing bins"
#	$(SILENT) $(MKDIR) $(BIN_DIR)
#	$(INSTALL_EXEC) $(OBJ_DIR)/COLLGRAPH $(BIN_DIR)

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

$(BIN_DIR)/COLLGRAPH: $(OBJ_DIR) $(BIN_DIR) $(OBJS)
	$(ECHO) "    ---- Creating $@"
	$(SILENT) $(CC) $(INCLUDES) $(LIB_DIRS) $(CFLAGS) -o $@ \
		$(OBJS) $(COLLGRAPH_LIBS)

##
## Standard suffix, version, packaging and dep rules
##

include $(RULES)
