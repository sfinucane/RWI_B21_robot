NAME = reaction

CC = gcc

BEE_HOMES = . .. `echo $(BEEHOME)/src` `echo ~brudy/bee/src`

FFIND := \
	LIST="$(BEE_HOMES)" ; \
	for i in $$LIST ; do \
		if [ -f $$i/$$FILE ] ; then \
			echo $$i/$$FILE; \
			exit 0; \
		fi; \
	done; \
	exit 1

CONF  = $(shell FILE=Makefile.conf  ; $(FFIND))
RULES = $(shell FILE=Makefile.rules ; $(FFIND))

##
## System config variables
##

include $(CONF)

##
## Variables
##

TARGETS += reaction
#TARGETS += colliTest reaction3

DEPENDENCIES += *.c
DISTCLEAN_FILES += $(TARGETS)

##
## local build variables
##

LIBS += -lrobot
LIBS += -lbuttonClient
LIBS += -lraiClient -lrai  -ltcx -lbUtils -lm $(OS_LIBS)

##
## Compulsory rules
##

all:
	$(MAKE) export $(DEPEND)
	$(MAKE) build

build: $(OBJ_DIR) $(TARGETS)

export:

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

$(TARGETS): %: $(OBJ_DIR)/%.o
	$(ECHO) "    ---- Creating $@"
#	$(SILENT) $(CC) -o $@ $(CFLAGS) $(OBJ_DIR)/$@.o $(LIB_DIRS) $(LIBS)
	$(CC) -o $@ $(CFLAGS) $(OBJ_DIR)/$@.o $(LIB_DIRS) $(LIBS)

##
## Standard suffix, version, packaging and dep rules
##

include $(RULES)
