NAME = plan
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

# CFLAGS += -DFORCE_GOAL_ORDER 

##
## local build variables
##

OBJS := $(OBJ_DIR)/PLAN.o $(OBJ_DIR)/file.o $(OBJ_DIR)/file2.o \
	$(OBJ_DIR)/handlers.o $(OBJ_DIR)/goals.o $(OBJ_DIR)/graphics.o

PLAN_LIBS = -lezx -lX11 -ltcx -lrobot $(OS_LIBS) -lbUtils -lcorr -lm

##
## Compulsory rules
##

all:
	$(MAKE) export $(DEPEND)
	$(MAKE) build

build: $(OBJ_DIR) \
	$(BIN_DIR) \
	$(BIN_DIR)/plan

export:
	$(ECHO) "    ---- Installing header files"
	$(SILENT) $(MKDIR) $(INC_DIR)
	$(INSTALL_H) `pwd`/PLAN-messages.h $(INC_DIR)
	$(ECHO) "    ---- Installing parameter files"
	$(SILENT) $(MKDIR) $(ETC_DIR)
	$(INSTALL_ETC) `pwd`/plan.ini $(ETC_DIR)


clean:
	$(RM) *.o *.a *~ core a.out obj obj_sun obj_solaris $(CLEAN_FILES)

distclean: clean
	$(RM) $(DEPEND) Makefile.depend* plan \
		$(NAME)-src.tgz $(NAME)-bin.tgz $(DISTCLEAN_FILES)

##
## local rules
##

$(OBJ_DIR) $(LIB_DIR) $(BIN_DIR):
	$(SILENT) $(MKDIR) $@

$(BIN_DIR)/plan: $(OBJS)
	$(ECHO) "    ---- Creating $@"
	$(SILENT) $(CC) $(INCLUDES) $(LIB_DIRS) $(CFLAGS) -o $@ \
		$(OBJS) $(PLAN_LIBS)
	ln -fs $(BIN_DIR)/plan plan

##
## Standard suffix, version, packaging and dep rules
##

include $(RULES)
