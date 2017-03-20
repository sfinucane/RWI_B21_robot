NAME = rawData
CONF  = ../../Makefile.conf
RULES = ../../Makefile.rules

##
## System config variables
##

include $(CONF)

##
## Variables
##

DEPENDENCIES += *.cc *.h *.c

##
## local build variables
##

# some systems need declarations for missing compiler declarations
ifeq ($(OS_TAG),_sun)
NO_GNU_BUT_SUN_OR_SOLARIS_SYSTEM = -DNO_GNU_BUT_SUN_OR_SOLARIS_SYSTEM -DD4MCD
else
ifeq ($(OS_TAG),_solaris)
NO_GNU_BUT_SUN_OR_SOLARIS_SYSTEM = -DNO_GNU_BUT_SUN_OR_SOLARIS_SYSTEM -DD4MCD
endif
endif

OBJS =  $(OBJ_DIR)/rawData.o \
	$(OBJ_DIR)/scriptData.o \
	$(OBJ_DIR)/channelData.o \
	$(OBJ_DIR)/testRawData.o \
	$(OBJ_DIR)/localizeTestRawData.o \
	$(OBJ_DIR)/cTestRawData.o

LIB_OBJS = $(OBJ_DIR)/rawData.o $(OBJ_DIR)/scriptData.o \
	$(OBJ_DIR)/channelData.o 

LIBS += -lrawData $(OS_LIBS) -lm 
# -ltcx

CFLAGS += -pedantic -Wall $(NO_GNU_BUT_SUN_OR_SOLARIS_SYSTEM)


CLEAN_FILES += Makefile.depend*

GLOBAL_LIB_DIR = ../$(PREFIX)/lib$(OS_TAG)$(POSTFIX)
GLOBAL_BIN_DIR = ../$(BIN_DIR)
GLOBAL_INC_DIR = ../$(INC_DIR)


$(INCLUDES) += -I$(GLOBAL_INC_DIR)

##
## Compulsory rules
##

all:
	echo $(GLOBAL_INC_DIR)
	$(MAKE) export rawdepend
	$(MAKE) build

rawdepend:
ifeq ($(DEPENDENCIES),)
else
	$(ECHO) "    ---- Analysing dependencies for $(DEPENDENCIES)"
	$(SILENT) $(CPP) -M $(CFLAGS) $(INCLUDES) -I$(GLOBAL_INC_DIR) \
	$(DEPENDENCIES)  | \
	$(AWK) '{if (index($$0,".o:") > 0) printf "$(OBJ_DIR)/"; print $0}' \
	> $(DEPEND)
endif

build:  $(OBJ_DIR) \
	$(OBJS) \
	$(GLOBAL_LIB_DIR) \
	$(GLOBAL_LIB_DIR)/librawData.a \
	testRawData \
	localizeTestRawData \
	cTestRawData 

export:
	$(ECHO) "    ---- Installing header files"
	$(SILENT) $(MKDIR) $(GLOBAL_INC_DIR)
	$(INSTALL_H) `pwd`/rawData.hh $(GLOBAL_INC_DIR)
	$(INSTALL_H) `pwd`/typedefRawData.h $(GLOBAL_INC_DIR)
	$(INSTALL_H) `pwd`/scriptData.hh $(GLOBAL_INC_DIR)
	$(INSTALL_H) `pwd`/channelData.h $(GLOBAL_INC_DIR)
	$(INSTALL_H) `pwd`/constDef.h $(GLOBAL_INC_DIR)

clean:
	$(RM) *.o *.a *~ core a.out obj obj_sun obj_solaris $(CLEAN_FILES) \
	testRawData localizeTestRawData cTestRawData

tags:
	$(SILENT) $(ETAGS) *.hh *.cc *.icc
	$(SILENT) (cd ../ ; $(ETAGS) --append $(NAME)/*.hh \
	$(NAME)/*.cc *.icc)
	$(SILENT) (cd ../../ ; $(ETAGS) --append $(NAME)/*.hh \
	$(NAME)/*.cc *.icc)

distclean: clean
	$(RM) $(DEPEND) Makefile.depend* \
		$(NAME)-src.tgz $(NAME)-bin.tgz $(DISTCLEAN_FILES)

##
## local rules
##

$(OBJ_DIR) $(GLOBAL_BIN_DIR) $(GLOBAL_LIB_DIR):
	$(SILENT) $(MKDIR) $@

librawData: $(GLOBAL_LIB_DIR)/librawData.a

$(GLOBAL_LIB_DIR)/librawData.a: $(OBJ_DIR) $(GLOBAL_LIB_DIR) $(LIB_OBJS)
	$(ECHO) "    ---- Creating $@ (LIB)"
	$(SILENT) $(AR) $@ $(LIB_OBJS)
	$(SILENT) $(RANLIB) $@

testRawData: $(OBJ_DIR) librawData \
			$(OBJ_DIR)/testRawData.o
	$(ECHO) "    ---- Creating $@ (C++)"
	$(SILENT) $(CXX) $(INCLUDES) -L$(GLOBAL_LIB_DIR) $(LIB_DIRS) $(CFLAGS) -o $@ \
			$(OBJ_DIR)/testRawData.o $(LIBS)

localizeTestRawData: $(OBJ_DIR) librawData \
			$(OBJ_DIR)/localizeTestRawData.o
	$(ECHO) "    ---- Creating $@ (C++)"
	$(SILENT) $(CXX) $(INCLUDES) -L$(GLOBAL_LIB_DIR) $(LIB_DIRS) $(CFLAGS) -o $@ \
			$(OBJ_DIR)/localizeTestRawData.o $(LIBS)

cTestRawData: $(OBJ_DIR) librawData \
			$(OBJ_DIR)/cTestRawData.o
	$(ECHO) "    ---- Creating $@ (C)"
	$(SILENT) $(CC) $(INCLUDES) -L$(GLOBAL_LIB_DIR) $(LIB_DIRS) $(CFLAGS) -o $@ \
			$(OBJ_DIR)/cTestRawData.o $(LIBS) -lg++ -lstdc++


##
## Standard suffix, version, packaging and dep rules
##

$(OBJ_DIR)/%.o %.o: %.cc
	$(ECHO) "    ---- Compiling $< (C++)"
	$(SILENT) $(CC) $(CFLAGS) $(INCLUDES) -I$(GLOBAL_INC_DIR) -c -o $@ $<

$(OBJ_DIR)/%.o %.o: %.c
	$(ECHO) "    ---- Compiling $< (C)"
	$(SILENT) $(CC) $(CFLAGS) $(INCLUDES) -I$(GLOBAL_INC_DIR) -c -o $@ $<

include $(RULES)


