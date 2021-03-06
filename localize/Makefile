NAME = localize
CONF  = ../Makefile.conf
RULES = ../Makefile.rules

##############################################################################
#                                         HOW TO ENABLE _NEW_ SCRIPT SUPPORT #
#                                                                            #
# if you want the new script                                                 #
# set envirnmet variable as follows and run make                             #
# setenv USE_NEW_SCRIPT_LIBRARY TRUE                                         #
# or set this variable at this point in Makefile (see next line)             #
##############################################################################

#USE_NEW_SCRIPT_LIBRARY=TRUE

##############################################################################


##
## System config variables
##

include $(CONF)
OBJ_DIR = .
#BIN_DIR = .

##
## Variables
##

CC = gcc
DEPENDENCIES += *.c *.h
CFLAGS    += -Wall -ansi -pedantic  -g
CFLAGS    += -DONLINE_MAPPING -DLOCALIZE_MAKE 

##
## local build variables
##

ifeq ($(USE_NEW_SCRIPT_LIBRARY),TRUE)
OBJS := $(OBJ_DIR)/sonar.o $(OBJ_DIR)/laser.o $(OBJ_DIR)/proximity.o \
	$(OBJ_DIR)/proximityTools.o $(OBJ_DIR)/map.o $(OBJ_DIR)/probGrid.o \
	$(OBJ_DIR)/function.o $(OBJ_DIR)/inline.o $(OBJ_DIR)/allocate.o \
	$(OBJ_DIR)/movement.o $(OBJ_DIR)/angle.o $(OBJ_DIR)/fileNames.o \
	$(OBJ_DIR)/angleTools.o $(OBJ_DIR)/communication.o \
	$(OBJ_DIR)/abstract.o \
	$(OBJ_DIR)/localTcx.o $(OBJ_DIR)/sensings.o \
	$(OBJ_DIR)/file.o $(OBJ_DIR)/correction.o $(OBJ_DIR)/probGridTools.o \
	$(OBJ_DIR)/graphic.o $(OBJ_DIR)/selection.o \
	$(OBJ_DIR)/dynamic.o $(OBJ_DIR)/planTcx.o $(OBJ_DIR)/colliTcx.o \
	$(OBJ_DIR)/activeLocalize.o  $(OBJ_DIR)/activeAutomat.o \
	$(OBJ_DIR)/question.o 	$(OBJ_DIR)/vision.o $(OBJ_DIR)/stack.o \
	$(OBJ_DIR)/condensation.o \
	$(OBJ_DIR)/scanAlignment.o



LOCALIZE_LIBS = -lcorr -llocalize -ltcx -lrobot $(OS_LIBS) \
		-lbUtils -lm -lezx -lgd -lX11
SCRIPT_LIB = -lrawData -lg++ -lstdc++
else
OBJS := $(OBJ_DIR)/sonar.o $(OBJ_DIR)/laser.o $(OBJ_DIR)/proximity.o \
	$(OBJ_DIR)/proximityTools.o $(OBJ_DIR)/map.o $(OBJ_DIR)/probGrid.o \
	$(OBJ_DIR)/function.o $(OBJ_DIR)/inline.o $(OBJ_DIR)/allocate.o \
	$(OBJ_DIR)/movement.o $(OBJ_DIR)/angle.o $(OBJ_DIR)/fileNames.o \
	$(OBJ_DIR)/angleTools.o $(OBJ_DIR)/communication.o \
	$(OBJ_DIR)/abstract.o \
	$(OBJ_DIR)/script.o $(OBJ_DIR)/localTcx.o $(OBJ_DIR)/sensings.o \
	$(OBJ_DIR)/file.o $(OBJ_DIR)/correction.o $(OBJ_DIR)/probGridTools.o \
	$(OBJ_DIR)/graphic.o $(OBJ_DIR)/selection.o \
	$(OBJ_DIR)/dynamic.o $(OBJ_DIR)/planTcx.o $(OBJ_DIR)/colliTcx.o \
	$(OBJ_DIR)/activeLocalize.o  $(OBJ_DIR)/activeAutomat.o \
	$(OBJ_DIR)/question.o 	$(OBJ_DIR)/vision.o \
	$(OBJ_DIR)/condensation.o \
	$(OBJ_DIR)/scanAlignment.o

CFLAGS += -DDO_NOT_USE_NEW_SCRIPT_LIBRARY

LOCALIZE_LIBS = -lcorr -llocalize -ltcx -lrobot $(OS_LIBS) \
		-lbUtils -lm -lezx -lgd -lX11
SCRIPT_LIB = -lstdc++

endif

USE_ZLIB=TRUE

ifeq ($(USE_ZLIB),TRUE)
CFLAGS += -DUSE_ZLIB
LOCALIZE_LIBS += -lz
endif

##
## Compulsory rules
##

all:
	$(MAKE) build

build: $(OBJ_DIR) \
	$(BIN_DIR) \
	$(LIB_DIR)/liblocalize.a \
	$(BIN_DIR)/LOCALIZE \
	$(BIN_DIR)/LOCAL_COLORS \
	$(BIN_DIR)/SET_ROBOT \
	$(BIN_DIR)/poslibtcx \
	$(BIN_DIR)/DISPLAY_LOCALIZE
#	RAW_DATA 

export: exportRawData
	$(ECHO) "    ---- Installing header files"
	$(SILENT) $(MKDIR) $(INC_DIR)
	$(INSTALL_H) `pwd`/LOCALIZE-messages.h $(INC_DIR)
	$(INSTALL_H) `pwd`/localize.h $(INC_DIR)
	$(INSTALL_H) `pwd`/liblocalize.h $(INC_DIR)
	$(INSTALL_H) `pwd`/sensings.h $(INC_DIR)
	$(INSTALL_H) `pwd`/correction.h $(INC_DIR)
	$(INSTALL_H) `pwd`/probGridTools.h $(INC_DIR)
#	$(INSTALL_H) `pwd`/general.h $(INC_DIR)


tags: tagsRawData
	$(SILENT) $(ETAGS) *.h *.c
	$(SILENT) (cd ../ ; $(ETAGS) --append $(NAME)/*.h \
	$(NAME)/*.c)


#install:
#	$(ECHO) "    ---- Installing bins"
#	$(SILENT) $(MKDIR) $(BIN_DIR)
#	$(INSTALL_EXEC) $(OBJ_DIR)/LOCALIZE $(BIN_DIR)

clean: cleanRawData
	$(RM) *.o *.a *~ core a.out obj obj_sun obj_solaris $(CLEAN_FILES) \
		plotRun robotDump


distclean: clean
	$(RM) $(DEPEND) Makefile.depend* \
		$(NAME)-src.tgz $(NAME)-bin.tgz $(DISTCLEAN_FILES)

##
## local rules
##

$(OBJ_DIR) $(LIB_DIR) $(BIN_DIR):
	$(SILENT) $(MKDIR) $@

$(LIB_DIR)/liblocalize.a: $(OBJ_DIR) $(LIB_DIR) localLib.o $(OBJ_DIR)/liblocalize.o
	$(ECHO) "    ---- Creating $@"
	$(SILENT) $(AR) $@ localLib.o $(OBJ_DIR)/liblocalize.o
	$(SILENT) $(RANLIB) $@

$(BIN_DIR)/LOCALIZE: $(OBJ_DIR) $(BIN_DIR)  $(OBJ_DIR)/LOCALIZE.o $(OBJS)
	$(ECHO) "    ---- Creating $@" 
	$(CC) $(INCLUDES) $(LIB_DIRS) $(CFLAGS) -o $@ \
	$(OBJ_DIR)/LOCALIZE.o $(OBJS) $(LOCALIZE_LIBS) $(SCRIPT_LIB)

addNoise: $(OBJ_DIR) $(BIN_DIR) $(OBJS) $(OBJ_DIR)/addNoise.o
	$(ECHO) "    ---- Creating $@" 
	$(SILENT) $(CC) $(INCLUDES) $(LIB_DIRS) $(CFLAGS) -o $(BIN_DIR)/$@ \
	$(OBJS) $(OBJ_DIR)/addNoise.o $(LOCALIZE_LIBS) $(SCRIPT_LIB)


$(BIN_DIR)/LOCAL_COLORS: $(OBJ_DIR) $(BIN_DIR) $(OBJS) $(OBJ_DIR)/showColors.o
	$(ECHO) "    ---- Creating $@"
	$(SILENT) $(CC) $(INCLUDES) $(LIB_DIRS) $(CFLAGS) -o $@ \
		$(OBJS) $(OBJ_DIR)/showColors.o $(LOCALIZE_LIBS) -lstdc++

$(BIN_DIR)/SET_ROBOT: $(BIN_DIR) $(OBJ_DIR)/setRobot.o 
	$(ECHO) "    ---- Creating $@"
	$(SILENT) $(CC) $(INCLUDES) $(LIB_DIRS) $(CFLAGS) -o $@ \
		$(OBJS) $(OBJ_DIR)/setRobot.o $(LOCALIZE_LIBS) $(SCRIPT_LIB) -lstdc++

control: $(BIN_DIR) $(OBJ_DIR)/controlLocalize.o 
	$(ECHO) "    ---- Creating $@"
	$(SILENT) $(CC) $(INCLUDES) $(LIB_DIRS) $(CFLAGS) -o $(BIN_DIR)/controlLocalize \
		$(OBJS) $(OBJ_DIR)/controlLocalize.o $(LOCALIZE_LIBS) 

plotRun: $(OBJ_DIR)/allocate.o $(OBJ_DIR)/plotRun.o \
	$(OBJ_DIR)/fileNames.o $(OBJ_DIR)/probGridTools.o
	$(SILENT) $(CC) $(INCLUDES) $(LIB_DIRS) $(CFLAGS) -o $@ \
		$(OBJ_DIR)/inline.o $(OBJ_DIR)/file.o $(OBJ_DIR)/fileNames.o \
		$(OBJ_DIR)/allocate.o \
		$(OBJ_DIR)/plotRun.o $(LOCALIZE_LIBS)

correctScript: $(OBJ_DIR)/correctScript.o
	$(SILENT) $(CC) $(INCLUDES) $(LIB_DIRS) $(CFLAGS) -o $@ \
		$(OBJ_DIR)/correctScript.o $(LOCALIZE_LIBS)

gnuplot2Fig: $(OBJ_DIR)/function.o $(OBJ_DIR)/allocate.o $(OBJ_DIR)/gnuplot2Fig.o \
	$(OBJ_DIR)/fileNames.o
	$(SILENT) $(CC) $(INCLUDES) $(LIB_DIRS) $(CFLAGS) -o $(BIN_DIR)/$@ \
		$(OBJ_DIR)/inline.o $(OBJ_DIR)/fileNames.o \
		$(OBJ_DIR)/allocate.o \
		$(OBJ_DIR)/gnuplot2Fig.o $(LOCALIZE_LIBS)

bayes: 	$(OBJ_DIR)/bayes1Dim.o
	$(SILENT) $(CC) $(INCLUDES) $(LIB_DIRS) $(CFLAGS) -o $@ \
		$(OBJ_DIR)/inline.o $(OBJ_DIR)/bayes1Dim.o $(LOCALIZE_LIBS)

evaluate: $(OBJ_DIR)/evaluate.o $(OBJ_DIR)/inline.o $(OBJ_DIR)/evaluationTools.o 
	$(SILENT) $(CC) $(INCLUDES) $(LIB_DIRS) $(CFLAGS) -o $(BIN_DIR)/$@ \
		$(OBJ_DIR)/evaluate.o  $(OBJ_DIR)/inline.o  \
		$(OBJ_DIR)/evaluationTools.o $(LOCALIZE_LIBS)

two-sample-t: $(OBJ_DIR)/two-sample-t.o $(OBJ_DIR)/inline.o $(OBJ_DIR)/evaluationTools.o 
	$(SILENT) $(CC) $(INCLUDES) $(LIB_DIRS) $(CFLAGS) -o $(BIN_DIR)/$@ \
		$(OBJ_DIR)/two-sample-t.o  $(OBJ_DIR)/inline.o  \
		$(OBJ_DIR)/evaluationTools.o $(LOCALIZE_LIBS)

mergePos: $(OBJ_DIR)/mergePos.o $(OBJ_DIR)/inline.o $(OBJ_DIR)/evaluationTools.o
	$(SILENT) $(CC) $(INCLUDES) $(LIB_DIRS) $(CFLAGS) -o $(BIN_DIR)/$@ \
		$(OBJ_DIR)/mergePos.o $(OBJ_DIR)/inline.o \
		$(OBJ_DIR)/evaluationTools.o $(LOCALIZE_LIBS)

robotDump: $(OBJ_DIR)/robotDump.o
		$(SILENT) $(CC) $(INCLUDES) $(LIB_DIRS) $(CFLAGS) -o $@ \
		$(OBJ_DIR)/inline.o \
		$(OBJ_DIR)/robotDump.o $(LOCALIZE_LIBS)

countadd: $(OBJ_DIR)/countadd.o
	$(SILENT) $(CC) $(INCLUDES) $(LIB_DIRS) $(CFLAGS) -o $@ \
		$(OBJ_DIR)/countadd.o $(LOCALIZE_LIBS)

probfunc: $(OBJ_DIR)/probfunc.o
	$(SILENT) $(CC) $(INCLUDES) $(LIB_DIRS) $(CFLAGS) -o $@ \
		$(OBJ_DIR)/probfunc.o $(LOCALIZE_LIBS)

dumpProbFunc: $(OBJ_DIR)/dumpProbFunc.o
	$(SILENT) $(CC) $(INCLUDES) $(LIB_DIRS) $(CFLAGS) -o $@ \
        $(OBJ_DIR)/dumpProbFunc.o $(LOCALIZE_LIBS)

probfuncgen: $(OBJ_DIR)/probfuncgen.o
	$(SILENT) $(CC) $(INCLUDES) $(LIB_DIRS) $(CFLAGS) -o $@ \
		$(OBJ_DIR)/probfuncgen.o $(LOCALIZE_LIBS)

createprobfunc: $(OBJ_DIR)/createprobfunc.o
	$(SILENT) $(CC) $(INCLUDES) $(LIB_DIRS) $(CFLAGS) -o $@ \
		$(OBJ_DIR)/createprobfunc.o $(LOCALIZE_LIBS)

alignmaps: $(OBJ_DIR)/alignmaps.o
	$(SILENT) $(CC) $(INCLUDES) $(LIB_DIRS) $(CFLAGS) -o $@ \
		$(OBJS) $(OBJ_DIR)/alignmaps.o $(LOCALIZE_LIBS)

match: $(OBJ_DIR)/match.o
	$(SILENT) $(CC) $(INCLUDES) $(LIB_DIRS) $(CFLAGS) -o $@ \
		$(OBJS) $(OBJ_DIR)/match.o $(LOCALIZE_LIBS)

explore: $(OBJ_DIR)/inline.o $(OBJ_DIR)/explore.o
	$(SILENT) $(CC) $(INCLUDES) $(LIB_DIRS) $(CFLAGS) -o $@ \
		$(OBJ_DIR)/inline.o $(OBJ_DIR)/explore.o $(LOCALIZE_LIBS)

multi: $(OBJ_DIR)/multi.o
	$(SILENT) $(CC) $(INCLUDES) $(LIB_DIRS) $(CFLAGS) -o $@ \
		$(OBJ_DIR)/multi.o $(LOCALIZE_LIBS)

$(BIN_DIR)/DISPLAY_LOCALIZE: $(OBJ_DIR)/inline.o $(OBJ_DIR)/displayLocalize.o $(OBJ_DIR)/function.o
	$(CC) $(INCLUDES) $(LIB_DIRS) -L/usr/local/lib -L/usr/lib/perl5/5.6.1/i386-linux/CORE $(CFLAGS) -rdynamic -fno-strict-aliasing -o $@ \
	/usr/lib/perl5/5.6.1/i386-linux/auto/DynaLoader/DynaLoader.a $(OBJ_DIR)/inline.o $(OBJ_DIR)/function.o $(OBJ_DIR)/displayLocalize.o /home/brudy/posServer/libposlib.a -lperl -lnsl -ldl -lc -lcrypt $(LOCALIZE_LIBS)

$(BIN_DIR)/poslibtcx: $(OBJ_DIR)/inline.o $(OBJ_DIR)/poslibtcx.o $(OBJ_DIR)/function.o
	$(CC) $(INCLUDES) $(LIB_DIRS) -L/usr/local/lib -L/usr/lib/perl5/5.6.1/i386-linux/CORE $(CFLAGS) -rdynamic -fno-strict-aliasing -o $@ \
	/usr/lib/perl5/5.6.1/i386-linux/auto/DynaLoader/DynaLoader.a $(OBJ_DIR)/inline.o $(OBJ_DIR)/function.o $(OBJ_DIR)/poslibtcx.o /home/brudy/posServer/libposlib.a -lperl -lnsl -ldl -lc -lcrypt -lbaseClient -lraiClient -lrai $(LOCALIZE_LIBS) 

join: $(OBJ_DIR)/join.o
	$(SILENT) $(CC) $(INCLUDES) $(LIB_DIRS) $(CFLAGS) -o $@ \
		$(OBJ_DIR)/join.o $(LOCALIZE_LIBS)

RAW_DATA: 
	$(SILENT) (cd rawData ; make build)

exportRawData:
	$(SILENT) (cd rawData ; make export)

cleanRawData:
	$(SILENT) (cd rawData ; make clean)

tagsRawData:
	$(SILENT) (cd rawData ; make tags)



##
## Standard suffix, version, packaging and dep rules
##

include $(RULES)


