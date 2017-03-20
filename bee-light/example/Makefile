NAME = BEESOFTAPI-examples
CONF  = ../../Makefile.conf
RULES = ../../Makefile.rules


##
## System config variables
##

include $(CONF)

ECHO          = @echo

##
## Either "@" for quiet or "" for verbose
##

SILENT = @
PREFIX    = ../../..

##
## Variables
##

DEPENDENCIES += *.c

##
## local build variables
##

OBJS := 

INCLUDE = -I../../../include

BASE_LIBS = -L../../../lib -lbee -ltcx -lpthread 


##
## Compulsory rules
##

all:
	$(MAKE) export $(DEPEND)
	$(MAKE) build

build: user\
	user2\
	user3\
	user4\
	user5\
	user6\
	user7\
	user8\
	dance

export:

clean:
	$(RM) *.o *.a *~ core a.out obj obj_sun obj_solaris $(CLEAN_FILES)

distclean: clean
	$(RM) $(DEPEND) Makefile.depend* user user? dance \
		$(NAME)-src.tgz $(NAME)-bin.tgz $(DISTCLEAN_FILES)

##
## local rules
##

user.o: user.c
	$(ECHO) "    ---- Creating $@"
	$(SILENT) gcc -c user.c $(INCLUDE) -lm

user2.o: user2.c
	$(ECHO) "    ---- Creating $@"
	$(SILENT) gcc -c user2.c $(INCLUDE)

user3.o: user3.c
	$(ECHO) "    ---- Creating $@"
	$(SILENT) gcc -c user3.c $(INCLUDE)

user4.o: user4.c
	$(ECHO) "    ---- Creating $@"
	$(SILENT) gcc -c user4.c  $(INCLUDE)

user5.o: user5.c
	$(ECHO) "    ---- Creating $@"
	$(SILENT) gcc -c user5.c $(INCLUDE)

user6.o: user6.c
	$(ECHO) "    ---- Creating $@"
	$(SILENT) gcc -c user6.c $(INCLUDE)

user7.o: user7.c
	$(ECHO) "    ---- Creating $@"
	$(SILENT) gcc -c user7.c $(INCLUDE)

user8.o: user8.c
	$(ECHO) "    ---- Creating $@"
	$(SILENT) gcc -c user8.c $(INCLUDE)

dance.o: dance.c
	$(ECHO) "    ---- Creating $@"
	$(SILENT) gcc -c dance.c $(INCLUDE)

user: $(OBJS) user.o
	$(ECHO) "    ---- Creating $@"
	$(SILENT) gcc -o $@ $(CFLAGS) $(OBJS) user.o \
		$(LIB_DIRS) $(BASE_LIBS)

user2: $(OBJS) user2.o
	$(ECHO) "    ---- Creating $@"
	$(SILENT) gcc -o $@ $(CFLAGS) $(OBJS) user2.o \
		$(LIB_DIRS) $(BASE_LIBS)

user3: $(OBJS) user3.o
	$(ECHO) "    ---- Creating $@"
	$(SILENT) gcc -o $@ $(CFLAGS) $(OBJS) user3.o \
		$(LIB_DIRS) $(BASE_LIBS)

user4: $(OBJS) user4.o
	$(ECHO) "    ---- Creating $@"
	$(SILENT) gcc -o $@ $(CFLAGS) $(OBJS) user4.o \
		$(LIB_DIRS) $(BASE_LIBS)

user5: $(OBJS) user5.o
	$(ECHO) "    ---- Creating $@"
	$(SILENT) gcc -o $@ $(CFLAGS) $(OBJS) user5.o \
		$(LIB_DIRS) $(BASE_LIBS)

user6: $(OBJS) user6.o
	$(ECHO) "    ---- Creating $@"
	$(SILENT) gcc -o $@ $(CFLAGS) $(OBJS) user6.o \
		$(LIB_DIRS) $(BASE_LIBS)

user7: $(OBJS) user7.o
	$(ECHO) "    ---- Creating $@"
	$(SILENT) gcc -o $@ $(CFLAGS) $(OBJS) user7.o \
		$(LIB_DIRS) $(BASE_LIBS)

user8: $(OBJS) user8.o
	$(ECHO) "    ---- Creating $@"
	$(SILENT) gcc -o $@ $(CFLAGS) $(OBJS) user8.o \
		$(LIB_DIRS) $(BASE_LIBS)

dance:  $(OBJS) dance.o
	$(ECHO) "    ---- Creating $@"
	$(SILENT) gcc -o $@ $(CFLAGS) $(OBJS) dance.o \
		$(LIB_DIRS) $(BASE_LIBS)


##
## Standard suffix, version, packaging and dep rules
##


include $(RULES)
