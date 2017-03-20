OBJ1 = checkBattery.o

# _HACK_ _GROSS_ This utilitity is dependent on WAY too much!
# ...and as a result gets caught by the ugly B14 base tactile
# support _HACK_ and so becomes dependent on mspmodule also.

LIB1 = -lBase -lMcp -lmspmodule -lmsp -labus -lRai -lUtils

# The name of the program I want to make, if any
PROG1 = checkBattery
SCRIPT = callCheckBattery

PROGS = $(PROG1) 
OBJS  = $(OBJ1)

CLEAN_FILES    = $(PROG1)
MRPROPER_FILES =

include $(RWI)/scripts/Makefile.defines

OPT_FLAGS = -O2

all: install

$(PROG1): $(OBJ1)
	$(CC) -o $(PROG1) $(OBJ1) $(LIB_DIRS) $(LIB1) $(INCLUDES)

# some programs need to be run by root since they write to hardware
# so we set the setuid bit and chown it to root
install: $(PROGS) $(SCRIPT)
	$(SUDO) $(RM) $(INSTALL_DIR)/bin/$(PROG1)
	cp $(PROG1) $(INSTALL_DIR)/bin
	$(SUDO) chown root $(INSTALL_DIR)/bin/$(PROG1)
	$(SUDO) $(CHMOD) a+s $(INSTALL_DIR)/bin/$(PROG1)
	$(SUDO) $(RM) $(INSTALL_DIR)/bin/$(SCRIPT)
	$(SUDO) sed -e "s:@@RWI@@:$(RWI):" \
		$(SCRIPT) > $(INSTALL_DIR)/bin/$(SCRIPT)
	$(SUDO) chown root $(INSTALL_DIR)/bin/$(SCRIPT)
	$(SUDO) $(CHMOD) a+sx $(INSTALL_DIR)/bin/$(SCRIPT)

include $(RWI)/scripts/Makefile.rules
