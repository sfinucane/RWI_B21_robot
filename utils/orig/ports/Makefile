include $(RWI)/scripts/Makefile.defines

EXE = ports

all: install

$(EXE): $(EXE).o
	gcc -static -o $@ $^

install: $(EXE)
	$(SUDO) $(RM) $(INSTALL_DIR)/bin/$(EXE)
	$(SUDO) mv $(EXE) $(INSTALL_DIR)/bin
	$(SUDO) strip $(INSTALL_DIR)/bin/$(EXE)
	$(SUDO) chown 0.0 $(INSTALL_DIR)/bin/$(EXE)
	$(SUDO) chmod 4755 $(INSTALL_DIR)/bin/$(EXE)

include $(RWI)/scripts/Makefile.rules
