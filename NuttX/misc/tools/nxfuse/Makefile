# ==========================================================================
#   Copyright (C) 2016 Ken Pettit. All rights reserved.
#   Author: Ken Pettit <pettitkd@gmail.com>
#
#   nxfuse is a Linux FUSE filesystem that allows native mounting of
#          NuttX filesystems within Linux.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name NuttX nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
# ==========================================================================

.silent:

-include .config

NUTTXDIR    ?= ../../nuttx

APPNAME     = nxfuse

OBJDIR		=  obj
DEPDIR		=  dep
SRCDIR      =  src

CC          =  $(CROSS_COMPILE)gcc
LDFLAGS     +=  -g
LIBFILES    +=  -lm -lpthread -lfuse
ifeq ($(RELEASE),)
CFLAGS      +=  -g -Wall -I include -DFAR= -DTRUE=1 -DFALSE=0 -Wno-unused-value -D_FILE_OFFSET_BITS=64
else
CFLAGS      +=  -O2 -Wall -I include -DFAR= -DTRUE=1 -DFALSE=0 -Wno-unused-value -D_FILE_OFFSET_BITS=64
endif

SOURCES     =  $(wildcard $(SRCDIR)/*.c)

ifeq ($(CONFIG_FS_NXFFS),y)
SOURCES     += $(wildcard $(SRCDIR)/nxffs/*.c)
endif

ifeq ($(CONFIG_FS_SMARTFS),y)
SOURCES     += $(wildcard $(SRCDIR)/smartfs/*.c)
endif

SRCTMP      =  $(patsubst $(SRCDIR)/%,%,$(SOURCES))
OBJTMP		=  $(SRCTMP:.c=.o)
OBJECTS		=  $(patsubst %,$(OBJDIR)/%,$(OBJTMP))
DEPS        =  $(patsubst $(OBJDIR)/%.o,$(DEPDIR)/%.d,$(OBJECTS))
CCONFIG     =  include/nuttx/config.h
CONFIG      =  .config

all: init $(APPNAME)

.PHONY: init clean context depend
init:
	@mkdir -p $(OBJDIR)/nxffs
	@mkdir -p $(DEPDIR)/nxffs
	@mkdir -p $(OBJDIR)/smartfs
	@mkdir -p $(DEPDIR)/smartfs

#Include our built dependencies
-include $(DEPS)

# Create dependencies

.depend: init Makefile $(CCONFIG) $(SOURCES)
	$(Q) $(MKDEP) $(ROOTDEPPATH) "$(CC)" -- $(CFLAGS) -- $(SOURCES) >Make.dep
	$(Q) touch $@

depend: .depend

# ============================================================
# Rule for compiling source files.
# ============================================================
$(OBJDIR)/%.o: $(SRCDIR)/%.c
	@echo Compiling $<
	@$(CC) $(CFLAGS) -c -o $@ $<
	@$(CC) -MM -MT $(OBJDIR)/$*.o $(CFLAGS) $(SRCDIR)/$*.c > $(DEPDIR)/$*.d

# ========================
# Rule to build nxfuse
# ========================
$(APPNAME): Makefile $(CONFIG) $(CCONFIG) $(OBJECTS)
	@echo Linking $@
	@$(CC) $(LDFLAGS) $(OBJECTS) $(LIBFILES) -o $@

$(CCONFIG): $(NUTTXDIR)/include/nuttx/config.h
	@echo "CP: config.h"
	@cp $(NUTTXDIR)/include/nuttx/config.h include/nuttx

# =================================
# Rule to retrieve the .config file
# =================================
$(CONFIG): $(NUTTXDIR)/.config
	@echo "CP: .config"
	@cp $(NUTTXDIR)/.config .
	@$(MAKE)

# =============================
# Rule to clean all build files
# =============================
.PHONY: clean
clean:
	@echo "=== cleaning ===";
	@rm -rf $(OBJDIR) $(DEPDIR)
	@rm -rf *.o
	@rm -f $(APPNAME) 
	@rm -f $(CCONFIG)
	@rm -f $(CONFIG)


# ==================================
# Rule to install nxfuse 
#
# Note: 
#    This is a simple installation
#    in lieu of having an actual
#    autoconf configure script.
#
# This install script always installs
# nxfuse to /usr/bin and the man
# page to /usr/share/man/man1
#
# Run 'make install' with sudo
# ==================================
.PHONY: install
install: $(APPNAME)
	@echo "=== installing to /usr/bin ===";
	@cp $(APPNAME) /usr/bin
	@cp man/nxfuse.1 /usr/share/man/man1

