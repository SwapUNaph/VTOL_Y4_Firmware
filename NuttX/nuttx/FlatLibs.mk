############################################################################
# FlatLibs.mk
#
#   Copyright (C) 2007-2012, 2014, 2016 Gregory Nutt. All rights reserved.
#   Author: Gregory Nutt <gnutt@nuttx.org>
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
#
############################################################################

# NUTTXLIBS is the list of NuttX libraries that is passed to the
#   processor-specific Makefile to build the final NuttX target.
#   Libraries in FSDIRS are excluded if file descriptor support
#   is disabled.
# USERLIBS is the list of libraries used to build the final user-space
#   application
# EXPORTLIBS is the list of libraries that should be exported by
#   'make export' is

NUTTXLIBS = lib$(DELIM)libsched$(LIBEXT)
USERLIBS =

# Driver support.  Generally depends on file descriptor support but there
# are some components in the drivers directory that are needed even if file
# descriptors are not supported.

NUTTXLIBS += lib$(DELIM)libdrivers$(LIBEXT)

# Add libraries for board support

NUTTXLIBS += lib$(DELIM)libconfigs$(LIBEXT)

# Add libraries for syscall support.

NUTTXLIBS += lib$(DELIM)libc$(LIBEXT) lib$(DELIM)libmm$(LIBEXT)
NUTTXLIBS += lib$(DELIM)libarch$(LIBEXT)
ifeq ($(CONFIG_LIB_SYSCALL),y)
NUTTXLIBS += lib$(DELIM)libstubs$(LIBEXT)
USERLIBS  += lib$(DELIM)libproxies$(LIBEXT)
endif

# Add libraries for C++ support.  CXX, CXXFLAGS, and COMPILEXX must
# be defined in Make.defs for this to work!

ifeq ($(CONFIG_HAVE_CXX),y)
NUTTXLIBS += lib$(DELIM)libcxx$(LIBEXT)
endif

# Add library for application support.

ifneq ($(APPDIR),)
NUTTXLIBS += lib$(DELIM)libapps$(LIBEXT)
endif

# Add libraries for network support

ifeq ($(CONFIG_NET),y)
NUTTXLIBS += lib$(DELIM)libnet$(LIBEXT)
endif

# Add libraries for Crypto API support

ifeq ($(CONFIG_CRYPTO),y)
NUTTXLIBS += lib$(DELIM)libcrypto$(LIBEXT)
endif

# Add libraries for file system support

ifeq ($(CONFIG_NFILE_DESCRIPTORS),0)
ifneq ($(CONFIG_NSOCKET_DESCRIPTORS),0)
NUTTXLIBS += lib$(DELIM)libfs$(LIBEXT)
endif
else
NUTTXLIBS += lib$(DELIM)libfs$(LIBEXT) lib$(DELIM)libbinfmt$(LIBEXT)
endif

# Add libraries for the NX graphics sub-system

ifeq ($(CONFIG_NX),y)
NUTTXLIBS += lib$(DELIM)libgraphics$(LIBEXT)
NUTTXLIBS += lib$(DELIM)libnx$(LIBEXT)
endif

# Add libraries for the Audio sub-system

ifeq ($(CONFIG_AUDIO),y)
NUTTXLIBS += lib$(DELIM)libaudio$(LIBEXT)
endif

# Add libraries for the Wireless sub-system

ifeq ($(CONFIG_WIRELESS),y)
NUTTXLIBS += lib$(DELIM)libwireless$(LIBEXT)
endif

# Export all libraries

EXPORTLIBS = $(NUTTXLIBS)
