############################################################################
# arch/ceva/src/Makefile
#
#   Copyright (C) 2018 Pinecone Inc. All rights reserved.
#   Author: Xiang Xiao <xiaoxiang@pinecone.net>
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

-include $(TOPDIR)/Make.defs
-include chip$(DELIM)Make.defs

ifeq ($(CONFIG_ARCH_TL420),y)
ARCH_SUBDIR = tl4
else ifeq ($(CONFIG_ARCH_TL421),y)
ARCH_SUBDIR = tl4
else ifeq ($(CONFIG_ARCH_XC5),y)
ARCH_SUBDIR = xc5
else ifeq ($(CONFIG_ARCH_XM6),y)
ARCH_SUBDIR = xm6
endif

CPPFLAGS += $(EXTRADEFINES)
CFLAGS += $(EXTRADEFINES)
CXXFLAGS += $(EXTRADEFINES)

ifeq ($(CONFIG_WINDOWS_NATIVE),y)
  ARCH_SRCDIR = $(TOPDIR)\arch\$(CONFIG_ARCH)\src
  NUTTX = "$(OUTDIR)\nuttx$(EXEEXT)"
  CFLAGS += -I$(ARCH_SRCDIR)\chip
  CFLAGS += -I$(ARCH_SRCDIR)\common
  CFLAGS += -I$(ARCH_SRCDIR)\$(ARCH_SUBDIR)
  CFLAGS += -I$(ARCH_SRCDIR)\$(CONFIG_ARCH_CHIP)
  CFLAGS += -I$(TOPDIR)\sched
else
  ARCH_SRCDIR = $(TOPDIR)/arch/$(CONFIG_ARCH)/src
ifeq ($(WINTOOL),y)
  NUTTX = "${shell cygpath -w $(OUTDIR)/nuttx$(EXEEXT)}"
  CFLAGS += -I "${shell cygpath -w $(ARCH_SRCDIR)/chip}"
  CFLAGS += -I "${shell cygpath -w $(ARCH_SRCDIR)/common}"
  CFLAGS += -I "${shell cygpath -w $(ARCH_SRCDIR)/$(ARCH_SUBDIR)}"
  CFLAGS += -I "${shell cygpath -w $(ARCH_SRCDIR)/$(CONFIG_ARCH_CHIP)}"
  CFLAGS += -I "${shell cygpath -w $(TOPDIR)/sched}"
  LDSCRIPT := ${shell cygpath -m $(LDSCRIPT)}
else
  NUTTX = "$(OUTDIR)/nuttx$(EXEEXT)"
  CFLAGS += -I$(ARCH_SRCDIR)/chip
  CFLAGS += -I$(ARCH_SRCDIR)/common
  CFLAGS += -I$(ARCH_SRCDIR)/$(ARCH_SUBDIR)
  CFLAGS += -I$(ARCH_SRCDIR)/$(CONFIG_ARCH_CHIP)
  CFLAGS += -I$(TOPDIR)/sched
endif
endif

# The "head" object

HEAD_OBJ = $(HEAD_ASRC:.S=$(OBJEXT))
STARTUP_OBJS ?= $(HEAD_OBJ)

# Flat build or kernel-mode objects

ASRCS = $(CHIP_ASRCS) $(CMN_ASRCS)
AOBJS = $(ASRCS:.S=$(OBJEXT))

CSRCS = $(CHIP_CSRCS) $(CMN_CSRCS)
COBJS = $(CSRCS:.c=$(OBJEXT))

SRCS = $(ASRCS) $(CSRCS)
OBJS = $(AOBJS) $(COBJS)

# User-mode objects

UASRCS = $(CHIP_UASRCS) $(CMN_UASRCS)
UAOBJS = $(UASRCS:.S=$(OBJEXT))

UCSRCS = $(CHIP_UCSRCS) $(CMN_UCSRCS)
UCOBJS = $(UCSRCS:.c=$(OBJEXT))

USRCS = $(UASRCS) $(UCSRCS)
UOBJS = $(UAOBJS) $(UCOBJS)

KBIN = libkarch$(LIBEXT)
UBIN = libuarch$(LIBEXT)
BIN  = libarch$(LIBEXT)

EXTRA_LIBS ?=
EXTRA_LIBPATHS ?=
LINKLIBS ?=

ifeq ($(CONFIG_WINDOWS_NATIVE),y)
  BOARDMAKE = $(if $(wildcard .\board\Makefile),y,)
  LIBPATHS += -I "$(OUTDIR)\staging"
ifeq ($(BOARDMAKE),y)
  LIBPATHS += -I "$(OUTDIR)\arch\$(CONFIG_ARCH)\src\board"
endif

else
  BOARDMAKE = $(if $(wildcard ./board/Makefile),y,)

ifeq ($(WINTOOL),y)
  LIBPATHS += -I "${shell cygpath -w "$(OUTDIR)/staging"}"
ifeq ($(BOARDMAKE),y)
  LIBPATHS += -I "${shell cygpath -w "$(OUTDIR)/arch/$(CONFIG_ARCH)/src/board"}"
endif

else
  LIBPATHS += -I "$(OUTDIR)/staging"
ifeq ($(BOARDMAKE),y)
  LIBPATHS += -I "$(OUTDIR)/arch/$(CONFIG_ARCH)/src/board"
endif
endif
endif

LDLIBS = $(patsubst %, -lib %,$(LINKLIBS))
ifeq ($(BOARDMAKE),y)
  LDLIBS += -lib libboard.lib
endif

VPATH += .
VPATH += chip
VPATH += common
VPATH += $(ARCH_SUBDIR)

VPATH := $(patsubst %,:$(SRCDIR)$(DELIM)%,$(VPATH))
ifeq ($(WINTOOL),y)
VPATH += :$(shell cygpath -m $(OUTDIR)$(DELIM)arch$(DELIM)$(CONFIG_ARCH)$(DELIM)src$(DELIM)chip)
else
VPATH += :$(OUTDIR)$(DELIM)arch$(DELIM)$(CONFIG_ARCH)$(DELIM)src$(DELIM)chip
endif

all: $(HEAD_OBJ) $(BIN)

.PHONY: board$(DELIM)libboard$(LIBEXT)

$(sort $(AOBJS) $(UAOBJS) $(HEAD_OBJ)): %$(OBJEXT): %.S
	$(call ASSEMBLE, $<, $@)

$(sort $(COBJS) $(UCOBJS)): %$(OBJEXT): %.c
	$(call COMPILE, $<, $@)

$(BIN) $(KBIN): $(OBJS)
	$(call ARCHIVE, $@, $(OBJS))

$(UBIN): $(UOBJS)
	$(call ARCHIVE, $@, $(UOBJS))

board$(DELIM)libboard$(LIBEXT):
ifeq ($(CONFIG_ARCH_XC5), y)
	$(Q) if [ ! -L "$(CROSSDEV)/cevaxccc-xdrv" ]; then \
		ln -sf "$(CROSSDEV)/cevaxccc-drv"  "$(CROSSDEV)/cevaxccc-xdrv";\
	fi
endif
	$(Q) $(MAKE) -C board TOPDIR="$(TOPDIR)" libboard$(LIBEXT) EXTRADEFINES=$(EXTRADEFINES)

nuttx$(EXEEXT): $(HEAD_OBJ) board$(DELIM)libboard$(LIBEXT) $(LDSCRIPT)
	$(Q) echo "LD: nuttx"
	$(Q) $(call PREPROCESS, $(LDSCRIPT), $(notdir $(LDSCRIPT)))
	$(Q) $(LD) $(LDFLAGS) $(LIBPATHS) $(EXTRA_LIBPATHS) \
		-o $(NUTTX) $(HEAD_OBJ) $(EXTRA_OBJS) \
		$(LDLIBS) $(EXTRA_LIBS) $(LIBGCC) \
		-l $(OUTDIR)$(DELIM)nuttx.lin $(notdir $(LDSCRIPT))
	$(Q) $(DISASM) -o $(OUTDIR)$(DELIM)nuttx.lst $(NUTTX)
	$(Q) $(OBJDUMP) $(NUTTX) > $(OUTDIR)$(DELIM)nuttx.dump
	$(Q) $(OBJCOPY) -b $(OUTDIR)$(DELIM)nuttx -c -split $(NUTTX)

# This is part of the top-level export target
# Note that there may not be a head object if layout is handled
# by the linker configuration.

export_startup: board$(DELIM)libboard$(LIBEXT) $(STARTUP_OBJS)
ifneq ($(STARTUP_OBJS),)
	$(Q) if [ -d "$(EXPORT_DIR)$(DELIM)startup" ]; then \
		cp -f $(STARTUP_OBJS) "$(EXPORT_DIR)$(DELIM)startup$(DELIM)."; \
	 else \
		echo "$(EXPORT_DIR)$(DELIM)startup does not exist"; \
	exit 1; \
	fi
endif

# Dependencies

.depend: Makefile chip$(DELIM)Make.defs $(SRCS)
ifeq ($(BOARDMAKE),y)
	$(Q) $(MAKE) -C board TOPDIR="$(TOPDIR)" depend
endif
	$(Q) $(MKDEP) $(patsubst %,--dep-path %,$(subst :, ,$(VPATH))) \
	 "$(CC)" -- $(CFLAGS) -- $^ >Make.dep
	$(Q) touch $@

depend: .depend

clean:
ifeq ($(BOARDMAKE),y)
	$(Q) $(MAKE) -C board TOPDIR="$(TOPDIR)" clean
endif
	$(call DELFILE, $(notdir $(LDSCRIPT)))
	$(call DELFILE, $(KBIN))
	$(call DELFILE, $(UBIN))
	$(call DELFILE, $(BIN))
	$(call CLEAN)

distclean: clean
ifeq ($(BOARDMAKE),y)
	$(Q) $(MAKE) -C board TOPDIR="$(TOPDIR)" distclean
endif
	$(call DELFILE, Make.dep)
	$(call DELFILE, .depend)

-include Make.dep
