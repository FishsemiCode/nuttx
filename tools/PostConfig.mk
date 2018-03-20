############################################################################
# tools/PostConfig.mk
#
#   Copyright (C) 2007-2012, 2014-2015 Gregory Nutt. All rights reserved.
#   Author: Pinecone <pinecone@pinecone.net>
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

SRCDIR   := $(patsubst %$(DELIM),%,$(dir $(firstword $(MAKEFILE_LIST))))

# override MKDEP from defconfig
MKDEP    := $(OUTDIR)$(DELIM)tools$(DELIM)mkdeps$(HOSTEXEEXT)

ifeq ($(WINTOOL),y)
MKDEP    += --winpath
SRCDIR   := $(shell cygpath -m $(SRCDIR))

POSTFLAGS := ${shell $(INCDIR) $(INCDIROPT) "$(CC)" `cygpath -w $(SRCDIR)`}
POSTFLAGS += ${shell $(INCDIR) $(INCDIROPT) "$(CC)" `cygpath -w $(OUTDIR)$(DELIM)include`}
else
POSTFLAGS := ${shell $(INCDIR) $(INCDIROPT) "$(CC)" $(SRCDIR)}
POSTFLAGS += ${shell $(INCDIR) $(INCDIROPT) "$(CC)" $(OUTDIR)$(DELIM)include}
endif

AFLAGS   += $(POSTFLAGS) $($(strip $(1))_AFLAGS)
CFLAGS   += $(POSTFLAGS) $($(strip $(1))_CFLAGS)
CPPFLAGS += $(POSTFLAGS) $($(strip $(1))_CPPFLAGS)
CXXFLAGS += $(POSTFLAGS) $($(strip $(1))_CXXFLAGS)

# CREATEDIR is the list of directories which need to create
$(if $(CREATEDIR), $(foreach DIR, $(CREATEDIR), $(call MKDIR, $(DIR))))
