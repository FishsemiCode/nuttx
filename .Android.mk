#
# Copyright (C) 2017 Pinecone
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

NUTTX_ROOT      := $(abspath $(call my-dir))
NUTTX_OUTDIR    := $(abspath $(TARGET_OUT_INTERMEDIATES)/NUTTX_OBJ)

NUTTX_ARMTOOL   := CROSSDEV=$(NUTTX_ROOT)/prebuilts/gcc/linux/arm/bin/arm-none-eabi-
NUTTX_ARMTOOL   += ARCROSSDEV=$(NUTTX_ROOT)/prebuilts/gcc/linux/arm/bin/arm-none-eabi-

NUTTX_TL4TOOL   := CROSSDEV=$(NUTTX_ROOT)/prebuilts/ceva/linux/tl4/

#abies/audio
$(NUTTX_OUTDIR)/abies/audio/nuttx/nuttx.a: FORCE
	$(hide) $(NUTTX_TL4TOOL) $(NUTTX_ROOT)/nuttx/tools/configure.sh -o $(dir $@) abies/audio
	$(hide) $(NUTTX_TL4TOOL) $(MAKE) -C $(NUTTX_ROOT)/nuttx O=$(dir $@) V=$(if $(hide),0,1)
	$(hide) $(NUTTX_TL4TOOL) $(MAKE) -C $(NUTTX_ROOT)/nuttx O=$(dir $@) savedefconfig
	$(hide) cp $(dir $@)/defconfig ${NUTTX_ROOT}/nuttx/configs/abies/audio

include $(CLEAR_VARS)
LOCAL_MODULE               := abies-audio.fw
LOCAL_PREBUILT_MODULE_FILE := $(NUTTX_OUTDIR)/abies/audio/nuttx/nuttx.a
LOCAL_MODULE_CLASS         := FIRMWARE
LOCAL_MODULE_PATH          := $(TARGET_OUT_VENDOR)/firmware
include $(BUILD_PREBUILT)

#banks/audio
$(NUTTX_OUTDIR)/banks/audio/nuttx/nuttx.a: FORCE
	$(hide) $(NUTTX_TL4TOOL) $(NUTTX_ROOT)/nuttx/tools/configure.sh -o $(dir $@) banks/audio
	$(hide) $(NUTTX_TL4TOOL) $(MAKE) -C $(NUTTX_ROOT)/nuttx O=$(dir $@) V=$(if $(hide),0,1)
	$(hide) $(NUTTX_TL4TOOL) $(MAKE) -C $(NUTTX_ROOT)/nuttx O=$(dir $@) savedefconfig
	$(hide) cp $(dir $@)/defconfig ${NUTTX_ROOT}/nuttx/configs/banks/audio

include $(CLEAR_VARS)
LOCAL_MODULE               := banks-audio.fw
LOCAL_PREBUILT_MODULE_FILE := $(NUTTX_OUTDIR)/banks/audio/nuttx/nuttx.a
LOCAL_MODULE_CLASS         := FIRMWARE
LOCAL_MODULE_PATH          := $(TARGET_OUT_VENDOR)/firmware
include $(BUILD_PREBUILT)

#banks/rpm
$(NUTTX_OUTDIR)/banks/rpm/nuttx/nuttx: FORCE
	$(hide) $(NUTTX_ARMTOOL) $(NUTTX_ROOT)/nuttx/tools/configure.sh -o $(dir $@) banks/rpm
	$(hide) $(NUTTX_ARMTOOL) $(MAKE) -C $(NUTTX_ROOT)/nuttx O=$(dir $@) V=$(if $(hide),0,1)
	$(hide) $(NUTTX_ARMTOOL) $(MAKE) -C $(NUTTX_ROOT)/nuttx O=$(dir $@) savedefconfig
	$(hide) cp $(dir $@)/defconfig ${NUTTX_ROOT}/nuttx/configs/banks/rpm

include $(CLEAR_VARS)
LOCAL_MODULE               := banks-rpm.fw
LOCAL_PREBUILT_MODULE_FILE := $(NUTTX_OUTDIR)/banks/rpm/nuttx/nuttx
LOCAL_MODULE_CLASS         := FIRMWARE
LOCAL_MODULE_PATH          := $(TARGET_OUT_VENDOR)/firmware
include $(BUILD_PREBUILT)

#banks/sensor
$(NUTTX_OUTDIR)/banks/sensor/nuttx/nuttx: FORCE
	$(hide) $(NUTTX_ARMTOOL) $(NUTTX_ROOT)/nuttx/tools/configure.sh -o $(dir $@) banks/sensor
	$(hide) $(NUTTX_ARMTOOL) $(MAKE) -C $(NUTTX_ROOT)/nuttx O=$(dir $@) V=$(if $(hide),0,1)
	$(hide) $(NUTTX_ARMTOOL) $(MAKE) -C $(NUTTX_ROOT)/nuttx O=$(dir $@) savedefconfig
	$(hide) cp $(dir $@)/defconfig ${NUTTX_ROOT}/nuttx/configs/banks/sensor

include $(CLEAR_VARS)
LOCAL_MODULE               := banks-sensor.fw
LOCAL_PREBUILT_MODULE_FILE := $(NUTTX_OUTDIR)/banks/sensor/nuttx/nuttx
LOCAL_MODULE_CLASS         := FIRMWARE
LOCAL_MODULE_PATH          := $(TARGET_OUT_VENDOR)/firmware
include $(BUILD_PREBUILT)
