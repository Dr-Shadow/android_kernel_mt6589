# Copyright Statement:
#
# This software/firmware and related documentation ("MediaTek Software") are
# protected under relevant copyright laws. The information contained herein
# is confidential and proprietary to MediaTek Inc. and/or its licensors.
# Without the prior written permission of MediaTek inc. and/or its licensors,
# any reproduction, modification, use or disclosure of MediaTek Software,
# and information contained herein, in whole or in part, shall be strictly prohibited.
#
# MediaTek Inc. (C) 2010. All rights reserved.
#
# BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
# THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
# RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
# AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
# EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
# MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
# NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
# SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
# SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
# THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
# THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
# CONTAINED IN MEDIATEK SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK
# SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
# STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND
# CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
# AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
# OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
# MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
#
# The following software/firmware and/or related documentation ("MediaTek Software")
# have been modified by MediaTek Inc. All revisions are subject to any receiver's
# applicable license agreements with MediaTek Inc.


include mediatek/build/Makefile
$(call codebase-path,kernel,.,mt6589,e960)
#$(call relative-include,common.mk)
include mediatek/build/kernel/common.mk
MODULE_PREFIX_KERNEL := CUSTOM_KERNEL_
MODULE_PREFIX_HAL    := CUSTOM_HAL_
machine-y       := $(call lc,$(MTK_PLATFORM))
ifeq ($(strip $(KBUILD_OUTPUT_SUPPORT)),yes)
MACHINE          := mediatek/platform/$(call lc,$(MTK_PLATFORM))/kernel/core/
machdirs         := mediatek/platform/$(call lc,$(MTK_PLATFORM))/kernel/core/
else
MACHINE         := $(MTK_PATH_PLATFORM)/core/
machdirs        := $(MTK_PATH_PLATFORM)/core/
endif
platdirs        :=

ifeq ($(strip $(KBUILD_OUTPUT_SUPPORT)),yes)
drivers-y       += mediatek/kernel/
drivers-y       += mediatek/custom/out/$(FULL_PROJECT)/kernel/
drivers-y       += mediatek/platform/$(call lc,$(MTK_PLATFORM))/kernel/drivers/
else
drivers-y       += $(MTK_PATH_PLATFORM)/drivers/
drivers-y       += $(MTK_PATH_SOURCE)/
drivers-y       += $(MTK_PATH_CUSTOM)/
endif

LINUXINCLUDE    += -I$(MTK_PATH_PLATFORM)/core/include/
LINUXINCLUDE    += -I$(MTK_PATH_SOURCE)/include/
KBUILD_CFLAGS   += $(call mtk.custom.generate-macros)

ifneq (,$(MTK_MODEM_SUPPORT))
KBUILD_CPPFLAGS += -D$(call uc,$(MTK_MODEM_SUPPORT))
endif

KBUILD_CFLAGS += -DUSER_BUILD_KERNEL

ifeq (,$(M))
clean: mtkclean
mtkclean:
	@echo "clean mediatek kernel source tree"
	@find $(MTK_PATH_PLATFORM) $(MTK_PATH_SOURCE) $(MTK_PATH_CUSTOM) $(RCS_FIND_IGNORE) \
	\( -name '*.[oas]' -o -name '*.ko' -o -name '.*.cmd' \
	-o -name '.*.d' -o -name '.*.tmp' -o -name '*.mod.c' \
	-o -name '*.symtypes' -o -name 'modules.order' \
	-o -name 'Module.markers' -o -name '.tmp_*.o.*' \
	-o -name '*.gcno' \) -type f -print | grep -v "/prebuilt/" | xargs rm -f
	find $(MTK_ROOT_GEN_CONFIG) -name kconfig -type f -print | xargs rm -f
endif

custom-files-kernel := $(strip $(call mtk.custom.generate-rules,prepare,kernel))
custom-files-hal    := $(strip $(call mtk.custom.generate-rules,prepare,hal))
_module_inc_kernel  :=
_module_inc_hal     :=
$(foreach i,\
  $(foreach t,$(dir $(patsubst $(MTK_ROOT_CUSTOM_OUT)/kernel/%,%,$(custom-files-kernel))), \
    $(eval _ := $(subst /, ,$(t))) \
    $(if $(filter 1,$(words $(_))),$(t),) \
    $(if $(filter 2,$(words $(_))),\
      $(if $(filter src inc,$(word 2,$(_))),$(t), \
        $(if $(filter $($(MODULE_PREFIX_KERNEL)$(call uc,$(word 1,$(_)))),$(word 2,$(_))),$(t),) \
  ),)) \
 ,$(eval _module_inc_kernel += $(filter-out $(_module_inc_kernel),$(i))) \
)

$(foreach i,\
  $(foreach t,$(dir $(patsubst $(MTK_ROOT_CUSTOM_OUT)/hal/%,%,$(custom-files-hal))), \
    $(eval _ := $(subst /, ,$(t))) \
    $(if $(filter 1,$(words $(_))),$(t),) \
    $(if $(filter 2,$(words $(_))),\
      $(if $(filter src inc,$(word 2,$(_))),$(t), \
        $(if $(filter $($(MODULE_PREFIX_HAL)$(call uc,$(word 1,$(_)))),$(word 2,$(_))),$(t),) \
  ),)) \
 ,$(eval _module_inc_hal += $(filter-out $(_module_inc_hal),$(i))) \
)
LINUXINCLUDE += $(strip $(addprefix -I$(MTK_ROOT_CUSTOM_OUT)/kernel/,$(_module_inc_kernel)))
LINUXINCLUDE += $(strip $(addprefix -I$(MTK_ROOT_CUSTOM_OUT)/hal/,$(_module_inc_hal)))
# add include path for ptgen output
LINUXINCLUDE += -I$(MTK_ROOT_CUSTOM_OUT)/common
#$(foreach p,$(strip $(addprefix -I$(MTK_ROOT_CUSTOM_OUT)/kernel/,$(_module_inc_kernel))),$(info --> $(p)))
#$(foreach p,$(strip $(addprefix -I$(MTK_ROOT_CUSTOM_OUT)/hal/,$(_module_inc_hal))),$(info --> $(p)))

generated_files := $(call mtk.config.generate-rules)
.config mediatek-configs: $(MTK_ROOT_GEN_CONFIG)/kconfig $(MTK_ROOT_GEN_CONFIG)
	@echo "[COPY] $< --> $@"
	@cp $< $@
	@echo "[COPY] $< --> .config"
	@cp $< .config
ifeq (mediatek-configs,$(MAKECMDGOALS))
include/config/auto.conf: ;
endif

