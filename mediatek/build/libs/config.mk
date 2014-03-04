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


# config.mk - add supports for configuration file generation

MTK_AUTOCONFIG_LIST := $(patsubst $(call relative-path,../../config/common/autoconfig/)%,%,\
  $(call wildcard2,$(call relative-path,../../config/common/autoconfig/*)))

#internal functions supporting configuration generation
define .mtk.config.delete-rule
$(1): $(2).delete
$(2).delete: 
	@echo "[DELETE] $(2)"
	@rm -rf $(2)
endef

define .mtk.config.generate-rule
$(1): $(2)
$(2): $(3)
	@echo "[CONFIG] copy $(3)"
	@echo "           to $(2)"
	@mkdir -p $(dir $(2))
	@cp -f $(3) $(2)
endef

define .mtk.config.generate-auto-merge-rules
.PHONY: mediatek-configs.$(1)
mediatek-configs: mediatek-configs.$(1)
mediatek-configs.$(1): $(2)
$(2): PRIVATE_FILE_LIST := $(3)
$(2): $(3) $(4)
	@echo "[CONFIG] generate $(2)"
	@$(5) $(3) > $(2)
endef

define .mtk.config.generate-auto-rules
$(eval include $(MTK_ROOT_CONFIG)/common/autoconfig/$1/config.mk) \
$(eval _merge_order_ := $(call wildcard2,$(foreach m,$(merge-order),$(foreach p,\
  $(call reverse,$(MTK_CUSTOM_FOLDERS)),$(MTK_ROOT_CONFIG)/$(p)/autoconfig/$1/$(m))))) \
$(eval _target_file_ := $(MTK_ROOT_GEN_CONFIG)/$1) \
$(if $(call seq,auto-merge,$(config-type)),$(eval \
  $(call .mtk.config.generate-auto-merge-rules,$1,$(_target_file_),$(_merge_order_)\
         ,$(MTK_ROOT_GEN_CONFIG),$(merge-command))
),$(eval #################### add your own config file parsing rule here ###################### )
)$(_target_file_)
endef

# * mtk.config.generate-rules - create rules for auto generating configuration files
# parameter: none
# return:    auto generated file list
#define mtk.config.generate-rules
#endef

# here we use the config out folder "mediatek/config/out/<prj>" as the flag 
# for recognizing if we just change our target project. every time user builds,
# folders for targets other than current one will be touched. Thus when user changes
# target, the target folder will be newer than previous one.
# So please set dependency from all project-dependent files (such .config) to mediatek/config/out/<pr>!
# -c no-such-file is used to suppress warning message
#$(shell touch -c no-such-file \
#        $(filter-out $(call wildcard2,$(call relative-path,../../config/out/$(FULL_PROJECT))), \
#        $(call wildcard2,$(call relative-path,../../config/out/*))))
$(shell touch -c no-such-file \
        $(filter-out $(call wildcard2,$(MTK_ROOT_GEN_CONFIG)), \
        $(call wildcard2,$(MTK_ROOT_CONFIG)/out/*/autoconfig)))

# * mtk.config.generate-rules - generate rules for.config folder generation
# example usage
# a. associate.config files with target "all"
#    $(call mtk.config.generate-rules,all)
define mtk.config.generate-rules
$(eval \
.PHONY: mediatek-configs
$(1): mediatek-configs
$(1): $(MTK_ROOT_GEN_CONFIG)
$(MTK_ROOT_GEN_CONFIG):
	@mkdir -p $$@
$(1): $(MTK_ROOT_CONFIG_OUT)/ProjectConfig.mk
$(MTK_ROOT_CONFIG_OUT):
	@mkdir -p $$@
$(MTK_ROOT_CONFIG_OUT)/ProjectConfig.mk: $(MTK_ROOT_CONFIG_OUT) $(MTK_PROJECT_CONFIGS)
	@echo "[CONFIG] generate $$@"
	@if [ -e $$@ ]; then chmod u+w $$@; fi
	@python $(MTK_PATH_BUILD)/tools/config/merge-project.py $(MTK_PROJECT_CONFIGS) > $$@
) \
$(if $(filter yes, $(strip $(BUILD_KERNEL))), \
     $(foreach c,$(MTK_AUTOCONFIG_LIST),$(call .mtk.config.generate-auto-rules,$c)), \
) \
$(if $(MTK_ROOT_CONFIG),$(strip \
  $(eval _confflist_ :=) $(eval _conffmap_  :=)  \
  $(foreach d,$(addprefix $(MTK_ROOT_CONFIG)/,$(MTK_CUSTOM_FOLDERS)),\
    $(eval _dirs := $(if $(CUSTOMER),$(filter-out $(MTK_ROOT_CONFIG)/$(CUSTOMER),$(d)),$(d))) \
   $(if $(_dirs),\
      $(eval _files := $(filter-out $(_confflist_), \
        $(patsubst $(d)/%,%,$(shell find -L $(_dirs) \( ! -name .\* \) -type f | \
           grep -v "autoconfig/\|ProjectConfig\.mk")))) \
      $(foreach f,$(_files), \
        $(eval _confflist_ += $(f)) \
        $(eval _conffmap_  += $(MTK_ROOT_CONFIG_OUT)/$(f):$(d)/$(f)) \
      ) \
    ,) \
  ) \
  $(if $(call wildcard2,$(MTK_ROOT_CONFIG_OUT)),\
    $(foreach f,$(filter-out $(MTK_ROOT_CONFIG_OUT)/ProjectConfig.mk \ 
                             $(call wildcard2,$(MTK_ROOT_CONFIG_OUT)/autoconfig/*) \
                             $(foreach f,$(_conffmap_),$(word 1,$(subst :, ,$f))),\
      $(shell find $(if $(2),$(addprefix $(MTK_ROOT_CONFIG_OUT)/,$(2)),$(MTK_ROOT_CONFIG_OUT)) \
          -type f 2> /dev/null)),\
      $(eval $(call .mtk.config.delete-rule,$(1),$(f)))) \
  ,) \
  $(foreach f,$(_conffmap_),\
    $(eval $(call .mtk.config.generate-rule,$(1),$(word 1,$(subst :, ,$(f))),$(word 2,$(subst :, ,$(f))))) \
    $(word 1,$(subst :, ,$(f))) \
)),)
endef
# Example Config File
# prepare mediatek/config/common/xxx/config.mk with following content:
# config-type   := auto-merge
# merge-command := python $(MTK_ROOT_BUILD)/tools/config/merge.py
# merge-order   := \
#     common \
#     platform \
#     $(if $(seq has-feature,wifi),wifi,) \
#     BT \
#     AEE \
#     project \
#     flavor
#     ...
# 

# Example Usage
# include ../Makefile
# $(call codebase-path)
# .PHONY: mtk-config-files
# *: mtk-config-folder
# mtk-config-folder: mtk-config-files
# mtk-config-files := $(strip $(call mtk.config.generate-rules,mtk-config-files))
# config-files: ;

