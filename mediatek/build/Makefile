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

ifneq (true,$(__mtk_main_makefile_include))
__mtk_main_makefile_include := true

# Makefile - global makefile for all building system to include
ifneq (,$(PROJECT))
TARGET_PRODUCT := $(PROJECT)
endif
ifeq (emulator,$(TARGET_PRODUCT))
TARGET_PRODUCT := generic
endif
_entering_makefile_list_ := $(MAKEFILE_LIST)
ifeq (,$(TARGET_PRODUCT))
  $(error TARGET_PRODUCT/PROJECT is not set)
endif

FULL_PROJECT := $(TARGET_PRODUCT)$(if $(FLAVOR),[$(FLAVOR)],)
MTK_PROJECT  := $(word 1,$(subst [, ,$(FULL_PROJECT)))
export FULL_PROJECT MTK_PROJECT

# * relative-include - include a file with path being relative to whom calls the function.
# this function is mutual exclusive to original include directive in the same makefile. 
# if anyone want to use this function, one must either
#   1. use relative-include instead of include through entire Makefile
#   2. make at least one call of relative-include before any include directive
# the only exception to make a inclusion before relative-include is to include this file.
# parameter:
# 1) file to include (relative to this Makefile's directory). can be empty (include nothing)
define relative-include
$(strip \
$(eval _relative_dir_depth_:=$(if $(_relative_dir_depth_),$(_relative_dir_depth_)/_,_rdpath/_)) \
$(eval $(_relative_dir_depth_):=$(strip $(if $($(_relative_dir_depth_)),$($(_relative_dir_depth_)),\
  $(patsubst %/,%,$(dir $(word $(words $(MAKEFILE_LIST)),$(MAKEFILE_LIST)))))))\
$(eval $(if $(1),include $($(_relative_dir_depth_))/$(1),))\
$(eval $(_relative_dir_depth_)/_:=)\
$(eval _relative_dir_depth_ := $(patsubst %/,%,$(dir $(_relative_dir_depth_)))))
endef
_:=
# * relative-path - return updated filename relative to whom calls the function
# return path relative to makefile's current working directory. it's quite useful when 
# there are too many inclusion between makefile and you don't know where the actual cwd is.
# parameter:
# 1) filename
define relative-path
$(strip \
$(eval _relative_dir_depth_:=$(if $(_relative_dir_depth_),$(_relative_dir_depth_)/_,_rdpath/_)) \
$(eval $(_relative_dir_depth_):=$(strip $(if $($(_relative_dir_depth_)),$($(_relative_dir_depth_)),\
  $(patsubst %/,%,$(dir $(word $(words $(MAKEFILE_LIST)),$(MAKEFILE_LIST)))))))\
$(if $(1),$($(_relative_dir_depth_))/$(1),)\
$(eval $(_relative_dir_depth_)/_:=)\
$(eval _relative_dir_depth_ := $(patsubst %/,%,$(dir $(_relative_dir_depth_)))))
endef

# * codebase-path - initialize ABS++ relative path variables
# parameter: (all optional!)
# 1) Path Postfix : the postfix (e.g., kernel/ ) to add after MTK_PATH_* variables
# 2) Path Prefix  : the prefix  (e.g., ../../) from caller makefile to codebase root.
# 3) Platform     : platform the current project for
# 4) Project      : project name
# exported values:
# MTK_ROOT_*          : to <top>/mediatek/*
# MTK_PATH_*          : to <top>/mediatek/*/<postfix>
# MTK_PATH_PLATFORM   : to <top>/mediatek/<platform>/<postfix>
# MTK_PATH_CUSTOM     : to <top>/mediatek/custom/out/<project>/<postfix>
# MTK_ROOT_GEN_CONFIG : to <top>/mediatek/config/out/<project>/autoconfig
#   under ABS, general RD users should only use MTK_CUSTOM here (to access custom folder)
#   other building system should directly support custom folder.
define codebase-path
$(eval $(if $(3),$(eval _3:=$(3)),$(eval _3:=$(call lc,$(MTK_PLATFORM))))) \
$(eval $(if $(4),$(eval _4:=$(4)),$(eval _4:=$(FULL_PROJECT)))) \
$(eval $(if $(2),$(eval _2:=$(2)/),$(eval _2:=$(call to-root)))) \
$(eval $(if $(1),$(eval _1:=/$(1)),$(eval _1:=/$(call from-root)))) \
$(eval TO_ROOT := $(_2)) \
$(eval MTK_ROOT := $(_2)mediatek) \
$(eval MTK_ROOT_PLATFORM := $(MTK_ROOT)/platform/$(_3)) \
$(eval MTK_ROOT_BUILD := $(MTK_ROOT)/build) \
$(eval MTK_ROOT_SOURCE = $(MTK_ROOT)) \
$(eval MTK_ROOT_SOURCE_OPERATOR := $(MTK_ROOT_SOURCE)/operator) \
$(eval MTK_ROOT_CUSTOM := $(MTK_ROOT)/custom) \
$(eval MTK_ROOT_CUSTOM_OUT := $(MTK_ROOT_CUSTOM)/out/$(FULL_PROJECT)) \
$(eval MTK_ROOT_CONFIG := $(MTK_ROOT)/config) \
$(eval MTK_ROOT_CONFIG_OUT := $(MTK_ROOT_CONFIG)/out/$(FULL_PROJECT)) \
$(eval MTK_ROOT_GEN_CONFIG := $(MTK_ROOT_CONFIG_OUT)/autoconfig) \
$(eval MTK_PATH_PLATFORM := $(MTK_ROOT_PLATFORM)$(_1)) \
$(eval MTK_PATH_BUILD := $(MTK_ROOT_BUILD)$(_1)) \
$(eval MTK_PATH_SOURCE := $(MTK_ROOT_SOURCE)$(_1)) \
$(eval MTK_PATH_CUSTOM := $(MTK_ROOT_CUSTOM_OUT)$(_1)) \
$(eval ###### if you add any exported variables below, please also update shell.mk ###### ) \
$(eval export TO_ROOT MTK_ROOT MTK_ROOT_CUSTOM_OUT MTK_ROOT_CONFIG_OUT) \
$(eval export MTK_ROOT_PLATFORM MTK_ROOT_BUILD MTK_ROOT_SOURCE MTK_ROOT_CUSTOM MTK_ROOT_CONFIG) \
$(eval export MTK_PATH_PLATFORM MTK_PATH_BUILD MTK_PATH_SOURCE MTK_PATH_CUSTOM MTK_ROOT_GEN_CONFIG)
endef

# include individual functionality from separate makefiles.
$(call relative-include,libs/gmsl)        # basic library: GNU GMSL + MTK extended Lib (GMEL)
$(call relative-include,libs/definitions.mk)   # common functions : Functions used in MTK build extension
$(call relative-include,libs/custom.mk)   # custom rules : Functions for custom folders
$(call relative-include,libs/config.mk)   # config rules : Functions for config generation

# let makefiles included in this makefile be invisible to all later makefiles
$(_relative_dir_depth_)/_:=
MAKEFILE_LIST := $(_entering_makefile_list_)
MAKEFILE_LIST := $(filter-out $(word $(words $(MAKEFILE_LIST)),$(MAKEFILE_LIST)),$(MAKEFILE_LIST))

endif # __mtk_main_makefile_include
