#!/bin/bash
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


# shell.sh - codebase variables initializer for shell script
# usage:
#   shell.sh  <PATH_TO_ROOT> <POSTFIX_FOR_MTK_PATH> <PROJECT_NAME>
# e.g., ../../../mediatek/build/shell.sh e1k ../../.. kernel
# then you'll have variables such as 
#   MTK_PATH_PLATFORM pointing to ../../../mediatek/platform/mt6516/kernel

if [ -z ${TARGET_PRODUCT} ]; then TARGET_PRODUCT=$3; fi
if [ -z ${TARGET_PRODUCT} ]; then
    echo "*** TARGET_PRODUCT is not set. stop"
    exit
fi


# export variables to shell environments
eval `TARGET_PRODUCT=${TARGET_PRODUCT} _prefix_=$1 _postfix_=$2 make -i -f $1/mediatek/build/libs/shell.mk`

# for legacy "Download folder". Will be removed once nobody use it.
function make_legacy_download_folder() {
  legacy_download_path=${TO_ROOT}/out/Download
  if [ ! -d ${legacy_download_path} ]; then
    mkdir -p ${legacy_download_path}/sdcard
    mkdir -p ${legacy_download_path}/flash
  fi
  if [ ! -e Download ]; then
      ln -s ${legacy_download_path} Download
  fi
  legacy_download_path=${TO_ROOT}/out/Download/$1
}

function copy_to_legacy_download_folder() {
  for item in $@; do
    if [ -e $item ]; then
      chmod u+w $item
      cp -f $item $legacy_download_path/
    fi
  done
}

function copy_to_legacy_download_flash_folder() {
  make_legacy_download_folder flash
  copy_to_legacy_download_folder $@
}

function copy_to_legacy_download_sdcard_folder() {
  make_legacy_download_folder sdcard
  copy_to_legacy_download_folder $@
}

