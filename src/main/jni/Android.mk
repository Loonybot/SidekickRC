LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_MODULE := sidekicktimer
LOCAL_SRC_FILES := fastclock.c

LOCAL_ARM_MODE := arm

include $(BUILD_SHARED_LIBRARY)
