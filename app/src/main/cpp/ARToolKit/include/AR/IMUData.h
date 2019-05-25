/*
 *  arConfig.h
 *  ARToolKit5
 *
 */

#pragma once

#ifndef IMUData_H
#define IMUData_H

#include <AR/ar.h>
#ifndef _WIN32 // errno is defined in stdlib.h on Windows.
#  ifdef EMSCRIPTEN // errno is not in sys/
#    include <errno.h>
#  else
#    include <sys/errno.h>
#  endif
#endif
#include <AR/config.h>
#include <AR/arConfig.h>
#ifdef __ANDROID__
#  include <jni.h>
#  include <android/log.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

//const int g_nBufferSize = 500;

//extern char g_szBuffer[];
//extern 
ARIMUData arIMUData;

#ifdef __cplusplus
}
#endif //#ifdef __cplusplus

#endif //#ifndef AR_H
