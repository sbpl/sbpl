/*
 * Copyright (c) 2008, Maxim Likhachev
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Pennsylvania nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

 #include <cstdio>
 #ifdef ROS
 #include <ros/ros.h>
 #endif

#ifndef __CONFIG_H_
#define __CONFIG_H_

/**
 * \brief if set, then heuristic is used if available
 */
#define USE_HEUR 1

/**
 * \brief memory debugging (available for vc++)
 */
#ifdef WIN32
#define MEM_CHECK 0
#endif

/**
 * \brief regular debugging
 */
#define DEBUG 0

/**
 * \brief timing debugging
 */
#define TIME_DEBUG 0

/**
 * \brief small epsilon for various floating error checking
 */
#define ERR_EPS 0.0000001

#define SBPL_LEVEL_NONE   0
#define SBPL_LEVEL_DEBUG  1
#define SBPL_LEVEL_INFO   2
#define SBPL_LEVEL_WARN   3
#define SBPL_LEVEL_ERROR  4
#define SBPL_LEVEL_FATAL  5

typedef int (*SBPL_PRINT_TEXT_FP)(int level, const char*);
typedef int (*SBPL_FFLUSH_TEXT_FP)(FILE*);

void SET_SBPL_PRINT_TEXT_FP(SBPL_PRINT_TEXT_FP fptr); 
void SET_SBPL_FFLUSH_TEXT_FP(SBPL_FFLUSH_TEXT_FP fptr);

int SBPL_PRINTALL(int level, const char* format, ...);
int SBPL_FPRINTALL(FILE* file, const char* format, ...);
int SBPL_FFLUSHALL(FILE* file);

// Standard Output Logger Macros
#ifdef ROS
#define SBPL_DEBUG                  ROS_DEBUG
#define SBPL_DEBUG_NAMED(a,...)     ROS_DEBUG_NAMED("SBPL_" #a,__VA_ARGS__)
#define SBPL_INFO                   ROS_INFO
#define SBPL_WARN                   ROS_WARN
#define SBPL_ERROR                  ROS_ERROR
#define SBPL_FATAL                  ROS_FATAL
#else
  #if DEBUG
#define SBPL_DEBUG(...)             SBPL_PRINTALL(SBPL_LEVEL_DEBUG, __VA_ARGS__)
#define SBPL_DEBUG_NAMED(file, ...) SBPL_FPRINTALL(file, __VA_ARGS__)
#define SBPL_INFO(...)              SBPL_PRINTALL(SBPL_LEVEL_INFO, __VA_ARGS__)
#define SBPL_WARN(...)              SBPL_PRINTALL(SBPL_LEVEL_WARN, __VA_ARGS__)
#define SBPL_ERROR(...)             SBPL_PRINTALL(SBPL_LEVEL_ERROR, __VA_ARGS__)
#define SBPL_FATAL(...)             SBPL_PRINTALL(SBPL_LEVEL_FATAL, __VA_ARGS__)
  #else
#define SBPL_DEBUG(...)
#define SBPL_DEBUG_NAMED(file, ...)
#define SBPL_INFO(...)
#define SBPL_WARN(...)
#define SBPL_ERROR(...)
#define SBPL_FATAL(...)
  #endif
#endif

// File Output Logger Macros
#ifdef ROS
#define SBPL_DEBUG_NAMED(a,...)     ROS_DEBUG_NAMED("SBPL_" #a,__VA_ARGS__)

#define SBPL_FOPEN(...)             (FILE*)1
#define SBPL_FCLOSE(...)
#define SBPL_PRINTF                 ROS_DEBUG
#define SBPL_FPRINTF(a,...)         ROS_DEBUG_NAMED("SBPL_" #a,__VA_ARGS__)
#define SBPL_FFLUSH(...)
#else
  #if DEBUG
#define SBPL_FOPEN                  fopen
#define SBPL_FCLOSE                 fclose
#define SBPL_PRINTF(...)            SBPL_PRINTALL(SBPL_LEVEL_NONE, __VA_ARGS__)
#define SBPL_FPRINTF(file, ...)     SBPL_FPRINTALL(file, __VA_ARGS__)
#define SBPL_FFLUSH(file)           SBPL_FFLUSHALL(file)
  #else
#define SBPL_FOPEN(file, ...)       (FILE*)1
#define SBPL_FCLOSE(...)
#define SBPL_PRINTF(...)
#define SBPL_FPRINTF(file, ...)
#define SBPL_FFLUSH(file)
  #endif
#endif

#endif

