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

//Macros for compiling with an without ROS
#ifdef ROS
#include <ros/ros.h>

#define SBPL_DEBUG          ROS_DEBUG
#define SBPL_DEBUG_NAMED(a,...) ROS_DEBUG_NAMED("SBPL_" #a,__VA_ARGS__)
#define SBPL_INFO           ROS_INFO
#define SBPL_WARN           ROS_WARN
#define SBPL_ERROR          ROS_ERROR
#define SBPL_FATAL          ROS_FATAL

#define SBPL_FOPEN(...)     (FILE*)1
#define SBPL_FCLOSE(...)
#define SBPL_PRINTF         ROS_DEBUG
#define SBPL_FPRINTF(a,...) ROS_DEBUG_NAMED("SBPL_" #a,__VA_ARGS__)
#define SBPL_FFLUSH(...)
#else
#include <cstdio>

#define SBPL_DEBUG          printf
#define SBPL_DEBUG_NAMED    fprintf
#define SBPL_INFO           printf
#define SBPL_WARN           printf
#define SBPL_ERROR          printf
#define SBPL_FATAL          printf

#define SBPL_FOPEN          fopen
#define SBPL_FCLOSE         fclose
#define SBPL_PRINTF         printf
#define SBPL_FPRINTF        fprintf
#define SBPL_FFLUSH         fflush
#endif

#endif

