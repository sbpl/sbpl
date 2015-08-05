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
 *     * Neither the name of the Carnegie Mellon University nor the names of its
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

#include <sbpl/config.h>
#include <cstdarg>
#include <cstdlib>
#include <cstring>

#define SBPL_PRINTF_BUFFER_SIZE 1024

static int SBPL_DEFAULT_DEBUG_OUTPUT_LOGGER(int, const char*);
static int SBPL_DEFAULT_DEBUG_OUTPUT_FLUSH(FILE*);

// set appropriate default logger
#if DEBUG && !defined(ROS)
static SBPL_PRINT_TEXT_FP sbpl_print_fp = SBPL_DEFAULT_DEBUG_OUTPUT_LOGGER;
static SBPL_FFLUSH_TEXT_FP sbpl_fflush_fp = SBPL_DEFAULT_DEBUG_OUTPUT_FLUSH;
#else
static SBPL_PRINT_TEXT_FP sbpl_print_fp = NULL;
static SBPL_FFLUSH_TEXT_FP sbpl_fflush_fp = NULL;
#endif

void SET_SBPL_PRINT_TEXT_FP(SBPL_PRINT_TEXT_FP fptr) { sbpl_print_fp = fptr; }
void SET_SBPL_FFLUSH_TEXT_FP(SBPL_FFLUSH_TEXT_FP fptr) { sbpl_fflush_fp = fptr; }

int SBPL_PRINTALL(int level, const char* format, ...)
{
	int retVal = 0;
    if (sbpl_print_fp) {
        // parse arguments and pass resultant string to configured logger
        va_list args;
        va_start(args, format);
    	char buffer[SBPL_PRINTF_BUFFER_SIZE] = {0};
        retVal = vsnprintf(buffer,SBPL_PRINTF_BUFFER_SIZE - 1, format, args);
        if (retVal < 0) {
            printf("SBPL_PRINTALL::ERROR, could not complete call to vsnprintf()");
        }
        else {
            if (retVal == SBPL_PRINTF_BUFFER_SIZE) {
                printf("SBPL_PRINTALL::ERROR, SBPL_PRINTF_BUFFER_SIZE: %d not large enough", SBPL_PRINTF_BUFFER_SIZE);
            }
            if(buffer[retVal-1] == '\n') { // Remove newline
                buffer[retVal-1] = '\0';
            }
            sbpl_print_fp(level, &buffer[0]);
        }
    	va_end(args);
    }
	return retVal;
}

int SBPL_FPRINTALL(FILE* file, const char* format, ...)
{
	int retVal = 0;
    va_list args;
    va_start(args, format);
    if ((file == stdout) && (sbpl_print_fp != NULL)) {
        char buffer[SBPL_PRINTF_BUFFER_SIZE] = {0};
        retVal = vsnprintf(buffer,SBPL_PRINTF_BUFFER_SIZE-1,format, args);
        if (retVal < 0) {
            printf("SBPL_PRINTALL::ERROR, could not complete call to vsnprintf()");
        }
        else {
            if (retVal == SBPL_PRINTF_BUFFER_SIZE) {
                printf("SBPL_PRINTALL::ERROR, SBPL_PRINTF_BUFFER_SIZE: %d not large enough", SBPL_PRINTF_BUFFER_SIZE);
            }
            if(buffer[retVal-1] == '\n') { // Remove newline
                    buffer[retVal-1] = '\0';
            }
            sbpl_print_fp(SBPL_LEVEL_NONE, &buffer[0]);
        }
    }
    else {
        retVal = vfprintf(file, format, args);
    }
    va_end (args);
    return retVal;
}

int SBPL_FFLUSHALL(FILE* file)
{
	int retVal = 0;
   	if (sbpl_fflush_fp) {
		retVal = sbpl_fflush_fp(file);
	}
	return retVal;
}

int SBPL_DEFAULT_DEBUG_OUTPUT_LOGGER(int level, const char* msg)
{
    switch (level)
    {
    case SBPL_LEVEL_NONE:
    case SBPL_LEVEL_DEBUG:
    case SBPL_LEVEL_INFO:
    case SBPL_LEVEL_WARN:
        return printf("%s\n", msg);
    case SBPL_LEVEL_ERROR:
    case SBPL_LEVEL_FATAL:
        return fprintf(stderr, "%s\n", msg);
    default:
        return -1;
    }
}

int SBPL_DEFAULT_DEBUG_OUTPUT_FLUSH(FILE* f)
{
    fflush(f);
}

