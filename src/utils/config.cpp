#include "sbpl/config.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#define SBPL_PRINTF_BUFFER_SIZE 1024

SBPL_PRINT_TEXT_FP sbpl_print_fp = NULL;
SBPL_FFLUSH_TEXT_FP sbpl_fflush_fp = NULL;

void SET_SBPL_PRINT_TEXT_FP(SBPL_PRINT_TEXT_FP fptr) { sbpl_print_fp = fptr; }
void SET_SBPL_FFLUSH_TEXT_FP(SBPL_FFLUSH_TEXT_FP fptr) { sbpl_fflush_fp = fptr; }

int SBPL_PRINTALL(int level, const char* format, ...)
{
	int retVal = 0;
        va_list args;
        va_start (args, format);
	if (sbpl_print_fp == NULL)
	{
		retVal = vprintf(format, args);
	}
	else
	{
        	char buffer[SBPL_PRINTF_BUFFER_SIZE] = {0};
	        retVal = vsnprintf (buffer,SBPL_PRINTF_BUFFER_SIZE-1,format, args);
	        if (retVal < 0)
	        {
	                printf("SBPL_PRINTALL::ERROR, could not complete call to vsnprintf()");
	        }
	        else
	        {
	                if ( (retVal == SBPL_PRINTF_BUFFER_SIZE) )
	                {
	                        printf("SBPL_PRINTALL::ERROR, SBPL_PRINTF_BUFFER_SIZE: %d not large enough", SBPL_PRINTF_BUFFER_SIZE);
	                }
	                if(buffer[retVal-1] == '\n') //Remove newline
	                {
	                        buffer[retVal-1] = '\0';
	                }
	                sbpl_print_fp(level, &buffer[0]);
	        }
	}
	va_end(args);
	return retVal;
}

int SBPL_FPRINTALL(FILE* file, const char* format, ...)
{
	int retVal = 0;
        va_list args;
        va_start (args, format);
        if ( (file == stdout) && (sbpl_print_fp != NULL) )
        {
                char buffer[SBPL_PRINTF_BUFFER_SIZE] = {0};
                retVal = vsnprintf (buffer,SBPL_PRINTF_BUFFER_SIZE-1,format, args);
                if (retVal < 0)
                {
	                printf("SBPL_PRINTALL::ERROR, could not complete call to vsnprintf()");
                }
                else
                {
                        if ( (retVal == SBPL_PRINTF_BUFFER_SIZE) )
                        {
	                        printf("SBPL_PRINTALL::ERROR, SBPL_PRINTF_BUFFER_SIZE: %d not large enough", SBPL_PRINTF_BUFFER_SIZE);
                        }
                        if(buffer[retVal-1] == '\n') //Remove newline
                        {
                                buffer[retVal-1] = '\0';
                        }
	                sbpl_print_fp(SBPL_LEVEL_NONE, &buffer[0]);
                }
        }
        else
        {
                retVal = vfprintf (file, format, args);
        }
        va_end (args);
        return retVal;
}

int SBPL_FFLUSHALL(FILE* file)
{
	int retVal = 0;
   	if(sbpl_fflush_fp == NULL)
	{
		retVal = fflush(file);
	}
	else
	{
		retVal = sbpl_fflush_fp(file);
	}
	return retVal;
}



