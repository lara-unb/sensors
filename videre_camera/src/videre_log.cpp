/**
 * @file    videre_camera.cpp
 * @author  George Andrew Brindeiro
 * @date    26/07/2012
 *
 * @attention Copyright (C) 2012
 * @attention Laboratório de Automação e Robótica (LARA)
 * @attention Universidade de Brasília (UnB)
 */

#include <videre_camera/videre_log.h>

void VC_LOG(LogLevel level, const char* msg, ...)
{
    va_list fmtargs;
    char buffer[1024];

    va_start(fmtargs, msg);
    vsnprintf(buffer, sizeof(buffer)-1, msg, fmtargs);
    va_end(fmtargs);

    switch(level)
    {
        case INFO:
            printf("[VC] %s\n", buffer);
            break;
        case ERROR:
            printf("\033[33m[VC] Error: %s\033[0;0m\n", buffer);
            break;
    }
}
