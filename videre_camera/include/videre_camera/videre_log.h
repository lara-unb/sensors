/**
 * @file     videre_log.h
 * @author   George Andrew Brindeiro
 * @date     26/07/2012
 *
 * @brief Simple colored logging with prefix
 *
 * Provides colored logging functionality for messages in VidereCamera
 *
 * Contact: georgebrindeiro@lara.unb.br
 *
 * Revisions:
 * [26/07/2012] Created
 */

#ifndef VIDERE_LOG_H
#define VIDERE_LOG_H

#include <cstdarg>
#include <cstdio>

enum LogLevel {ERROR, INFO};

void VC_LOG(LogLevel level, const char* msg, ...);

#endif //VIDERE_LOG_H
