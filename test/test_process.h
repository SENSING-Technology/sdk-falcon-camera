#ifndef __TEST_PROCESS_H
#define __TEST_PROCESS_H

#include <iostream>
#include <stdio.h>

#include "nvpfm.hpp"
#include "nvpfm.h"

using namespace std;

void depthcallback(void *data, void *userdata);
void sensorcfgcallback(void *pdata, int len, void *userdata);
void othercallback(void *data, void *userdata);
void imucallback(void *data, void *userdata);
void rgbcallback(void *data, void *userdata);
void leftircallback(void *data, void *userdata);
void rightircallback(void *data, void *userdata);
// void exposurecallback(void *pdata, int len, void *userdata);
void highprecisioncallback(void *pdata, int len, void *userdata);
void repeatfiltercallback(void *pdata, int len, void *userdata);
void spatialfiltercallback(void *pdata, int len, void *userdata);
void highlightfiltercallback(void *pdata, int len, void *userdata);

#endif // __TEST_PROCESS_H