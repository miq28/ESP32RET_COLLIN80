#pragma once

#include "Arduino.h"

#if defined(WEACT_STUDIO_CAN485_V1)

void rgbStatusInit();
void rgbStatusLoop();

void rgbCanRxActivity();
void rgbCanTxActivity();
void rgbCanError(bool active);

void rgbWifiSTAConnected(bool connected);
void rgbWifiAPClientConnected(bool connected);
void rgbWifiModeSTA(bool active);
void rgbWifiModeAP(bool active);

#else

inline void rgbStatusInit() {}
inline void rgbStatusLoop() {}

inline void rgbCanRxActivity() {}
inline void rgbCanTxActivity() {}
inline void rgbCanError(bool active) {}

inline void rgbWifiSTAConnected(bool connected) {}
inline void rgbWifiAPClientConnected(bool connected) {}
inline void rgbWifiModeSTA(bool active) {}
inline void rgbWifiModeAP(bool active) {}

#endif