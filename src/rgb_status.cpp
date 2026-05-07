#include "rgb_status.h"

#if defined(WEACT_STUDIO_CAN485_V1)

#include <Adafruit_NeoPixel.h>
#include "config.h"

static Adafruit_NeoPixel pixel(1, RGB_LED, NEO_GRB + NEO_KHZ800);

static volatile uint32_t rxUntil = 0;
static volatile uint32_t txUntil = 0;
static volatile bool errorState = false;

static constexpr uint32_t BLINK_TIME_MS = 30;

static volatile bool wifiSTAActive = false;
static volatile bool wifiAPActive = false;
static volatile bool wifiSTAConnected = false;
static volatile bool wifiAPClientConnected = false;

static void applyColor(uint8_t r, uint8_t g, uint8_t b)
{
    pixel.setPixelColor(0, pixel.Color(r, g, b));
    pixel.show();
}

void rgbStatusInit()
{
    pixel.begin();
    pixel.clear();
    pixel.show();
}

void rgbCanRxActivity()
{
    rxUntil = millis() + BLINK_TIME_MS;
}

void rgbCanTxActivity()
{
    txUntil = millis() + BLINK_TIME_MS;
}

void rgbCanError(bool active)
{
    errorState = active;
}

void rgbStatusLoop()
{
    uint32_t now = millis();

    bool slowBlink = ((now % 1000) < BLINK_TIME_MS);

    // ===== WIFI OVERLAY =====

    bool wifiOverlayActive = false;

    uint8_t wifiR = 0;
    uint8_t wifiG = 0;
    uint8_t wifiB = 0;

    // ===== STA =====
    if (wifiSTAActive)
    {
        // disconnected -> orange blink
        if (!wifiSTAConnected && slowBlink)
        {
            wifiOverlayActive = true;

            wifiR = 255;
            wifiG = 64;
            wifiB = 0;
        }
    }

    // ===== AP =====
    else if (wifiAPActive)
    {
        // no client -> white blink
        if (!wifiAPClientConnected && slowBlink)
        {
            wifiOverlayActive = true;

            wifiR = 255;
            wifiG = 255;
            wifiB = 255;
        }
    }

    // Highest priority
    if (errorState)
    {
        applyColor(255, 0, 0);
        return;
    }

    bool rxActive = (int32_t)(rxUntil - now) > 0;
    bool txActive = (int32_t)(txUntil - now) > 0;

    uint8_t r = 0;
    uint8_t g = 0;
    uint8_t b = 0;

    // RX + TX simultaneously
    if (rxActive && txActive)
    {
        g = 255;
        b = 255;
    }
    else if (rxActive)
    {
        b = 255;
    }
    else if (txActive)
    {
        g = 255;
    }

    // ===== WIFI OVERLAY =====
    if (wifiOverlayActive)
    {
        r = wifiR;
        g = wifiG;
        b = wifiB;
    }

    applyColor(r, g, b);
}

void rgbWifiSTAConnected(bool connected)
{
    wifiSTAConnected = connected;
}

void rgbWifiAPClientConnected(bool connected)
{
    wifiAPClientConnected = connected;
}

void rgbWifiModeSTA(bool active)
{
    wifiSTAActive = active;

    if (active)
        wifiAPActive = false;
}

void rgbWifiModeAP(bool active)
{
    wifiAPActive = active;

    if (active)
        wifiSTAActive = false;
}

#endif