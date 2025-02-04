// © Jakub Jandus 2023

#include "NTP.h"

// NTP
// const char *ntpServer = "pool.ntp.org";
// const char *ntpServer = "time.google.com";

const char *ntpServer = "192.168.1.1";

const int retryTimeout = 3600; // milliseconds do not set to less than 3000

// NTP setup, this should synchronize the time within milliseconds and keep it synchronized periodically
bool NTPSetup()
{
    bool successInit = false;
    // time is kept in UTC format as to avoid any timezone or daylight savings issues
    NTP.setMaxNumSyncRetry(5);
    NTP.setNTPTimeout(retryTimeout);
    successInit = NTP.begin(ntpServer, false);
    return successInit;
}

// NTP manual update - do not call unless needed
bool NTPUpdateManual()
{
    NTP.getTime();
    return true;
}