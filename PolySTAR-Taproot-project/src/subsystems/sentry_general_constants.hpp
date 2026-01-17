#pragma once

#include <stdint.h>

/**
 * Time (ms) the sentry will wait before listening to jetson commands
 * We do this since we can't read the refSerial data (for some unknown reason)
 */ 
constexpr uint32_t START_MATCH_WAIT_TIME = 30 * 1000;