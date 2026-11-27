#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initialize native ESP32-S3 USB Serial/JTAG binary transport (no UART bridge lag).
 */
esp_err_t protocol_init(void);

/**
 * Core 0 protocol task: parses both SMC3 v0.70 + SimSync 5-byte frames AND
 * native ESP32-S3 Turbo v2 binary 12-bit frames simultaneously, streams telemetry,
 * and handles NVS persistence safely off Core 1.
 */
void protocol_task(void *arg);

#ifdef __cplusplus
}
#endif
