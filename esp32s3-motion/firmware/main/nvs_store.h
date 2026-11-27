#pragma once

#include "esp_err.h"
#include "motion_config.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t nvs_store_init(void);
esp_err_t nvs_store_load(controller_cfg_t *cfg);
esp_err_t nvs_store_save(const controller_cfg_t *cfg);

#ifdef __cplusplus
}
#endif
