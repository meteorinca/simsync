#include "nvs_store.h"
#include "pid_control.h"

#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"

static const char *TAG = "nvs_store";
#define NVS_NS  "simsync"
#define NVS_KEY "cfg_v1"

esp_err_t nvs_store_init(void) {
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS partition reset required, erasing...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    return err;
}

esp_err_t nvs_store_load(controller_cfg_t *cfg) {
    pid_default_config(cfg);

    nvs_handle_t h = 0;
    esp_err_t err = nvs_open(NVS_NS, NVS_READONLY, &h);
    if (err != ESP_OK) {
        ESP_LOGI(TAG, "No saved NVS config yet, using safe bench defaults");
        return err;
    }

    controller_cfg_t loaded;
    size_t len = sizeof(loaded);
    err = nvs_get_blob(h, NVS_KEY, &loaded, &len);
    nvs_close(h);

    if (err == ESP_OK && len == sizeof(loaded) &&
        loaded.magic == cfg->magic && loaded.version == cfg->version) {
        *cfg = loaded;
        ESP_LOGI(TAG, "Loaded saved config from NVS (active_dof=%u, pwm=%u/%u kHz)",
                 cfg->active_dof, cfg->pwm12_khz, cfg->pwm3_khz);
        return ESP_OK;
    }

    ESP_LOGW(TAG, "Saved NVS config invalid/outdated, keeping defaults");
    return ESP_ERR_INVALID_STATE;
}

esp_err_t nvs_store_save(const controller_cfg_t *cfg) {
    nvs_handle_t h = 0;
    esp_err_t err = nvs_open(NVS_NS, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_open failed: %s", esp_err_to_name(err));
        return err;
    }

    err = nvs_set_blob(h, NVS_KEY, cfg, sizeof(*cfg));
    if (err == ESP_OK) {
        err = nvs_commit(h);
    }
    nvs_close(h);

    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Saved controller config to NVS flash");
    } else {
        ESP_LOGE(TAG, "NVS save failed: %s", esp_err_to_name(err));
    }
    return err;
}
