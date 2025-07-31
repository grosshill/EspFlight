#ifndef DEBUG_UTILS_H
#define DEBUG_UTILS_H
#include "esp_err.h"
#include "esp_log.h"

#ifdef DEBUG
#define EF_ERR_CHECK(x, tag) do {                       \
    esp_err_t err_rc_ = (x);                            \
    if (err_rc_ != ESP_OK) {                            \
        ESP_LOGI(tag, "Error in %s | %s",               \
                 __func__, esp_err_to_name(err_rc_));   \
        return err_rc_;                                 \
    }                                                   \
} while(0)
#else
#define EF_ERR_CHECK(x, tag) do {                       \
    (x);                                                \
} while(0)
#endif

#endif