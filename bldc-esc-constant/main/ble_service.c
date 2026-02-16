#include "ble_service.h"

#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#include "esp_check.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "host/ble_hs.h"
#include "host/ble_gatt.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

static const char *TAG = "BLE_SERVICE";

#define BLE_DEVICE_NAME "AirPurifier_PARK"

static ble_throttle_setter_t s_throttle_setter = NULL;
static uint16_t s_speed_char_handle;
static uint8_t s_own_addr_type;

static const ble_uuid128_t s_service_uuid = BLE_UUID128_INIT(
    0x10, 0x76, 0x65, 0x9a,
    0x31, 0x6c,
    0x43, 0xbf,
    0x95, 0x55,
    0x52, 0xdd, 0xbb, 0x45, 0xaa, 0x01
);

static const ble_uuid128_t s_char_uuid = BLE_UUID128_INIT(
    0x10, 0x76, 0x65, 0x9a,
    0x31, 0x6c,
    0x43, 0xbf,
    0x95, 0x55,
    0x52, 0xdd, 0xbb, 0x45, 0xaa, 0x02
);

static void ble_app_advertise(void);

static int speed_char_access_cb(uint16_t conn_handle,
                                uint16_t attr_handle,
                                struct ble_gatt_access_ctxt *ctxt,
                                void *arg)
{
    (void)conn_handle;
    (void)attr_handle;
    (void)arg;

    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) {
        return BLE_ATT_ERR_UNLIKELY;
    }

    if (ctxt->om == NULL || OS_MBUF_PKTLEN(ctxt->om) < 1) {
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }

    uint8_t received_value = 0;
    const int rc_copy = ble_hs_mbuf_to_flat(ctxt->om, &received_value, sizeof(received_value), NULL);
    if (rc_copy != 0) {
        return BLE_ATT_ERR_UNLIKELY;
    }

    if (received_value > 100) {
        received_value = 100;
    }

    ESP_LOGI(TAG, "Received throttle percent: %u", received_value);

    if (s_throttle_setter == NULL) {
        return BLE_ATT_ERR_UNLIKELY;
    }

    esp_err_t err = s_throttle_setter(received_value);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set throttle, err=0x%x", err);
        return BLE_ATT_ERR_UNLIKELY;
    }

    return 0;
}

static const struct ble_gatt_svc_def gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &s_service_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = &s_char_uuid.u,
                .access_cb = speed_char_access_cb,
                .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
                .val_handle = &s_speed_char_handle,
            },
            {0}
        },
    },
    {0}
};

static int gap_event_cb(struct ble_gap_event *event, void *arg)
{
    (void)arg;

    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            ESP_LOGI(TAG, "BLE connected");
        } else {
            ESP_LOGI(TAG, "BLE connect failed; restarting advertising");
            ble_app_advertise();
        }
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "BLE disconnected; restarting advertising");
        ble_app_advertise();
        return 0;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        ESP_LOGI(TAG, "Advertising completed; restarting advertising");
        ble_app_advertise();
        return 0;

    default:
        return 0;
    }
}

static void ble_app_advertise(void)
{
    struct ble_hs_adv_fields adv_fields = {0};
    adv_fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    adv_fields.uuids128 = &s_service_uuid;
    adv_fields.num_uuids128 = 1;
    adv_fields.uuids128_is_complete = 1;

    int rc = ble_gap_adv_set_fields(&adv_fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_adv_set_fields failed: %d", rc);
        return;
    }

    struct ble_hs_adv_fields rsp_fields = {0};
    const char *device_name = ble_svc_gap_device_name();
    rsp_fields.name = (const uint8_t *)device_name;
    rsp_fields.name_len = strlen(device_name);
    rsp_fields.name_is_complete = 1;

    rc = ble_gap_adv_rsp_set_fields(&rsp_fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_adv_rsp_set_fields failed: %d", rc);
        return;
    }

    struct ble_gap_adv_params adv_params = {0};
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    rc = ble_gap_adv_start(s_own_addr_type, NULL, BLE_HS_FOREVER, &adv_params, gap_event_cb, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_adv_start failed: %d", rc);
    }
}

static void ble_on_sync(void)
{
    int rc = ble_hs_id_infer_auto(0, &s_own_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_hs_id_infer_auto failed: %d", rc);
        return;
    }

    ble_app_advertise();
}

static void ble_host_task(void *param)
{
    (void)param;
    nimble_port_run();
    nimble_port_freertos_deinit();
}

esp_err_t ble_service_init(ble_throttle_setter_t throttle_setter)
{
    ESP_RETURN_ON_FALSE(throttle_setter != NULL, ESP_ERR_INVALID_ARG, TAG, "throttle_setter is NULL");
    s_throttle_setter = throttle_setter;

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_RETURN_ON_ERROR(nvs_flash_erase(), TAG, "nvs_flash_erase failed");
        err = nvs_flash_init();
    }
    ESP_RETURN_ON_ERROR(err, TAG, "nvs_flash_init failed");

    err = nimble_port_init();
    ESP_RETURN_ON_ERROR(err, TAG, "nimble_port_init failed");

    ble_svc_gap_init();
    ble_svc_gatt_init();
    const int rc_name = ble_svc_gap_device_name_set(BLE_DEVICE_NAME);
    ESP_RETURN_ON_FALSE(rc_name == 0, ESP_FAIL, TAG, "ble_svc_gap_device_name_set failed: %d", rc_name);

    int rc = ble_gatts_count_cfg(gatt_svcs);
    ESP_RETURN_ON_FALSE(rc == 0, ESP_FAIL, TAG, "ble_gatts_count_cfg failed: %d", rc);

    rc = ble_gatts_add_svcs(gatt_svcs);
    ESP_RETURN_ON_FALSE(rc == 0, ESP_FAIL, TAG, "ble_gatts_add_svcs failed: %d", rc);

    ble_hs_cfg.sync_cb = ble_on_sync;

    nimble_port_freertos_init(ble_host_task);

    ESP_LOGI(TAG, "NimBLE initialized with device name: %s", BLE_DEVICE_NAME);
    return ESP_OK;
}
