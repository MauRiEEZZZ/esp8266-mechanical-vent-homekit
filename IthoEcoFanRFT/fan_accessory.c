
#include <homekit/homekit.h>
#include <homekit/characteristics.h>

void my_accessory_identify(homekit_value_t _value) {
  printf("accessory identify\n");
}

homekit_characteristic_t ch_fan_active = HOMEKIT_CHARACTERISTIC_(ACTIVE, 0);
homekit_characteristic_t ch_fan_rotation_speed = HOMEKIT_CHARACTERISTIC_(ROTATION_SPEED, 0, .min_step = (float[]) {50});

homekit_characteristic_t ch_timer_short_name = HOMEKIT_CHARACTERISTIC_(NAME, "Ventileer kort");
homekit_characteristic_t ch_timer_short_on = HOMEKIT_CHARACTERISTIC_(ON, false);

homekit_characteristic_t ch_timer_long_name = HOMEKIT_CHARACTERISTIC_(NAME, "Ventileer lang");
homekit_characteristic_t ch_timer_long_on = HOMEKIT_CHARACTERISTIC_(ON, false);

homekit_accessory_t *accessories[] = {
    HOMEKIT_ACCESSORY(.id=1, .category=homekit_accessory_category_fan, .services=(homekit_service_t*[]) {
        HOMEKIT_SERVICE(ACCESSORY_INFORMATION, .characteristics=(homekit_characteristic_t*[]) {
            HOMEKIT_CHARACTERISTIC(NAME, "RFT Ventilatie via Homekit"),
            HOMEKIT_CHARACTERISTIC(MANUFACTURER, "Itho CVE"),
            HOMEKIT_CHARACTERISTIC(SERIAL_NUMBER, "123-456-789"),
            HOMEKIT_CHARACTERISTIC(MODEL, "IthoCVE ECO RFT Ventilatie Low Medium High"),
            HOMEKIT_CHARACTERISTIC(FIRMWARE_REVISION, "1.0"),
            HOMEKIT_CHARACTERISTIC(IDENTIFY, my_accessory_identify),
            NULL
        }),
        HOMEKIT_SERVICE(FAN2, .primary=true, .characteristics=(homekit_characteristic_t*[]){
          HOMEKIT_CHARACTERISTIC(NAME, "Ventilatie"),
          &ch_fan_active,
          &ch_fan_rotation_speed,
          NULL
        }),
        NULL
    }),
    HOMEKIT_ACCESSORY(.id=2, .category=homekit_accessory_category_switch, .services=(homekit_service_t*[]) {
        HOMEKIT_SERVICE(ACCESSORY_INFORMATION, .characteristics=(homekit_characteristic_t*[]) {
            HOMEKIT_CHARACTERISTIC(NAME, "Ventileer kort"),
            HOMEKIT_CHARACTERISTIC(IDENTIFY, my_accessory_identify),
            NULL
        }),
        HOMEKIT_SERVICE(SWITCH, .primary=true, .characteristics=(homekit_characteristic_t*[]){
          &ch_timer_short_name,
          &ch_timer_short_on,
          NULL
        }),
        NULL
    }),
    HOMEKIT_ACCESSORY(.id=3, .category=homekit_accessory_category_switch, .services=(homekit_service_t*[]) {
        HOMEKIT_SERVICE(ACCESSORY_INFORMATION, .characteristics=(homekit_characteristic_t*[]) {
            HOMEKIT_CHARACTERISTIC(NAME, "Ventileer lang"),
            HOMEKIT_CHARACTERISTIC(IDENTIFY, my_accessory_identify),
            NULL
        }),
        HOMEKIT_SERVICE(SWITCH, .primary=false, .characteristics=(homekit_characteristic_t*[]){
          &ch_timer_long_name, 
          &ch_timer_long_on,
          NULL
        }),
        NULL
    }),
    NULL
};

homekit_server_config_t config = {
    .accessories = accessories,
    .password = "111-11-111"
};
