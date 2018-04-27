


#define WIFI_CONF_FORMAT {0, 0, 0, 1}
const uint8_t wifi_conf_format[] = WIFI_CONF_FORMAT;
#define WIFI_CONF_START 30
struct dataStruct {
  uint8_t format[4];
  float p;
  float i;
  float d;
  int16_t MAXACCEL;
  int16_t MAXVELOC;
} DataConf = {
  WIFI_CONF_FORMAT,
  1.00,
  -0.1,
  0.006,
  10000,
  10000
};

