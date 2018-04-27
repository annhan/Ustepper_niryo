


void printWiFiConf(void) {
  Serial.println(DataConf.p);
  Serial.println(DataConf.i);
  Serial.println(DataConf.d);
  Serial.println(DataConf.MAXACCEL);
  Serial.println(DataConf.MAXVELOC);
}


bool loadStepConf() {
  Serial.println("A");
  if (EEPROM.read(WIFI_CONF_START + 0) == wifi_conf_format[0] &&
      EEPROM.read(WIFI_CONF_START + 1) == wifi_conf_format[1] &&
      EEPROM.read(WIFI_CONF_START + 2) == wifi_conf_format[2] &&
      EEPROM.read(WIFI_CONF_START + 3) == wifi_conf_format[3])
  {
    for (unsigned int t = 0; t < sizeof(DataConf); t++) {
      *((char*)&DataConf + t) = EEPROM.read(WIFI_CONF_START + t); //& lÃ  Ä‘á»‹a chá»‰  cá»§a biáº¿n Struc, *lÃ  data tá»©c lÃ  gÃ¡n data trong Ã´ nhá»› struc báº±ng eprom Ä‘á»�c dc (char*) lÃ  Ã©p kiá»ƒu dá»¯ liá»‡u
    }
    printWiFiConf();
    return true;
  } else {
    Serial.println("file");
    return false;
  }
}
void saveStepConf(void) {
  for (unsigned int t = 0; t < sizeof(DataConf); t++) {
    EEPROM.write(WIFI_CONF_START + t, *((char*)&DataConf + t));
  }
  //EEPROM.commit();
  printWiFiConf();
}

