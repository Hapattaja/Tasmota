#ifdef USE_BLE_ESP32
#ifdef ESP32                       // ESP32 family only. Use define USE_HM10 for ESP8266 support
#if defined CONFIG_IDF_TARGET_ESP32 || defined CONFIG_IDF_TARGET_ESP32C3|| defined CONFIG_IDF_TARGET_ESP32S3

#ifdef USE_RUUVI_ESP32

#define XSNS_126                        126

#include <vector>

int Ruuvi32_ScanCompleteCallback(NimBLEScanResults results);
int Ruuvi32_AdvertismentCallback(BLE_ESP32::ble_advertisment_t *pStruct);

struct ruuvi_sensor_t {
  uint8_t MAC[6];
  int8_t RSSI;
  float humidity = NAN;
  float temperature = NAN;
  float pressure = NAN;
  float acceleration = NAN;
  float acceleration_x = NAN;
  float acceleration_y = NAN;
  float acceleration_z = NAN;
  float battery_voltage = NAN;
  float tx_power = NAN;
  float movement_counter = NAN;
  float measurement_sequence_number = NAN;
  bool sent = 0;
};

std::vector<ruuvi_sensor_t> RuuviSensors;
SemaphoreHandle_t ruuviSensorSlotMutex  = (SemaphoreHandle_t) nullptr;

bool parse_ruuvi_data_byte(const uint8_t *adv_data, int adv_data_len, ruuvi_sensor_t &result);
void Ruuvi32_ResponseAppend(ruuvi_sensor_t *p);

bool parse_ruuvi_data_byte(const uint8_t *adv_data, int adv_data_len, ruuvi_sensor_t &result) {
  const uint8_t data_type = adv_data[0];
  const auto *data = &adv_data[1];
  switch (data_type) {
    case 0x03: {  // RAWv1
      if (adv_data_len != 14)
        return false;

      const uint8_t temp_sign = (data[1] >> 7) & 1;
      const float temp_val = (data[1] & 0x7F) + (data[2] / 100.0f);
      const float temperature = temp_sign == 0 ? temp_val : -1 * temp_val;

      const float humidity = data[0] * 0.5f;
      const float pressure = (uint16_t(data[3] << 8) + uint16_t(data[4]) + 50000.0f) / 100.0f;
      const float acceleration_x = (int16_t(data[5] << 8) + int16_t(data[6])) / 1000.0f;
      const float acceleration_y = (int16_t(data[7] << 8) + int16_t(data[8])) / 1000.0f;
      const float acceleration_z = (int16_t(data[9] << 8) + int16_t(data[10])) / 1000.0f;
      const float battery_voltage = (uint16_t(data[11] << 8) + uint16_t(data[12])) / 1000.0f;

      result.humidity = humidity;
      result.temperature = temperature;
      result.pressure = pressure;
      result.acceleration_x = acceleration_x;
      result.acceleration_y = acceleration_y;
      result.acceleration_z = acceleration_z;
      result.acceleration = sqrtf(acceleration_x * acceleration_x + acceleration_y * acceleration_y + acceleration_z * acceleration_z);
      result.battery_voltage = battery_voltage;

      return true;
    }
    case 0x05: {  // RAWv2
      if (adv_data_len != 24)
        return false;

      const float temperature = (int16_t(data[0] << 8) + int16_t(data[1])) * 0.005f;
      const float humidity = (uint16_t(data[2] << 8) | uint16_t(data[3])) / 400.0f;
      const float pressure = ((uint16_t(data[4] << 8) | uint16_t(data[5])) + 50000.0f) / 100.0f;
      const float acceleration_x = (int16_t(data[6] << 8) + int16_t(data[7])) / 1000.0f;
      const float acceleration_y = (int16_t(data[8] << 8) + int16_t(data[9])) / 1000.0f;
      const float acceleration_z = (int16_t(data[10] << 8) + int16_t(data[11])) / 1000.0f;

      const uint16_t power_info = (uint16_t(data[12] << 8) | data[13]);
      const float battery_voltage = ((power_info >> 5) + 1600.0f) / 1000.0f;
      const float tx_power = ((power_info & 0x1F) * 2.0f) - 40.0f;

      const float movement_counter = float(data[14]);
      const float measurement_sequence_number = float(uint16_t(data[15] << 8) | uint16_t(data[16]));

      result.temperature = data[0] == 0x7F && data[1] == 0xFF ? NAN : temperature;
      result.humidity = data[2] == 0xFF && data[3] == 0xFF ? NAN : humidity;
      result.pressure = data[4] == 0xFF && data[5] == 0xFF ? NAN : pressure;
      result.acceleration_x = data[6] == 0xFF && data[7] == 0xFF ? NAN : acceleration_x;
      result.acceleration_y = data[8] == 0xFF && data[9] == 0xFF ? NAN : acceleration_y;
      result.acceleration_z = data[10] == 0xFF && data[11] == 0xFF ? NAN : acceleration_z;
      result.acceleration = result.acceleration_x == NAN || result.acceleration_y == NAN || result.acceleration_z == NAN
                                ? NAN
                                : sqrtf(acceleration_x * acceleration_x + acceleration_y * acceleration_y +
                                        acceleration_z * acceleration_z);
      result.battery_voltage = (power_info >> 5) == 0x7FF ? NAN : battery_voltage;
      result.tx_power = (power_info & 0x1F) == 0x1F ? NAN : tx_power;
      result.movement_counter = movement_counter;
      result.measurement_sequence_number = measurement_sequence_number;

      return true;
    }
    default:
      return false;
  }
}

uint32_t Ruuvi32_AddOrUpdateSensor(ruuvi_sensor_t *sensor, std::vector<ruuvi_sensor_t>& sensors) {
    for (uint32_t i = 0; i < sensors.size(); i++) {
        if(memcmp(sensor->MAC, sensors[i].MAC, 6) == 0) {
            AddLog(LOG_LEVEL_DEBUG_MORE, PSTR("Ruuvi: Updated sensor %02x%02x%02x%02x%02x%02x"), sensor->MAC[0], sensor->MAC[1], sensor->MAC[2], sensor->MAC[3], sensor->MAC[4], sensor->MAC[5]);
            sensors[i] = *sensor;
            return i;
        }
    }
    sensors.push_back(*sensor);
    AddLog(LOG_LEVEL_DEBUG_MORE, PSTR("Ruuvi: Added sensor %02x%02x%02x%02x%02x%02x"), sensor->MAC[0], sensor->MAC[1], sensor->MAC[2], sensor->MAC[3], sensor->MAC[4], sensor->MAC[5]);
    return sensors.size() - 1;
}

/*********************************************************************************************\
 * BLE callbacks section
 * These are called from main thread only.
\*********************************************************************************************/

int Ruuvi32_ScanCompleteCallback(NimBLEScanResults results){
  // we actually don't need to do anything here....
  if (BLE_ESP32::BLEDebugMode > 0) AddLog(LOG_LEVEL_DEBUG_MORE, PSTR("Ruuvi32: Scan completed"));
  return 0;
}

int Ruuvi32_AdvertismentCallback(BLE_ESP32::ble_advertisment_t *pStruct)
{
  if (BLE_ESP32::BLEDebugMode > 0) {
    AddLog(LOG_LEVEL_DEBUG_MORE, PSTR("Ruuvi32: Advertisement callback"));
  }

  const uint8_t *addr = pStruct->addr;

  BLEAdvertisedDevice *advertisedDevice = pStruct->advertisedDevice;
  if (!advertisedDevice->haveManufacturerData()){
    return 0;
  }

  std::string data = advertisedDevice->getManufacturerData();
  int manufacturerDataLen = data.length();

  if (manufacturerDataLen) {
    const uint8_t *manufacturerData = (const uint8_t *)data.data();

    if (manufacturerDataLen >= 14 && manufacturerData[0] == 0x99 && manufacturerData[1] == 0x04) {
        ruuvi_sensor_t sensor{};
	      memcpy(sensor.MAC, pStruct->addr, 6);
	      sensor.RSSI = pStruct->RSSI;

        if (parse_ruuvi_data_byte(manufacturerData + 2, manufacturerDataLen - 2, sensor)) {
          AddLog(LOG_LEVEL_DEBUG_MORE, PSTR("%s: MAC: %02x%02x%02x%02x%02x%02x Temperature: %f °C Humidity: %f %% Pressure: %f hPa Tx power: %f Battery: %d V RSSI: %d"),
                "Ruuvi",
                sensor.MAC[0], sensor.MAC[1], sensor.MAC[2], sensor.MAC[3], sensor.MAC[4], sensor.MAC[5],
                sensor.temperature, sensor.humidity, sensor.pressure, sensor.tx_power, sensor.battery_voltage,
                sensor.RSSI);

      	  const char *alias = BLE_ESP32::getAlias(addr);
          if (!alias || !(*alias)){
             return 0;
          }
	        int slot = Ruuvi32_AddOrUpdateSensor(&sensor, RuuviSensors);
      }
      return 0;
    }
  }
  return 0;
}

#ifdef USE_WEBSERVER

/*********************************************************************************************\
 * Presentation
\*********************************************************************************************/

const char HTTP_RUUVI32[] PROGMEM = "{s}Ruuvi ESP32{m}%d{e}";
const char HTTP_RUUVI32_HL[] PROGMEM = "{s}<hr>{m}<hr>{e}";

void Ruuvi32_Show(bool json) {

    WSContentSend_PD(HTTP_RUUVI32_HL);
    WSContentSend_PD(HTTP_RUUVI32, RuuviSensors.size());

    if (RuuviSensors.size() > 0) {

      for (uint32_t i = 0; i < RuuviSensors.size(); i++) {
        ruuvi_sensor_t sensor = RuuviSensors[i];
        const char *name;
        const char *alias = BLE_ESP32::getAlias(sensor.MAC);
        if (alias && *alias){
          name = alias;
        }
        else {
          char mac_string[20];
          ToHex_P(sensor.MAC, 6, mac_string, 20, 0);
          name = mac_string;
        }

        // TODO: Report more variables
  	    TempHumDewShow(0, (0 == TasmotaGlobal.tele_period), (PSTR("Ruuvi %s"), name), sensor.temperature, sensor.humidity);
      }
    }

    WSContentSend_PD(HTTP_RUUVI32_HL);
}
#endif  // USE_WEBSERVER

// TODO: Sensor numbers: Ruuvi, Ruuvi-2, Ruuvi-3 etc. ?
void Ruuvi32_ResponseAppend(ruuvi_sensor_t *p)
{
  const char *alias = BLE_ESP32::getAlias(p->MAC);
  if (!(alias && *alias)) {
    return;
  }

  ResponseAppend_P(PSTR("\"Ruuvi-%s\":{"), alias);
  ResponseAppend_P(PSTR("\"alias\":\"%s\","), alias);
  ResponseAppend_P(PSTR("\"mac\":\"%02x%02x%02x%02x%02x%02x\""), p->MAC[0], p->MAC[1], p->MAC[2], p->MAC[3], p->MAC[4], p->MAC[5]);

  if (!isnan(p->temperature)) {
    float temperature = ConvertTempToFahrenheit(p->temperature); // convert if SO8 on
    ResponseAppend_P(PSTR(",\"" D_JSON_TEMPERATURE "\":%.2f"), temperature);
  }
  if (!isnan(p->humidity)) {
    ResponseAppend_P(PSTR(",\"" D_JSON_HUMIDITY "\":%.2f"), p->humidity);
  }
  if (!isnan(p->pressure)) {
    ResponseAppend_P(PSTR(",\"" D_JSON_PRESSURE "\":%.2f"), p->pressure);
  }
  if (!isnan(p->battery_voltage)) {
    ResponseAppend_P(PSTR(",\"Battery\":%u"), p->battery_voltage);
  }
  if (!isnan(p->RSSI)) {
    ResponseAppend_P(PSTR(",\"RSSI\":%d"), p->RSSI);
  }

  ResponseAppend_P(PSTR("}"));
}

void Ruuvi32_ResponseAppend() {
  Ruuvi32_RemoveNonPresentSensors();

  for (uint32_t i = 0; i < RuuviSensors.size(); i++) {
    ResponseAppend_P(PSTR(","));
    ruuvi_sensor_t* sensor = &RuuviSensors[i];
    Ruuvi32_ResponseAppend(sensor);
    if (!sensor->sent) {
      // Not actually used, maybe MQTT?
      AddLog(LOG_LEVEL_DEBUG_MORE, PSTR("Ruuvi: Sent %02x%02x%02x%02x%02x%02x"), sensor->MAC[0], sensor->MAC[1], sensor->MAC[2], sensor->MAC[3], sensor->MAC[4], sensor->MAC[5]);
      sensor->sent = 1;
    }
  }
}

void Ruuvi32_RemoveNonPresentSensors(){
  // PROBLEM: when we take this, it hangs the BLE loop.
  // BUT, devicePresent uses the remove devices for which the adverts have timed out
  for (int i = RuuviSensors.size()-1; i >= 0 ; i--) {
    if (!BLE_ESP32::devicePresent(RuuviSensors[i].MAC)){
      ruuvi_sensor_t* sensor = &RuuviSensors[i];
      AddLog(LOG_LEVEL_DEBUG, PSTR("Ruuvi: Remove MAC: %02x%02x%02x%02x%02x%02x"), sensor->MAC[0], sensor->MAC[1], sensor->MAC[2], sensor->MAC[3], sensor->MAC[4], sensor->MAC[5]);
      TasAutoMutex localmutex(&ruuviSensorSlotMutex, "Ruuvi32Timeout");
      RuuviSensors.erase(RuuviSensors.begin() + i);
    }
  }
}

/*********************************************************************************************\
 * init BLE_32
\*********************************************************************************************/

void Ruuvi32_Init() {

  AddLog(LOG_LEVEL_INFO, PSTR("Ruuvi32: init: register callbacks"));

  BLE_ESP32::registerForAdvertismentCallbacks((const char *)"Ruuvi32", Ruuvi32_AdvertismentCallback);
  BLE_ESP32::registerForScanCallbacks((const char *)"Ruuvi32", Ruuvi32_ScanCompleteCallback);

  return;
}

/*********************************************************************************************\
 * constants
\*********************************************************************************************/

bool Xsns126(uint8_t function)
{
  bool result = false;

  switch (function) {
    case FUNC_INIT:
      Ruuvi32_Init();
      break;
    case FUNC_EVERY_SECOND:
      //Ruuvi32_EverySecond();
      break;
    case FUNC_JSON_APPEND:
      Ruuvi32_ResponseAppend();
      break;
 #ifdef USE_WEBSERVER
     case FUNC_WEB_SENSOR:
       Ruuvi32_Show(0);
       break;
 #endif  // USE_WEBSERVER
    }
  return result;
}

#endif  // USE_Ruuvi_ESP32
#endif  // CONFIG_IDF_TARGET_ESP32 or CONFIG_IDF_TARGET_ESP32C3
#endif  // ESP32

#endif // USE_BLE_ESP32