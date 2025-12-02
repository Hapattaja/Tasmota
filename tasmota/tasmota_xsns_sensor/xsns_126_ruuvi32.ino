#ifdef USE_BLE_ESP32
#ifdef ESP32                       // ESP32 family only. Use define USE_HM10 for ESP8266 support
#if defined CONFIG_IDF_TARGET_ESP32 || defined CONFIG_IDF_TARGET_ESP32C3|| defined CONFIG_IDF_TARGET_ESP32S3

#ifdef USE_RUUVI_ESP32

#define XSNS_126                        126

#include <vector>

// Commands
#define D_CMND_RUUVI_SET "RuuviSet"
#define D_CMND_RUUVI_CLEAR "RuuviClear"
#define D_CMND_RUUVI_DEBUG "RuuviDebug"

const char kRuuviCommands[] PROGMEM = "|" D_CMND_RUUVI_SET "|" D_CMND_RUUVI_CLEAR "|" D_CMND_RUUVI_DEBUG;
void (* const RuuviCommand[])(void) PROGMEM = { &CmndRuuviSet, &CmndRuuviClear, &CmndRuuviDebug };

uint8_t RuuviDebugMode = 0;

struct ruuvi_sensor_data_t {
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
};

#define RUUVI_NAME_MAXLEN 32

struct ruuvi_sensor_t {
  uint8_t MAC[6];
  char name[RUUVI_NAME_MAXLEN + 1] = "";
  float temperature_offset = 0;
  float humidity_offset = 0;
  float pressure_offset = 0;
  ruuvi_sensor_data_t data;
  bool is_present = 0;
};

std::vector<ruuvi_sensor_t> RuuviSensors;

int Ruuvi32_ScanCompleteCallback(NimBLEScanResults results);
int Ruuvi32_AdvertismentCallback(BLE_ESP32::ble_advertisment_t *pStruct);

bool Ruuvi32_ParseData(const uint8_t *adv_data, int adv_data_len, ruuvi_sensor_data_t &result);
void Ruuvi32_ResponseAppend(ruuvi_sensor_data_t *p);
void Ruuvi32_ResponseAppendSensor(ruuvi_sensor_t *sensor);
ruuvi_sensor_t* Ruuvi32_GetSensor(const uint8_t mac[6]);
uint32_t Ruuvi32_AddOrUpdateSensor(ruuvi_sensor_t* sensor);

bool Ruuvi32_ParseData(const uint8_t *adv_data, int adv_data_len, ruuvi_sensor_data_t &result) {
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

// Updates sensor data
ruuvi_sensor_t* Ruuvi32_GetSensor(const uint8_t mac[6])
{
    for (uint32_t i = 0; i < RuuviSensors.size(); i++) {
        if(memcmp(mac, RuuviSensors[i].MAC, 6) == 0) {
            ruuvi_sensor_t* sensor = &(RuuviSensors[i]);
            if (RuuviDebugMode > 0) {
              AddLog(LOG_LEVEL_DEBUG, PSTR("Ruuvi: Got sensor %02x%02x%02x%02x%02x%02x"), sensor->MAC[0], sensor->MAC[1], sensor->MAC[2], sensor->MAC[3], sensor->MAC[4], sensor->MAC[5]);
            }
            return sensor;
        }
    }
    return nullptr;
}

// Adds or updates a sensor
uint32_t Ruuvi32_AddOrUpdateSensor(ruuvi_sensor_t* sensor)
{
    if (RuuviDebugMode > 0) {
      AddLog(LOG_LEVEL_DEBUG,PSTR("Ruuvi: AddOrUpdateSensor"));
    }

    for (uint32_t i = 0; i < RuuviSensors.size(); i++) {
        if(memcmp(sensor->MAC, RuuviSensors[i].MAC, 6) == 0) {
            if (RuuviDebugMode > 0) {
              AddLog(LOG_LEVEL_DEBUG, PSTR("Ruuvi: Updated sensor %02x%02x%02x%02x%02x%02x"), sensor->MAC[0], sensor->MAC[1], sensor->MAC[2], sensor->MAC[3], sensor->MAC[4], sensor->MAC[5]);
            }
            RuuviSensors[i] = *sensor;
            return i;
        }
    }

    RuuviSensors.push_back(*sensor);

    if (RuuviDebugMode > 0) {
      AddLog(LOG_LEVEL_DEBUG, PSTR("Ruuvi: Added sensor %02x%02x%02x%02x%02x%02x"), sensor->MAC[0], sensor->MAC[1], sensor->MAC[2], sensor->MAC[3], sensor->MAC[4], sensor->MAC[5]);
    }

    return RuuviSensors.size() - 1;
}

// /*********************************************************************************************\
//  * BLE callbacks section
//  * These are called from main thread only.
// \*********************************************************************************************/

int Ruuvi32_ScanCompleteCallback(NimBLEScanResults results){
  // we actually don't need to do anything here....
  if (BLE_ESP32::BLEDebugMode > 0) {
    AddLog(LOG_LEVEL_DEBUG, PSTR("Ruuvi32: Scan completed"));
  }
  return 0;
}

int Ruuvi32_AdvertismentCallback(BLE_ESP32::ble_advertisment_t *pStruct)
{
  if (BLE_ESP32::BLEDebugMode > 0) {
    if (RuuviDebugMode > 0) {
      AddLog(LOG_LEVEL_DEBUG, PSTR("Ruuvi: Advertisement callback"));
    }
  }

  const uint8_t *addr = pStruct->addr;

  if (RuuviDebugMode > 0) {
    AddLog(LOG_LEVEL_DEBUG, PSTR("Ruuvi: Found device (%02x%02x%02x%02x%02x%02x)"), addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
  }

  ruuvi_sensor_t* sensor = Ruuvi32_GetSensor(addr);
  if (sensor == nullptr) {
    if (RuuviDebugMode > 0) {
      AddLog(LOG_LEVEL_DEBUG, PSTR("Ruuvi: Sensor is not pre-defined."));
    }
    return 0;
  }

  if (RuuviDebugMode > 0) {
    AddLog(LOG_LEVEL_DEBUG, PSTR("Ruuvi: Pre-defined sensor found."));
  }

  // // const char *alias = BLE_ESP32::getAlias(addr);
  //     // if (!alias || !(*alias)){
  //     //    return 0;
  //     // }

  const BLEAdvertisedDevice *advertisedDevice = pStruct->advertisedDevice;
  if (!advertisedDevice->haveManufacturerData()){
    if (RuuviDebugMode > 0) {
      AddLog(LOG_LEVEL_DEBUG, PSTR("Ruuvi: No manufacturer data found."));
    }

    return 0;
  }

  std::string data = advertisedDevice->getManufacturerData();
  int manufacturerDataLen = data.length();

  if (manufacturerDataLen) {
    const uint8_t *manufacturerData = (const uint8_t *)data.data();

    if (manufacturerDataLen >= 14 && manufacturerData[0] == 0x99 && manufacturerData[1] == 0x04) {
        ruuvi_sensor_data_t data;
        if (Ruuvi32_ParseData(manufacturerData + 2, manufacturerDataLen - 2, data)) {

          if (RuuviDebugMode > 0) {
            AddLog(LOG_LEVEL_DEBUG, PSTR("Ruuvi: Sensor is recognized as Ruuvi sensor."));
          }

  	      data.RSSI = pStruct->RSSI;
          sensor->data = data;
          sensor->is_present = 1;

          if (RuuviDebugMode > 0) {
            AddLog(LOG_LEVEL_DEBUG, PSTR("%s: MAC: %02x%02x%02x%02x%02x%02x Temperature: %f °C Humidity: %f %% Pressure: %f hPa Tx power: %f Battery: %f V RSSI: %d"),
                "Ruuvi",
                sensor->MAC[0], sensor->MAC[1], sensor->MAC[2], sensor->MAC[3], sensor->MAC[4], sensor->MAC[5],
                data.temperature, data.humidity, data.pressure, data.tx_power, data.battery_voltage, data.RSSI);
          }
      }
    }
  }
  return 0;
}

void Ruuvi32_SensorListResp()
{
  Response_P(PSTR("{\"Ruuvi\":["));

  for (uint32_t i = 0; i < RuuviSensors.size(); i++) {
    if (i > 0) {
      ResponseAppend_P(PSTR(","));
    }
    ruuvi_sensor_t* sensor = &(RuuviSensors[i]);
    ResponseAppend_P(PSTR("{\"mac\":\"%02x%02x%02x%02x%02x%02x\", \"name\":\"%s\", \"temperature_offset\":%f, \"humidity_offset\":%f, \"pressure_offset\":%f}"),
      sensor->MAC[0], sensor->MAC[1], sensor->MAC[2], sensor->MAC[3], sensor->MAC[4], sensor->MAC[5],
      sensor->name,
      sensor->temperature_offset, sensor->humidity_offset, sensor->pressure_offset);
  }
  ResponseAppend_P(PSTR("]}"));
}

/*********************************************************************************************\
 * Commands
\*********************************************************************************************/

void CmndRuuviDebug(void){
  if (XdrvMailbox.payload >= 0) {
    RuuviDebugMode = XdrvMailbox.payload;
  }
  ResponseCmndNumber(RuuviDebugMode);
}

void CmndRuuviClear()
{
  RuuviSensors.clear();
  Ruuvi32_SensorListResp();
}

void CmndRuuviSet()
{
  int op = XdrvMailbox.index;
  if (RuuviDebugMode > 0) {
    AddLog(LOG_LEVEL_DEBUG,PSTR("Ruuvi: RuuviSet %d %s"), op, XdrvMailbox.data);
  }

  if (XdrvMailbox.data_len > 0)
  {
    JsonParser parser(XdrvMailbox.data);
    JsonParserObject root = parser.getRootObject();
    if (!root) {
      ResponseCmndChar_P(PSTR("invalidjson"));
      return;
    }

    JsonParserToken val = root[PSTR("mac")];
    if (!val) {      
      ResponseCmndChar_P(PSTR("missingmac"));
      return;
    }

    ruuvi_sensor_t sensor;

    const char *mac_string = val.getStr();
    if (BLE_ESP32::fromHex(sensor.MAC, mac_string, sizeof(sensor.MAC)) != 6) {
      ResponseCmndChar("invalidmac");
      return;
    }
    val = root[PSTR("name")];
    if (val) {
      if (RuuviDebugMode > 0) {
        AddLog(LOG_LEVEL_DEBUG,PSTR("Ruuvi: RuuviSet: Copying name to sensor."), sizeof(sensor.name));
      }
      strncpy(sensor.name, val.getStr(), sizeof(sensor.name));
      sensor.name[sizeof(sensor.name) - 1] = 0;
      if (RuuviDebugMode > 0) {
        AddLog(LOG_LEVEL_DEBUG,PSTR("Ruuvi: RuuviSet: Sensor name = %s"), sensor.name);
      }
    }

    val = root[PSTR("temperature_offset")];
    if (val) {
      sensor.temperature_offset = val.getFloat();
      if (RuuviDebugMode > 0) {
        AddLog(LOG_LEVEL_DEBUG,PSTR("Ruuvi: RuuviSet: Temperature offset parsed (%f)."), sensor.temperature_offset);
      }
    }
    val = root[PSTR("humidity_offset")];
    if (val) {
      sensor.humidity_offset = val.getFloat();
      if (RuuviDebugMode > 0) {
        AddLog(LOG_LEVEL_DEBUG,PSTR("Ruuvi: RuuviSet: Humidity offset parsed (%f)."), sensor.humidity_offset);
      }
    }
    val = root[PSTR("pressure_offset")];
    if (val) {
      sensor.pressure_offset = val.getFloat();
      if (RuuviDebugMode > 0) {
        AddLog(LOG_LEVEL_DEBUG,PSTR("Ruuvi: RuuviSet: Pressure offset parsed (%f)"), sensor.pressure_offset);
      }
    }

    if (RuuviDebugMode) {
      AddLog(LOG_LEVEL_DEBUG,PSTR("Ruuvi: RuuviSet: Json parsed"));
    }

    Ruuvi32_AddOrUpdateSensor(&sensor);
  }

  Ruuvi32_SensorListResp();
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
        ruuvi_sensor_t* sensor = &(RuuviSensors[i]);
        const char *name;

        if (sensor->name && *sensor->name) {
          name = sensor->name;
        }
        else {
          char mac_string[20];
          ToHex_P(sensor->MAC, 6, mac_string, 20, 0);
          name = mac_string;
        }

        // TODO: Report more variables
  	    TempHumDewShow(0, (0 == TasmotaGlobal.tele_period), (PSTR("%s"), name), sensor->data.temperature + sensor->temperature_offset, sensor->data.humidity + sensor->humidity_offset);
      }
    }

    WSContentSend_PD(HTTP_RUUVI32_HL);
}
#endif  // USE_WEBSERVER

void Ruuvi32_ResponseAppendSensor(ruuvi_sensor_t *sensor)
{
  ruuvi_sensor_data_t *data = &sensor->data;

  ResponseAppend_P(PSTR("\"%s\":{"), sensor->name);
  //ResponseAppend_P(PSTR("\"name\":\"%s\","), name);
  ResponseAppend_P(PSTR("\"mac\":\"%02x%02x%02x%02x%02x%02x\""), sensor->MAC[0], sensor->MAC[1], sensor->MAC[2], sensor->MAC[3], sensor->MAC[4], sensor->MAC[5]);

  if (!isnan(data->temperature)) {
    float temperature = ConvertTempToFahrenheit(data->temperature + sensor->temperature_offset); // convert if SO8 on
    ResponseAppend_P(PSTR(",\"" D_JSON_TEMPERATURE "\":%.2f"), temperature);
  }
  if (!isnan(data->humidity)) {
    ResponseAppend_P(PSTR(",\"" D_JSON_HUMIDITY "\":%.2f"), data->humidity + sensor->humidity_offset);
  }
  if (!isnan(data->pressure)) {
    ResponseAppend_P(PSTR(",\"" D_JSON_PRESSURE "\":%.2f"), data->pressure + sensor->pressure_offset);
  }
  if (!isnan(data->battery_voltage)) {
    ResponseAppend_P(PSTR(",\"Battery\":%.3f"), data->battery_voltage);
  }
  if (!isnan(data->RSSI)) {
    ResponseAppend_P(PSTR(",\"RSSI\":%d"), data->RSSI);
  }

  ResponseAppend_P(PSTR("}"));
}

void Ruuvi32_UpdateSensorsState()
{
  for (int i = RuuviSensors.size()-1; i >= 0; i--) {
    ruuvi_sensor_t* sensor = &(RuuviSensors[i]);
    if (sensor->is_present && !BLE_ESP32::devicePresent(sensor->MAC)) {
      AddLog(LOG_LEVEL_DEBUG, PSTR("Ruuvi: Device is not present %02x%02x%02x%02x%02x%02x."), sensor->MAC[0], sensor->MAC[1], sensor->MAC[2], sensor->MAC[3], sensor->MAC[4], sensor->MAC[5]);
      sensor->is_present = 0;
    }
  }
}

void Ruuvi32_ResponseAppend(void)
{
  Ruuvi32_UpdateSensorsState();

  for (uint32_t i = 0; i < RuuviSensors.size(); i++) {
    ruuvi_sensor_t* sensor = &(RuuviSensors[i]);
    if (sensor->is_present)
    {
      ResponseAppend_P(PSTR(","));
      Ruuvi32_ResponseAppendSensor(sensor);
      if (RuuviDebugMode > 0) {
        AddLog(LOG_LEVEL_DEBUG_MORE, PSTR("Ruuvi: Response %02x%02x%02x%02x%02x%02x"), sensor->MAC[0], sensor->MAC[1], sensor->MAC[2], sensor->MAC[3], sensor->MAC[4], sensor->MAC[5]);
      }
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

bool Xsns126(uint32_t function)
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
    case FUNC_COMMAND:
      result = DecodeCommand(kRuuviCommands, RuuviCommand);
      break;
    }

  return result;
}

#endif  // USE_Ruuvi_ESP32
#endif  // CONFIG_IDF_TARGET_ESP32 or CONFIG_IDF_TARGET_ESP32C3
#endif  // ESP32

#endif // USE_BLE_ESP32