#ifdef USE_SENSORFILTER

#define XDRV_127             127

#include <map>

// Commands
#define D_CMND_SENSORFILTER_SET "SensorFilterSet"
#define D_CMND_SENSORFILTER_CLEAR "SensorFilterClear"
#define D_CMND_SENSORFILTER_DEBUG "SensorFilterDebug"

const char kSensorFilterCommands[] PROGMEM = "|" D_CMND_SENSORFILTER_SET "|" D_CMND_SENSORFILTER_CLEAR "|" D_CMND_SENSORFILTER_DEBUG;
void (* const SensorFilterCommand[])(void) PROGMEM = { &CmndSensorFilterSet, &CmndSensorFilterClear, &CmndSensorFilterDebug };

uint8_t SensorFilterDebugMode = 1;

struct sensor_filter_t {
    float offset = NAN;
};

std::map<String, sensor_filter_t> sensor_filters;

sensor_filter_t* SensorFilter_GetFilter(const String key);


// TODO: An alternative solution could be to parse TasmotaGlobal.mqtt_data and modify it directly to change sensor values.
// That way it could be possible to change any sensor values without changing the code. However there can be performance issues.

// JsonParserToken SensorFilterGetLocalSensor(String sensor_name) {
//   // String buf = ResponseData();   // copy the string into a new buffer that will be modified
//   // AddLog(LOG_LEVEL_DEBUG, PSTR("ResponseData: %s"), buf.c_str());

//   // // Change existing mqtt_data...
//   // String rstring = "{\"Time\":\"2022-11-27T16:44:40\",\"DS18B20-1\":{\"Id\":\"00000007E5FC\",\"Temperature\":\"6.0\"},\"DS18B20-2\":{\"Id\":\"0000000623DD\",\"Temperature\":\"5.0\"}";
//   // TasmotaGlobal.mqtt_data = rstring;

//   // String buf2 = ResponseData();   // copy the string into a new buffer that will be modified
//   // AddLog(LOG_LEVEL_DEBUG, PSTR("Changed data: %s"), buf2.c_str());

//   // //const char *r = rstring.c_str();
//   // //memcpy(TasmotaGlobal.mqtt_data, r, strlen(r) * sizeof(char));
// }

sensor_filter_t* SensorFilter_GetFilter(const String key)
{
    std::map<String, sensor_filter_t>::iterator it = sensor_filters.find(key);
    if (it != sensor_filters.end()) {
        return &(it->second);
    }
    return nullptr;
}

void SensorFilter_ListResp()
{
    Response_P(PSTR("{\"SensorFilter\":["));

    int i = 0;
    for (auto it = sensor_filters.begin(); it != sensor_filters.end(); it++) {

        if (i > 0) {
            ResponseAppend_P(PSTR(","));
        }
        ResponseAppend_P(PSTR("{\"key\":\"%s\", \"offset\":%f}"), it->first.c_str(), it->second.offset);
        i++;
    }

    ResponseAppend_P(PSTR("]}"));
}

/*********************************************************************************************\
 * Commands
\*********************************************************************************************/

void CmndSensorFilterDebug(){
  if (XdrvMailbox.payload >= 0) {
    SensorFilterDebugMode = XdrvMailbox.payload;
  }
  ResponseCmndNumber(SensorFilterDebugMode);
}

void CmndSensorFilterClear()
{
    sensor_filters.clear();
    SensorFilter_ListResp();
}

void CmndSensorFilterSet()
{
    int op = XdrvMailbox.index;
    if (SensorFilterDebugMode > 0) {
        AddLog(LOG_LEVEL_DEBUG,PSTR("SensorFilter: FilterSet %d %s"), op, XdrvMailbox.data);
    }

    if (XdrvMailbox.data_len > 0)
    {
        JsonParser parser(XdrvMailbox.data);
        JsonParserObject root = parser.getRootObject();
        if (!root) {
            ResponseCmndChar_P(PSTR("invalidjson"));
            return;
        }

        JsonParserToken val = root[PSTR("key")];
        if (!val) {
            ResponseCmndChar_P(PSTR("missingkey"));
            return;
        }

        String key = val.getStr();
        if (SensorFilterDebugMode > 0) {
            AddLog(LOG_LEVEL_DEBUG,PSTR("SensorFilter: Key parsed %s."), key.c_str());
        }

        sensor_filter_t filter;

        val = root[PSTR("offset")];
        if (val) {
            filter.offset = val.getFloat();
            if (SensorFilterDebugMode > 0) {
                AddLog(LOG_LEVEL_DEBUG,PSTR("SensorFilter: Offset parsed %f"), filter.offset);
            }
        }

        sensor_filters[key] = filter;
    }

    SensorFilter_ListResp();
}

/*********************************************************************************************\
 * Interface
\*********************************************************************************************/

bool Xdrv127(uint8_t function)
{
  bool result = false;

  switch (function) {
    case FUNC_COMMAND:
      result = DecodeCommand(kSensorFilterCommands, SensorFilterCommand);
      break;
  }

  return result;
}

#endif // USE_SENSORFILTER
