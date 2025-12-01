/*

SpotHinta API client for Tasmota https://spot-hinta.fi/JustNow

# Rule to power on or off the relay based on the result from SpotHintaJustNowRank API request.
# - Executes every hour and once after 60 seconds for trying to verify that we are getting a information for the correct hour.
# - If request fails, retries after 600 seconds.
# - Adjust rank and price in the rule.

RULE1
ON Time#Initialized DO backlog event SpotHintaCheck ENDON
ON event#SpotHintaCheck DO backlog SpotHintaJustNowRank {"rank":4,"price":4,"backup":[2,3,4,5]} ENDON
ON SpotHintaJustNowRank#result>=0 DO power1 %value% ENDON
ON SpotHintaJustNowRank#status<0 DO RuleTimer1 600 ENDON
ON time#minute|60 DO backlog event SpotHintaCheck;RuleTimer1 60 ENDON
ON rules#Timer=1 DO event SpotHintaCheck ENDON
ON Power1#State DO teleperiod ENDON

# Important! Make sure that device's time zone matches entered backup hours.
Backlog0 Timezone 99; TimeStd 0,0,10,1,4,120; TimeDst 0,0,3,1,3,180

# Report immediately when heater heating state changes (PowerDelta 50%)
PowerDelta1 50

# Disable saving power state and use after restart:
SetOption0 0

# Switch relay(s) to their last saved state:
PowerOnState 3

# Tasmota devices configured for native discovery:
SetOption19 0

# For testing without actually do the request to API, use result parameter in settings, for example:
SpotHintaJustNowRank {"rank":5,"price":5,"backup":[2,3,4,5,20,21],"result":0}
SpotHintaJustNowRank {"rank":5,"price":5,"backup":[2,3,4,5,20,21],"result":%var1%}

*/

#ifdef USE_SPOTHINTA

#define XDRV_126             126

// Commands
#define D_CMND_SPOTHINTA_JUSTNOWRANK "SpotHintaJustNowRank"
#define D_CMND_SPOTHINTA_DEBUG "SpotHintaDebug"

const char kSpotHintaCommands[] PROGMEM = "|" D_CMND_SPOTHINTA_JUSTNOWRANK "|" D_CMND_SPOTHINTA_DEBUG;
void (* const SpotHintaCommand[])(void) PROGMEM = { &CmndSpotHintaJustNowRank, &CmndSpotHintaDebug };

uint8_t SpotHinta_DebugMode = 0;

struct spothinta_settings_t {    
    uint8_t rank = 0; // "Rank" limit (number of cheapest hours today)
    uint8_t price = 0; // Price limit in cents. If price NOW is below this relay is turned ON 
    
    // Backup hours if API is not answering or Internet connection is down.
    bool backup_hours[24] = { 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0 };
    //                       00 01 02 03 04 05 06 07 08 09 10 11 12 13 14 15 16 17 18 19 20 21 22 23 

    // // During these hours relay is always ON.
    // bool booster_hours[24] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }; 
    //  //                       00 01 02 03 04 05 06 07 08 09 10 11 12 13 14 15 16 17 18 19 20 21 22 23 
};

const char* SpotHinta_ResultTypes[] { "Normal", "Backup" };

uint8_t SpotHinta_JustNowRank(spothinta_settings_t *settings);

int8_t SpotHinta_JustNowRank(uint8_t rank, uint8_t price)
{
    int8_t result;

    WiFiClient http_client;
    HTTPClient http;

    char url[50];
    snprintf(url, sizeof(url), "http://api.spot-hinta.fi/JustNowRank/%d/%d", rank, price);
    http.begin(http_client, UrlEncode(url));
    http.addHeader("accept", "text/plain");

    if (SpotHinta_DebugMode > 0) {
        AddLog(LOG_LEVEL_DEBUG,PSTR("SpotHinta: GET %s"), url);
    }
    result = -1;
    int status_code = http.GET();
    if (status_code == 200 || status_code == 400) {
        const char* read = http.getString().c_str(); // TODO: This should be truncated to prevent running out of memory.
        if (status_code == 200 && 0 == strcasecmp(read, "200")) {
            result = 1;
        }
        else if (status_code == 400 && 0 == strcasecmp(read, "400")) {
            result = 0;
        }
        else {
            result = -1;
            if (SpotHinta_DebugMode > 0) {
                AddLog(LOG_LEVEL_DEBUG,PSTR("SpotHinta: Invalid content %s"), read);
            }
        }    
    }
    else {
        if (SpotHinta_DebugMode > 0) {
            AddLog(LOG_LEVEL_DEBUG,PSTR("SpotHinta: Invalid response."));
        }
    }
    http.end();
    
    if (SpotHinta_DebugMode > 0) {
        AddLog(LOG_LEVEL_DEBUG,PSTR("SpotHinta: Status code %d, result: %d"),status_code, result);
    }

    return result;
}

/*********************************************************************************************\
 * Commands
\*********************************************************************************************/

void CmndSpotHintaDebug() {
  if (XdrvMailbox.payload >= 0) {
    SpotHinta_DebugMode = XdrvMailbox.payload;
  }
  ResponseCmndNumber(SpotHinta_DebugMode);
}

void CmndSpotHintaJustNowRank() {
    if (SpotHinta_DebugMode > 0) {
        AddLog(LOG_LEVEL_DEBUG,PSTR("SpotHinta: JustNowRank"));
    }

    int op = XdrvMailbox.index;
    if (XdrvMailbox.data_len == 0)
    {
        ResponseCmndChar_P("Missing settings");
        return;
    }
 
    spothinta_settings_t settings;

    JsonParser parser(XdrvMailbox.data);
    JsonParserObject root = parser.getRootObject();
    if (!root) {
        ResponseCmndChar_P(PSTR("Invalid JSON"));
        return;
    }

    JsonParserToken token = root[PSTR("rank")];
    if (token) {
        if (!token.isUint()) {
            ResponseCmndChar_P(PSTR("Rank is not an unsigned integer"));
            return;
        }
        settings.rank = token.getUInt();
        if (SpotHinta_DebugMode > 0) {
            AddLog(LOG_LEVEL_DEBUG,PSTR("SpotHinta: Rank parsed %u"), settings.rank);
        }
        if (settings.rank < 0 || settings.rank > 23) {
            ResponseCmndChar_P(PSTR("Rank out of range"));
            return;
        }
    }

    token = root[PSTR("price")];
    if (token) {
        if (!token.isUint()) {
            ResponseCmndChar_P(PSTR("Price is not an unsigned integer"));
            return;
        }
        settings.price = token.getUInt();
        if (settings.price < 0 && settings.price > 1000) {
            ResponseCmndChar_P(PSTR("Invalid price"));
            return;
        }
        if (SpotHinta_DebugMode > 0) {
            AddLog(LOG_LEVEL_DEBUG,PSTR("SpotHinta: Price parsed %u"), settings.price);
        }
    }

    token = root[PSTR("backup")];
    if (token) {
        if (!token.isArray()) {
            ResponseCmndChar_P(PSTR("Backup is not array"));
            return;
        }        
        JsonParserArray hourArray = token.getArray();
        
        // Clear default backup hours
        for(int i = 0; i < sizeof(settings.backup_hours); i++) {
            settings.backup_hours[i] = 0;
        }        
        for (JsonParserToken token : hourArray) {
            if (token) {
                if (!token.isUint()) {
                    ResponseCmndChar_P(PSTR("Invalid hour."));
                    return;
                }
                uint8_t hour = token.getUInt();
                if (hour < 0 || hour > 23) {
                    ResponseCmndChar_P(PSTR("Hour out of range (0...23)"));
                    return;
                }
                if (SpotHinta_DebugMode > 1) {
                    AddLog(LOG_LEVEL_DEBUG,PSTR("SpotHinta: Set backup hour %d"), hour);
                }
                settings.backup_hours[hour] = true;
            }
        }
    }

    int8_t result;
    token = root[PSTR("result")];
    if (token) {
        if (!token.isInt() && !token.isUint()) {
            ResponseCmndChar_P(PSTR("Invalid result override."));
            return;           
        }
        // Test result for debugging
        result = token.getInt();
        if (SpotHinta_DebugMode > 1) {
            AddLog(LOG_LEVEL_DEBUG,PSTR("SpotHinta: Using result override %d"), result);
        }
    }
    else {
        // Make real API call
        result = SpotHinta_JustNowRank(settings.rank, settings.price);
    }
    
    int8_t status = 1;
    if (result < 0)
    {        
        status = -1;
        if (SpotHinta_IsBackupHour(settings.backup_hours, RtcTime.hour)) {
            result = 1;
        }
        else {
            result = 0;
        }
    }

    // Result is always 0 or 1.
    // Status is 1 if successful response from SpotHinta API.
    // Status is -1 if non-successful response and backup hours are used.
    // Settings are the settings used on request to SpotHinta API.

    Response_P(PSTR("{\"%s\":{"), XdrvMailbox.command);
    ResponseAppend_P(PSTR("\"Result\":%d,"), result);
    ResponseAppend_P(PSTR("\"Status\":%d,"), status);
    ResponseAppend_P(PSTR("\"Settings\":{\"Rank\":\"%u\",\"Price\":%u,\"Backup\":["), settings.rank, settings.price);
    int i = 0;
    for (uint8_t hour = 0; hour < sizeof(settings.backup_hours); ++hour) {
        if (settings.backup_hours[hour]) {
            if (i > 0) {
                ResponseAppend_P(PSTR(","));
            }
            ResponseAppend_P(PSTR("%u"), hour);
            i++;
        }
    }
    ResponseAppend_P(PSTR("]}}}"));
}

bool SpotHinta_IsBackupHour(bool backup_hours[], uint8_t hour)
{
    if (SpotHinta_DebugMode > 0) {
        AddLog(LOG_LEVEL_DEBUG,PSTR("SpotHinta: IsBackupHour (current hour %u)"), hour);
    }

    if (backup_hours[RtcTime.hour]) {            
        if (SpotHinta_DebugMode > 0) {
            AddLog(LOG_LEVEL_DEBUG,PSTR("SpotHinta: Current hour is a backup hour."));
        }
        return true;
    }
    else {
        if (SpotHinta_DebugMode > 0) {
            AddLog(LOG_LEVEL_DEBUG,PSTR("SpotHinta: Current hour is not a backup hour."));
        }
    }
    return false;
};

/*********************************************************************************************\
 * Interface
\*********************************************************************************************/

bool Xdrv126(uint32_t function)
{
  bool result = false;

  switch (function) {
    case FUNC_COMMAND:
      result = DecodeCommand(kSpotHintaCommands, SpotHintaCommand);
      break;
  }

  return result;
}

#endif // USE_SPOTHINTA
