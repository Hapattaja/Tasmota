/*
  xdrv_128_hygrostat.ino - Hygrostat controller for Tasmota

  Based on xdrv_39_thermostat.ini (Copyright (C) 2021  Javier Arigita)
*/

#ifdef USE_HYGROSTAT

#define XDRV_128             128

// Enable/disable debugging
//#define DEBUG_HYGROSTAT

// Enable/disable experimental PI auto-tuning inspired by the Arduino
// Autotune Library by Brett Beauregard
//#define USE_PI_AUTOTUNING // (Ziegler-Nichols closed loop method)

#ifdef DEBUG_HYGROSTAT
#define DOMOTICZ_MAX_IDX     4
#define DOMOTICZ_IDX1        791
#define DOMOTICZ_IDX2        792
#define DOMOTICZ_IDX3        799
#define DOMOTICZ_IDX4        800
#define DOMOTICZ_IDX5        801
#endif // DEBUG_HYGROSTAT

// Commands
#define D_CMND_HYGROSTATMODESET "HygrostatModeSet"
#define D_CMND_HUMIDIFYINGMODESET "HumidifyingModeSet"
#define D_CMND_HUMTOODRYPROTECTSET "HumToodryProtectSet"
#define D_CMND_CONTROLLERMODESET "ControllerModeSet"
#define D_CMND_INPUTSWITCHSET "InputSwitchSet"
#define D_CMND_INPUTSWITCHUSE "InputSwitchUse"
#define D_CMND_OUTPUTRELAYSET "OutputRelaySet"
#define D_CMND_TIMEALLOWRAMPUPSET "TimeAllowRampupSet"
//#define D_CMND_HUMFORMATSET "HumFormatSet"
#define D_CMND_HUMMEASUREDSET "HumMeasuredSet"
#define D_CMND_HUMTARGETSET "HumTargetSet"
#define D_CMND_HUMMEASUREDGRDREAD "HumMeasuredGrdRead"
#define D_CMND_HUMSENSNUMBERSET "HumSensNumberSet"
#define D_CMND_SENSORINPUTSET "SensorInputSet"
#define D_CMND_STATEEMERGENCYSET "StateEmergencySet"
#define D_CMND_TIMEMANUALTOAUTOSET "TimeManualToAutoSet"
#define D_CMND_TIMEONLIMITSET "TimeOnLimitSet"
#define D_CMND_PROPBANDSET "PropBandSet"
#define D_CMND_TIMERESETSET "TimeResetSet"
#define D_CMND_TIMEPICYCLESET "TimePiCycleSet"
#define D_CMND_HUMANTIWINDUPRESETSET "HumAntiWindupResetSet"
#define D_CMND_HUMHYSTSET "HumHystSet"
#ifdef USE_PI_AUTOTUNING
#define D_CMND_PERFLEVELAUTOTUNE "PerfLevelAutotune"
#endif // USE_PI_AUTOTUNING
#define D_CMND_TIMEMAXACTIONSET "TimeMaxActionSet"
#define D_CMND_TIMEMINACTIONSET "TimeMinActionSet"
#define D_CMND_TIMEMINTURNOFFACTIONSET "TimeMinTurnoffActionSet"
#define D_CMND_HUMRUPDELTINSET "HumRupDeltInSet"
#define D_CMND_HUMRUPDELTOUTSET "HumRupDeltOutSet"
#define D_CMND_TIMERAMPUPMAXSET "TimeRampupMaxSet"
#define D_CMND_TIMERAMPUPCYCLESET "TimeRampupCycleSet"
#define D_CMND_HUMRAMPUPPIACCERRSET "HumRampupPiAccErrSet"
#define D_CMND_TIMEPIPROPORTREAD "TimePiProportRead"
#define D_CMND_TIMEPIINTEGRREAD "TimePiIntegrRead"
#define D_CMND_TIMESENSLOSTSET "TimeSensLostSet"
#define D_CMND_DIAGNOSTICMODESET "DiagnosticModeSet"
#define D_CMND_CTRDUTYCYCLEREAD "CtrDutyCycleRead"
#define D_CMND_ENABLEOUTPUTSET "EnableOutputSet"

enum HygrostatModes { HYGROSTAT_OFF, HYGROSTAT_AUTOMATIC_OP, HYGROSTAT_MANUAL_OP, HYGROSTAT_MODES_MAX };
#ifdef USE_PI_AUTOTUNING
enum ControllerModes { CTR_HYBRID, CTR_PI, CTR_RAMP_UP, CTR_PI_AUTOTUNE, CTR_MODES_MAX };
enum ControllerHybridPhases { CTR_HYBRID_RAMP_UP, CTR_HYBRID_PI, CTR_HYBRID_PI_AUTOTUNE };
enum AutotuneStates { AUTOTUNE_OFF, AUTOTUNE_ON, AUTOTUNE_MAX };
enum AutotunePerformanceParam { AUTOTUNE_PERF_FAST, AUTOTUNE_PERF_NORMAL, AUTOTUNE_PERF_SLOW, AUTOTUNE_PERF_MAX };
#else
enum ControllerModes { CTR_HYBRID, CTR_PI, CTR_RAMP_UP, CTR_MODES_MAX };
enum ControllerHybridPhases { CTR_HYBRID_RAMP_UP, CTR_HYBRID_PI };
#endif // USE_PI_AUTOTUNING
enum HumidifyingModes { HUMIDIFYING, DEHUMIDIFYING, HUMIDIFYING_MODES_MAX };
enum InterfaceStates { IFACE_OFF, IFACE_ON };
enum InputUsage { INPUT_NOT_USED, INPUT_USED };
enum CtrCycleStates { CYCLE_OFF, CYCLE_ON };
enum EmergencyStates { EMERGENCY_OFF, EMERGENCY_ON };
enum SensorType { SENSOR_MQTT, SENSOR_LOCAL, SENSOR_MAX };
enum HumConvType { HUM_CONV_ABSOLUTE, HUM_CONV_RELATIVE };
enum DiagnosticModes { DIAGNOSTIC_OFF, DIAGNOSTIC_ON };
enum HygrostatSupportedInputSwitches {
  HYGROSTAT_INPUT_NONE,
  HYGROSTAT_INPUT_SWT1 = 1,            // Buttons
  HYGROSTAT_INPUT_SWT2,
  HYGROSTAT_INPUT_SWT3,
  HYGROSTAT_INPUT_SWT4,
  HYGROSTAT_INPUT_SWT5,
  HYGROSTAT_INPUT_SWT6,
  HYGROSTAT_INPUT_SWT7,
  HYGROSTAT_INPUT_SWT8
};
enum HygrostatSupportedOutputRelays {
  HYGROSTAT_OUTPUT_NONE,
  HYGROSTAT_OUTPUT_REL1 = 1,           // Relays
  HYGROSTAT_OUTPUT_REL2,
  HYGROSTAT_OUTPUT_REL3,
  HYGROSTAT_OUTPUT_REL4,
  HYGROSTAT_OUTPUT_REL5,
  HYGROSTAT_OUTPUT_REL6,
  HYGROSTAT_OUTPUT_REL7,
  HYGROSTAT_OUTPUT_REL8
};

typedef union {
  uint32_t data;
  struct {
    uint32_t hygrostat_mode : 2;       // Operation mode of the hygrostat system
    uint32_t controller_mode : 2;       // Operation mode of the hygrostat controller
    uint32_t humidifier_mode : 1;          // Humidifying mode of the hygrostat (0 = humidifier / 1 = dehumidifier)
    uint32_t sensor_alive : 1;          // Flag stating if humidity sensor is alive (0 = inactive, 1 = active)
    uint32_t sensor_type : 1;           // Sensor type: MQTT/local
    uint32_t command_output : 1;        // Flag stating the desired command to the output (0 = inactive, 1 = active)
    uint32_t status_output : 1;         // Flag stating state of the output (0 = inactive, 1 = active)
    uint32_t status_input : 1;          // Flag stating state of the input (0 = inactive, 1 = active)
    uint32_t use_input : 1;             // Flag stating if the input switch shall be used to switch to manual mode
    uint32_t phase_hybrid_ctr : 2;      // Phase of the hybrid controller (Ramp-up, PI or Autotune)
    uint32_t status_cycle_active : 1;   // Status showing if cycle is active (Output ON) or not (Output OFF)
    uint32_t counter_seconds : 6;       // Second counter used to track minutes
    uint32_t output_relay_number : 4;   // Output relay number
    uint32_t input_switch_number : 3;   // Input switch number
    uint32_t enable_output : 1;         // Enables / disables the physical output
#ifdef USE_PI_AUTOTUNING
    uint32_t autotune_flag : 1;         // Enable/disable autotune
    uint32_t autotune_perf_mode : 2;    // Autotune performance mode
#else
    uint32_t free : 3;                  // Free bits
#endif // USE_PI_AUTOTUNING
  };
} HygrostatStateBitfield;

typedef union {
  uint8_t data;
  struct {
    uint8_t state_emergency : 1;       // State for hygrostat emergency
    uint8_t diagnostic_mode : 1;       // Diagnostic mode selected
    uint8_t output_inconsist_ctr : 2;  // Counter of the minutes where the output state is inconsistent with the command
  };
} HygrostatDiagBitfield;

#ifdef DEBUG_HYGROSTAT
const char DOMOTICZ_MES[] PROGMEM = "{\"idx\":%d,\"nvalue\":%d,\"svalue\":\"%s\"}";
uint16_t Domoticz_Virtual_Switches[DOMOTICZ_MAX_IDX] = { DOMOTICZ_IDX1, DOMOTICZ_IDX3, DOMOTICZ_IDX4, DOMOTICZ_IDX5 };
#endif // DEBUG_HYGROSTAT

const char kHygrostatCommands[] PROGMEM = "|" D_CMND_HYGROSTATMODESET "|" D_CMND_HUMIDIFYINGMODESET "|"
  D_CMND_HUMTOODRYPROTECTSET "|" D_CMND_CONTROLLERMODESET "|" D_CMND_INPUTSWITCHSET "|" D_CMND_INPUTSWITCHUSE "|"
  D_CMND_OUTPUTRELAYSET "|" D_CMND_TIMEALLOWRAMPUPSET "|" /*D_CMND_HUMFORMATSET "|"*/ D_CMND_HUMMEASUREDSET "|"
  D_CMND_HUMTARGETSET "|" D_CMND_HUMMEASUREDGRDREAD "|" D_CMND_SENSORINPUTSET "|" D_CMND_STATEEMERGENCYSET "|"
  D_CMND_TIMEMANUALTOAUTOSET "|" D_CMND_PROPBANDSET "|" D_CMND_TIMERESETSET "|" D_CMND_TIMEPICYCLESET "|"
#ifdef USE_PI_AUTOTUNING
  D_CMND_HUMANTIWINDUPRESETSET "|" D_CMND_HUMHYSTSET "|" D_CMND_PERFLEVELAUTOTUNE "|" D_CMND_TIMEMAXACTIONSET "|"
#else
  D_CMND_HUMANTIWINDUPRESETSET "|" D_CMND_HUMHYSTSET "|" D_CMND_TIMEMAXACTIONSET "|"
#endif // USE_PI_AUTOTUNING
  D_CMND_TIMEMINACTIONSET "|" D_CMND_TIMEMINTURNOFFACTIONSET "|" D_CMND_HUMRUPDELTINSET "|" D_CMND_HUMRUPDELTOUTSET "|"
  D_CMND_TIMERAMPUPMAXSET "|" D_CMND_TIMERAMPUPCYCLESET "|" D_CMND_HUMRAMPUPPIACCERRSET "|" D_CMND_TIMEPIPROPORTREAD "|"
  D_CMND_TIMEPIINTEGRREAD "|" D_CMND_TIMESENSLOSTSET "|" D_CMND_DIAGNOSTICMODESET "|" D_CMND_CTRDUTYCYCLEREAD "|"
  D_CMND_ENABLEOUTPUTSET;

void (* const HygrostatCommand[])(void) PROGMEM = {
  &CmndHygrostatModeSet, &CmndHumidifyingModeSet, &CmndHumToodryProtectSet, &CmndControllerModeSet, &CmndInputSwitchSet,
  &CmndInputSwitchUse, &CmndOutputRelaySet, &CmndTimeAllowRampupSet, /*&CmndHumFormatSet, */&CmndHumMeasuredSet,
  &CmndHumTargetSet, &CmndHumMeasuredGrdRead, &CmndSensorInputSet, &CmndStateEmergencySet, &CmndTimeManualToAutoSet,
  &CmndPropBandSet, &CmndTimeResetSet, &CmndTimePiCycleSet, &CmndHumAntiWindupResetSet, &CmndHumHystSet,
#ifdef USE_PI_AUTOTUNING
  &CmndPerfLevelAutotune, &CmndTimeMaxActionSet, &CmndTimeMinActionSet, &CmndTimeMinTurnoffActionSet, &CmndHumRupDeltInSet,
#else
  &CmndTimeMaxActionSet, &CmndTimeMinActionSet, &CmndTimeMinTurnoffActionSet, &CmndHumRupDeltInSet,
#endif // USE_PI_AUTOTUNING
  &CmndHumRupDeltOutSet, &CmndTimeRampupMaxSet, &CmndTimeRampupCycleSet, &CmndHumRampupPiAccErrSet,
  &CmndTimePiProportRead, &CmndTimePiIntegrRead, &CmndTimeSensLostSet, &CmndDiagnosticModeSet, &CmndCtrDutyCycleRead,
  &CmndEnableOutputSet };

struct HYGROSTAT {
  HygrostatStateBitfield status;                                             // Bittfield including states as well as several flags
  uint32_t timestamp_hum_measured_update = 0;                                // Timestamp of latest measurement update
  uint32_t timestamp_hum_meas_change_update = 0;                             // Timestamp of latest measurement value change (> or < to previous)
  uint32_t timestamp_output_off = 0;                                          // Timestamp of latest hygrostat output Off state
  uint32_t timestamp_input_on = 0;                                            // Timestamp of latest input On state
  uint32_t time_hygrostat_total = 0;                                         // Time hygrostat on within a specific timeframe
  uint32_t time_ctr_checkpoint = 0;                                           // Time to finalize the control cycle within the PI strategy or to switch to PI from Rampup in seconds
  uint32_t time_ctr_changepoint = 0;                                          // Time until switching off output within the controller in seconds
  int32_t hum_measured_gradient = 0;                                         // Humidity measured gradient from sensor in thousandths of percents per hour
  int16_t hum_target_level = HYGROSTAT_HUM_INIT;                           // Target level of the hygrostat in tenths of percents
  int16_t hum_target_level_ctr = HYGROSTAT_HUM_INIT;                       // Target level set for the controller
  int16_t hum_pi_accum_error = 0;                                            // Humidity accumulated error for the PI controller in hundredths of percents
  int16_t hum_pi_error = 0;                                                  // Humidity error for the PI controller in hundredths of percents
  int32_t time_proportional_pi;                                               // Time proportional part of the PI controller
  int32_t time_integral_pi;                                                   // Time integral part of the PI controller
  int32_t time_total_pi;                                                      // Time total (proportional + integral) of the PI controller
  uint16_t kP_pi = 0;                                                         // kP value for the PI controller multiplied by 100 (to avoid floating point operations)
  uint16_t kI_pi = 0;                                                         // kI value for the PI controller multiplied by 100 (to avoid floating point operations)
  int32_t hum_rampup_meas_gradient = 0;                                      // Humidity measured gradient from sensor in thousandths of percents per hour calculated during ramp-up
  uint32_t timestamp_rampup_start = 0;                                        // Timestamp where the ramp-up controller mode has been started
  uint32_t time_rampup_deadtime = 0;                                          // Time constant of the hygrostat system (step response time)
  uint32_t time_rampup_nextcycle = 0;                                         // Time where the ramp-up controller shall start the next cycle
  int16_t hum_measured = 0;                                                  // Humidity measurement received from sensor in tenths of percents
  int16_t hum_rampup_output_off = 0;                                         // Humidity to switch off relay output within the ramp-up controller in tenths of percents
  uint8_t time_output_delay = HYGROSTAT_TIME_OUTPUT_DELAY;                   // Output delay between state change and real actuation event (f.i. valve open/closed)
  uint8_t counter_rampup_cycles = 0;                                          // Counter of ramp-up cycles
  uint8_t hum_rampup_pi_acc_error = HYGROSTAT_HUM_PI_RAMPUP_ACC_E;         // Accumulated error when switching from ramp-up controller to PI in hundreths of percents
  uint8_t hum_rampup_delta_out = HYGROSTAT_HUM_RAMPUP_DELTA_OUT;           // Minimum delta humidity to target to get out of the rampup mode, in tenths of percents
  uint8_t hum_rampup_delta_in = HYGROSTAT_HUM_RAMPUP_DELTA_IN;             // Minimum delta humidity to target to get into rampup mode, in tenths of percents
  uint8_t val_prop_band = HYGROSTAT_PROP_BAND;                               // Proportional band of the PI controller in degrees celsius
  int16_t hum_rampup_start = 0;                                              // Humidity at start of ramp-up controller in tenths of percents
  int16_t hum_rampup_cycle = 0;                                              // Humidity set at the beginning of each ramp-up cycle in tenths of percents
  uint16_t time_rampup_max = HYGROSTAT_TIME_RAMPUP_MAX;                      // Time maximum ramp-up controller duration in minutes
  uint16_t time_rampup_cycle = HYGROSTAT_TIME_RAMPUP_CYCLE;                  // Time ramp-up cycle in minutes
  uint16_t time_allow_rampup = HYGROSTAT_TIME_ALLOW_RAMPUP;                  // Time in minutes after last target update to allow ramp-up controller phase
  uint16_t time_sens_lost = HYGROSTAT_TIME_SENS_LOST;                        // Maximum time w/o sensor update to set it as lost in minutes
  uint16_t time_manual_to_auto = HYGROSTAT_TIME_MANUAL_TO_AUTO;              // Time without input switch active to change from manual to automatic in minutes
  uint32_t time_reset = HYGROSTAT_TIME_RESET;                                // Reset time of the PI controller in seconds
  uint16_t time_pi_cycle = HYGROSTAT_TIME_PI_CYCLE;                          // Cycle time for the hygrostat controller in minutes
  uint16_t time_max_action = HYGROSTAT_TIME_MAX_ACTION;                      // Maximum hygrostat time per cycle in minutes
  uint16_t time_min_action = HYGROSTAT_TIME_MIN_ACTION;                      // Minimum hygrostat time per cycle in minutes
  uint16_t time_min_turnoff_action = HYGROSTAT_TIME_MIN_TURNOFF_ACTION;      // Minimum turnoff time in minutes, below it the hygrostat will stay on
  int16_t hum_toodry_protect = HYGROSTAT_HUM_TOODRY_PROTECT;                 // Minimum humidity for too dry protection, in tenths of percents
  uint8_t hum_reset_anti_windup = HYGROSTAT_HUM_RESET_ANTI_WINDUP;         // Range where reset antiwindup is disabled, in tenths of percents
  int8_t hum_hysteresis = HYGROSTAT_HUM_HYSTERESIS;                        // Range hysteresis for humidity PI controller, in tenths of percents
  HygrostatDiagBitfield diag;                                                // Bittfield including diagnostic flags
#ifdef USE_PI_AUTOTUNING
  uint8_t dutycycle_step_autotune = HYGROSTAT_DUTYCYCLE_AUTOTUNE;            // Duty cycle for the step response of the autotune PI function in %
  uint8_t peak_ctr = 0;                                                       // Peak counter for the autotuning function
  uint8_t hum_band_no_peak_det = HYGROSTAT_HUM_BAND_NO_PEAK_DET;           // Humidity band in thenths of percents within no peak will be detected
  uint8_t val_prop_band_atune = 0;                                            // Proportional band calculated from the the PI autotune function in degrees celsius
  uint32_t time_reset_atune = 0;                                              // Reset time calculated from the PI autotune function in seconds
  uint16_t pU_pi_atune = 0;                                                   // pU value ("Ultimate" period) period of self-sustaining oscillations determined when the controller gain was set to Ku in minutes (for PI autotune)
  uint16_t kU_pi_atune = 0;                                                   // kU value ("Ultimate" gain) determined by increasing controller gain until self-sustaining oscillations are achieved (for PI autotune)
  uint16_t kP_pi_atune = 0;                                                   // kP value calculated by the autotune PI function multiplied by 100 (to avoid floating point operations)
  uint16_t kI_pi_atune = 0;                                                   // kI value calulated by the autotune PI function multiplied by 100 (to avoid floating point operations)
  int16_t hum_peaks_atune[HYGROSTAT_PEAKNUMBER_AUTOTUNE];                   // Array to store humidity peaks to be used by the autotune PI function
  int16_t hum_abs_max_atune;                                                 // Max humidity reached within autotune
  int16_t hum_abs_min_atune;                                                 // Min humidity reached within autotune
  uint16_t time_peak_timestamps_atune[HYGROSTAT_PEAKNUMBER_AUTOTUNE];        // Array to store timestamps in minutes of the humidity peaks to be used by the autotune PI function
  uint16_t time_std_dev_peak_det_ok = HYGROSTAT_TIME_STD_DEV_PEAK_DET_OK;    // Standard deviation in minutes of the oscillation periods within the peak detection is successful
#endif // USE_PI_AUTOTUNING
} Hygrostat[HYGROSTAT_CONTROLLER_OUTPUTS];

/*********************************************************************************************/

void HygrostatInit(uint8_t ctr_output)
{
  // Init Hygrostat[ctr_output].status bitfield:
  Hygrostat[ctr_output].status.hygrostat_mode = HYGROSTAT_OFF;
  Hygrostat[ctr_output].status.controller_mode = CTR_HYBRID;
  Hygrostat[ctr_output].status.humidifier_mode = HUMIDIFYING;
  Hygrostat[ctr_output].status.sensor_alive = IFACE_OFF;
  Hygrostat[ctr_output].status.sensor_type = SENSOR_MQTT;
  //Hygrostat[ctr_output].status.hum_format = HUM_CELSIUS;
  Hygrostat[ctr_output].status.command_output = IFACE_OFF;
  Hygrostat[ctr_output].status.status_output = IFACE_OFF;
  Hygrostat[ctr_output].status.phase_hybrid_ctr = CTR_HYBRID_PI;
  Hygrostat[ctr_output].status.status_cycle_active = CYCLE_OFF;
  Hygrostat[ctr_output].diag.state_emergency = EMERGENCY_OFF;
  Hygrostat[ctr_output].status.counter_seconds = 0;
  Hygrostat[ctr_output].status.output_relay_number = (HYGROSTAT_RELAY_NUMBER + ctr_output);
  Hygrostat[ctr_output].status.input_switch_number = (HYGROSTAT_SWITCH_NUMBER + ctr_output);
  Hygrostat[ctr_output].status.use_input = INPUT_NOT_USED;
  Hygrostat[ctr_output].status.enable_output = IFACE_ON;
  Hygrostat[ctr_output].diag.output_inconsist_ctr = 0;
  Hygrostat[ctr_output].diag.diagnostic_mode = DIAGNOSTIC_ON;
#ifdef USE_PI_AUTOTUNING
  Hygrostat[ctr_output].status.autotune_flag = AUTOTUNE_OFF;
  Hygrostat[ctr_output].status.autotune_perf_mode = AUTOTUNE_PERF_FAST;
#endif // USE_PI_AUTOTUNING
  // Make sure the Output is OFF
  if (Hygrostat[ctr_output].status.enable_output == IFACE_ON) {
    ExecuteCommandPower(Hygrostat[ctr_output].status.output_relay_number, POWER_OFF, SRC_HYGROSTAT);
  }
}

bool HygrostatMinuteCounter(uint8_t ctr_output)
{
  bool result = false;
  Hygrostat[ctr_output].status.counter_seconds++;    // increment time

  if ((Hygrostat[ctr_output].status.counter_seconds % 60) == 0) {
    result = true;
    Hygrostat[ctr_output].status.counter_seconds = 0;
  }
  return result;
}

inline bool HygrostatSwitchIdValid(uint8_t switchId)
{
  return (switchId >= HYGROSTAT_INPUT_SWT1 && switchId <= HYGROSTAT_INPUT_SWT8);
}

inline bool HygrostatRelayIdValid(uint8_t relayId)
{
  return (relayId >= HYGROSTAT_OUTPUT_REL1 && relayId <= HYGROSTAT_OUTPUT_REL8);
}

uint8_t HygrostatInputStatus(uint8_t input_switch)
{
  bool ifId = HygrostatSwitchIdValid(input_switch);
  uint8_t value = 0;
  if(ifId) {
    value = SwitchGetVirtual(ifId - HYGROSTAT_INPUT_SWT1);
  }
  return value;
}

uint8_t HygrostatOutputStatus(uint8_t output_switch)
{
  return (uint8_t)bitRead(TasmotaGlobal.power, (output_switch - 1));
}

int16_t HygrostatCelsiusToFahrenheit(const int32_t deg, uint8_t conv_type) {
  int32_t value;
  value = (int32_t)(((int32_t)deg * (int32_t)90) / (int32_t)50);
  if (conv_type == HUM_CONV_ABSOLUTE) {
    value += (int32_t)320;
  }

  // Protect overflow
  if (value <= (int32_t)(INT16_MIN)) {
    value = (int32_t)(INT16_MIN);
  }
  else if (value >= (int32_t)INT16_MAX) {
    value = (int32_t)INT16_MAX;
  }

  return (int16_t)value;
}

int16_t HygrostatFahrenheitToCelsius(const int32_t deg, uint8_t conv_type) {
  int16_t offset = 0;
  int32_t value;
  if (conv_type == HUM_CONV_ABSOLUTE) {
    offset = 320;
  }

  value = (int32_t)(((deg - (int32_t)offset) * (int32_t)50) / (int32_t)90);

  // Protect overflow
  if (value <= (int32_t)(INT16_MIN)) {
    value = (int32_t)(INT16_MIN);
  }
  else if (value >= (int32_t)INT16_MAX) {
    value = (int32_t)INT16_MAX;
  }

  return (int16_t)value;
}

void HygrostatSignalPreProcessingSlow(uint8_t ctr_output)
{
  // Update input sensor status
  if ((TasmotaGlobal.uptime - Hygrostat[ctr_output].timestamp_hum_measured_update) > ((uint32_t)Hygrostat[ctr_output].time_sens_lost * 60)) {
    Hygrostat[ctr_output].status.sensor_alive = IFACE_OFF;
    Hygrostat[ctr_output].hum_measured_gradient = 0;
    Hygrostat[ctr_output].hum_measured = 0;
  }
}

void HygrostatSignalPostProcessingSlow(uint8_t ctr_output)
{
  // Increate counter when inconsistent output state exists
  if ((Hygrostat[ctr_output].status.status_output != Hygrostat[ctr_output].status.command_output)
    &&(Hygrostat[ctr_output].status.enable_output == IFACE_ON)) {
    Hygrostat[ctr_output].diag.output_inconsist_ctr++;
  }
  else {
    Hygrostat[ctr_output].diag.output_inconsist_ctr = 0;
  }
}

void HygrostatSignalProcessingFast(uint8_t ctr_output)
{
  // Update real status of the input
  Hygrostat[ctr_output].status.status_input = (uint32_t)HygrostatInputStatus(Hygrostat[ctr_output].status.input_switch_number);
  // Update timestamp of last input
  if (Hygrostat[ctr_output].status.status_input == IFACE_ON) {
    Hygrostat[ctr_output].timestamp_input_on = TasmotaGlobal.uptime;
  }
  // Update real status of the output
  Hygrostat[ctr_output].status.status_output = (uint32_t)HygrostatOutputStatus(Hygrostat[ctr_output].status.output_relay_number);
}

void HygrostatCtrState(uint8_t ctr_output)
{
#ifdef USE_PI_AUTOTUNING
  bool flag_heating = (Hygrostat[ctr_output].status.humidifier_mode == HUMIDIFYING);
#endif //USE_PI_AUTOTUNING

  switch (Hygrostat[ctr_output].status.controller_mode) {
    // Hybrid controller (Ramp-up + PI)
    case CTR_HYBRID:
        HygrostatHybridCtrPhase(ctr_output);
      break;
    // PI controller
    case CTR_PI:
#ifdef USE_PI_AUTOTUNING
      // If Autotune has been enabled (via flag)
      // AND we have just reached the setpoint humidity
      // AND the humidity gradient is negative for humidifying and positive for dehumidifying
      // then switch state to PI autotuning
      if ((Hygrostat[ctr_output].status.autotune_flag == AUTOTUNE_ON)
        &&(Hygrostat[ctr_output].hum_measured == Hygrostat[ctr_output].hum_target_level)
        && ((flag_heating && (Hygrostat[ctr_output].hum_measured_gradient < 0))
          ||(!flag_heating && (Hygrostat[ctr_output].hum_measured_gradient > 0))))
      {
        Hygrostat[ctr_output].status.controller_mode = CTR_PI_AUTOTUNE;
        HygrostatPeakDetectorInit(ctr_output);
      }
#endif // USE_PI_AUTOTUNING
      break;
    // Ramp-up controller (predictive)
    case CTR_RAMP_UP:
      break;
#ifdef USE_PI_AUTOTUNING
    // PI autotune
    case CTR_PI_AUTOTUNE:
      // If autotune finalized (flag Off)
      // then go back to the PI controller
      if (Hygrostat[ctr_output].status.autotune_flag == AUTOTUNE_OFF)
      {
        Hygrostat[ctr_output].status.controller_mode = CTR_PI;
      }
      break;
#endif //USE_PI_AUTOTUNING
  }
}

void HygrostatHybridCtrPhase(uint8_t ctr_output)
{
  bool flag_heating = (Hygrostat[ctr_output].status.humidifier_mode == HUMIDIFYING);
  if (Hygrostat[ctr_output].status.controller_mode == CTR_HYBRID) {
    switch (Hygrostat[ctr_output].status.phase_hybrid_ctr) {
      // Ramp-up phase with gradient control
      case CTR_HYBRID_RAMP_UP:
          // If ramp-up offtime counter has been initalized
          // AND ramp-up offtime counter value reached
          if((Hygrostat[ctr_output].time_ctr_checkpoint != 0)
            && (TasmotaGlobal.uptime >= Hygrostat[ctr_output].time_ctr_checkpoint)) {
            // Reset pause period
            Hygrostat[ctr_output].time_ctr_checkpoint = 0;
            // Reset timers
            Hygrostat[ctr_output].time_ctr_changepoint = 0;
            // Set PI controller
            Hygrostat[ctr_output].status.phase_hybrid_ctr = CTR_HYBRID_PI;
          }
        break;
      // PI controller phase
      case CTR_HYBRID_PI:
          // If no output action for a pre-defined time
          // AND hum target has changed
          // AND value of hum target - actual humidity bigger than threshold for humidifying and lower for dehumidifying
          // then go to ramp-up
          if (((TasmotaGlobal.uptime - Hygrostat[ctr_output].timestamp_output_off) > (60 * (uint32_t)Hygrostat[ctr_output].time_allow_rampup))
            && (Hygrostat[ctr_output].hum_target_level != Hygrostat[ctr_output].hum_target_level_ctr)
            && ( ( (Hygrostat[ctr_output].hum_target_level - Hygrostat[ctr_output].hum_measured > Hygrostat[ctr_output].hum_rampup_delta_in)
                && (flag_heating))
              || ( (Hygrostat[ctr_output].hum_measured - Hygrostat[ctr_output].hum_target_level > Hygrostat[ctr_output].hum_rampup_delta_in)
                && (!flag_heating)))) {
              Hygrostat[ctr_output].timestamp_rampup_start = TasmotaGlobal.uptime;
              Hygrostat[ctr_output].hum_rampup_start = Hygrostat[ctr_output].hum_measured;
              Hygrostat[ctr_output].hum_rampup_meas_gradient = 0;
              Hygrostat[ctr_output].time_rampup_deadtime = 0;
              Hygrostat[ctr_output].counter_rampup_cycles = 1;
              Hygrostat[ctr_output].time_ctr_changepoint = 0;
              Hygrostat[ctr_output].time_ctr_checkpoint = 0;
              Hygrostat[ctr_output].status.phase_hybrid_ctr = CTR_HYBRID_RAMP_UP;
          }
#ifdef USE_PI_AUTOTUNING
          // If Autotune has been enabled (via flag)
          // AND we have just reached the setpoint humidity
          // AND the humidity gradient is negative for humidifying and positive for dehumidifying
          // then switch state to PI autotuning
          if ((Hygrostat[ctr_output].status.autotune_flag == AUTOTUNE_ON)
            &&(Hygrostat[ctr_output].hum_measured == Hygrostat[ctr_output].hum_target_level)
            && ((flag_heating && (Hygrostat[ctr_output].hum_measured_gradient < 0))
              ||(!flag_heating && (Hygrostat[ctr_output].hum_measured_gradient > 0))))
          {
            Hygrostat[ctr_output].status.phase_hybrid_ctr = CTR_HYBRID_PI_AUTOTUNE;
            HygrostatPeakDetectorInit(ctr_output);
          }
#endif // USE_PI_AUTOTUNING
        break;
#ifdef USE_PI_AUTOTUNING
        // PI autotune controller phase
      case CTR_HYBRID_PI_AUTOTUNE:
        // If autotune finalized (flag Off)
        // then go back to the PI controller
        if (Hygrostat[ctr_output].status.autotune_flag == AUTOTUNE_OFF)
        {
          Hygrostat[ctr_output].status.phase_hybrid_ctr = CTR_HYBRID_PI;
        }
      break;
#endif // USE_PI_AUTOTUNING
    }
  }
#ifdef DEBUG_HYGROSTAT
  HygrostatVirtualSwitchCtrState(ctr_output);
#endif // DEBUG_HYGROSTAT
}

bool HygrostatStateAutoToManual(uint8_t ctr_output)
{
  bool change_state = false;
  // If input is used
  // AND switch input is active
  //  OR humidity sensor is not alive
  // then go to manual
  if ((Hygrostat[ctr_output].status.use_input == INPUT_USED)
    &&((Hygrostat[ctr_output].status.status_input == IFACE_ON)
      || (Hygrostat[ctr_output].status.sensor_alive == IFACE_OFF))) {
    change_state = true;
  }
  return change_state;
}

bool HygrostatStateManualToAuto(uint8_t ctr_output)
{
  bool change_state = false;

  // If switch input inactive
  // AND sensor alive
  // AND no switch input action (time in current state) bigger than a pre-defined time
  // then go to automatic
  if ((Hygrostat[ctr_output].status.status_input == IFACE_OFF)
    &&(Hygrostat[ctr_output].status.sensor_alive ==  IFACE_ON)
    && ((TasmotaGlobal.uptime - Hygrostat[ctr_output].timestamp_input_on) > ((uint32_t)Hygrostat[ctr_output].time_manual_to_auto * 60))) {
    change_state = true;
  }
  return change_state;
}

void HygrostatEmergencyShutdown(uint8_t ctr_output)
{
  // Emergency switch to HYGROSTAT_OFF
  Hygrostat[ctr_output].status.hygrostat_mode = HYGROSTAT_OFF;
  Hygrostat[ctr_output].status.command_output = IFACE_OFF;
  if (Hygrostat[ctr_output].status.enable_output == IFACE_ON) {
    HygrostatOutputRelay(ctr_output, Hygrostat[ctr_output].status.command_output);
  }
}

void HygrostatState(uint8_t ctr_output)
{
  switch (Hygrostat[ctr_output].status.hygrostat_mode) {
    // State if Off or Emergency
    case HYGROSTAT_OFF:
      // No change of state possible without external command
      break;
    // State automatic, hygrostat active following the command target hum.
    case HYGROSTAT_AUTOMATIC_OP:
      if (HygrostatStateAutoToManual(ctr_output)) {
        // If sensor not alive change to HYGROSTAT_MANUAL_OP
        Hygrostat[ctr_output].status.hygrostat_mode = HYGROSTAT_MANUAL_OP;
      }
      HygrostatCtrState(ctr_output);
      break;
    // State manual operation following input switch
    case HYGROSTAT_MANUAL_OP:
      if (HygrostatStateManualToAuto(ctr_output)) {
        // Input switch inactive and timeout reached change to HYGROSTAT_AUTOMATIC_OP
        Hygrostat[ctr_output].status.hygrostat_mode = HYGROSTAT_AUTOMATIC_OP;
      }
      break;
  }
}

void HygrostatOutputRelay(uint8_t ctr_output, uint32_t command)
{
  // If command received to enable output
  // AND current output status is OFF
  // then switch output to ON
  if ((command == IFACE_ON)
    && (Hygrostat[ctr_output].status.status_output == IFACE_OFF)) {
//#ifndef DEBUG_HYGROSTAT
    if (Hygrostat[ctr_output].status.enable_output == IFACE_ON) {
      ExecuteCommandPower(Hygrostat[ctr_output].status.output_relay_number, POWER_ON, SRC_HYGROSTAT);
    }
//#endif // DEBUG_HYGROSTAT
    Hygrostat[ctr_output].status.status_output = IFACE_ON;
#ifdef DEBUG_HYGROSTAT
    HygrostatVirtualSwitch(ctr_output);
#endif // DEBUG_HYGROSTAT
  }
  // If command received to disable output
  // AND current output status is ON
  // then switch output to OFF
  else if ((command == IFACE_OFF) && (Hygrostat[ctr_output].status.status_output == IFACE_ON)) {
//#ifndef DEBUG_HYGROSTAT
    if (Hygrostat[ctr_output].status.enable_output == IFACE_ON) {
      ExecuteCommandPower(Hygrostat[ctr_output].status.output_relay_number, POWER_OFF, SRC_HYGROSTAT);
    }
//#endif // DEBUG_HYGROSTAT
    Hygrostat[ctr_output].timestamp_output_off = TasmotaGlobal.uptime;
    Hygrostat[ctr_output].status.status_output = IFACE_OFF;
#ifdef DEBUG_HYGROSTAT
    HygrostatVirtualSwitch(ctr_output);
#endif // DEBUG_HYGROSTAT
  }
}

void HygrostatCalculatePI(uint8_t ctr_output)
{
  // General comment: Some variables have been increased in resolution to avoid loosing accuracy in division operations

  bool flag_heating = (Hygrostat[ctr_output].status.humidifier_mode == HUMIDIFYING);
  int32_t aux_hum_error;

  // Calculate error
  aux_hum_error = (int32_t)(Hygrostat[ctr_output].hum_target_level_ctr - Hygrostat[ctr_output].hum_measured) * 10;

  // Invert error for cooling
  if (Hygrostat[ctr_output].status.humidifier_mode == DEHUMIDIFYING) {
    aux_hum_error *= -1;
  }

  // Protect overflow
  if (aux_hum_error <= (int32_t)(INT16_MIN)) {
    Hygrostat[ctr_output].hum_pi_error = (int16_t)(INT16_MIN);
  }
  else if (aux_hum_error >= (int32_t)INT16_MAX) {
    Hygrostat[ctr_output].hum_pi_error = (int16_t)INT16_MAX;
  }
  else {
    Hygrostat[ctr_output].hum_pi_error = (int16_t)aux_hum_error;
  }

  // Kp = 100/PI.propBand. PI.propBand(Xp) = Proportional range (4K in 4K/200 controller)
  Hygrostat[ctr_output].kP_pi = 100 / (uint16_t)(Hygrostat[ctr_output].val_prop_band);
  // Calculate proportional
  Hygrostat[ctr_output].time_proportional_pi = ((int32_t)(Hygrostat[ctr_output].hum_pi_error * (int16_t)Hygrostat[ctr_output].kP_pi) * ((int32_t)Hygrostat[ctr_output].time_pi_cycle * 60)) / 10000;

  // Minimum proportional action limiter
  // If proportional action is less than the minimum action time
  // AND proportional > 0
  // then adjust to minimum value
  if ((Hygrostat[ctr_output].time_proportional_pi < abs(((int32_t)Hygrostat[ctr_output].time_min_action * 60)))
    && (Hygrostat[ctr_output].time_proportional_pi > 0)) {
    Hygrostat[ctr_output].time_proportional_pi = ((int32_t)Hygrostat[ctr_output].time_min_action * 60);
  }

  if (Hygrostat[ctr_output].time_proportional_pi < 0) {
    Hygrostat[ctr_output].time_proportional_pi = 0;
  }
  else if (Hygrostat[ctr_output].time_proportional_pi > ((int32_t)Hygrostat[ctr_output].time_pi_cycle * 60)) {
    Hygrostat[ctr_output].time_proportional_pi = ((int32_t)Hygrostat[ctr_output].time_pi_cycle * 60);
  }

  // Calculate integral (resolution increased to avoid use of floats in consequent operations)
  Hygrostat[ctr_output].kI_pi = (uint16_t)((((uint32_t)Hygrostat[ctr_output].kP_pi * (uint32_t)Hygrostat[ctr_output].time_pi_cycle * 6000)) / (uint32_t)Hygrostat[ctr_output].time_reset);

  // Reset of antiwindup
  // If error does not lay within the integrator scope range, do not use the integral
  // and accumulate error = 0
  if (abs((Hygrostat[ctr_output].hum_pi_error) / 10) > Hygrostat[ctr_output].hum_reset_anti_windup) {
    Hygrostat[ctr_output].time_integral_pi = 0;
    Hygrostat[ctr_output].hum_pi_accum_error = 0;
  }
  // Normal use of integrator
  // result will be calculated with the cummulated previous error anterior
  // and current error will be cummulated to the previous one
  else {
    // Hysteresis limiter
    // If error is less than or equal than hysteresis, limit output to 0, when humidity
    // is rising, never when falling. Limit cummulated error. If this is not done,
    // there will be very strong control actions from the integral part due to a
    // very high cummulated error when beingin hysteresis. This triggers high
    // integral actions

    // Update accumulated error
    aux_hum_error = (int32_t)Hygrostat[ctr_output].hum_pi_accum_error + (int32_t)Hygrostat[ctr_output].hum_pi_error;

    // Protect overflow
    if (aux_hum_error <= (int32_t)INT16_MIN) {
      Hygrostat[ctr_output].hum_pi_accum_error = INT16_MIN;
    }
    else if (aux_hum_error >= (int32_t)INT16_MAX) {
      Hygrostat[ctr_output].hum_pi_accum_error = INT16_MAX;
    }
    else {
      Hygrostat[ctr_output].hum_pi_accum_error = (int16_t)aux_hum_error;
    }

    // If we are under setpoint
    // AND we are within the hysteresis
    // AND the humidity is rising for heating or sinking for cooling
    if ( (Hygrostat[ctr_output].hum_pi_error >= 0)
      && (abs((Hygrostat[ctr_output].hum_pi_error) / 10) <= (int16_t)Hygrostat[ctr_output].hum_hysteresis)
      && (  ((Hygrostat[ctr_output].hum_measured_gradient > 0)
          && (flag_heating))
        || ( (Hygrostat[ctr_output].hum_measured_gradient < 0)
          && (!flag_heating)))) {
      // Reduce accumulator error 20% in each cycle
      Hygrostat[ctr_output].hum_pi_accum_error *= 0.8;
    }
    // If we are over setpoint
    // AND humidity is rising for heating or sinking for cooling
    else if ((Hygrostat[ctr_output].hum_pi_error < 0)
      && (  ((Hygrostat[ctr_output].hum_measured_gradient > 0)
          && (flag_heating))
        || ( (Hygrostat[ctr_output].hum_measured_gradient < 0)
          && (!flag_heating)))) {
      // Reduce accumulator error 20% in each cycle
      Hygrostat[ctr_output].hum_pi_accum_error *= 0.8;
    }

    // Limit lower limit of acumErr to 0
    if (Hygrostat[ctr_output].hum_pi_accum_error < 0) {
      Hygrostat[ctr_output].hum_pi_accum_error = 0;
    }

    // Integral calculation
    Hygrostat[ctr_output].time_integral_pi = (((int32_t)Hygrostat[ctr_output].hum_pi_accum_error * (int32_t)Hygrostat[ctr_output].kI_pi) * (int32_t)((uint32_t)Hygrostat[ctr_output].time_pi_cycle * 60)) / 1000000;

    // Antiwindup of the integrator
    // If integral calculation is bigger than cycle time, adjust result
    // to the cycle time and error will not be cummulated
    if (Hygrostat[ctr_output].time_integral_pi > ((uint32_t)Hygrostat[ctr_output].time_pi_cycle * 60)) {
      Hygrostat[ctr_output].time_integral_pi = ((uint32_t)Hygrostat[ctr_output].time_pi_cycle * 60);
    }
  }

  // Calculate output
  Hygrostat[ctr_output].time_total_pi = Hygrostat[ctr_output].time_proportional_pi + Hygrostat[ctr_output].time_integral_pi;

  // Antiwindup of the output
  // If result is bigger than cycle time, the result will be adjusted
  // to the cylce time minus safety time and error will not be cummulated
  if (Hygrostat[ctr_output].time_total_pi >= ((int32_t)Hygrostat[ctr_output].time_pi_cycle * 60)) {
    // Limit to cycle time //at least switch down a minimum time
    Hygrostat[ctr_output].time_total_pi = ((int32_t)Hygrostat[ctr_output].time_pi_cycle * 60);
  }
  else if (Hygrostat[ctr_output].time_total_pi < 0) {
    Hygrostat[ctr_output].time_total_pi = 0;
  }

  // Target value limiter
  // If target value has been reached or we are over it for heating or under it for cooling
  if (Hygrostat[ctr_output].hum_pi_error <= 0) {
    // If we are over the hysteresis or the gradient is positive for heating or negative for cooling
    if ((abs((Hygrostat[ctr_output].hum_pi_error) / 10) > Hygrostat[ctr_output].hum_hysteresis)
      || (  ((Hygrostat[ctr_output].hum_measured_gradient >= 0)
          && (flag_heating))
        || ( (Hygrostat[ctr_output].hum_measured_gradient <= 0)
          && (!flag_heating)))){
      Hygrostat[ctr_output].time_total_pi = 0;
    }
  }
  // If target value has not been reached
  // AND we are within the histeresis
  // AND gradient is positive for heating or negative for cooling
  // then set value to 0
  else if ((Hygrostat[ctr_output].hum_pi_error > 0)
    && (abs((Hygrostat[ctr_output].hum_pi_error) / 10) <= Hygrostat[ctr_output].hum_hysteresis)
    && (((Hygrostat[ctr_output].hum_measured_gradient > 0)
        && (flag_heating))
      || ( (Hygrostat[ctr_output].hum_measured_gradient < 0)
        && (!flag_heating)))) {
    Hygrostat[ctr_output].time_total_pi = 0;
  }

  // Minimum action limiter
  // If result is less than the minimum action time, adjust to minimum value
  if ((Hygrostat[ctr_output].time_total_pi <= abs(((int32_t)Hygrostat[ctr_output].time_min_action * 60)))
    && (Hygrostat[ctr_output].time_total_pi != 0)) {
    Hygrostat[ctr_output].time_total_pi = ((int32_t)Hygrostat[ctr_output].time_min_action * 60);
  }
  // Maximum action limiter
  // If result is more than the maximum action time, adjust to maximum value
  else if (Hygrostat[ctr_output].time_total_pi > abs(((int32_t)Hygrostat[ctr_output].time_max_action * 60))) {
    Hygrostat[ctr_output].time_total_pi = ((int32_t)Hygrostat[ctr_output].time_max_action * 60);
  }
  // If switched off less time than safety time, do not switch off
  else if (Hygrostat[ctr_output].time_total_pi > (((int32_t)Hygrostat[ctr_output].time_pi_cycle * 60) - ((int32_t)Hygrostat[ctr_output].time_min_turnoff_action * 60))) {
    Hygrostat[ctr_output].time_total_pi = ((int32_t)Hygrostat[ctr_output].time_pi_cycle * 60);
  }

  // Adjust output switch point
  Hygrostat[ctr_output].time_ctr_changepoint = TasmotaGlobal.uptime + (uint32_t)Hygrostat[ctr_output].time_total_pi;
  // Adjust next cycle point
  Hygrostat[ctr_output].time_ctr_checkpoint = TasmotaGlobal.uptime + ((uint32_t)Hygrostat[ctr_output].time_pi_cycle * 60);
}

void HygrostatWorkAutomaticPI(uint8_t ctr_output)
{
  bool flag_heating = (Hygrostat[ctr_output].status.humidifier_mode == HUMIDIFYING);
  if ( (TasmotaGlobal.uptime >= Hygrostat[ctr_output].time_ctr_checkpoint)
    || (Hygrostat[ctr_output].hum_target_level != Hygrostat[ctr_output].hum_target_level_ctr)
    || (  (( (Hygrostat[ctr_output].hum_measured < Hygrostat[ctr_output].hum_target_level)
          && (Hygrostat[ctr_output].hum_measured_gradient < 0)
          && (flag_heating))
        || ((Hygrostat[ctr_output].hum_measured > Hygrostat[ctr_output].hum_target_level)
          && (Hygrostat[ctr_output].hum_measured_gradient > 0)
          && (!flag_heating)))
        && (Hygrostat[ctr_output].status.status_cycle_active == CYCLE_OFF))) {
    Hygrostat[ctr_output].hum_target_level_ctr = Hygrostat[ctr_output].hum_target_level;
    HygrostatCalculatePI(ctr_output);
    // Reset cycle active
    Hygrostat[ctr_output].status.status_cycle_active = CYCLE_OFF;
  }
  if (TasmotaGlobal.uptime < Hygrostat[ctr_output].time_ctr_changepoint) {
    Hygrostat[ctr_output].status.status_cycle_active = CYCLE_ON;
    Hygrostat[ctr_output].status.command_output = IFACE_ON;
  }
  else {
    Hygrostat[ctr_output].status.command_output = IFACE_OFF;
  }
}

void HygrostatWorkAutomaticRampUp(uint8_t ctr_output)
{
  uint32_t time_in_rampup;
  int16_t aux_hum_delta;
  int16_t hum_delta_rampup;
  bool flag_heating = (Hygrostat[ctr_output].status.humidifier_mode == HUMIDIFYING);

  // Update timestamp for humidity at start of ramp-up if humidity still
  // dropping for heating or rising for cooling
  if (    ((Hygrostat[ctr_output].hum_measured < Hygrostat[ctr_output].hum_rampup_start)
        && (flag_heating))
      ||  ((Hygrostat[ctr_output].hum_measured > Hygrostat[ctr_output].hum_rampup_start)
        && (!flag_heating)))
     {
    Hygrostat[ctr_output].hum_rampup_start = Hygrostat[ctr_output].hum_measured;
  }

  // Update time in ramp-up as well as delta hum
  time_in_rampup = TasmotaGlobal.uptime - Hygrostat[ctr_output].timestamp_rampup_start;
  hum_delta_rampup = Hygrostat[ctr_output].hum_measured - Hygrostat[ctr_output].hum_rampup_start;
  // Init command output status to true
  Hygrostat[ctr_output].status.command_output = IFACE_ON;
  // Update humidity target level for controller
  Hygrostat[ctr_output].hum_target_level_ctr = Hygrostat[ctr_output].hum_target_level;

  // If time in ramp-up < max time
  // AND humidity measured < target for heating or > for cooling
  if ((time_in_rampup <= (60 * (uint32_t)Hygrostat[ctr_output].time_rampup_max))
    && (  ((Hygrostat[ctr_output].hum_measured < Hygrostat[ctr_output].hum_target_level)
        && (flag_heating))
      ||  ((Hygrostat[ctr_output].hum_measured > Hygrostat[ctr_output].hum_target_level)
        && (!flag_heating)))){
    // DEADTIME point reached
    // If humidity measured minus humidity at start of ramp-up >= threshold
    // AND deadtime still 0
    if ( (abs(hum_delta_rampup) >= Hygrostat[ctr_output].hum_rampup_delta_out)
      && (Hygrostat[ctr_output].time_rampup_deadtime == 0)) {
      // Set deadtime, assuming it is half of the time until slope, since humidity resistance of the hum. fall needs to be considered
      // minus open time of the valve (arround 3 minutes). If rise/sink very fast limit it to delay of output valve
      int32_t time_aux;
      time_aux = ((time_in_rampup / 2) - Hygrostat[ctr_output].time_output_delay);
      if (time_aux >= Hygrostat[ctr_output].time_output_delay) {
        Hygrostat[ctr_output].time_rampup_deadtime = (uint32_t)time_aux;
      }
      else {
        Hygrostat[ctr_output].time_rampup_deadtime = Hygrostat[ctr_output].time_output_delay;
      }
      // Calculate absolute gradient since start of ramp-up (considering deadtime) in thousandths of º/hour
      Hygrostat[ctr_output].hum_rampup_meas_gradient = (int32_t)((360000 * (int32_t)hum_delta_rampup) / (int32_t)time_in_rampup);
      Hygrostat[ctr_output].time_rampup_nextcycle = TasmotaGlobal.uptime + ((uint32_t)Hygrostat[ctr_output].time_rampup_cycle * 60);
      // Set auxiliary variables
      Hygrostat[ctr_output].hum_rampup_cycle = Hygrostat[ctr_output].hum_measured;
      Hygrostat[ctr_output].time_ctr_changepoint = TasmotaGlobal.uptime + (60 * (uint32_t)Hygrostat[ctr_output].time_rampup_max);
      Hygrostat[ctr_output].hum_rampup_output_off =  Hygrostat[ctr_output].hum_target_level_ctr;
    }
    // Gradient calculation every time_rampup_cycle
    else if ((Hygrostat[ctr_output].time_rampup_deadtime > 0) && (TasmotaGlobal.uptime >= Hygrostat[ctr_output].time_rampup_nextcycle)) {
      // Calculate hum. gradient in º/hour and set again time_rampup_nextcycle and hum_rampup_cycle
      // hum_rampup_meas_gradient = ((3600 * hum_delta_rampup) / (os.time() - time_rampup_nextcycle))
      hum_delta_rampup = Hygrostat[ctr_output].hum_measured - Hygrostat[ctr_output].hum_rampup_cycle;
      uint32_t time_total_rampup = (uint32_t)Hygrostat[ctr_output].time_rampup_cycle * 60 * Hygrostat[ctr_output].counter_rampup_cycles;
      // Translate into gradient per hour (thousandths of ° per hour)
      Hygrostat[ctr_output].hum_rampup_meas_gradient = int32_t((360000 * (int32_t)hum_delta_rampup) / (int32_t)time_total_rampup);
      if (   ((Hygrostat[ctr_output].hum_rampup_meas_gradient > 0)
          && ((flag_heating)))
        ||   ((Hygrostat[ctr_output].hum_rampup_meas_gradient < 0)
          && ((!flag_heating)))) {
        // Calculate time to switch Off and come out of ramp-up
        // y-y1 = m(x-x1) -> x = ((y-y1) / m) + x1 -> y1 = hum_rampup_cycle, x1 = (time_rampup_nextcycle - time_rampup_cycle), m = gradient in º/sec
        // Better Alternative -> (y-y1)/(x-x1) = ((y2-y1)/(x2-x1)) -> where y = hum (target) and x = time (to switch off, what its needed)
        // x = ((y-y1)/(y2-y1))*(x2-x1) + x1 - deadtime
        aux_hum_delta =Hygrostat[ctr_output].hum_target_level_ctr - Hygrostat[ctr_output].hum_rampup_cycle;
        Hygrostat[ctr_output].time_ctr_changepoint = (uint32_t)(uint32_t)(((uint32_t)(aux_hum_delta) * (uint32_t)(time_total_rampup)) / (uint32_t)hum_delta_rampup) + (uint32_t)Hygrostat[ctr_output].time_rampup_nextcycle - (uint32_t)time_total_rampup - (uint32_t)Hygrostat[ctr_output].time_rampup_deadtime;

        // Calculate humidity for switching off the output
        // y = (((y2-y1)/(x2-x1))*(x-x1)) + y1
        Hygrostat[ctr_output].hum_rampup_output_off = (int16_t)(((int32_t)hum_delta_rampup * (int32_t)(Hygrostat[ctr_output].time_ctr_changepoint - (TasmotaGlobal.uptime - (time_total_rampup)))) / (int32_t)(time_total_rampup * Hygrostat[ctr_output].counter_rampup_cycles)) + Hygrostat[ctr_output].hum_rampup_cycle;
        // Set auxiliary variables
        Hygrostat[ctr_output].time_rampup_nextcycle = TasmotaGlobal.uptime + ((uint32_t)Hygrostat[ctr_output].time_rampup_cycle * 60);
        Hygrostat[ctr_output].hum_rampup_cycle = Hygrostat[ctr_output].hum_measured;
        // Reset period counter
        Hygrostat[ctr_output].counter_rampup_cycles = 1;
      }
      else {
        // Increase the period counter
        Hygrostat[ctr_output].counter_rampup_cycles++;
        // Set another period
        Hygrostat[ctr_output].time_rampup_nextcycle = TasmotaGlobal.uptime + ((uint32_t)Hygrostat[ctr_output].time_rampup_cycle * 60);
        // Reset time_ctr_changepoint and hum_rampup_output_off
        Hygrostat[ctr_output].time_ctr_changepoint = TasmotaGlobal.uptime + (60 * (uint32_t)Hygrostat[ctr_output].time_rampup_max) - time_in_rampup;
        Hygrostat[ctr_output].hum_rampup_output_off =  Hygrostat[ctr_output].hum_target_level_ctr;
      }
      // Set time to get out of ramp-up
      Hygrostat[ctr_output].time_ctr_checkpoint = Hygrostat[ctr_output].time_ctr_changepoint + Hygrostat[ctr_output].time_rampup_deadtime;
    }

    // Set output switch ON or OFF
    // If deadtime has not been calculated
    // or checkpoint has not been calculated
    // or it is not yet time and humidity to switch it off acc. to calculations
    // or gradient is <= 0 for heating of >= 0 for cooling
    if ((Hygrostat[ctr_output].time_rampup_deadtime == 0)
      || (Hygrostat[ctr_output].time_ctr_checkpoint == 0)
      || (TasmotaGlobal.uptime < Hygrostat[ctr_output].time_ctr_changepoint)
      || (  ((Hygrostat[ctr_output].hum_measured < Hygrostat[ctr_output].hum_rampup_output_off)
          && (flag_heating))
        ||  ((Hygrostat[ctr_output].hum_measured > Hygrostat[ctr_output].hum_rampup_output_off)
          && (!flag_heating)))
      || (  ((Hygrostat[ctr_output].hum_rampup_meas_gradient <= 0)
          && (flag_heating))
        ||  ((Hygrostat[ctr_output].hum_rampup_meas_gradient >= 0)
          && (!flag_heating)))) {
      Hygrostat[ctr_output].status.command_output = IFACE_ON;
    }
    else {
      Hygrostat[ctr_output].status.command_output = IFACE_OFF;
    }
  }
  else {
    // If we have not reached the humidity, start with an initial value for accumulated error for the PI controller
    if (  ((Hygrostat[ctr_output].hum_measured < Hygrostat[ctr_output].hum_target_level_ctr)
        && (flag_heating))
      ||  ((Hygrostat[ctr_output].hum_measured > Hygrostat[ctr_output].hum_target_level_ctr)
        && (!flag_heating))) {
      Hygrostat[ctr_output].hum_pi_accum_error = Hygrostat[ctr_output].hum_rampup_pi_acc_error;
    }
    // Set to now time to get out of ramp-up
    Hygrostat[ctr_output].time_ctr_checkpoint = TasmotaGlobal.uptime;
    // Switch Off output
    Hygrostat[ctr_output].status.command_output = IFACE_OFF;
  }
}

#ifdef USE_PI_AUTOTUNING

void HygrostatPeakDetectorInit(uint8_t ctr_output)
{
  for (uint8_t i = 0; i < HYGROSTAT_PEAKNUMBER_AUTOTUNE; i++) {
    Hygrostat[ctr_output].hum_peaks_atune[i] = 0;
  }
  Hygrostat[ctr_output].pU_pi_atune = 0;
  Hygrostat[ctr_output].kP_pi_atune = 0;
  Hygrostat[ctr_output].kI_pi_atune = 0;
  Hygrostat[ctr_output].kU_pi_atune = 0;
  Hygrostat[ctr_output].peak_ctr = 0;
  Hygrostat[ctr_output].hum_abs_max_atune = 0;
  Hygrostat[ctr_output].hum_abs_min_atune = 100;
  Hygrostat[ctr_output].time_ctr_checkpoint = TasmotaGlobal.uptime + HYGROSTAT_TIME_MAX_AUTOTUNE;
}

void HygrostatPeakDetector(uint8_t ctr_output)
{
  uint8_t peak_num = Hygrostat[ctr_output].peak_ctr;
  int16_t peak_avg = 0;
  bool peak_transition = false;
  // Update Max/Min Hygrostat[ctr_output].hum_abs_max_atune
  if (Hygrostat[ctr_output].hum_measured > Hygrostat[ctr_output].hum_abs_max_atune) {
    Hygrostat[ctr_output].hum_abs_max_atune = Hygrostat[ctr_output].hum_measured;
  }
  if (Hygrostat[ctr_output].hum_measured < Hygrostat[ctr_output].hum_abs_min_atune) {
    Hygrostat[ctr_output].hum_abs_min_atune = Hygrostat[ctr_output].hum_measured;
  }
  // For heating, even peak numbers look for maxes, odd for minds, the contrary for cooling
  // If we did not found all peaks yet
  if (peak_num < HYGROSTAT_PEAKNUMBER_AUTOTUNE) {
    bool flag_heating = (Hygrostat[ctr_output].status.humidifier_mode == HUMIDIFYING);
    bool cond_peak_1 = (   (Hygrostat[ctr_output].hum_measured > Hygrostat[ctr_output].hum_peaks_atune[peak_num])
                        && (flag_heating)
                      ||   (Hygrostat[ctr_output].hum_measured < Hygrostat[ctr_output].hum_peaks_atune[peak_num])
                        && (!flag_heating));
    bool cond_peak_2 = (   (Hygrostat[ctr_output].hum_measured < Hygrostat[ctr_output].hum_peaks_atune[peak_num])
                        && (flag_heating)
                      ||   (Hygrostat[ctr_output].hum_measured > Hygrostat[ctr_output].hum_peaks_atune[peak_num])
                        && (!flag_heating));
    bool cond_gradient_1 = (   (Hygrostat[ctr_output].hum_measured_gradient > 0)
                            && (flag_heating)
                          ||   (Hygrostat[ctr_output].hum_measured_gradient < 0)
                            && (!flag_heating));
    bool cond_gradient_2 = (   (Hygrostat[ctr_output].hum_measured_gradient < 0)
                            && (flag_heating)
                          ||   (Hygrostat[ctr_output].hum_measured_gradient > 0)
                            && (!flag_heating));
    // If peak number is even (look for max if heating and min if cooling)
    if ((peak_num % 2) == 0) {
      // If current humidity higher (heating) or lower (cooling) than registered value for peak
      // AND humidity gradient > 0 for heating or < 0 for cooling
      // then, update value
      if (cond_peak_1 && cond_gradient_1) {
        Hygrostat[ctr_output].hum_peaks_atune[peak_num] = Hygrostat[ctr_output].hum_measured;
      }
      // Else if current humidity lower (heating) or higher (cooling) then registered value for peak
      // AND difference to peak is outside of the peak no detection band
      // then the current peak value is the peak (max for heating, min for cooling), switch detection
      if ( (cond_peak_2)
        && (abs(Hygrostat[ctr_output].hum_measured - Hygrostat[ctr_output].hum_peaks_atune[peak_num]) > Hygrostat[ctr_output].hum_band_no_peak_det)) {
        // Register peak timestamp;
        Hygrostat[ctr_output].time_peak_timestamps_atune[peak_num] = (TasmotaGlobal.uptime / 60);
        Hygrostat[ctr_output].peak_ctr++;
        peak_transition = true;
      }
    }
    // Peak number is odd (look for min if heating and max if cooling)
    else {
      // If current humidity lower (heating) or higher (cooling) than registered value for peak
      // AND humidity gradient < 0 for heating or > 0 for cooling
      // then, update value
      if (cond_peak_2 && cond_gradient_2) {
        Hygrostat[ctr_output].hum_peaks_atune[peak_num] = Hygrostat[ctr_output].hum_measured;
      }
      // Else if current humidity higher (heating) or lower (cooling) then registered value for peak
      // AND difference to peak is outside of the peak no detection band
      // then the current peak value is the peak (min for heating, max for cooling), switch detection
      if ( (cond_peak_1)
        && (abs(Hygrostat[ctr_output].hum_measured - Hygrostat[ctr_output].hum_peaks_atune[peak_num]) > Hygrostat[ctr_output].hum_band_no_peak_det)) {
        // Calculate period
        // Register peak timestamp;
        Hygrostat[ctr_output].time_peak_timestamps_atune[peak_num] = (TasmotaGlobal.uptime / 60);
        Hygrostat[ctr_output].peak_ctr++;
        peak_transition = true;
      }
    }
  }
  else {
    // Peak detection done, proceed to evaluate results
    HygrostatAutotuneParamCalc(ctr_output);
    Hygrostat[ctr_output].status.autotune_flag = AUTOTUNE_OFF;
  }

  // If peak detection not finalized but bigger than 3 and we have just found a peak, check if results can be extracted
  if ((Hygrostat[ctr_output].peak_ctr > 2) && (peak_transition)) {
    //Update peak_num
    peak_num = Hygrostat[ctr_output].peak_ctr;
    // Calculate average value among the last 3 peaks
    peak_avg = (abs(Hygrostat[ctr_output].hum_peaks_atune[peak_num - 1]
                    - Hygrostat[ctr_output].hum_peaks_atune[peak_num - 2])
              + abs(Hygrostat[ctr_output].hum_peaks_atune[peak_num - 2]
                    - Hygrostat[ctr_output].hum_peaks_atune[peak_num - 3])) / 2;

    if ((20 * (int32_t)peak_avg) < (int32_t)(Hygrostat[ctr_output].hum_abs_max_atune - Hygrostat[ctr_output].hum_abs_min_atune)) {
      // Calculate average humidity among all peaks
      for (uint8_t i = 0; i < peak_num; i++) {
        peak_avg += Hygrostat[ctr_output].hum_peaks_atune[i];
      }
      peak_avg /= peak_num;
      // If last period crosses the average value, result valid
      if (10 * abs(Hygrostat[ctr_output].hum_peaks_atune[peak_num - 1] - Hygrostat[ctr_output].hum_peaks_atune[peak_num - 2]) < (Hygrostat[ctr_output].hum_abs_max_atune - peak_avg)) {
        // Peak detection done, proceed to evaluate results
        HygrostatAutotuneParamCalc(ctr_output);
        Hygrostat[ctr_output].status.autotune_flag = AUTOTUNE_OFF;
      }
    }
  }
  peak_transition = false;
}

void HygrostatAutotuneParamCalc(uint8_t ctr_output)
{
  uint8_t peak_num = Hygrostat[ctr_output].peak_ctr;

  // Calculate the tunning parameters
  // Resolution increased to avoid float operations
  Hygrostat[ctr_output].kU_pi_atune = (uint16_t)(100 * ((uint32_t)400000 * (uint32_t)(Hygrostat[ctr_output].dutycycle_step_autotune)) / ((uint32_t)(Hygrostat[ctr_output].hum_abs_max_atune - Hygrostat[ctr_output].hum_abs_min_atune) * (uint32_t)314159));
  Hygrostat[ctr_output].pU_pi_atune = (Hygrostat[ctr_output].time_peak_timestamps_atune[peak_num - 1] - Hygrostat[ctr_output].time_peak_timestamps_atune[peak_num - 2]);

  switch (Hygrostat[ctr_output].status.autotune_perf_mode) {
    case AUTOTUNE_PERF_FAST:
      // Calculate kP/Ki autotune
      Hygrostat[ctr_output].kP_pi_atune = (4 * Hygrostat[ctr_output].kU_pi_atune) / 10;
      break;
    case AUTOTUNE_PERF_NORMAL:
      // Calculate kP/Ki autotune
      Hygrostat[ctr_output].kP_pi_atune = (18 * Hygrostat[ctr_output].kU_pi_atune) / 100;
      break;
    case AUTOTUNE_PERF_SLOW:
      // Calculate kP/Ki autotune
      Hygrostat[ctr_output].kP_pi_atune = (13 * Hygrostat[ctr_output].kU_pi_atune) / 100;
      break;
  }

  // Resolution increased to avoid float operations
  Hygrostat[ctr_output].kI_pi_atune = (12 * (6000 * Hygrostat[ctr_output].kU_pi_atune / Hygrostat[ctr_output].pU_pi_atune)) / 10;

  // Calculate PropBand Autotune
  Hygrostat[ctr_output].val_prop_band_atune = 100 / Hygrostat[ctr_output].kP_pi_atune;
  // Calculate Reset Time Autotune
  Hygrostat[ctr_output].time_reset_atune = (uint32_t)((((uint32_t)Hygrostat[ctr_output].kP_pi_atune * (uint32_t)Hygrostat[ctr_output].time_pi_cycle * 6000)) / (uint32_t)Hygrostat[ctr_output].kI_pi_atune);
}

void HygrostatWorkAutomaticPIAutotune(uint8_t ctr_output)
{
  bool flag_heating = (Hygrostat[ctr_output].status.humidifier_mode == HUMIDIFYING);
  // If no timeout of the PI Autotune function
  // AND no change in setpoint
  if ((TasmotaGlobal.uptime < Hygrostat[ctr_output].time_ctr_checkpoint)
    &&(Hygrostat[ctr_output].hum_target_level_ctr == Hygrostat[ctr_output].hum_target_level)) {
    if (TasmotaGlobal.uptime >= Hygrostat[ctr_output].time_ctr_checkpoint) {
      Hygrostat[ctr_output].hum_target_level_ctr = Hygrostat[ctr_output].hum_target_level;
      // Calculate time_ctr_changepoint
      Hygrostat[ctr_output].time_ctr_changepoint = TasmotaGlobal.uptime + (((uint32_t)Hygrostat[ctr_output].time_pi_cycle * (uint32_t)Hygrostat[ctr_output].dutycycle_step_autotune) / (uint32_t)100);
      // Reset cycle active
      Hygrostat[ctr_output].status.status_cycle_active = CYCLE_OFF;
    }
    // Set Output On/Off depending on the changepoint
    if (TasmotaGlobal.uptime < Hygrostat[ctr_output].time_ctr_changepoint) {
      Hygrostat[ctr_output].status.status_cycle_active = CYCLE_ON;
      Hygrostat[ctr_output].status.command_output = IFACE_ON;
    }
    else {
      Hygrostat[ctr_output].status.command_output = IFACE_OFF;
    }
    // Update peak values
    HygrostatPeakDetector(ctr_output);
  }
  else {
    // Disable Autotune flag
    Hygrostat[ctr_output].status.autotune_flag = AUTOTUNE_OFF;
  }

  if (Hygrostat[ctr_output].status.autotune_flag == AUTOTUNE_OFF) {
    // Set output Off
    Hygrostat[ctr_output].status.command_output = IFACE_OFF;
  }
}
#endif //USE_PI_AUTOTUNING

void HygrostatCtrWork(uint8_t ctr_output)
{
  switch (Hygrostat[ctr_output].status.controller_mode) {
    // Hybrid controller (Ramp-up + PI)
    case CTR_HYBRID:
      switch (Hygrostat[ctr_output].status.phase_hybrid_ctr) {
        case CTR_HYBRID_RAMP_UP:
          HygrostatWorkAutomaticRampUp(ctr_output);
          break;
        case CTR_HYBRID_PI:
          HygrostatWorkAutomaticPI(ctr_output);
          break;
#ifdef USE_PI_AUTOTUNING
        // PI autotune
        case CTR_HYBRID_PI_AUTOTUNE:
          HygrostatWorkAutomaticPIAutotune(ctr_output);
          break;
#endif //USE_PI_AUTOTUNING
      }
      break;
    // PI controller
    case CTR_PI:
      HygrostatWorkAutomaticPI(ctr_output);
      break;
    // Ramp-up controller (predictive)
    case CTR_RAMP_UP:
      HygrostatWorkAutomaticRampUp(ctr_output);
      break;
#ifdef USE_PI_AUTOTUNING
    // PI autotune
    case CTR_PI_AUTOTUNE:
      HygrostatWorkAutomaticPIAutotune(ctr_output);
      break;
#endif //USE_PI_AUTOTUNING
  }
}

void HygrostatWork(uint8_t ctr_output)
{
  switch (Hygrostat[ctr_output].status.hygrostat_mode) {
    // State if hygrostat Off or Emergency
    case HYGROSTAT_OFF:
      Hygrostat[ctr_output].status.command_output = IFACE_OFF;
      break;
    // State automatic hygrostat active following to command target hum.
    case HYGROSTAT_AUTOMATIC_OP:
      HygrostatCtrWork(ctr_output);

      break;
    // State manual operation following input switch
    case HYGROSTAT_MANUAL_OP:
      Hygrostat[ctr_output].time_ctr_checkpoint = 0;
      Hygrostat[ctr_output].status.command_output = Hygrostat[ctr_output].status.status_input;
      break;
  }
  HygrostatOutputRelay(ctr_output, Hygrostat[ctr_output].status.command_output);
}

void HygrostatDiagnostics(uint8_t ctr_output)
{
  // Diagnostic related to the plausibility of the output state
  if ((Hygrostat[ctr_output].diag.diagnostic_mode == DIAGNOSTIC_ON)
    &&(Hygrostat[ctr_output].diag.output_inconsist_ctr >= HYGROSTAT_TIME_MAX_OUTPUT_INCONSIST)) {
    Hygrostat[ctr_output].status.hygrostat_mode = HYGROSTAT_OFF;
    Hygrostat[ctr_output].diag.state_emergency = EMERGENCY_ON;
  }

  // Diagnostic related to the plausibility of the output power implemented
  // already into the energy driver

  // If diagnostics fail, emergency enabled and hygrostat shutdown triggered
  if (Hygrostat[ctr_output].diag.state_emergency == EMERGENCY_ON) {
    HygrostatEmergencyShutdown(ctr_output);
  }
}

void HygrostatController(uint8_t ctr_output)
{
  HygrostatState(ctr_output);
  HygrostatWork(ctr_output);
}

bool HygrostatTimerArm(uint8_t ctr_output, int16_t humVal)
{
  bool result = false;
  // HumVal unit is tenths of percents
  if ((humVal >= -1000)
    && (humVal <= 2000)
    && (humVal >= Hygrostat[ctr_output].hum_toodry_protect)) {
      Hygrostat[ctr_output].hum_target_level = humVal;
      Hygrostat[ctr_output].status.hygrostat_mode = HYGROSTAT_AUTOMATIC_OP;
      result = true;
  }
  // Returns true if setpoint plausible and hygrostat armed, false on the contrary
  return result;
}

void HygrostatTimerDisarm(uint8_t ctr_output)
{
  Hygrostat[ctr_output].hum_target_level = HYGROSTAT_HUM_INIT;
  Hygrostat[ctr_output].status.hygrostat_mode = HYGROSTAT_OFF;
}

#ifdef DEBUG_HYGROSTAT

void HygrostatVirtualSwitch(uint8_t ctr_output)
{
  char domoticz_in_topic[] = DOMOTICZ_IN_TOPIC;
  if (ctr_output < DOMOTICZ_MAX_IDX) {
    Response_P(DOMOTICZ_MES, Domoticz_Virtual_Switches[ctr_output], (0 == Hygrostat[ctr_output].status.command_output) ? 0 : 1, "");
    MqttPublish(domoticz_in_topic);
  }
}

void HygrostatVirtualSwitchCtrState(uint8_t ctr_output)
{
  char domoticz_in_topic[] = DOMOTICZ_IN_TOPIC;
  Response_P(DOMOTICZ_MES, DOMOTICZ_IDX2, (0 == Hygrostat[0].status.phase_hybrid_ctr) ? 0 : 1, "");
  MqttPublish(domoticz_in_topic);
}

void HygrostatDebug(uint8_t ctr_output)
{
  char result_chr[FLOATSZ];
  AddLog(LOG_LEVEL_DEBUG, PSTR(""));
  AddLog(LOG_LEVEL_DEBUG, PSTR("------ Hygrostat Start ------"));
  dtostrfd(Hygrostat[ctr_output].status.counter_seconds, 0, result_chr);
  AddLog(LOG_LEVEL_DEBUG, PSTR("Hygrostat[ctr_output].status.counter_seconds: %s"), result_chr);
  dtostrfd(Hygrostat[ctr_output].status.hygrostat_mode, 0, result_chr);
  AddLog(LOG_LEVEL_DEBUG, PSTR("Hygrostat[ctr_output].status.hygrostat_mode: %s"), result_chr);
  dtostrfd(Hygrostat[ctr_output].diag.state_emergency, 0, result_chr);
  AddLog(LOG_LEVEL_DEBUG, PSTR("Hygrostat[ctr_output].diag.state_emergency: %s"), result_chr);
  dtostrfd(Hygrostat[ctr_output].diag.output_inconsist_ctr, 0, result_chr);
  AddLog(LOG_LEVEL_DEBUG, PSTR("Hygrostat[ctr_output].diag.output_inconsist_ctr: %s"), result_chr);
  dtostrfd(Hygrostat[ctr_output].status.controller_mode, 0, result_chr);
  AddLog(LOG_LEVEL_DEBUG, PSTR("Hygrostat[ctr_output].status.controller_mode: %s"), result_chr);
  dtostrfd(Hygrostat[ctr_output].status.command_output, 0, result_chr);
  AddLog(LOG_LEVEL_DEBUG, PSTR("Hygrostat[ctr_output].status.command_output: %s"), result_chr);
  dtostrfd(Hygrostat[ctr_output].status.status_output, 0, result_chr);
  AddLog(LOG_LEVEL_DEBUG, PSTR("Hygrostat[ctr_output].status.status_output: %s"), result_chr);
  dtostrfd(Hygrostat[ctr_output].status.status_input, 0, result_chr);
  AddLog(LOG_LEVEL_DEBUG, PSTR("Hygrostat[ctr_output].status.status_input: %s"), result_chr);
  dtostrfd(Hygrostat[ctr_output].status.phase_hybrid_ctr, 0, result_chr);
  AddLog(LOG_LEVEL_DEBUG, PSTR("Hygrostat[ctr_output].status.phase_hybrid_ctr: %s"), result_chr);
  dtostrfd(Hygrostat[ctr_output].status.sensor_alive, 0, result_chr);
  AddLog(LOG_LEVEL_DEBUG, PSTR("Hygrostat[ctr_output].status.sensor_alive: %s"), result_chr);
  dtostrfd(Hygrostat[ctr_output].status.status_cycle_active, 0, result_chr);
  AddLog(LOG_LEVEL_DEBUG, PSTR("Hygrostat[ctr_output].status.status_cycle_active: %s"), result_chr);
  dtostrfd(Hygrostat[ctr_output].hum_pi_error, 0, result_chr);
  AddLog(LOG_LEVEL_DEBUG, PSTR("Hygrostat[ctr_output].hum_pi_error: %s"), result_chr);
  dtostrfd(Hygrostat[ctr_output].hum_pi_accum_error, 0, result_chr);
  AddLog(LOG_LEVEL_DEBUG, PSTR("Hygrostat[ctr_output].hum_pi_accum_error: %s"), result_chr);
  dtostrfd(Hygrostat[ctr_output].time_proportional_pi, 0, result_chr);
  AddLog(LOG_LEVEL_DEBUG, PSTR("Hygrostat[ctr_output].time_proportional_pi: %s"), result_chr);
  dtostrfd(Hygrostat[ctr_output].time_integral_pi, 0, result_chr);
  AddLog(LOG_LEVEL_DEBUG, PSTR("Hygrostat[ctr_output].time_integral_pi: %s"), result_chr);
  dtostrfd(Hygrostat[ctr_output].time_total_pi, 0, result_chr);
  AddLog(LOG_LEVEL_DEBUG, PSTR("Hygrostat[ctr_output].time_total_pi: %s"), result_chr);
  dtostrfd(Hygrostat[ctr_output].hum_measured_gradient, 0, result_chr);
  AddLog(LOG_LEVEL_DEBUG, PSTR("Hygrostat[ctr_output].hum_measured_gradient: %s"), result_chr);
  dtostrfd(Hygrostat[ctr_output].time_rampup_deadtime, 0, result_chr);
  AddLog(LOG_LEVEL_DEBUG, PSTR("Hygrostat[ctr_output].time_rampup_deadtime: %s"), result_chr);
  dtostrfd(Hygrostat[ctr_output].hum_rampup_meas_gradient, 0, result_chr);
  AddLog(LOG_LEVEL_DEBUG, PSTR("Hygrostat[ctr_output].hum_rampup_meas_gradient: %s"), result_chr);
  dtostrfd(Hygrostat[ctr_output].time_ctr_changepoint, 0, result_chr);
  AddLog(LOG_LEVEL_DEBUG, PSTR("Hygrostat[ctr_output].time_ctr_changepoint: %s"), result_chr);
  dtostrfd(Hygrostat[ctr_output].hum_rampup_output_off, 0, result_chr);
  AddLog(LOG_LEVEL_DEBUG, PSTR("Hygrostat[ctr_output].hum_rampup_output_off: %s"), result_chr);
  dtostrfd(Hygrostat[ctr_output].time_ctr_checkpoint, 0, result_chr);
  AddLog(LOG_LEVEL_DEBUG, PSTR("Hygrostat[ctr_output].time_ctr_checkpoint: %s"), result_chr);
  dtostrfd(TasmotaGlobal.uptime, 0, result_chr);
  AddLog(LOG_LEVEL_DEBUG, PSTR("uptime: %s"), result_chr);
  dtostrfd(TasmotaGlobal.power, 0, result_chr);
  AddLog(LOG_LEVEL_DEBUG, PSTR("power: %s"), result_chr);
  AddLog(LOG_LEVEL_DEBUG, PSTR("------ Hygrostat End ------"));
  AddLog(LOG_LEVEL_DEBUG, PSTR(""));
}
#endif // DEBUG_HYGROSTAT

uint8_t HygrostatGetDutyCycle(uint8_t ctr_output)
{
  uint8_t value = 0;
  if ( (Hygrostat[ctr_output].status.controller_mode == CTR_PI)
    || ((Hygrostat[ctr_output].status.controller_mode == CTR_HYBRID)
      &&(Hygrostat[ctr_output].status.phase_hybrid_ctr == CTR_HYBRID_PI))) {
    value = 100*Hygrostat[ctr_output].time_total_pi / ((uint32_t)60*(uint32_t)Hygrostat[ctr_output].time_pi_cycle);
  }
  else if ( (Hygrostat[ctr_output].status.controller_mode == CTR_RAMP_UP)
        || ((Hygrostat[ctr_output].status.controller_mode == CTR_HYBRID)
          &&(Hygrostat[ctr_output].status.phase_hybrid_ctr == CTR_HYBRID_RAMP_UP))) {
    if (Hygrostat[ctr_output].status.status_output == IFACE_ON) {
      value = 100;
    }
    else {
      value = 0;
    }
  }
  return value;
}

void HygrostatGetLocalSensor(uint8_t ctr_output) {
  String buf = ResponseData();   // copy the string into a new buffer that will be modified
  JsonParser parser((char*)buf.c_str());
  JsonParserObject root = parser.getRootObject();
  if (root) {
    String sensor_name = HYGROSTAT_SENSOR_NAME;
    const char* value_c;
    if (  (HYGROSTAT_SENSOR_NUMBER > 1)
        &&(HYGROSTAT_CONTROLLER_OUTPUTS > 1)
        &&(ctr_output < HYGROSTAT_SENSOR_NUMBER)) {
      sensor_name.concat("_" + (ctr_output + 1));
    }
    JsonParserToken value_token = root[sensor_name].getObject()[PSTR("Humidity")];
    if (value_token.isNum()) {
      int16_t value = value_token.getFloat() * 10;
      if ( (value >= -1000)
        && (value <= 2000)
        && (Hygrostat[ctr_output].status.sensor_type == SENSOR_LOCAL)) {
        uint32_t timestamp = TasmotaGlobal.uptime;
        // Calculate humidity gradient if humidity value has changed
        if (value != Hygrostat[ctr_output].hum_measured) {
          int32_t hum_delta = (value - Hygrostat[ctr_output].hum_measured); // in tenths of percents
          uint32_t time_delta = (timestamp - Hygrostat[ctr_output].timestamp_hum_meas_change_update); // in seconds
          Hygrostat[ctr_output].hum_measured_gradient = (int32_t)((360000 * hum_delta) / ((int32_t)time_delta)); // thousandths of percents per hour
          Hygrostat[ctr_output].hum_measured = value;
          Hygrostat[ctr_output].timestamp_hum_meas_change_update = timestamp;
        }
        Hygrostat[ctr_output].timestamp_hum_measured_update = timestamp;
        Hygrostat[ctr_output].status.sensor_alive = IFACE_ON;
      }
    }
  }
}

/*********************************************************************************************\
 * Commands
\*********************************************************************************************/

void CmndHygrostatModeSet(void)
{
  if ((XdrvMailbox.index > 0) && (XdrvMailbox.index <= HYGROSTAT_CONTROLLER_OUTPUTS)) {
    uint8_t ctr_output = XdrvMailbox.index - 1;
    if (XdrvMailbox.data_len > 0) {
      uint8_t value = (uint8_t)(CharToFloat(XdrvMailbox.data));
      if ((value >= HYGROSTAT_OFF) && (value < HYGROSTAT_MODES_MAX)) {
        Hygrostat[ctr_output].status.hygrostat_mode = value;
        Hygrostat[ctr_output].timestamp_input_on = 0;     // Reset last manual switch timer if command set externally
      }
      if ((value == HYGROSTAT_OFF) && (Hygrostat[ctr_output].status.enable_output == IFACE_ON)) {
        // Make sure the relay is switched to off once if the hygrostat is being disabled,
        // or it will get stuck on (danger!)
        Hygrostat[ctr_output].status.command_output = IFACE_OFF;
        HygrostatOutputRelay(ctr_output, Hygrostat[ctr_output].status.command_output);
      }
    }
    ResponseCmndIdxNumber((int)Hygrostat[ctr_output].status.hygrostat_mode);
  }
}

void CmndHumidifyingModeSet(void)
{
  if ((XdrvMailbox.index > 0) && (XdrvMailbox.index <= HYGROSTAT_CONTROLLER_OUTPUTS)) {
    uint8_t ctr_output = XdrvMailbox.index - 1;
    if (XdrvMailbox.data_len > 0) {
      uint8_t value = (uint8_t)(CharToFloat(XdrvMailbox.data));
      if ((value >= HUMIDIFYING) && (value < HUMIDIFYING_MODES_MAX)) {
        Hygrostat[ctr_output].status.humidifier_mode = value;
        // Trigger a restart of the controller
        Hygrostat[ctr_output].time_ctr_checkpoint = TasmotaGlobal.uptime;
      }
    }
    ResponseCmndIdxNumber((int)Hygrostat[ctr_output].status.humidifier_mode);
  }
}

void CmndHumToodryProtectSet(void)
{
  if ((XdrvMailbox.index > 0) && (XdrvMailbox.index <= HYGROSTAT_CONTROLLER_OUTPUTS)) {
    uint8_t ctr_output = XdrvMailbox.index - 1;
    int16_t value;
    if (XdrvMailbox.data_len > 0) {
      value = (int16_t)(CharToFloat(XdrvMailbox.data) * 10);
      if ( (value >= -1000)
        && (value <= 2000)) {
        Hygrostat[ctr_output].hum_toodry_protect = value;
      }
    }
    value = Hygrostat[ctr_output].hum_toodry_protect;
    ResponseCmndIdxFloat((float)value / 10, 1);
  }
}

void CmndControllerModeSet(void)
{
  if ((XdrvMailbox.index > 0) && (XdrvMailbox.index <= HYGROSTAT_CONTROLLER_OUTPUTS)) {
    uint8_t ctr_output = XdrvMailbox.index - 1;
    if (XdrvMailbox.data_len > 0) {
      uint8_t value = (uint8_t)(XdrvMailbox.payload);
      if ((value >= CTR_HYBRID) && (value < CTR_MODES_MAX)) {
        Hygrostat[ctr_output].status.controller_mode = value;
        // Reset controller variables
        Hygrostat[ctr_output].timestamp_rampup_start = TasmotaGlobal.uptime;
        Hygrostat[ctr_output].hum_rampup_start = Hygrostat[ctr_output].hum_measured;
        Hygrostat[ctr_output].hum_rampup_meas_gradient = 0;
        Hygrostat[ctr_output].time_rampup_deadtime = 0;
        Hygrostat[ctr_output].counter_rampup_cycles = 1;
        Hygrostat[ctr_output].time_ctr_changepoint = 0;
        Hygrostat[ctr_output].time_ctr_checkpoint = 0;
      }
    }
    ResponseCmndIdxNumber((int)Hygrostat[ctr_output].status.controller_mode);
  }
}

void CmndInputSwitchSet(void)
{
  if ((XdrvMailbox.index > 0) && (XdrvMailbox.index <= HYGROSTAT_CONTROLLER_OUTPUTS)) {
    uint8_t ctr_output = XdrvMailbox.index - 1;
    if (XdrvMailbox.data_len > 0) {
      uint8_t value = (uint8_t)(XdrvMailbox.payload);
      if (HygrostatSwitchIdValid(value)) {
        Hygrostat[ctr_output].status.input_switch_number = value;
        Hygrostat[ctr_output].timestamp_input_on = TasmotaGlobal.uptime;
      }
    }
    ResponseCmndIdxNumber((int)Hygrostat[ctr_output].status.input_switch_number);
  }
}

void CmndInputSwitchUse(void)
{
  if ((XdrvMailbox.index >= INPUT_NOT_USED) && (XdrvMailbox.index <= INPUT_USED)) {
    uint8_t ctr_output = XdrvMailbox.index - 1;
    if (XdrvMailbox.data_len > 0) {
      Hygrostat[ctr_output].status.use_input = (uint32_t)(XdrvMailbox.payload);
    }
    ResponseCmndIdxNumber((int)Hygrostat[ctr_output].status.use_input);
  }
}

void CmndSensorInputSet(void)
{
  if ((XdrvMailbox.index > 0) && (XdrvMailbox.index <= HYGROSTAT_CONTROLLER_OUTPUTS)) {
    uint8_t ctr_output = XdrvMailbox.index - 1;
    if (XdrvMailbox.data_len > 0) {
      uint8_t value = (uint8_t)(XdrvMailbox.payload);
      if ((value >= SENSOR_MQTT) && (value < SENSOR_MAX)) {
        Hygrostat[ctr_output].status.sensor_type = value;
      }
    }
    ResponseCmndIdxNumber((int)Hygrostat[ctr_output].status.sensor_type);
  }
}

void CmndOutputRelaySet(void)
{
  if ((XdrvMailbox.index > 0) && (XdrvMailbox.index <= HYGROSTAT_CONTROLLER_OUTPUTS)) {
    uint8_t ctr_output = XdrvMailbox.index - 1;
    if (XdrvMailbox.data_len > 0) {
      uint8_t value = (uint8_t)(XdrvMailbox.payload);
      if (HygrostatRelayIdValid(value)) {
        Hygrostat[ctr_output].status.output_relay_number = value;
      }
    }
    ResponseCmndIdxNumber((int)Hygrostat[ctr_output].status.output_relay_number);
  }
}

void CmndTimeAllowRampupSet(void)
{
  if ((XdrvMailbox.index > 0) && (XdrvMailbox.index <= HYGROSTAT_CONTROLLER_OUTPUTS)) {
    uint8_t ctr_output = XdrvMailbox.index - 1;
    if (XdrvMailbox.data_len > 0) {
      uint32_t value = (uint32_t)(XdrvMailbox.payload);
      if ((value >= 0) && (value < 1440)) {
        Hygrostat[ctr_output].time_allow_rampup = (uint16_t)value;
      }
    }
    ResponseCmndIdxNumber((int)((uint32_t)Hygrostat[ctr_output].time_allow_rampup));
  }
}

void CmndHumMeasuredSet(void)
{
  if ((XdrvMailbox.index > 0) && (XdrvMailbox.index <= HYGROSTAT_CONTROLLER_OUTPUTS)) {
    uint8_t ctr_output = XdrvMailbox.index - 1;
    int16_t value;
    if (XdrvMailbox.data_len > 0) {
        value = (int16_t)(CharToFloat(XdrvMailbox.data) * 10);
      if ( (value >= -1000)
        && (value <= 2000)
        && (Hygrostat[ctr_output].status.sensor_type == SENSOR_MQTT)) {
        uint32_t timestamp = TasmotaGlobal.uptime;
        // Calculate humidity gradient if humidity value has changed
        if (value != Hygrostat[ctr_output].hum_measured) {
          int32_t hum_delta = (value - Hygrostat[ctr_output].hum_measured); // in tenths of percents
          uint32_t time_delta = (timestamp - Hygrostat[ctr_output].timestamp_hum_meas_change_update); // in seconds
          Hygrostat[ctr_output].hum_measured_gradient = (int32_t)((360000 * hum_delta) / ((int32_t)time_delta)); // thousandths of percents per hour
          Hygrostat[ctr_output].hum_measured = value;
          Hygrostat[ctr_output].timestamp_hum_meas_change_update = timestamp;
        }
        Hygrostat[ctr_output].timestamp_hum_measured_update = timestamp;
        Hygrostat[ctr_output].status.sensor_alive = IFACE_ON;
      }
    }
    value = Hygrostat[ctr_output].hum_measured;
    ResponseCmndIdxFloat((float)value / 10, 1);
  }
}

void CmndHumTargetSet(void)
{
  if ((XdrvMailbox.index > 0) && (XdrvMailbox.index <= HYGROSTAT_CONTROLLER_OUTPUTS)) {
    uint8_t ctr_output = XdrvMailbox.index - 1;
    int16_t value;
    if (XdrvMailbox.data_len > 0) {
      value = (int16_t)(CharToFloat(XdrvMailbox.data) * 10);
      if ( (value >= -1000)
        && (value <= 2000)
        && (value >= Hygrostat[ctr_output].hum_toodry_protect)) {
        Hygrostat[ctr_output].hum_target_level = value;
      }
    }
    value = Hygrostat[ctr_output].hum_target_level;
    ResponseCmndIdxFloat((float)value / 10, 1);
  }
}

void CmndHumMeasuredGrdRead(void)
{
  if ((XdrvMailbox.index > 0) && (XdrvMailbox.index <= HYGROSTAT_CONTROLLER_OUTPUTS)) {
    uint8_t ctr_output = XdrvMailbox.index - 1;
    int16_t value;
    value = Hygrostat[ctr_output].hum_measured_gradient;
    ResponseCmndIdxFloat(((float)value) / 1000, 1);
  }
}

void CmndStateEmergencySet(void)
{
  if ((XdrvMailbox.index > 0) && (XdrvMailbox.index <= HYGROSTAT_CONTROLLER_OUTPUTS)) {
    uint8_t ctr_output = XdrvMailbox.index - 1;
    if (XdrvMailbox.data_len > 0) {
      uint8_t value = (uint8_t)(XdrvMailbox.payload);
      if ((value >= 0) && (value <= 1)) {
        Hygrostat[ctr_output].diag.state_emergency = (uint16_t)value;
      }
    }
    ResponseCmndIdxNumber((int)Hygrostat[ctr_output].diag.state_emergency);
  }
}

void CmndTimeManualToAutoSet(void)
{
  if ((XdrvMailbox.index > 0) && (XdrvMailbox.index <= HYGROSTAT_CONTROLLER_OUTPUTS)) {
    uint8_t ctr_output = XdrvMailbox.index - 1;
    if (XdrvMailbox.data_len > 0) {
      uint32_t value = (uint32_t)(XdrvMailbox.payload);
      if ((value >= 0) && (value <= 1440)) {
        Hygrostat[ctr_output].time_manual_to_auto = (uint16_t)value;
      }
    }
    ResponseCmndIdxNumber((int)((uint32_t)Hygrostat[ctr_output].time_manual_to_auto));
  }
}

void CmndPropBandSet(void)
{
  if ((XdrvMailbox.index > 0) && (XdrvMailbox.index <= HYGROSTAT_CONTROLLER_OUTPUTS)) {
    uint8_t ctr_output = XdrvMailbox.index - 1;
    if (XdrvMailbox.data_len > 0) {
      uint8_t value = (uint8_t)(XdrvMailbox.payload);
      if ((value >= 0) && (value <= 20)) {
        Hygrostat[ctr_output].val_prop_band = value;
      }
    }
    ResponseCmndIdxNumber((int)Hygrostat[ctr_output].val_prop_band);
  }
}

void CmndTimeResetSet(void)
{
  if ((XdrvMailbox.index > 0) && (XdrvMailbox.index <= HYGROSTAT_CONTROLLER_OUTPUTS)) {
    uint8_t ctr_output = XdrvMailbox.index - 1;
    if (XdrvMailbox.data_len > 0) {
      uint32_t value = (uint32_t)(XdrvMailbox.payload);
      if ((value >= 0) && (value <= 86400)) {
        Hygrostat[ctr_output].time_reset = value;
      }
    }
    ResponseCmndIdxNumber((int)Hygrostat[ctr_output].time_reset);
  }
}

void CmndTimePiProportRead(void)
{
  if ((XdrvMailbox.index > 0) && (XdrvMailbox.index <= HYGROSTAT_CONTROLLER_OUTPUTS)) {
    uint8_t ctr_output = XdrvMailbox.index - 1;
    ResponseCmndIdxNumber((int)Hygrostat[ctr_output].time_proportional_pi);
  }
}

void CmndTimePiIntegrRead(void)
{
  if ((XdrvMailbox.index > 0) && (XdrvMailbox.index <= HYGROSTAT_CONTROLLER_OUTPUTS)) {
    uint8_t ctr_output = XdrvMailbox.index - 1;
    ResponseCmndIdxNumber((int)Hygrostat[ctr_output].time_integral_pi);
  }
}

void CmndTimePiCycleSet(void)
{
  if ((XdrvMailbox.index > 0) && (XdrvMailbox.index <= HYGROSTAT_CONTROLLER_OUTPUTS)) {
    uint8_t ctr_output = XdrvMailbox.index - 1;
    if (XdrvMailbox.data_len > 0) {
      uint32_t value = (uint32_t)(XdrvMailbox.payload);
      if ((value >= 0) && (value <= 1440)) {
        Hygrostat[ctr_output].time_pi_cycle = (uint16_t)value;
      }
    }
    ResponseCmndIdxNumber((int)((uint32_t)Hygrostat[ctr_output].time_pi_cycle));
  }
}

void CmndHumAntiWindupResetSet(void)
{
  if ((XdrvMailbox.index > 0) && (XdrvMailbox.index <= HYGROSTAT_CONTROLLER_OUTPUTS)) {
    uint8_t ctr_output = XdrvMailbox.index - 1;
    uint8_t value;
    if (XdrvMailbox.data_len > 0) {
      value = (uint8_t)(CharToFloat(XdrvMailbox.data) * 10);
      if ( (value >= 0)
        && (value <= 100)) {
        Hygrostat[ctr_output].hum_reset_anti_windup = value;
      }
    }
    value = Hygrostat[ctr_output].hum_reset_anti_windup;
    ResponseCmndIdxFloat((float)value / 10, 1);
  }
}

void CmndHumHystSet(void)
{
  if ((XdrvMailbox.index > 0) && (XdrvMailbox.index <= HYGROSTAT_CONTROLLER_OUTPUTS)) {
    uint8_t ctr_output = XdrvMailbox.index - 1;
    int8_t value;
    if (XdrvMailbox.data_len > 0) {
      value = (int8_t)(CharToFloat(XdrvMailbox.data) * 10);
      if ( (value >= -100)
        && (value <= 100)) {
        Hygrostat[ctr_output].hum_hysteresis = value;
      }
    }
    value = Hygrostat[ctr_output].hum_hysteresis;
    ResponseCmndIdxFloat((float)value / 10, 1);
  }
}

#ifdef USE_PI_AUTOTUNING
void CmndPerfLevelAutotune(void)
{
  if ((XdrvMailbox.index > 0) && (XdrvMailbox.index <= HYGROSTAT_CONTROLLER_OUTPUTS)) {
    uint8_t ctr_output = XdrvMailbox.index - 1;
    if (XdrvMailbox.data_len > 0) {
      uint8_t value = (uint8_t)(XdrvMailbox.payload);
      if ((value >= 0) && (value <= AUTOTUNE_PERF_MAX)) {
        Hygrostat[ctr_output].status.autotune_perf_mode = value;
      }
    }
    ResponseCmndIdxNumber((int)Hygrostat[ctr_output].status.autotune_perf_mode);
  }
}
#endif // USE_PI_AUTOTUNING

void CmndTimeMaxActionSet(void)
{
  if ((XdrvMailbox.index > 0) && (XdrvMailbox.index <= HYGROSTAT_CONTROLLER_OUTPUTS)) {
    uint8_t ctr_output = XdrvMailbox.index - 1;
    if (XdrvMailbox.data_len > 0) {
      uint32_t value = (uint32_t)(XdrvMailbox.payload);
      if ((value >= 0) && (value <= 1440)) {
        Hygrostat[ctr_output].time_max_action = (uint16_t)value;
      }
    }
    ResponseCmndIdxNumber((int)((uint32_t)Hygrostat[ctr_output].time_max_action));
  }
}

void CmndTimeMinActionSet(void)
{
  if ((XdrvMailbox.index > 0) && (XdrvMailbox.index <= HYGROSTAT_CONTROLLER_OUTPUTS)) {
    uint8_t ctr_output = XdrvMailbox.index - 1;
    if (XdrvMailbox.data_len > 0) {
      uint32_t value = (uint32_t)(XdrvMailbox.payload);
      if ((value >= 0) && (value <= 1440)) {
        Hygrostat[ctr_output].time_min_action = (uint16_t)value;
      }
    }
    ResponseCmndIdxNumber((int)((uint32_t)Hygrostat[ctr_output].time_min_action));
  }
}

void CmndTimeSensLostSet(void)
{
  if ((XdrvMailbox.index > 0) && (XdrvMailbox.index <= HYGROSTAT_CONTROLLER_OUTPUTS)) {
    uint8_t ctr_output = XdrvMailbox.index - 1;
    if (XdrvMailbox.data_len > 0) {
      uint32_t value = (uint32_t)(XdrvMailbox.payload);
      if ((value >= 0) && (value <= 1440)) {
        Hygrostat[ctr_output].time_sens_lost = (uint16_t)value;
      }
    }
    ResponseCmndIdxNumber((int)((uint32_t)Hygrostat[ctr_output].time_sens_lost));
  }
}

void CmndTimeMinTurnoffActionSet(void)
{
  if ((XdrvMailbox.index > 0) && (XdrvMailbox.index <= HYGROSTAT_CONTROLLER_OUTPUTS)) {
    uint8_t ctr_output = XdrvMailbox.index - 1;
    if (XdrvMailbox.data_len > 0) {
      uint32_t value = (uint32_t)(XdrvMailbox.payload);
      if ((value >= 0) && (value <= 1440)) {
        Hygrostat[ctr_output].time_min_turnoff_action = (uint16_t)value;
      }
    }
    ResponseCmndIdxNumber((int)((uint32_t)Hygrostat[ctr_output].time_min_turnoff_action));
  }
}

void CmndHumRupDeltInSet(void)
{
  if ((XdrvMailbox.index > 0) && (XdrvMailbox.index <= HYGROSTAT_CONTROLLER_OUTPUTS)) {
    uint8_t ctr_output = XdrvMailbox.index - 1;
    uint8_t value;
    if (XdrvMailbox.data_len > 0) {
      value = (uint8_t)(CharToFloat(XdrvMailbox.data) * 10);
      if ( (value >= 0)
        && (value <= 100)) {
        Hygrostat[ctr_output].hum_rampup_delta_in = value;
      }
    }
    value = Hygrostat[ctr_output].hum_rampup_delta_in;
    ResponseCmndIdxFloat((float)value / 10, 1);
  }
}

void CmndHumRupDeltOutSet(void)
{
  if ((XdrvMailbox.index > 0) && (XdrvMailbox.index <= HYGROSTAT_CONTROLLER_OUTPUTS)) {
    uint8_t ctr_output = XdrvMailbox.index - 1;
    uint8_t value;
    if (XdrvMailbox.data_len > 0) {
      value = (uint8_t)(CharToFloat(XdrvMailbox.data) * 10);
      if ( (value >= 0)
        && (value <= 100)) {
        Hygrostat[ctr_output].hum_rampup_delta_out = value;
      }
    }
    value = Hygrostat[ctr_output].hum_rampup_delta_out;
    ResponseCmndIdxFloat((float)value / 10, 1);
  }
}

void CmndTimeRampupMaxSet(void)
{
  if ((XdrvMailbox.index > 0) && (XdrvMailbox.index <= HYGROSTAT_CONTROLLER_OUTPUTS)) {
    uint8_t ctr_output = XdrvMailbox.index - 1;
    if (XdrvMailbox.data_len > 0) {
      uint32_t value = (uint32_t)(XdrvMailbox.payload);
      if ((value >= 0) && (value <= 1440)) {
        Hygrostat[ctr_output].time_rampup_max = (uint16_t)value;
      }
    }
    ResponseCmndIdxNumber((int)((uint32_t)Hygrostat[ctr_output].time_rampup_max));
  }
}

void CmndTimeRampupCycleSet(void)
{
  if ((XdrvMailbox.index > 0) && (XdrvMailbox.index <= HYGROSTAT_CONTROLLER_OUTPUTS)) {
    uint8_t ctr_output = XdrvMailbox.index - 1;
    if (XdrvMailbox.data_len > 0) {
      uint32_t value = (uint32_t)(XdrvMailbox.payload);
      if ((value >= 0) && (value <= 1440)) {
        Hygrostat[ctr_output].time_rampup_cycle = (uint16_t)value;
      }
    }
    ResponseCmndIdxNumber((int)Hygrostat[ctr_output].time_rampup_cycle);
  }
}

void CmndHumRampupPiAccErrSet(void)
{
  if ((XdrvMailbox.index > 0) && (XdrvMailbox.index <= HYGROSTAT_CONTROLLER_OUTPUTS)) {
    uint8_t ctr_output = XdrvMailbox.index - 1;
    uint16_t value;
    if (XdrvMailbox.data_len > 0) {
      value = (uint16_t)(CharToFloat(XdrvMailbox.data) * 100);
      if ( (value >= 0)
        && (value <= 2500)) {
        Hygrostat[ctr_output].hum_rampup_pi_acc_error = value;
      }
    }
    value = Hygrostat[ctr_output].hum_rampup_pi_acc_error;
    ResponseCmndIdxFloat((float)value / 100, 1);
  }
}

void CmndDiagnosticModeSet(void)
{
  if ((XdrvMailbox.index > 0) && (XdrvMailbox.index <= HYGROSTAT_CONTROLLER_OUTPUTS)) {
    uint8_t ctr_output = XdrvMailbox.index - 1;
    if (XdrvMailbox.data_len > 0) {
      uint8_t value = (uint8_t)(CharToFloat(XdrvMailbox.data));
      if ((value >= DIAGNOSTIC_OFF) && (value <= DIAGNOSTIC_ON)) {
        Hygrostat[ctr_output].diag.diagnostic_mode = value;
      }
    }
    ResponseCmndIdxNumber((int)Hygrostat[ctr_output].diag.diagnostic_mode);
  }
}

void CmndCtrDutyCycleRead(void)
{
  if ((XdrvMailbox.index > 0) && (XdrvMailbox.index <= HYGROSTAT_CONTROLLER_OUTPUTS)) {
    uint8_t ctr_output = XdrvMailbox.index - 1;

        ResponseCmndIdxNumber((int)HygrostatGetDutyCycle(ctr_output) );
  }
}

void CmndEnableOutputSet(void)
{
  if ((XdrvMailbox.index > 0) && (XdrvMailbox.index <= HYGROSTAT_CONTROLLER_OUTPUTS)) {
    uint8_t ctr_output = XdrvMailbox.index - 1;
    if (XdrvMailbox.data_len > 0) {
      uint8_t value = (uint8_t)(CharToFloat(XdrvMailbox.data));
      if ((value >= IFACE_OFF) && (value <= IFACE_ON)) {
        Hygrostat[ctr_output].status.enable_output = value;
      }
    }
    ResponseCmndIdxNumber((int)Hygrostat[ctr_output].status.enable_output);
  }
}



/*********************************************************************************************\
 * Web UI
\*********************************************************************************************/


// To be done, add all of this defines in according languages file when all will be finished
// Avoid multiple changes on all language files during developement
// --------------------------------------------------
// xdrv_88_hygrostat.ino
#define D_HYGROSTAT             "Hygrostat"
#define D_HYGROSTAT_SET_POINT   "Set Point"
#define D_HYGROSTAT_SENSOR      "Current"
#define D_HYGROSTAT_GRADIENT    "Gradient"
#define D_HYGROSTAT_DUTY_CYCLE  "Duty cycle"
#define D_HYGROSTAT_CYCLE_TIME  "Cycle time"
#define D_HYGROSTAT_PI_AUTOTUNE "PI Auto tuning"
// --------------------------------------------------


#ifdef USE_WEBSERVER
const char HTTP_HYGROSTAT_INFO[]        PROGMEM = "{s}" D_HYGROSTAT "{m}%s{e}";
const char HTTP_HYGROSTAT_HUMIDITY[]    PROGMEM = "{s}%s " D_HUMIDITY "{m}%*_f " D_UNIT_PERCENT "%c{e}";
const char HTTP_HYGROSTAT_DUTY_CYCLE[]  PROGMEM = "{s}" D_HYGROSTAT_DUTY_CYCLE "{m}%d " D_UNIT_PERCENT "{e}";
const char HTTP_HYGROSTAT_CYCLE_TIME[]  PROGMEM = "{s}" D_HYGROSTAT_CYCLE_TIME "{m}%d " D_UNIT_MINUTE "{e}";
const char HTTP_HYGROSTAT_PI_AUTOTUNE[] PROGMEM = "{s}" D_HYGROSTAT_PI_AUTOTUNE "{m}%s{e}";
const char HTTP_HYGROSTAT_HL[]          PROGMEM = "{s}<hr>{m}<hr>{e}";

#endif  // USE_WEBSERVER

void HygrostatShow(uint8_t ctr_output, bool json)
{
  if (json) {
    float f_target_hum = Hygrostat[ctr_output].hum_target_level / 10.0f;
    ResponseAppend_P(PSTR(",\"Hygrostat%i\":{"), ctr_output);
    ResponseAppend_P(PSTR("%s\"%s\":%i"), "", D_CMND_HYGROSTATMODESET, Hygrostat[ctr_output].status.hygrostat_mode);
    ResponseAppend_P(PSTR("%s\"%s\":%2_f"), ",", D_CMND_HUMTARGETSET, &f_target_hum);
    ResponseAppend_P(PSTR("%s\"%s\":%i"), ",", D_CMND_CTRDUTYCYCLEREAD, HygrostatGetDutyCycle(ctr_output));
    ResponseJsonEnd();
    return;
  }
#ifdef USE_WEBSERVER

  WSContentSend_P(HTTP_HYGROSTAT_HL);

  if (Hygrostat[ctr_output].status.hygrostat_mode == HYGROSTAT_OFF) {
    WSContentSend_P(HTTP_HYGROSTAT_INFO, D_DISABLED );

  } else {
    char c_unit = '%';
    float f_humidity ;

    WSContentSend_P(HTTP_HYGROSTAT_INFO, D_ENABLED );

    f_humidity = Hygrostat[ctr_output].hum_target_level / 10.0f ;
    WSContentSend_PD(HTTP_HYGROSTAT_HUMIDITY, D_HYGROSTAT_SET_POINT, Settings->flag2.humidity_resolution, &f_humidity, c_unit);

    f_humidity = Hygrostat[ctr_output].hum_measured / 10.0f;
    WSContentSend_PD(HTTP_HYGROSTAT_HUMIDITY, D_HYGROSTAT_SENSOR, Settings->flag2.humidity_resolution, &f_humidity, c_unit);

    int16_t value = Hygrostat[ctr_output].hum_measured_gradient;
    f_humidity = value / 1000.0f;
    WSContentSend_PD(HTTP_HYGROSTAT_HUMIDITY, D_HYGROSTAT_GRADIENT, Settings->flag2.humidity_resolution, &f_humidity, c_unit);
    WSContentSend_P(HTTP_HYGROSTAT_DUTY_CYCLE, HygrostatGetDutyCycle(ctr_output) );
    WSContentSend_P(HTTP_HYGROSTAT_CYCLE_TIME, Hygrostat[ctr_output].time_pi_cycle );

  #ifdef USE_PI_AUTOTUNING
    WSContentSend_P(HTTP_HYGROSTAT_PI_AUTOTUNE, D_ENABLED  );
  #else
    WSContentSend_P(HTTP_HYGROSTAT_PI_AUTOTUNE, D_DISABLED );
  #endif

  }

#endif  // USE_WEBSERVER
}



/*********************************************************************************************\
 * Interface
\*********************************************************************************************/

bool Xdrv88(uint8_t function)
{
  bool result = false;
  uint8_t ctr_output;

  switch (function) {
    case FUNC_INIT:
      for (ctr_output = 0; ctr_output < HYGROSTAT_CONTROLLER_OUTPUTS; ctr_output++) {
        HygrostatInit(ctr_output);
      }
      break;
    case FUNC_LOOP:
      for (ctr_output = 0; ctr_output < HYGROSTAT_CONTROLLER_OUTPUTS; ctr_output++) {
        if (Hygrostat[ctr_output].status.hygrostat_mode != HYGROSTAT_OFF) {
          HygrostatSignalProcessingFast(ctr_output);
          HygrostatDiagnostics(ctr_output);
        }
      }
      break;
    case FUNC_SERIAL:
      break;
    case FUNC_EVERY_SECOND:
      for (ctr_output = 0; ctr_output < HYGROSTAT_CONTROLLER_OUTPUTS; ctr_output++) {
        if ((HygrostatMinuteCounter(ctr_output))
          && (Hygrostat[ctr_output].status.hygrostat_mode != HYGROSTAT_OFF)) {
          HygrostatSignalPreProcessingSlow(ctr_output);
          HygrostatController(ctr_output);
          HygrostatSignalPostProcessingSlow(ctr_output);
#ifdef DEBUG_HYGROSTAT
          HygrostatDebug(ctr_output);
#endif // DEBUG_HYGROSTAT
        }
      }
      break;
    case FUNC_SHOW_SENSOR:
      for (ctr_output = 0; ctr_output < HYGROSTAT_CONTROLLER_OUTPUTS; ctr_output++) {
        if (Hygrostat[ctr_output].status.hygrostat_mode != HYGROSTAT_OFF) {
          HygrostatGetLocalSensor(ctr_output);
        }
      }
      break;
    case FUNC_JSON_APPEND:
      for (ctr_output = 0; ctr_output < HYGROSTAT_CONTROLLER_OUTPUTS; ctr_output++) {
        HygrostatShow(ctr_output, true);
      }
      break;

#ifdef USE_WEBSERVER
    case FUNC_WEB_SENSOR:
      for (ctr_output = 0; ctr_output < HYGROSTAT_CONTROLLER_OUTPUTS; ctr_output++) {
        HygrostatShow(ctr_output, false);
      }
      break;
#endif  // USE_WEBSERVER

    case FUNC_COMMAND:
      result = DecodeCommand(kHygrostatCommands, HygrostatCommand);
      break;
  }
  return result;
}

#endif // USE_HYGROSTAT
