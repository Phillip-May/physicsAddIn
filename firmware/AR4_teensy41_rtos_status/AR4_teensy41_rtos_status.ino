#include <Arduino.h>
#define USE_ARDUINO_DEFINES
#include <arduino_freertos.h>
#include <queue.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <avr/eeprom.h>
#include <stddef.h>

#include "RobotMotionCore.h"
#include "SerialFraming.h"

namespace {

constexpr uint32_t kSerialBaud = 115200;
constexpr uint8_t kJointCount = 9;
constexpr uint8_t kAxisCount = 6;
constexpr uint8_t kQueueDepth = 8;
constexpr uint8_t kMotionQueueDepth = RobotMotionCore::kMotionLookaheadQueuedCommands;
constexpr size_t kMessageBytes = 1536;
constexpr uint32_t kDefaultStatusPeriodMs = 250;
constexpr uint32_t kMinimumStatusPeriodMs = 20;
constexpr uint32_t kDefaultLimitPollPeriodMs = 5;
constexpr uint32_t kMinimumLimitPollPeriodMs = 1;

constexpr uint8_t kLimitPins[kJointCount] = {
  26, 27, 28, 29, 30, 31, 36, 37, 38
};

constexpr uint8_t kLimitPinModes[kJointCount] = {
  INPUT_PULLUP, INPUT_PULLUP, INPUT_PULLUP, INPUT_PULLUP, INPUT_PULLUP, INPUT_PULLUP, INPUT_PULLUP, INPUT_PULLUP, INPUT_PULLUP
};

constexpr uint8_t kLimitActiveState[kJointCount] = {
  HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH
};

constexpr uint8_t kEstopPin = 39;
constexpr uint8_t kHeartbeatPin = LED_BUILTIN;
constexpr uint8_t kJogStepPins[kAxisCount] = {0, 2, 4, 6, 8, 10};
constexpr uint8_t kJogDirPins[kAxisCount] = {1, 3, 5, 7, 9, 11};
uint8_t g_jogDirInvert[kAxisCount] = {0, 1, 1, 0, 1, 1};
constexpr uint16_t kJogMaxSteps = 500;
constexpr uint16_t kJogStepGapMs = 10;
constexpr uint16_t kJogPulseLowUs = 30;
constexpr uint16_t kMasterBackoffStepsDefault = 700;
constexpr uint16_t kMasterBackoffStepsMin = 10;
constexpr uint16_t kMasterBackoffStepsMax = 5000;
constexpr uint16_t kMasterFastStepGapMs = 3;
constexpr uint16_t kMasterSlowStepGapMs = 12;
constexpr double kMasterReturnDegPerSec = 5.0;
constexpr double kMoveJDefaultSpeedDegPerSec = RobotMotionCore::kDefaultMoveJSpeedDegPerSec;
constexpr double kMoveJMaxSpeedDegPerSec = RobotMotionCore::kDefaultMoveJMaxSpeedDegPerSec;
constexpr uint32_t kMoveJMinTickGapUs = RobotMotionCore::kDefaultMoveJMinTickGapUs;
constexpr double kMoveJPlanSampleDeg = RobotMotionCore::kDefaultMoveJSampleDeg;
constexpr uint32_t kMoveJPlanMaxTicks = RobotMotionCore::kDefaultMoveJPlanMaxTicks;
// Initial limit-switch sweep direction. Runtime mastering may override it per robot.
int8_t g_masterDirection[kAxisCount] = {1, -1, 1, -1, -1, 1};

// Normalised to +1 or -1: zero would mean "do not move", which is never a valid sweep direction.
inline int32_t masterDirectionFor(uint8_t index) {
  return g_masterDirection[index] >= 0 ? 1 : -1;
}
constexpr double kAxisLimitPosDeg[kAxisCount] = {170.0, 90.0, 52.0, 180.0, 105.0, 180.0};
constexpr double kAxisLimitNegDeg[kAxisCount] = {170.0, 42.0, 89.0, 180.0, 105.0, 180.0};
constexpr double kDefaultStepsPerDegree[kAxisCount] = {88.888, 111.111, 111.111, 99.555, 43.720, 44.444};
constexpr double kCalibrationBaseOffsetDeg[kAxisCount] = {-6.2, 3.8, 1.4, -0.8, 5.6, 0.5};

struct SerialMessage {
  char text[kMessageBytes];
};

struct RobotStatus {
  uint32_t sequence;
  uint32_t millisNow;
  uint8_t estopActive;
  uint8_t limitsRaw[kJointCount];
  uint8_t limitsActive[kJointCount];
  uint32_t limitChangeCount[kJointCount];
  uint32_t limitLastChangeMs[kJointCount];
  uint32_t limitPollCount;
  uint32_t limitPollPeriodMs;
  uint32_t limitPollMeasuredHzX100;
  uint8_t jogArmed;
  uint8_t masteringActive;
  uint8_t masteringJoint;
  uint8_t mastered[kAxisCount];
  int32_t currentSteps[kAxisCount];
  int32_t zeroSteps[kAxisCount];
  int32_t masterLimitSteps[kAxisCount];
  int32_t positionDegX100[kAxisCount];
  long long positionDegX10000000[kAxisCount];
  int32_t masterOffsetDegX100[kAxisCount];
  int32_t masterOffsetDegX10000000[kAxisCount];
  int32_t stepsPerDegreeX1000[kAxisCount];
  long long stepsPerDegreeX10000000[kAxisCount];
};

QueueHandle_t g_commandQueue = nullptr;
QueueHandle_t g_motionQueue = nullptr;
QueueHandle_t g_motionExecutionQueue = nullptr;
QueueHandle_t g_txQueue = nullptr;

volatile bool g_streamStatus = true;
volatile uint32_t g_statusPeriodMs = kDefaultStatusPeriodMs;
volatile uint32_t g_limitPollPeriodMs = kDefaultLimitPollPeriodMs;
uint32_t g_statusSequence = 0;
uint32_t g_rxDropped = 0;
uint32_t g_txDropped = 0;
// Checksum failures reported in the status frame.
uint32_t g_rxBadChecksum = 0;
// Last accepted sequence number in the current host session. Duplicates are acknowledged, not run.
int32_t g_lastCommandSeq = -1;
uint32_t g_limitPollCount = 0;
uint32_t g_limitPollMeasuredHzX100 = 0;
uint8_t g_limitRaw[kJointCount] = {};
uint8_t g_limitActive[kJointCount] = {};
uint32_t g_limitChangeCount[kJointCount] = {};
uint32_t g_limitLastChangeMs[kJointCount] = {};
volatile bool g_jogArmed = false;
volatile bool g_stopRequested = false;
// Persistent stop signal for lower-priority jog and mastering tasks.
volatile bool g_abortRequested = false;
const char *g_configLoadResult = "not_stored";
// Whether the EEPROM holds a valid config, cached: the status task emits at up to 50 Hz and each
// check would otherwise re-scan flash sectors.
bool g_configStored = false;

constexpr uint32_t kConfigMagic = 0x34524143UL;  // "CAR4", little-endian

constexpr uint16_t kConfigVersion = 2;
constexpr size_t kConfigEepromAddress = 0;
const char *loadPersistentConfig();
void savePersistentConfig();
void clearPersistentConfig();
bool persistentConfigStored();

volatile bool g_masteringActive = false;
volatile uint8_t g_masteringJoint = 0;
volatile bool g_moveActive = false;
uint8_t g_mastered[kAxisCount] = {};
int32_t g_currentSteps[kAxisCount] = {};
int32_t g_zeroSteps[kAxisCount] = {};
int32_t g_masterLimitSteps[kAxisCount] = {};
double g_masterLimitDistanceDeg[kAxisCount] = {};
double g_masterOffsetDeg[kAxisCount] = {};
double g_stepsPerDegree[kAxisCount] = {
  kDefaultStepsPerDegree[0],
  kDefaultStepsPerDegree[1],
  kDefaultStepsPerDegree[2],
  kDefaultStepsPerDegree[3],
  kDefaultStepsPerDegree[4],
  kDefaultStepsPerDegree[5]
};
RobotMotionCore::RobotModel g_motionModel = {};
// Welding schedules, uploaded by the host. A movel carries only an index, and resolveWeaveParams
// turns it into parameters here using the same code the simulator used, so an index cannot come to
// mean two different welds. Lives in RAM like the robot model, so the host re-sends on connect.
RobotMotionCore::WeaveScheduleTable g_weaveSchedules = {};
RobotMotionCore::MotionProgramSettings g_motionSettings = {};
RobotMotionCore::MotionProgram g_motionProgramBuffer = {};
RobotMotionCore::MotionCommandRing g_motionCommandRing = {};
DMAMEM RobotMotionCore::MotionSegmentProgram g_motionSegmentProgramBuffer = {};
RobotMotionCore::MotionSegmentSampler g_motionSegmentSamplerBuffer = {};
// Requires 86,024 bytes in RAM1; available RAM2 cannot hold it with the segment program.
RobotMotionCore::MotionBaseSegmentProgram g_motionBaseSegmentProgramBuffer = {};
bool g_motionModelLoaded = false;
bool g_motionSettingsLoaded = false;
// State carried across lookahead windows. Kept in RAM1 because RAM2 holds the segment program.
RobotMotionCore::MotionWindowRunner g_motionWindowRunner = {};
// Triggers whose resolved time reached past the window that planned them. Carried in absolute run
// time and cleared wherever the other carried motion state is.
RobotMotionCore::PendingMotionTriggerQueue g_pendingTriggers = {};
volatile uint32_t g_motionExecutionGeneration = 1;

constexpr uint16_t kMotionExecutionQueueDepth = 1024;

struct MotionExecutionSample {
  uint32_t generation;
  uint32_t dueUs;
  int32_t targetSteps[kAxisCount];
  int32_t commandId;
  int32_t completedCommandId;
  uint8_t commandType;
  uint8_t finalSample;
  uint32_t sampleIndex;
  uint32_t profileSpeedMmSecX1000;
  int32_t triggerId;
};

void enqueueMoveLResult(const char *msg, const char *code, uint32_t sample, uint32_t ticks, int32_t id = -1);
void enqueueMoveLRunResult(const char *msg,
                           const char *code,
                           uint32_t sample,
                           uint32_t ticks,
                           int32_t id,
                           uint32_t elapsedMs,
                           uint32_t expectedMs,
                           uint32_t sampleDurationUs,
                           uint32_t sampleCount,
                           uint32_t lineMmX1000,
                           uint32_t speedMmSecX1000,
                           uint32_t zeroStepSamples,
                           bool blended,
                           uint32_t blendSamples,
                           uint32_t blendArcMmX1000,
                           const char *segmentKind,
                           uint32_t profileSpeedMmSecX1000,
                           uint32_t peakSpeedMmSecX1000,
                           uint32_t startSpeedMmSecX1000,
                           uint32_t endSpeedMmSecX1000,
                           uint32_t blendRadiusMmX1000,
                           uint32_t blendDeviationMmX1000,
                           uint32_t blendContourDeviationMmX1000,
                           uint32_t capReasonMask,
                           uint32_t commandSpeedMmSecX1000,
                           uint32_t finalSpeedMmSecX1000,
                           uint32_t capJointVelocityMmSecX1000,
                           uint32_t capJointAccelerationMmSecX1000,
                           uint32_t capSingularityMmSecX1000,
                           uint32_t capStepRateMmSecX1000,
                           uint32_t capJerkMmSecX1000,
                           const RobotMotionCore::StepExecutorStats *stepStats);
void enqueueMotionQueued(const char *msg, int32_t id);
void enqueueMotionLookaheadPlanned(uint8_t commandCount,
                                   uint8_t executeCount,
                                   int32_t firstId,
                                   int32_t lastExecuteId,
                                   uint32_t buildMs,
                                   uint32_t plannedDurationMs,
                                   uint8_t sampledVerification);

void enqueueTx(const char *text) {
  if (!text) {
    return;
  }

  SerialMessage message = {};
  strncpy(message.text, text, sizeof(message.text) - 1);
  if (g_txQueue == nullptr || xQueueSend(g_txQueue, &message, 0) != pdPASS) {
    g_txDropped++;
  }
}

uint32_t readMotionExecutionGeneration() {
  taskENTER_CRITICAL();
  const uint32_t generation = g_motionExecutionGeneration;
  taskEXIT_CRITICAL();
  return generation;
}

uint32_t bumpMotionExecutionGeneration() {
  taskENTER_CRITICAL();
  ++g_motionExecutionGeneration;
  if (g_motionExecutionGeneration == 0) {
    g_motionExecutionGeneration = 1;
  }
  const uint32_t generation = g_motionExecutionGeneration;
  taskEXIT_CRITICAL();
  return generation;
}

const char *jsonFieldValue(const SerialMessage &message, const char *key) {
  const size_t keyLen = strlen(key);
  const char *cursor = message.text;
  while ((cursor = strstr(cursor, key)) != nullptr) {
    const char *value = cursor + keyLen;
    while (*value == ' ' || *value == '\t' || *value == '\r' || *value == '\n') {
      value++;
    }
    if (*value == ':') {
      value++;
      while (*value == ' ' || *value == '\t' || *value == '\r' || *value == '\n') {
        value++;
      }
      return value;
    }
    cursor += keyLen;
  }
  return nullptr;
}

bool readStringField(const SerialMessage &message, const char *key, char *out, size_t outSize) {
  if (!out || outSize == 0) return false;
  out[0] = '\0';
  const char *cursor = jsonFieldValue(message, key);
  if (!cursor || *cursor != '"') return false;
  cursor++;
  size_t used = 0;
  while (*cursor != '\0' && *cursor != '"') {
    if (*cursor == '\\') return false;
    if (used + 1 >= outSize) return false;
    out[used++] = *cursor++;
  }
  if (*cursor != '"') return false;
  out[used] = '\0';
  return true;
}

bool commandIs(const SerialMessage &message, const char *expected) {
  char command[32] = {};
  return readStringField(message, "\"cmd\"", command, sizeof(command)) &&
         strcmp(command, expected) == 0;
}

bool jsonTokenEnded(const char *cursor) {
  return *cursor == '\0' || *cursor == ',' || *cursor == '}' || *cursor == ']' ||
         *cursor == ' ' || *cursor == '\t' || *cursor == '\r' || *cursor == '\n';
}

bool readBoolField(const SerialMessage &message, const char *key, bool fallback) {
  const char *value = jsonFieldValue(message, key);
  if (!value) return fallback;
  if (strncmp(value, "true", 4) == 0 && jsonTokenEnded(value + 4)) return true;
  if (strncmp(value, "false", 5) == 0 && jsonTokenEnded(value + 5)) return false;
  return fallback;
}

// Clears all state carried between planning windows. The next window seeds from the arm's joints.
void resetMotionRunState() {
  RobotMotionCore::clearMotionCommandRing(&g_motionCommandRing);
  RobotMotionCore::clearPendingMotionTriggers(&g_pendingTriggers);
  RobotMotionCore::beginMotionWindowRunner(g_motionModel,
                                           g_motionSettings,
                                           nullptr,
                                           &g_motionCommandRing,
                                           &g_motionProgramBuffer,
                                           &g_motionSegmentProgramBuffer,
                                           &g_motionSegmentSamplerBuffer,
                                           &g_motionBaseSegmentProgramBuffer,
                                           &g_motionWindowRunner);
}

void requestStop(bool acknowledge) {
  g_stopRequested = true;
  g_abortRequested = true;
  bumpMotionExecutionGeneration();
  g_masteringActive = false;
  g_masteringJoint = 0;
  g_moveActive = false;

  for (uint8_t i = 0; i < kAxisCount; ++i) {
    digitalWrite(kJogStepPins[i], HIGH);
  }

  if (g_commandQueue != nullptr) {
    xQueueReset(g_commandQueue);
  }
  if (g_motionQueue != nullptr) {
    xQueueReset(g_motionQueue);
  }
  if (g_motionExecutionQueue != nullptr) {
    xQueueReset(g_motionExecutionQueue);
  }
  resetMotionRunState();
  if (g_txQueue != nullptr) {
    xQueueReset(g_txQueue);
  }
  if (acknowledge) {
    enqueueTx(g_jogArmed ? "{\"msg\":\"stop_ack\",\"armed\":true}" : "{\"msg\":\"stop_ack\",\"armed\":false}");
  }
  g_stopRequested = false;
}

inline bool jogAbortPending() {
  return g_stopRequested || g_abortRequested;
}

void applyLogicalStepDelta(uint8_t joint, int32_t requestedDir, uint32_t completedSteps) {
  if (joint < 1 || joint > kAxisCount || completedSteps == 0) {
    return;
  }
  const uint8_t index = joint - 1;
  const int32_t signedSteps = requestedDir > 0
                                  ? static_cast<int32_t>(completedSteps)
                                  : -static_cast<int32_t>(completedSteps);
  taskENTER_CRITICAL();
  g_currentSteps[index] += signedSteps;
  taskEXIT_CRITICAL();
}

uint32_t readPeriodMs(const SerialMessage &message) {
  const char *valueText = jsonFieldValue(message, "\"period_ms\"");
  if (valueText == nullptr) return kDefaultStatusPeriodMs;
  const uint32_t value = static_cast<uint32_t>(strtoul(valueText, nullptr, 10));
  return value == 0 ? kDefaultStatusPeriodMs : value;
}

int32_t readIntField(const SerialMessage &message, const char *key, int32_t fallback) {
  const char *valueText = jsonFieldValue(message, key);
  return valueText ? static_cast<int32_t>(strtol(valueText, nullptr, 10)) : fallback;
}

double readFloatField(const SerialMessage &message, const char *key, double fallback) {
  const char *valueText = jsonFieldValue(message, key);
  return valueText ? strtod(valueText, nullptr) : fallback;
}

int32_t motionCommandId(const SerialMessage &message) {
  return readIntField(message, "\"id\"", -1);
}

bool readArrayValue(const SerialMessage &message, const char *key, uint8_t index, double *value) {
  const char *cursor = jsonFieldValue(message, key);
  if (cursor == nullptr || *cursor != '[') return false;
  cursor++;

  for (uint8_t i = 0; i <= index; ++i) {
    while (*cursor == ' ' || *cursor == '\t' || *cursor == ',') {
      cursor++;
    }
    if (*cursor == '\0' || *cursor == ']') {
      return false;
    }

    char *end = nullptr;
    const double parsed = strtod(cursor, &end);
    if (end == cursor) {
      return false;
    }
    if (i == index) {
      if (value) {
        *value = parsed;
      }
      return true;
    }
    cursor = end;
  }

  return false;
}

int32_t readArrayIntField(const SerialMessage &message, const char *key, uint8_t index, int32_t fallback) {
  double value = 0.0;
  if (!readArrayValue(message, key, index, &value)) {
    return fallback;
  }
  return static_cast<int32_t>(llround(value));
}

double readArrayDoubleField(const SerialMessage &message, const char *key, uint8_t index, double fallback) {
  double value = fallback;
  return readArrayValue(message, key, index, &value) ? value : fallback;
}

int32_t readSignedDirection(const SerialMessage &message) {
  const int32_t dir = readIntField(message, "\"dir\"", 0);
  if (dir < 0) {
    return -1;
  }
  if (dir > 0) {
    return 1;
  }
  return 0;
}

int32_t axisZeroStep(uint8_t index) {
  return g_zeroSteps[index];
}

int32_t axisStepLimit(uint8_t index) {
  return static_cast<int32_t>(llround((kAxisLimitPosDeg[index] + kAxisLimitNegDeg[index]) * g_stepsPerDegree[index]));
}

double axisPositionDeg(uint8_t index, int32_t steps) {
  return static_cast<double>(steps - axisZeroStep(index)) / g_stepsPerDegree[index];
}

double masterLimitPositionDeg(uint8_t index, double offsetDeg) {
  const int32_t masterDir = masterDirectionFor(index);
  return masterDir > 0
             ? (kAxisLimitPosDeg[index] + kCalibrationBaseOffsetDeg[index] + offsetDeg)
             : (-kAxisLimitNegDeg[index] + kCalibrationBaseOffsetDeg[index] + offsetDeg);
}

void updateMasterReference(uint8_t index, double limitPositionDeg) {
  const int32_t sign = limitPositionDeg >= 0.0 ? 1 : -1;
  const double distanceDeg = fabs(limitPositionDeg);
  g_masterLimitSteps[index] = g_currentSteps[index];
  g_masterLimitDistanceDeg[index] = distanceDeg;
  g_zeroSteps[index] = g_masterLimitSteps[index] - sign * static_cast<int32_t>(llround(distanceDeg * g_stepsPerDegree[index]));
}

int32_t axisPositionDegX100(uint8_t index, int32_t steps) {
  return static_cast<int32_t>(llround(axisPositionDeg(index, steps) * 100.0));
}

uint16_t stepGapMsForJointSpeed(uint8_t index, double degPerSec) {
  if (index >= kAxisCount || degPerSec <= 0.0f) {
    return kJogStepGapMs;
  }

  const double stepsPerSecond = g_stepsPerDegree[index] * degPerSec;
  if (stepsPerSecond <= 0.0f) {
    return kJogStepGapMs;
  }

  const uint32_t gapMs = static_cast<uint32_t>(llround(1000.0 / stepsPerSecond));
  return static_cast<uint16_t>(gapMs < 1 ? 1 : gapMs);
}

bool limitIsActive(uint8_t joint) {
  if (joint < 1 || joint > kJointCount) {
    return false;
  }
  taskENTER_CRITICAL();
  const bool active = g_limitActive[joint - 1] == 1;
  taskEXIT_CRITICAL();
  return active;
}

RobotStatus readRobotStatus() {
  RobotStatus status = {};
  status.sequence = g_statusSequence++;
  status.millisNow = millis();
  status.estopActive = digitalRead(kEstopPin) == LOW ? 1 : 0;

  taskENTER_CRITICAL();
  for (uint8_t i = 0; i < kJointCount; ++i) {
    status.limitsRaw[i] = g_limitRaw[i];
    status.limitsActive[i] = g_limitActive[i];
    status.limitChangeCount[i] = g_limitChangeCount[i];
    status.limitLastChangeMs[i] = g_limitLastChangeMs[i];
  }
  status.limitPollCount = g_limitPollCount;
  status.limitPollPeriodMs = g_limitPollPeriodMs;
  status.limitPollMeasuredHzX100 = g_limitPollMeasuredHzX100;
  status.jogArmed = g_jogArmed ? 1 : 0;
  status.masteringActive = g_masteringActive ? 1 : 0;
  status.masteringJoint = g_masteringJoint;
  for (uint8_t i = 0; i < kAxisCount; ++i) {
    status.mastered[i] = g_mastered[i];
    status.currentSteps[i] = g_currentSteps[i];
    status.zeroSteps[i] = g_zeroSteps[i];
    status.masterLimitSteps[i] = g_masterLimitSteps[i];
    status.positionDegX100[i] = axisPositionDegX100(i, g_currentSteps[i]);
    status.positionDegX10000000[i] = static_cast<long long>(llround(axisPositionDeg(i, g_currentSteps[i]) * 10000000.0));
    status.masterOffsetDegX100[i] = static_cast<int32_t>(llround(g_masterOffsetDeg[i] * 100.0));
    status.masterOffsetDegX10000000[i] = static_cast<int32_t>(llround(g_masterOffsetDeg[i] * 10000000.0));
    status.stepsPerDegreeX1000[i] = static_cast<int32_t>(llround(g_stepsPerDegree[i] * 1000.0));
    status.stepsPerDegreeX10000000[i] = static_cast<long long>(llround(g_stepsPerDegree[i] * 10000000.0));
  }
  taskEXIT_CRITICAL();

  return status;
}

void enqueueStatus() {
  const RobotStatus status = readRobotStatus();

  SerialMessage message = {};
  const int written = snprintf(
      message.text,
      sizeof(message.text),
      "{\"msg\":\"robot_status\",\"seq\":%lu,\"ms\":%lu,\"estop\":%u,"
      "\"limits_raw\":[%u,%u,%u,%u,%u,%u,%u,%u,%u],"
      "\"limits_active\":[%u,%u,%u,%u,%u,%u,%u,%u,%u],"
      "\"limit_active_state\":[%u,%u,%u,%u,%u,%u,%u,%u,%u],"
      "\"limit_changes\":[%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu],"
      "\"limit_last_change_ms\":[%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu],"
      "\"jog\":{\"armed\":%u,\"allowed_joints\":[1,2,3,4,5,6],\"max_steps\":%u},"
      "\"mastering\":{\"active\":%u,\"active_joint\":%u,"
      "\"mastered\":[%u,%u,%u,%u,%u,%u],"
      "\"master_dir\":[%d,%d,%d,%d,%d,%d],"
      "\"jog_dir_invert\":[%u,%u,%u,%u,%u,%u],"
      "\"config_stored\":%u,"
      "\"current_steps\":[%ld,%ld,%ld,%ld,%ld,%ld],"
      "\"zero_steps\":[%ld,%ld,%ld,%ld,%ld,%ld],"
      "\"master_limit_steps\":[%ld,%ld,%ld,%ld,%ld,%ld],"
      "\"position_deg_x100\":[%ld,%ld,%ld,%ld,%ld,%ld],"
      "\"position_deg_x10000000\":[%lld,%lld,%lld,%lld,%lld,%lld],"
      "\"offset_deg_x100\":[%ld,%ld,%ld,%ld,%ld,%ld],"
      "\"offset_deg_x10000000\":[%ld,%ld,%ld,%ld,%ld,%ld],"
      "\"steps_per_deg_x1000\":[%ld,%ld,%ld,%ld,%ld,%ld],"
      "\"steps_per_deg_x10000000\":[%lld,%lld,%lld,%lld,%lld,%lld]},"
      "\"limit_poll\":{\"period_ms\":%lu,\"count\":%lu,\"measured_hz_x100\":%lu},"
      "\"queue\":{\"rx\":%lu,\"tx\":%lu,\"rx_dropped\":%lu,\"tx_dropped\":%lu,\"rx_bad_checksum\":%lu}}",
      static_cast<unsigned long>(status.sequence),
      static_cast<unsigned long>(status.millisNow),
      status.estopActive,
      status.limitsRaw[0], status.limitsRaw[1], status.limitsRaw[2],
      status.limitsRaw[3], status.limitsRaw[4], status.limitsRaw[5],
      status.limitsRaw[6], status.limitsRaw[7], status.limitsRaw[8],
      status.limitsActive[0], status.limitsActive[1], status.limitsActive[2],
      status.limitsActive[3], status.limitsActive[4], status.limitsActive[5],
      status.limitsActive[6], status.limitsActive[7], status.limitsActive[8],
      kLimitActiveState[0] == HIGH ? 1 : 0,
      kLimitActiveState[1] == HIGH ? 1 : 0,
      kLimitActiveState[2] == HIGH ? 1 : 0,
      kLimitActiveState[3] == HIGH ? 1 : 0,
      kLimitActiveState[4] == HIGH ? 1 : 0,
      kLimitActiveState[5] == HIGH ? 1 : 0,
      kLimitActiveState[6] == HIGH ? 1 : 0,
      kLimitActiveState[7] == HIGH ? 1 : 0,
      kLimitActiveState[8] == HIGH ? 1 : 0,
      static_cast<unsigned long>(status.limitChangeCount[0]),
      static_cast<unsigned long>(status.limitChangeCount[1]),
      static_cast<unsigned long>(status.limitChangeCount[2]),
      static_cast<unsigned long>(status.limitChangeCount[3]),
      static_cast<unsigned long>(status.limitChangeCount[4]),
      static_cast<unsigned long>(status.limitChangeCount[5]),
      static_cast<unsigned long>(status.limitChangeCount[6]),
      static_cast<unsigned long>(status.limitChangeCount[7]),
      static_cast<unsigned long>(status.limitChangeCount[8]),
      static_cast<unsigned long>(status.limitLastChangeMs[0]),
      static_cast<unsigned long>(status.limitLastChangeMs[1]),
      static_cast<unsigned long>(status.limitLastChangeMs[2]),
      static_cast<unsigned long>(status.limitLastChangeMs[3]),
      static_cast<unsigned long>(status.limitLastChangeMs[4]),
      static_cast<unsigned long>(status.limitLastChangeMs[5]),
      static_cast<unsigned long>(status.limitLastChangeMs[6]),
      static_cast<unsigned long>(status.limitLastChangeMs[7]),
      static_cast<unsigned long>(status.limitLastChangeMs[8]),
      status.jogArmed,
      kJogMaxSteps,
      status.masteringActive,
      status.masteringJoint,
      status.mastered[0], status.mastered[1], status.mastered[2],
      status.mastered[3], status.mastered[4], status.mastered[5],
      g_masterDirection[0], g_masterDirection[1], g_masterDirection[2],
      g_masterDirection[3], g_masterDirection[4], g_masterDirection[5],
      g_jogDirInvert[0], g_jogDirInvert[1], g_jogDirInvert[2],
      g_jogDirInvert[3], g_jogDirInvert[4], g_jogDirInvert[5],
      g_configStored ? 1u : 0u,
      static_cast<long>(status.currentSteps[0]),
      static_cast<long>(status.currentSteps[1]),
      static_cast<long>(status.currentSteps[2]),
      static_cast<long>(status.currentSteps[3]),
      static_cast<long>(status.currentSteps[4]),
      static_cast<long>(status.currentSteps[5]),
      static_cast<long>(status.zeroSteps[0]),
      static_cast<long>(status.zeroSteps[1]),
      static_cast<long>(status.zeroSteps[2]),
      static_cast<long>(status.zeroSteps[3]),
      static_cast<long>(status.zeroSteps[4]),
      static_cast<long>(status.zeroSteps[5]),
      static_cast<long>(status.masterLimitSteps[0]),
      static_cast<long>(status.masterLimitSteps[1]),
      static_cast<long>(status.masterLimitSteps[2]),
      static_cast<long>(status.masterLimitSteps[3]),
      static_cast<long>(status.masterLimitSteps[4]),
      static_cast<long>(status.masterLimitSteps[5]),
      static_cast<long>(status.positionDegX100[0]),
      static_cast<long>(status.positionDegX100[1]),
      static_cast<long>(status.positionDegX100[2]),
      static_cast<long>(status.positionDegX100[3]),
      static_cast<long>(status.positionDegX100[4]),
      static_cast<long>(status.positionDegX100[5]),
      status.positionDegX10000000[0],
      status.positionDegX10000000[1],
      status.positionDegX10000000[2],
      status.positionDegX10000000[3],
      status.positionDegX10000000[4],
      status.positionDegX10000000[5],
      static_cast<long>(status.masterOffsetDegX100[0]),
      static_cast<long>(status.masterOffsetDegX100[1]),
      static_cast<long>(status.masterOffsetDegX100[2]),
      static_cast<long>(status.masterOffsetDegX100[3]),
      static_cast<long>(status.masterOffsetDegX100[4]),
      static_cast<long>(status.masterOffsetDegX100[5]),
      static_cast<long>(status.masterOffsetDegX10000000[0]),
      static_cast<long>(status.masterOffsetDegX10000000[1]),
      static_cast<long>(status.masterOffsetDegX10000000[2]),
      static_cast<long>(status.masterOffsetDegX10000000[3]),
      static_cast<long>(status.masterOffsetDegX10000000[4]),
      static_cast<long>(status.masterOffsetDegX10000000[5]),
      static_cast<long>(status.stepsPerDegreeX1000[0]),
      static_cast<long>(status.stepsPerDegreeX1000[1]),
      static_cast<long>(status.stepsPerDegreeX1000[2]),
      static_cast<long>(status.stepsPerDegreeX1000[3]),
      static_cast<long>(status.stepsPerDegreeX1000[4]),
      static_cast<long>(status.stepsPerDegreeX1000[5]),
      status.stepsPerDegreeX10000000[0],
      status.stepsPerDegreeX10000000[1],
      status.stepsPerDegreeX10000000[2],
      status.stepsPerDegreeX10000000[3],
      status.stepsPerDegreeX10000000[4],
      status.stepsPerDegreeX10000000[5],
      static_cast<unsigned long>(status.limitPollPeriodMs),
      static_cast<unsigned long>(status.limitPollCount),
      static_cast<unsigned long>(status.limitPollMeasuredHzX100),
      static_cast<unsigned long>(uxQueueMessagesWaiting(g_commandQueue)),
      static_cast<unsigned long>(uxQueueMessagesWaiting(g_txQueue)),
      static_cast<unsigned long>(g_rxDropped),
      static_cast<unsigned long>(g_txDropped),
      static_cast<unsigned long>(g_rxBadChecksum));

  if (written <= 0 || written >= static_cast<int>(sizeof(message.text)) ||
      xQueueSend(g_txQueue, &message, 0) != pdPASS) {
    g_txDropped++;
  }
}

void enqueueMotionStatusIfDue() {
  if (!g_streamStatus) {
    return;
  }

  static uint32_t lastMotionStatusMs = 0;
  const uint32_t now = millis();
  const uint32_t periodMs = g_statusPeriodMs < kMinimumStatusPeriodMs ? kMinimumStatusPeriodMs : g_statusPeriodMs;
  if (lastMotionStatusMs != 0 && now - lastMotionStatusMs < periodMs) {
    return;
  }

  lastMotionStatusMs = now;
  enqueueStatus();
}

void enqueueJogRejected(const char *code) {
  SerialMessage response = {};
  snprintf(response.text,
           sizeof(response.text),
           "{\"msg\":\"jog_rejected\",\"code\":\"%s\"}",
           code ? code : "unknown");
  if (xQueueSend(g_txQueue, &response, 0) != pdPASS) {
    g_txDropped++;
  }
}

uint32_t pulseJointStepsAtGap(uint8_t joint, uint32_t steps, int32_t requestedDir, uint16_t gapMs) {
  if (joint < 1 || joint > kAxisCount || steps == 0 || requestedDir == 0) {
    return 0;
  }

  const uint8_t index = static_cast<uint8_t>(joint - 1);
  const bool positiveDirection = requestedDir > 0;
  digitalWrite(kJogDirPins[index], positiveDirection != (g_jogDirInvert[index] != 0) ? HIGH : LOW);
  delayMicroseconds(10);

  uint32_t completed = 0;
  for (uint32_t step = 0; step < steps; ++step) {
    if (jogAbortPending()) {
      break;
    }
    digitalWrite(kJogStepPins[index], LOW);
    delayMicroseconds(kJogPulseLowUs);
    digitalWrite(kJogStepPins[index], HIGH);
    completed++;
    applyLogicalStepDelta(joint, requestedDir, 1);
    vTaskDelay(pdMS_TO_TICKS(gapMs));
  }
  digitalWrite(kJogStepPins[index], HIGH);
  return completed;
}

uint16_t pulseJointSteps(uint8_t joint, uint16_t steps, int32_t requestedDir) {
  return static_cast<uint16_t>(pulseJointStepsAtGap(joint, steps, requestedDir, kJogStepGapMs));
}

void handleJogJoint(const SerialMessage &message) {
  if (!g_jogArmed) {
    enqueueJogRejected("not_armed");
    return;
  }

  const int32_t joint = readIntField(message, "\"joint\"", 0);
  const int32_t stepsValue = readIntField(message, "\"steps\"", 0);
  if (joint < 1 || joint > kAxisCount) {
    enqueueJogRejected("joint_not_allowed");
    return;
  }
  if (stepsValue <= 0 || stepsValue > kJogMaxSteps) {
    enqueueJogRejected("bad_steps");
    return;
  }

  const int32_t dir = readSignedDirection(message);
  if (dir == 0) {
    enqueueJogRejected("bad_direction");
    return;
  }

  g_abortRequested = false;

  const uint16_t completedSteps = pulseJointSteps(static_cast<uint8_t>(joint), static_cast<uint16_t>(stepsValue), dir);

  SerialMessage response = {};
  if (completedSteps < static_cast<uint16_t>(stepsValue) || jogAbortPending()) {
    snprintf(response.text,
             sizeof(response.text),
             "{\"msg\":\"jog_stopped\",\"joint\":%ld,\"steps\":%u,\"requested_steps\":%ld,\"dir\":%ld}",
             static_cast<long>(joint),
             completedSteps,
             static_cast<long>(stepsValue),
             static_cast<long>(dir));
  } else {
    snprintf(response.text,
             sizeof(response.text),
             "{\"msg\":\"jog_done\",\"joint\":%ld,\"steps\":%ld,\"dir\":%ld}",
             static_cast<long>(joint),
             static_cast<long>(stepsValue),
             static_cast<long>(dir));
  }
  if (xQueueSend(g_txQueue, &response, 0) != pdPASS) {
    g_txDropped++;
  }
}

void enqueueMoveJResult(const char *msg, const char *code, uint32_t ticks, int32_t id = -1) {
  SerialMessage response = {};
  if (id >= 0) {
    snprintf(response.text,
             sizeof(response.text),
             "{\"msg\":\"%s\",\"id\":%ld,\"code\":\"%s\",\"ticks\":%lu}",
             msg ? msg : "movej_rejected",
             static_cast<long>(id),
             code ? code : "ok",
             static_cast<unsigned long>(ticks));
  } else {
    snprintf(response.text,
             sizeof(response.text),
             "{\"msg\":\"%s\",\"code\":\"%s\",\"ticks\":%lu}",
             msg ? msg : "movej_rejected",
             code ? code : "ok",
             static_cast<unsigned long>(ticks));
  }
  if (xQueueSend(g_txQueue, &response, 0) != pdPASS) {
    g_txDropped++;
  }
}

void enqueueMoveJPlanResult(const char *msg, const char *code, const RobotMotionCore::MoveJPlan *plan, int32_t id = -1) {
  SerialMessage response = {};
  if (plan) {
    if (id >= 0) {
      snprintf(response.text,
               sizeof(response.text),
               "{\"msg\":\"%s\",\"id\":%ld,\"code\":\"%s\",\"ticks\":%lu,\"tick_gap_us\":%lu,\"duration_ms\":%lu}",
               msg ? msg : "movej_plan",
               static_cast<long>(id),
               code ? code : "ok",
               static_cast<unsigned long>(plan->maxStepCount),
               static_cast<unsigned long>(plan->tickGapUs),
               static_cast<unsigned long>(llround(plan->durationSec * 1000.0)));
    } else {
      snprintf(response.text,
               sizeof(response.text),
               "{\"msg\":\"%s\",\"code\":\"%s\",\"ticks\":%lu,\"tick_gap_us\":%lu,\"duration_ms\":%lu}",
               msg ? msg : "movej_plan",
               code ? code : "ok",
               static_cast<unsigned long>(plan->maxStepCount),
               static_cast<unsigned long>(plan->tickGapUs),
               static_cast<unsigned long>(llround(plan->durationSec * 1000.0)));
    }
  } else {
    if (id >= 0) {
      snprintf(response.text,
               sizeof(response.text),
               "{\"msg\":\"%s\",\"id\":%ld,\"code\":\"%s\",\"ticks\":0}",
               msg ? msg : "movej_rejected",
               static_cast<long>(id),
               code ? code : "unknown");
    } else {
      snprintf(response.text,
               sizeof(response.text),
               "{\"msg\":\"%s\",\"code\":\"%s\",\"ticks\":0}",
               msg ? msg : "movej_rejected",
               code ? code : "unknown");
    }
  }
  if (xQueueSend(g_txQueue, &response, 0) != pdPASS) {
    g_txDropped++;
  }
}

bool buildMoveJInput(const SerialMessage &message, RobotMotionCore::MoveJInput *input, const char **rejectCode) {
  if (!input) {
    if (rejectCode) *rejectCode = "bad_target";
    return false;
  }
  memset(input, 0, sizeof(*input));
  if (rejectCode) {
    *rejectCode = "ok";
  }
  for (uint8_t i = 0; i < kAxisCount; ++i) {
    input->limits.min[i] = -kAxisLimitNegDeg[i];
    input->limits.max[i] = kAxisLimitPosDeg[i];
    input->limitDirection[i] = masterDirectionFor(i);
    input->limitActive[i] = limitIsActive(static_cast<uint8_t>(i + 1)) ? 1 : 0;
  }
  RobotMotionCore::applyDefaultMoveJPlannerSettings(input);
  input->speedDegPerSec = readFloatField(message, "\"speed_deg_s\"", input->defaultSpeedDegPerSec);
  input->estopActive = digitalRead(kEstopPin) == LOW ? 1 : 0;

  for (uint8_t i = 0; i < kAxisCount; ++i) {
    if (!readArrayValue(message, "\"target_deg\"", i, &input->targetDeg[i])) {
      if (rejectCode) *rejectCode = "bad_target";
      return false;
    }
  }

  taskENTER_CRITICAL();
  for (uint8_t i = 0; i < kAxisCount; ++i) {
    input->mastered[i] = g_mastered[i];
    input->currentSteps[i] = g_currentSteps[i];
    input->zeroSteps[i] = g_zeroSteps[i];
    input->stepsPerDegree[i] = g_stepsPerDegree[i];
  }
  taskEXIT_CRITICAL();

  return true;
}

void currentJointRadians(double q[kAxisCount]) {
  int32_t currentSteps[kAxisCount] = {};
  int32_t zeroSteps[kAxisCount] = {};
  double stepsPerDegree[kAxisCount] = {};
  taskENTER_CRITICAL();
  for (uint8_t i = 0; i < kAxisCount; ++i) {
    currentSteps[i] = g_currentSteps[i];
    zeroSteps[i] = g_zeroSteps[i];
    stepsPerDegree[i] = g_stepsPerDegree[i];
  }
  taskEXIT_CRITICAL();

  constexpr double kDegToRadLocal = 3.14159265358979323846 / 180.0;
  for (uint8_t i = 0; i < kAxisCount; ++i) {
    q[i] = stepsPerDegree[i] > 0.0
               ? (static_cast<double>(currentSteps[i] - zeroSteps[i]) / stepsPerDegree[i]) * kDegToRadLocal
               : 0.0;
  }
}

bool allAxesMastered() {
  taskENTER_CRITICAL();
  bool mastered = true;
  for (uint8_t i = 0; i < kAxisCount; ++i) {
    mastered = mastered && g_mastered[i] != 0;
  }
  taskEXIT_CRITICAL();
  return mastered;
}

bool hasMoveJTarget(const SerialMessage &message) {
  double value = 0.0;
  for (uint8_t i = 0; i < kAxisCount; ++i) {
    if (!readArrayValue(message, "\"target_deg\"", i, &value)) {
      return false;
    }
  }
  return true;
}

bool stepBatchedTowardDesired(const int32_t desiredSteps[kAxisCount],
                              int32_t actualSteps[kAxisCount],
                              uint32_t lastStepUs[kAxisCount],
                              uint32_t *totalTicks,
                              const char **abortCode) {
  bool shouldStep[kAxisCount] = {};
  int32_t directions[kAxisCount] = {};
  uint8_t stepCount = 0;
  const uint32_t nowUs = micros();

  for (uint8_t i = 0; i < kAxisCount; ++i) {
    const int32_t delta = desiredSteps[i] - actualSteps[i];
    if (delta == 0) continue;
    if (static_cast<uint32_t>(nowUs - lastStepUs[i]) < kMoveJMinTickGapUs) continue;
    const int32_t direction = delta > 0 ? 1 : -1;
    const int32_t limitDirection = masterDirectionFor(i);
    if (direction == limitDirection && limitIsActive(static_cast<uint8_t>(i + 1))) {
      g_stopRequested = true;
      bumpMotionExecutionGeneration();
      if (abortCode) *abortCode = "limit_switch";
      return false;
    }
    shouldStep[i] = true;
    directions[i] = direction;
    ++stepCount;
  }

  if (stepCount == 0) return false;

  for (uint8_t i = 0; i < kAxisCount; ++i) {
    if (!shouldStep[i]) continue;
    const bool positiveDirection = directions[i] > 0;
    digitalWrite(kJogDirPins[i], positiveDirection != (g_jogDirInvert[i] != 0) ? HIGH : LOW);
  }
  delayMicroseconds(10);

  for (uint8_t i = 0; i < kAxisCount; ++i) {
    if (shouldStep[i]) digitalWrite(kJogStepPins[i], LOW);
  }
  delayMicroseconds(kJogPulseLowUs);

  const uint32_t completedUs = micros();
  for (uint8_t i = 0; i < kAxisCount; ++i) {
    if (!shouldStep[i]) continue;
    digitalWrite(kJogStepPins[i], HIGH);
    actualSteps[i] += directions[i];
    lastStepUs[i] = completedUs;
    if (totalTicks) ++(*totalTicks);
  }

  taskENTER_CRITICAL();
  for (uint8_t i = 0; i < kAxisCount; ++i) {
    if (shouldStep[i]) g_currentSteps[i] += directions[i];
  }
  taskEXIT_CRITICAL();
  return true;
}

bool stepTargetsReached(const int32_t actualSteps[kAxisCount], const int32_t targetSteps[kAxisCount]) {
  for (uint8_t i = 0; i < kAxisCount; ++i) {
    if (actualSteps[i] != targetSteps[i]) return false;
  }
  return true;
}

bool followQueuedStepTarget(const int32_t startSteps[kAxisCount],
                            const int32_t targetSteps[kAxisCount],
                            int32_t actualSteps[kAxisCount],
                            uint32_t lastStepUs[kAxisCount],
                            uint32_t startUs,
                            uint32_t deadlineUs,
                            uint32_t generation,
                            uint32_t *totalTicks,
                            const char **abortCode) {
  const uint32_t spanUs = static_cast<uint32_t>(deadlineUs - startUs);
  while (!g_stopRequested) {
    if (readMotionExecutionGeneration() != generation) {
      if (abortCode) *abortCode = "stale_generation";
      return false;
    }
    const uint32_t nowUs = micros();
    const bool pastDeadline = static_cast<int32_t>(nowUs - deadlineUs) >= 0;
    double alpha = 1.0;
    if (!pastDeadline && spanUs > 0) {
      alpha = static_cast<double>(static_cast<uint32_t>(nowUs - startUs)) / static_cast<double>(spanUs);
      if (alpha < 0.0) alpha = 0.0;
      if (alpha > 1.0) alpha = 1.0;
    }

    int32_t desiredSteps[kAxisCount] = {};
    for (uint8_t i = 0; i < kAxisCount; ++i) {
      desiredSteps[i] = startSteps[i] + static_cast<int32_t>(llround(static_cast<double>(targetSteps[i] - startSteps[i]) * alpha));
    }
    if (pastDeadline) {
      for (uint8_t i = 0; i < kAxisCount; ++i) desiredSteps[i] = targetSteps[i];
    }

    if (pastDeadline && stepTargetsReached(actualSteps, targetSteps)) return true;

    const bool stepped = stepBatchedTowardDesired(desiredSteps, actualSteps, lastStepUs, totalTicks, abortCode);
    enqueueMotionStatusIfDue();
    if (readMotionExecutionGeneration() != generation) {
      if (abortCode) *abortCode = "stale_generation";
      return false;
    }
    if (g_stopRequested) return false;
    if (!stepped) {
      const uint32_t waitNowUs = micros();
      if (static_cast<int32_t>(deadlineUs - waitNowUs) > 2000) {
        vTaskDelay(pdMS_TO_TICKS(1));
      } else {
        delayMicroseconds(50);
      }
    }
  }
  if (abortCode) *abortCode = "stopped";
  return false;
}

bool hasMoveLTargetPose(const SerialMessage &message) {
  double value = 0.0;
  for (uint8_t i = 0; i < 12; ++i) {
    if (!readArrayValue(message, "\"target_tcp\"", i, &value)) {
      return false;
    }
  }
  return true;
}

bool hasMoveLTargetConfig(const SerialMessage &message) {
  double value = 0.0;
  for (uint8_t i = 0; i < kAxisCount; ++i) {
    if (!readArrayValue(message, "\"target_deg\"", i, &value)) {
      return false;
    }
  }
  return true;
}

void enqueueMotionCommand(const SerialMessage &message, bool linearMove) {
  const int32_t id = motionCommandId(message);
  if (!g_jogArmed) {
    if (linearMove) {
      enqueueMoveLResult("movel_rejected", "not_armed", 0, 0, id);
    } else {
      enqueueMoveJResult("movej_rejected", "not_armed", 0, id);
    }
    return;
  }
  if (g_masteringActive) {
    if (linearMove) {
      enqueueMoveLResult("movel_rejected", "busy", 0, 0, id);
    } else {
      enqueueMoveJResult("movej_rejected", "busy", 0, id);
    }
    return;
  }
  if (!allAxesMastered()) {
    if (linearMove) {
      enqueueMoveLResult("movel_rejected", "not_mastered", 0, 0, id);
    } else {
      enqueueMoveJResult("movej_rejected", "not_mastered", 0, id);
    }
    return;
  }
  if (linearMove) {
    if (!g_motionModelLoaded) {
      enqueueMoveLResult("movel_rejected", "model_not_loaded", 0, 0, id);
      return;
    }
    if (!hasMoveLTargetPose(message)) {
      enqueueMoveLResult("movel_rejected", "bad_pose", 0, 0, id);
      return;
    }
    if (!hasMoveLTargetConfig(message)) {
      enqueueMoveLResult("movel_rejected", "bad_target", 0, 0, id);
      return;
    }
  } else if (!hasMoveJTarget(message)) {
    enqueueMoveJResult("movej_rejected", "bad_target", 0, id);
    return;
  }

  if (g_motionQueue == nullptr || xQueueSend(g_motionQueue, &message, 0) != pdPASS) {
    if (linearMove) {
      enqueueMoveLResult("movel_rejected", "queue_full", 0, 0, id);
    } else {
      enqueueMoveJResult("movej_rejected", "queue_full", 0, id);
    }
    return;
  }

  enqueueMotionQueued(linearMove ? "movel_queued" : "movej_queued", id);
}

void handlePlanMoveJ(const SerialMessage &message) {
  RobotMotionCore::MoveJInput input = {};
  RobotMotionCore::MoveJPlan plan = {};
  const char *rejectCode = "unknown";
  if (!buildMoveJInput(message, &input, &rejectCode)) {
    enqueueMoveJPlanResult("movej_plan_rejected", rejectCode, nullptr);
    return;
  }
  const RobotMotionCore::RejectCode planResult = RobotMotionCore::planMoveJ(input, &plan);
  if (planResult != RobotMotionCore::RejectCode::Ok) {
    enqueueMoveJPlanResult("movej_plan_rejected", RobotMotionCore::rejectCodeName(planResult), nullptr);
    return;
  }
  enqueueMoveJPlanResult("movej_plan_ok", "ok", &plan);
}

bool readArrayValues(const SerialMessage &message, const char *key, uint8_t count, double *values) {
  if (!values) {
    return false;
  }
  for (uint8_t i = 0; i < count; ++i) {
    if (!readArrayValue(message, key, i, &values[i]) || !isfinite(values[i])) {
      return false;
    }
  }
  return true;
}

bool readOptionalArrayValues(const SerialMessage &message, const char *key, uint8_t count, double *values) {
  if (strstr(message.text, key) == nullptr) {
    return true;
  }
  return readArrayValues(message, key, count, values);
}

RobotMotionCore::MotionProgramSettings defaultAr4MotionSettings() {
  RobotMotionCore::MotionProgramSettings settings = RobotMotionCore::defaultMotionProgramSettings();
  settings.defaultJointSpeedDegPerSec = kMoveJDefaultSpeedDegPerSec;
  settings.jointSampleDeg = kMoveJPlanSampleDeg;
  settings.singularityPolicy = RobotMotionCore::SingularityPolicy::SlowDown;
  settings.verifySampledDynamics = 0;
  double velocityLimit[kAxisCount] = {};
  double accelerationLimit[kAxisCount] = {};
  for (uint8_t i = 0; i < kAxisCount; ++i) {
    velocityLimit[i] = kMoveJMaxSpeedDegPerSec * 3.14159265358979323846 / 180.0;
    accelerationLimit[i] = settings.defaultJointAccelerationRadSec2;
  }
  RobotMotionCore::applyJointDynamicsLimitsToMotionSettings(&settings, velocityLimit, accelerationLimit, nullptr);
  RobotMotionCore::applyControllerStepLimitsToMotionSettings(&settings, g_stepsPerDegree, kMoveJMinTickGapUs);
  return settings;
}

double positiveFiniteOr(double value, double fallback) {
  return isfinite(value) && value > 0.0 ? value : fallback;
}

bool readPositiveSettingIfPresent(const SerialMessage &message, const char *key, double *target) {
  if (jsonFieldValue(message, key) == nullptr) {
    return true;
  }
  const double value = readFloatField(message, key, -1.0);
  if (!isfinite(value) || value <= 0.0) {
    return false;
  }
  if (target) {
    *target = value;
  }
  return true;
}

bool motionSettingsSchemaAccepted(const SerialMessage &message) {
  return readIntField(message,
                      "\"motion_settings_schema\"",
                      -1) ==
         static_cast<int32_t>(RobotMotionCore::kMotionSettingsSchemaVersion);
}

bool motionSettingsMutationBusy() {
  if (g_moveActive || g_masteringActive) {
    return true;
  }
  if (g_motionQueue != nullptr && uxQueueMessagesWaiting(g_motionQueue) > 0) {
    return true;
  }
  if (g_motionCommandRing.count > 0) {
    return true;
  }
  return false;
}

void enqueueMotionSettings() {
  RobotMotionCore::MotionProgramSettings settings = {};
  bool loaded = false;
  taskENTER_CRITICAL();
  settings = g_motionSettingsLoaded ? g_motionSettings : defaultAr4MotionSettings();
  loaded = g_motionSettingsLoaded;
  taskEXIT_CRITICAL();

  SerialMessage response = {};
  snprintf(response.text,
           sizeof(response.text),
           "{\"msg\":\"motion_settings\",\"loaded\":%s,"
           "\"motion_settings_schema\":%lu,"
           "\"control_period_s\":%.17g,"
           "\"default_joint_speed_deg_s\":%.17g,"
           "\"default_linear_speed_mm_s\":%.17g,"
           "\"default_linear_accel_mm_s2\":%.17g,"
           "\"default_linear_jerk_mm_s3\":%.17g,"
           "\"default_tool_angular_speed_rad_s\":%.17g,"
           "\"default_tool_angular_accel_rad_s2\":%.17g,"
           "\"default_tool_angular_jerk_rad_s3\":%.17g,"
           "\"singularity_threshold_rad\":%.17g}",
           loaded ? "true" : "false",
           static_cast<unsigned long>(RobotMotionCore::kMotionSettingsSchemaVersion),
           settings.controlPeriodSec,
           settings.defaultJointSpeedDegPerSec,
           settings.defaultLinearSpeedMmPerSec,
           settings.defaultLinearAccelerationMmSec2,
           settings.defaultLinearJerkMmSec3,
           settings.defaultToolAngularSpeedRadSec,
           settings.defaultToolAngularAccelerationRadSec2,
           settings.defaultToolAngularJerkRadSec3,
           settings.singularityThresholdRad);
  if (xQueueSend(g_txQueue, &response, 0) != pdPASS) {
    g_txDropped++;
  }
}

void handleSetMotionSettings(const SerialMessage &message) {
  if (motionSettingsMutationBusy()) {
    enqueueTx("{\"msg\":\"motion_settings_set_failed\",\"code\":\"busy\"}");
    return;
  }
  if (!motionSettingsSchemaAccepted(message)) {
    enqueueTx("{\"msg\":\"motion_settings_set_failed\",\"code\":\"bad_schema\"}");
    return;
  }

  RobotMotionCore::MotionProgramSettings settings = {};
  taskENTER_CRITICAL();
  settings = g_motionSettingsLoaded ? g_motionSettings : defaultAr4MotionSettings();
  taskEXIT_CRITICAL();

  if (!readPositiveSettingIfPresent(message, "\"control_period_s\"", &settings.controlPeriodSec) ||
      !readPositiveSettingIfPresent(message, "\"default_joint_speed_deg_s\"", &settings.defaultJointSpeedDegPerSec) ||
      !readPositiveSettingIfPresent(message, "\"default_linear_speed_mm_s\"", &settings.defaultLinearSpeedMmPerSec) ||
      !readPositiveSettingIfPresent(message, "\"default_linear_accel_mm_s2\"", &settings.defaultLinearAccelerationMmSec2) ||
      !readPositiveSettingIfPresent(message, "\"default_linear_jerk_mm_s3\"", &settings.defaultLinearJerkMmSec3) ||
      !readPositiveSettingIfPresent(message, "\"default_tool_angular_speed_rad_s\"", &settings.defaultToolAngularSpeedRadSec) ||
      !readPositiveSettingIfPresent(message, "\"default_tool_angular_accel_rad_s2\"", &settings.defaultToolAngularAccelerationRadSec2) ||
      !readPositiveSettingIfPresent(message, "\"default_tool_angular_jerk_rad_s3\"", &settings.defaultToolAngularJerkRadSec3) ||
      !readPositiveSettingIfPresent(message, "\"singularity_threshold_rad\"", &settings.singularityThresholdRad)) {
    enqueueTx("{\"msg\":\"motion_settings_set_failed\",\"code\":\"bad_settings\"}");
    return;
  }

  taskENTER_CRITICAL();
  g_motionSettings = settings;
  g_motionSettingsLoaded = true;
  taskEXIT_CRITICAL();

  enqueueTx("{\"msg\":\"motion_settings_set_done\"}");
  enqueueMotionSettings();
}

void handleSetWeaveSchedule(const SerialMessage &message) {
  const int32_t schema = readIntField(message, "\"weave_schedule_schema\"", 0);
  if (schema != static_cast<int32_t>(RobotMotionCore::kWeaveScheduleSchemaVersion)) {
    enqueueTx("{\"msg\":\"weave_schedule_set_failed\",\"code\":\"bad_schema\"}");
    return;
  }
  const int32_t index = readIntField(message, "\"index\"", -1);
  if (index < 0 || index >= static_cast<int32_t>(RobotMotionCore::kMaxWeaveSchedules)) {
    enqueueTx("{\"msg\":\"weave_schedule_set_failed\",\"code\":\"bad_index\"}");
    return;
  }
  const int32_t shape = readIntField(message, "\"shape\"", 0);
  const int32_t rateMode = readIntField(message, "\"rate_mode\"", 0);
  if (shape < 0 || shape > static_cast<int32_t>(RobotMotionCore::WeaveShape::LShape) ||
      rateMode < 0 || rateMode > static_cast<int32_t>(RobotMotionCore::WeaveRateMode::Wavelength)) {
    enqueueTx("{\"msg\":\"weave_schedule_set_failed\",\"code\":\"bad_weave\"}");
    return;
  }

  RobotMotionCore::WeaveParams weave = RobotMotionCore::defaultWeaveParams();
  weave.shape = static_cast<RobotMotionCore::WeaveShape>(shape);
  weave.rateMode = static_cast<RobotMotionCore::WeaveRateMode>(rateMode);
  weave.frequencyHz = readFloatField(message, "\"freq_hz\"", 0.0);
  weave.wavelengthMm = readFloatField(message, "\"wavelength_mm\"", 0.0);
  weave.amplitudeLeftMm = readFloatField(message, "\"amp_left_mm\"", 0.0);
  weave.amplitudeRightMm = readFloatField(message, "\"amp_right_mm\"", 0.0);
  weave.elevationMm = readFloatField(message, "\"elevation_mm\"", 0.0);
  weave.planeAngleDeg = readFloatField(message, "\"plane_angle_deg\"", 0.0);
  weave.biasMm = readFloatField(message, "\"bias_mm\"", 0.0);
  weave.dwellLeft = readFloatField(message, "\"dwell_left\"", 0.0);
  weave.dwellCenter = readFloatField(message, "\"dwell_center\"", 0.0);
  weave.dwellRight = readFloatField(message, "\"dwell_right\"", 0.0);
  weave.scheduleIndex = static_cast<uint8_t>(index);
  const bool valid = readIntField(message, "\"valid\"", 0) != 0;

  taskENTER_CRITICAL();
  g_weaveSchedules.schedules[index] = weave;
  g_weaveSchedules.valid[index] = valid ? 1 : 0;
  taskEXIT_CRITICAL();

  char reply[96];
  snprintf(reply, sizeof(reply),
           "{\"msg\":\"weave_schedule_set_done\",\"index\":%ld,\"valid\":%d,\"shape\":\"%s\"}",
           static_cast<long>(index), valid ? 1 : 0, RobotMotionCore::weaveShapeName(weave.shape));
  enqueueTx(reply);
}

void enqueueWeaveSchedules() {
  // One line per schedule, matching how they are set. A single reply carrying all eight would not
  // fit the fixed serial message size.
  for (uint8_t index = 0; index < RobotMotionCore::kMaxWeaveSchedules; ++index) {
    const RobotMotionCore::WeaveParams &weave = g_weaveSchedules.schedules[index];
    // Sized for the complete JSON line; truncation would make the response unparsable.
    char line[512];
    const int written = snprintf(line, sizeof(line),
             "{\"msg\":\"weave_schedule\",\"weave_schedule_schema\":%lu,\"index\":%u,\"valid\":%u,"
             "\"shape\":%d,\"rate_mode\":%d,\"freq_hz\":%.4f,\"wavelength_mm\":%.4f,"
             "\"amp_left_mm\":%.4f,\"amp_right_mm\":%.4f,\"elevation_mm\":%.4f,"
             "\"plane_angle_deg\":%.4f,\"bias_mm\":%.4f,"
             "\"dwell_left\":%.4f,\"dwell_center\":%.4f,\"dwell_right\":%.4f}",
             static_cast<unsigned long>(RobotMotionCore::kWeaveScheduleSchemaVersion),
             static_cast<unsigned>(index),
             static_cast<unsigned>(g_weaveSchedules.valid[index]),
             static_cast<int>(weave.shape), static_cast<int>(weave.rateMode),
             weave.frequencyHz, weave.wavelengthMm,
             weave.amplitudeLeftMm, weave.amplitudeRightMm, weave.elevationMm,
             weave.planeAngleDeg, weave.biasMm,
             weave.dwellLeft, weave.dwellCenter, weave.dwellRight);
    if (written < 0 || static_cast<size_t>(written) >= sizeof(line)) {
      enqueueTx("{\"msg\":\"weave_schedule_failed\",\"code\":\"reply_too_long\"}");
      return;
    }
    enqueueTx(line);
  }
}

void handleLoadRobotModel(const SerialMessage &message) {
  RobotMotionCore::RobotModel model = {};
  RobotMotionCore::MotionProgramSettings settings = defaultAr4MotionSettings();
  model.toolBindPose = RobotMotionCore::identityTransform();
  if (!motionSettingsSchemaAccepted(message)) {
    enqueueTx("{\"msg\":\"robot_model_load_failed\",\"code\":\"bad_motion_schema\"}");
    return;
  }
  if (!readArrayValues(message, "\"dhm\"", 24, model.dhm) ||
      !readArrayValues(message, "\"q_home\"", kAxisCount, model.qHome) ||
      !readArrayValues(message, "\"q_min\"", kAxisCount, model.qMin) ||
      !readArrayValues(message, "\"q_max\"", kAxisCount, model.qMax) ||
      !readArrayValues(message, "\"dhm_signs\"", kAxisCount, model.dhmSigns) ||
      !readArrayValues(message, "\"tool_bind\"", 12, model.toolBindPose.values)) {
    enqueueTx("{\"msg\":\"robot_model_load_failed\",\"code\":\"bad_model\"}");
    return;
  }
  model.valid = 1;
  if (!RobotMotionCore::modelIsValid(model)) {
    enqueueTx("{\"msg\":\"robot_model_load_failed\",\"code\":\"invalid_model\"}");
    return;
  }

  if (!readOptionalArrayValues(message, "\"joint_velocity_max_rad_s\"", kAxisCount, settings.jointVelocityLimitRadSec) ||
      !readOptionalArrayValues(message, "\"joint_acceleration_max_rad_s2\"", kAxisCount, settings.jointAccelerationLimitRadSec2)) {
    enqueueTx("{\"msg\":\"robot_model_load_failed\",\"code\":\"bad_dynamics\"}");
    return;
  }
  double jointJerk[kAxisCount] = {};
  if (!readOptionalArrayValues(message, "\"joint_jerk_max_rad_s3\"", kAxisCount, jointJerk)) {
    enqueueTx("{\"msg\":\"robot_model_load_failed\",\"code\":\"bad_dynamics\"}");
    return;
  }
  RobotMotionCore::applyJointDynamicsLimitsToMotionSettings(&settings,
                                                            settings.jointVelocityLimitRadSec,
                                                            settings.jointAccelerationLimitRadSec2,
                                                            jointJerk);
  const double controllerTickUs = positiveFiniteOr(readFloatField(message, "\"controller_min_tick_gap_us\"", kMoveJMinTickGapUs), kMoveJMinTickGapUs);
  settings.controlPeriodSec = positiveFiniteOr(readFloatField(message, "\"control_period_s\"", settings.controlPeriodSec), settings.controlPeriodSec);
  settings.defaultJointSpeedDegPerSec = positiveFiniteOr(readFloatField(message, "\"default_joint_speed_deg_s\"", settings.defaultJointSpeedDegPerSec), settings.defaultJointSpeedDegPerSec);
  settings.defaultLinearSpeedMmPerSec = positiveFiniteOr(readFloatField(message, "\"default_linear_speed_mm_s\"", settings.defaultLinearSpeedMmPerSec), settings.defaultLinearSpeedMmPerSec);
  settings.defaultLinearAccelerationMmSec2 = positiveFiniteOr(readFloatField(message, "\"default_linear_accel_mm_s2\"", settings.defaultLinearAccelerationMmSec2), settings.defaultLinearAccelerationMmSec2);
  settings.defaultLinearJerkMmSec3 = positiveFiniteOr(readFloatField(message, "\"default_linear_jerk_mm_s3\"", settings.defaultLinearJerkMmSec3), settings.defaultLinearJerkMmSec3);
  settings.defaultToolAngularSpeedRadSec = positiveFiniteOr(readFloatField(message, "\"default_tool_angular_speed_rad_s\"", settings.defaultToolAngularSpeedRadSec), settings.defaultToolAngularSpeedRadSec);
  settings.defaultToolAngularAccelerationRadSec2 = positiveFiniteOr(readFloatField(message, "\"default_tool_angular_accel_rad_s2\"", settings.defaultToolAngularAccelerationRadSec2), settings.defaultToolAngularAccelerationRadSec2);
  settings.defaultToolAngularJerkRadSec3 = positiveFiniteOr(readFloatField(message, "\"default_tool_angular_jerk_rad_s3\"", settings.defaultToolAngularJerkRadSec3), settings.defaultToolAngularJerkRadSec3);
  settings.singularityThresholdRad = positiveFiniteOr(readFloatField(message, "\"singularity_threshold_rad\"", settings.singularityThresholdRad), settings.singularityThresholdRad);
  RobotMotionCore::applyControllerStepLimitsToMotionSettings(&settings, g_stepsPerDegree, controllerTickUs);

  taskENTER_CRITICAL();
  g_motionModel = model;
  g_motionSettings = settings;
  g_motionModelLoaded = true;
  g_motionSettingsLoaded = true;
  taskEXIT_CRITICAL();
  enqueueTx("{\"msg\":\"robot_model_load_done\"}");
}

void enqueueMotionLookaheadPlanned(uint8_t commandCount,
                                   uint8_t executeCount,
                                   int32_t firstId,
                                   int32_t lastExecuteId,
                                   uint32_t buildMs,
                                   uint32_t plannedDurationMs,
                                   uint8_t sampledVerification) {
  SerialMessage response = {};
  snprintf(response.text,
           sizeof(response.text),
           "{\"msg\":\"motion_lookahead_planned\",\"queued_count\":%u,\"execute_count\":%u,"
           "\"first_id\":%ld,\"last_execute_id\":%ld,\"build_ms\":%lu,"
           "\"planned_ms\":%lu,\"sample_verify\":%u}",
           static_cast<unsigned>(commandCount),
           static_cast<unsigned>(executeCount),
           static_cast<long>(firstId),
           static_cast<long>(lastExecuteId),
           static_cast<unsigned long>(buildMs),
           static_cast<unsigned long>(plannedDurationMs),
           static_cast<unsigned>(sampledVerification));
  if (xQueueSend(g_txQueue, &response, 0) != pdPASS) {
    g_txDropped++;
  }
}

void enqueueMoveLResult(const char *msg, const char *code, uint32_t sample, uint32_t ticks, int32_t id) {
  SerialMessage response = {};
  if (id >= 0) {
    snprintf(response.text,
             sizeof(response.text),
             "{\"msg\":\"%s\",\"id\":%ld,\"code\":\"%s\",\"sample\":%lu,\"ticks\":%lu}",
             msg ? msg : "movel_rejected",
             static_cast<long>(id),
             code ? code : "unknown",
             static_cast<unsigned long>(sample),
             static_cast<unsigned long>(ticks));
  } else {
    snprintf(response.text,
             sizeof(response.text),
             "{\"msg\":\"%s\",\"code\":\"%s\",\"sample\":%lu,\"ticks\":%lu}",
             msg ? msg : "movel_rejected",
             code ? code : "unknown",
             static_cast<unsigned long>(sample),
             static_cast<unsigned long>(ticks));
  }
  if (xQueueSend(g_txQueue, &response, 0) != pdPASS) {
    g_txDropped++;
  }
}

void enqueueMoveLRunResult(const char *msg,
                           const char *code,
                           uint32_t sample,
                           uint32_t ticks,
                           int32_t id,
                           uint32_t elapsedMs,
                           uint32_t expectedMs,
                           uint32_t sampleDurationUs,
                           uint32_t sampleCount,
                           uint32_t lineMmX1000,
                           uint32_t speedMmSecX1000,
                           uint32_t zeroStepSamples,
                           bool blended,
                           uint32_t blendSamples,
                           uint32_t blendArcMmX1000,
                           const char *segmentKind,
                           uint32_t profileSpeedMmSecX1000,
                           uint32_t peakSpeedMmSecX1000,
                           uint32_t startSpeedMmSecX1000,
                           uint32_t endSpeedMmSecX1000,
                           uint32_t blendRadiusMmX1000,
                           uint32_t blendDeviationMmX1000,
                           uint32_t blendContourDeviationMmX1000,
                           uint32_t capReasonMask,
                           uint32_t commandSpeedMmSecX1000,
                           uint32_t finalSpeedMmSecX1000,
                           uint32_t capJointVelocityMmSecX1000,
                           uint32_t capJointAccelerationMmSecX1000,
                           uint32_t capSingularityMmSecX1000,
                           uint32_t capStepRateMmSecX1000,
                           uint32_t capJerkMmSecX1000,
                           const RobotMotionCore::StepExecutorStats *stepStats) {
  SerialMessage response = {};
  const uint32_t stepSamples = stepStats ? stepStats->samples : sampleCount;
  const uint32_t stepZeroSamples = stepStats ? stepStats->zeroStepSamples : zeroStepSamples;
  const uint32_t stepReversalSamples = stepStats ? stepStats->reversalSamples : 0;
  const uint32_t maxStepDelta = stepStats ? stepStats->maxStepDelta : 0;
  const uint32_t totalStepDelta = stepStats ? stepStats->totalAbsStepDelta : 0;
  if (id >= 0) {
    snprintf(response.text,
             sizeof(response.text),
             "{\"msg\":\"%s\",\"id\":%ld,\"code\":\"%s\",\"sample\":%lu,\"ticks\":%lu,"
             "\"elapsed_ms\":%lu,\"expected_ms\":%lu,\"sample_us\":%lu,\"sample_count\":%lu,"
             "\"line_mm_x1000\":%lu,\"speed_mm_s_x1000\":%lu,\"zero_step_samples\":%lu,"
             "\"blended\":%s,\"blend_samples\":%lu,\"blend_arc_mm_x1000\":%lu,"
             "\"segment_kind\":\"%s\",\"profile_speed_mm_s_x1000\":%lu,"
             "\"peak_speed_mm_s_x1000\":%lu,\"start_speed_mm_s_x1000\":%lu,"
             "\"end_speed_mm_s_x1000\":%lu,\"step_samples\":%lu,"
             "\"step_zero_samples\":%lu,\"step_reversal_samples\":%lu,"
             "\"max_step_delta\":%lu,\"total_step_delta\":%lu,"
             "\"blend_radius_mm_x1000\":%lu,\"blend_trim_mm_x1000\":%lu,"
             "\"blend_deviation_mm_x1000\":%lu,\"blend_contour_deviation_mm_x1000\":%lu,"
             "\"cap_reason_mask\":%lu,\"command_speed_mm_s_x1000\":%lu,"
             "\"final_speed_mm_s_x1000\":%lu,\"cap_joint_velocity_mm_s_x1000\":%lu,"
             "\"cap_joint_accel_mm_s_x1000\":%lu,\"cap_singularity_mm_s_x1000\":%lu,"
             "\"cap_step_rate_mm_s_x1000\":%lu,\"cap_jerk_mm_s_x1000\":%lu}",
             msg ? msg : "movel_done",
             static_cast<long>(id),
             code ? code : "unknown",
             static_cast<unsigned long>(sample),
             static_cast<unsigned long>(ticks),
             static_cast<unsigned long>(elapsedMs),
             static_cast<unsigned long>(expectedMs),
             static_cast<unsigned long>(sampleDurationUs),
             static_cast<unsigned long>(sampleCount),
             static_cast<unsigned long>(lineMmX1000),
             static_cast<unsigned long>(speedMmSecX1000),
             static_cast<unsigned long>(stepZeroSamples),
             blended ? "true" : "false",
             static_cast<unsigned long>(blendSamples),
             static_cast<unsigned long>(blendArcMmX1000),
             segmentKind ? segmentKind : "unknown",
             static_cast<unsigned long>(profileSpeedMmSecX1000),
             static_cast<unsigned long>(peakSpeedMmSecX1000),
             static_cast<unsigned long>(startSpeedMmSecX1000),
             static_cast<unsigned long>(endSpeedMmSecX1000),
             static_cast<unsigned long>(stepSamples),
             static_cast<unsigned long>(stepZeroSamples),
             static_cast<unsigned long>(stepReversalSamples),
             static_cast<unsigned long>(maxStepDelta),
             static_cast<unsigned long>(totalStepDelta),
             static_cast<unsigned long>(blendRadiusMmX1000),
             static_cast<unsigned long>(blendDeviationMmX1000),
             static_cast<unsigned long>(blendDeviationMmX1000),
             static_cast<unsigned long>(blendContourDeviationMmX1000),
             static_cast<unsigned long>(capReasonMask),
             static_cast<unsigned long>(commandSpeedMmSecX1000),
             static_cast<unsigned long>(finalSpeedMmSecX1000),
             static_cast<unsigned long>(capJointVelocityMmSecX1000),
             static_cast<unsigned long>(capJointAccelerationMmSecX1000),
             static_cast<unsigned long>(capSingularityMmSecX1000),
             static_cast<unsigned long>(capStepRateMmSecX1000),
             static_cast<unsigned long>(capJerkMmSecX1000));
  } else {
    snprintf(response.text,
             sizeof(response.text),
             "{\"msg\":\"%s\",\"code\":\"%s\",\"sample\":%lu,\"ticks\":%lu,"
             "\"elapsed_ms\":%lu,\"expected_ms\":%lu,\"sample_us\":%lu,\"sample_count\":%lu,"
             "\"line_mm_x1000\":%lu,\"speed_mm_s_x1000\":%lu,\"zero_step_samples\":%lu,"
             "\"blended\":%s,\"blend_samples\":%lu,\"blend_arc_mm_x1000\":%lu,"
             "\"segment_kind\":\"%s\",\"profile_speed_mm_s_x1000\":%lu,"
             "\"peak_speed_mm_s_x1000\":%lu,\"start_speed_mm_s_x1000\":%lu,"
             "\"end_speed_mm_s_x1000\":%lu,\"step_samples\":%lu,"
             "\"step_zero_samples\":%lu,\"step_reversal_samples\":%lu,"
             "\"max_step_delta\":%lu,\"total_step_delta\":%lu,"
             "\"blend_radius_mm_x1000\":%lu,\"blend_trim_mm_x1000\":%lu,"
             "\"blend_deviation_mm_x1000\":%lu,\"blend_contour_deviation_mm_x1000\":%lu,"
             "\"cap_reason_mask\":%lu,\"command_speed_mm_s_x1000\":%lu,"
             "\"final_speed_mm_s_x1000\":%lu,\"cap_joint_velocity_mm_s_x1000\":%lu,"
             "\"cap_joint_accel_mm_s_x1000\":%lu,\"cap_singularity_mm_s_x1000\":%lu,"
             "\"cap_step_rate_mm_s_x1000\":%lu,\"cap_jerk_mm_s_x1000\":%lu}",
             msg ? msg : "movel_done",
             code ? code : "unknown",
             static_cast<unsigned long>(sample),
             static_cast<unsigned long>(ticks),
             static_cast<unsigned long>(elapsedMs),
             static_cast<unsigned long>(expectedMs),
             static_cast<unsigned long>(sampleDurationUs),
             static_cast<unsigned long>(sampleCount),
             static_cast<unsigned long>(lineMmX1000),
             static_cast<unsigned long>(speedMmSecX1000),
             static_cast<unsigned long>(stepZeroSamples),
             blended ? "true" : "false",
             static_cast<unsigned long>(blendSamples),
             static_cast<unsigned long>(blendArcMmX1000),
             segmentKind ? segmentKind : "unknown",
             static_cast<unsigned long>(profileSpeedMmSecX1000),
             static_cast<unsigned long>(peakSpeedMmSecX1000),
             static_cast<unsigned long>(startSpeedMmSecX1000),
             static_cast<unsigned long>(endSpeedMmSecX1000),
             static_cast<unsigned long>(stepSamples),
             static_cast<unsigned long>(stepZeroSamples),
             static_cast<unsigned long>(stepReversalSamples),
             static_cast<unsigned long>(maxStepDelta),
             static_cast<unsigned long>(totalStepDelta),
             static_cast<unsigned long>(blendRadiusMmX1000),
             static_cast<unsigned long>(blendDeviationMmX1000),
             static_cast<unsigned long>(blendDeviationMmX1000),
             static_cast<unsigned long>(blendContourDeviationMmX1000),
             static_cast<unsigned long>(capReasonMask),
             static_cast<unsigned long>(commandSpeedMmSecX1000),
             static_cast<unsigned long>(finalSpeedMmSecX1000),
             static_cast<unsigned long>(capJointVelocityMmSecX1000),
             static_cast<unsigned long>(capJointAccelerationMmSecX1000),
             static_cast<unsigned long>(capSingularityMmSecX1000),
             static_cast<unsigned long>(capStepRateMmSecX1000),
             static_cast<unsigned long>(capJerkMmSecX1000));
  }
  if (xQueueSend(g_txQueue, &response, 0) != pdPASS) {
    g_txDropped++;
  }
}

void enqueueMotionQueued(const char *msg, int32_t id) {
  SerialMessage response = {};
  const UBaseType_t depth = g_motionQueue != nullptr ? uxQueueMessagesWaiting(g_motionQueue) : 0;
  if (id >= 0) {
    snprintf(response.text,
             sizeof(response.text),
             "{\"msg\":\"%s\",\"id\":%ld,\"depth\":%lu}",
             msg,
             static_cast<long>(id),
             static_cast<unsigned long>(depth));
  } else {
    snprintf(response.text,
             sizeof(response.text),
             "{\"msg\":\"%s\",\"depth\":%lu}",
             msg,
             static_cast<unsigned long>(depth));
  }
  if (xQueueSend(g_txQueue, &response, 0) != pdPASS) {
    g_txDropped++;
  }
}

bool buildMoveLInput(const SerialMessage &message, RobotMotionCore::MoveLInput *input, const char **rejectCode) {
  if (!input) {
    if (rejectCode) *rejectCode = "bad_pose";
    return false;
  }
  memset(input, 0, sizeof(*input));
  if (!g_motionModelLoaded) {
    if (rejectCode) *rejectCode = "model_not_loaded";
    return false;
  }
  if (!readArrayValues(message, "\"target_tcp\"", 12, input->targetTcp.values)) {
    if (rejectCode) *rejectCode = "bad_pose";
    return false;
  }
  double targetDeg[kAxisCount] = {};
  if (!readArrayValues(message, "\"target_deg\"", kAxisCount, targetDeg)) {
    if (rejectCode) *rejectCode = "bad_target";
    return false;
  }
  taskENTER_CRITICAL();
  input->model = g_motionModel;
  taskEXIT_CRITICAL();
  currentJointRadians(input->startQ);
  constexpr double kDegToRadLocal = 3.14159265358979323846 / 180.0;
  for (uint8_t i = 0; i < kAxisCount; ++i) {
    input->targetConfigQ[i] = targetDeg[i] * kDegToRadLocal;
  }
  input->speedMmPerSec = readFloatField(message, "\"speed_mm_s\"", 25.0);
  input->sampleMm = readFloatField(message, "\"sample_mm\"", 0.0);
  input->singularityThresholdRad = readFloatField(message, "\"singularity_rad\"", 5.0 * 3.14159265358979323846 / 180.0);
  input->singularityPolicy = RobotMotionCore::SingularityPolicy::SlowDown;
  input->requireTargetConfig = 1;
  input->targetConfigToleranceRad = 0.5 * kDegToRadLocal;
  if (rejectCode) *rejectCode = "ok";
  return true;
}

void handlePlanMoveL(const SerialMessage &message) {
  if (!allAxesMastered()) {
    enqueueMoveLResult("movel_plan_rejected", "not_mastered", 0, 0);
    return;
  }
  RobotMotionCore::MoveLInput input = {};
  const char *rejectCode = "unknown";
  if (!buildMoveLInput(message, &input, &rejectCode)) {
    enqueueMoveLResult("movel_plan_rejected", rejectCode, 0, 0);
    return;
  }
  RobotMotionCore::MoveLPlan plan = {};
  const RobotMotionCore::RejectCode result = RobotMotionCore::planMoveLEndpoint(input, &plan);
  if (result != RobotMotionCore::RejectCode::Ok) {
    enqueueMoveLResult("movel_plan_rejected", RobotMotionCore::rejectCodeName(result), 0, 0);
    return;
  }
  enqueueMoveLResult("movel_plan_ok", "endpoint_ok", plan.sampleCount, 0);
}

bool buildMotionProgramCommand(const SerialMessage &message,
                               RobotMotionCore::MotionCommand *command,
                               const char **rejectCode) {
  if (!command) {
    if (rejectCode) *rejectCode = "bad_target";
    return false;
  }
  memset(command, 0, sizeof(*command));
  command->id = motionCommandId(message);
  command->sampleMm = readFloatField(message, "\"sample_mm\"", 0.0);
  command->blendMm = readFloatField(message, "\"blend_mm\"", 0.0);

  command->trigger = RobotMotionCore::defaultMotionTrigger();
  const int32_t triggerId = readIntField(message, "\"trigger_id\"", -1);
  if (triggerId >= 0) {
    command->trigger.id = triggerId;
    command->trigger.reference = readIntField(message, "\"trigger_ref_start\"", 0) != 0
        ? RobotMotionCore::MotionTriggerReference::Start
        : RobotMotionCore::MotionTriggerReference::End;
    command->trigger.distanceMm = readFloatField(message, "\"trigger_dist_mm\"", 0.0);
    command->trigger.timeMs = readFloatField(message, "\"trigger_time_ms\"", 0.0);
  }

  double targetDeg[kAxisCount] = {};
  if (!readArrayValues(message, "\"target_deg\"", kAxisCount, targetDeg)) {
    if (rejectCode) *rejectCode = "bad_target";
    return false;
  }
  constexpr double kDegToRadLocal = 3.14159265358979323846 / 180.0;
  for (uint8_t i = 0; i < kAxisCount; ++i) {
    command->targetQ[i] = targetDeg[i] * kDegToRadLocal;
  }

  if (commandIs(message, "movel")) {
    command->type = RobotMotionCore::MotionCommandType::MoveL;
    if (!g_motionModelLoaded) {
      if (rejectCode) *rejectCode = "model_not_loaded";
      return false;
    }
    if (!readArrayValues(message, "\"target_tcp\"", 12, command->targetTcp.values)) {
      if (rejectCode) *rejectCode = "bad_pose";
      return false;
    }
    command->linearSpeedMmPerSec = readFloatField(message, "\"speed_mm_s\"", 25.0);
    command->jointSpeedDegPerSec = kMoveJDefaultSpeedDegPerSec;

    const int32_t weaveIndex = readIntField(message, "\"weave_index\"", -2);
    if (weaveIndex != -2) {
      RobotMotionCore::WeaveParams request = RobotMotionCore::defaultWeaveParams();
      request.enabled = 1;
      if (weaveIndex >= 0) {
        request.scheduleIndex = static_cast<uint8_t>(weaveIndex);
        // A stored schedule supplies its own shape; this only has to read as active so the lookup
        // happens.
        request.shape = RobotMotionCore::WeaveShape::Sine;
      } else {
        // Inline parameters travel with the move. jsonFieldValue searches the whole line, so the
        // nested object's keys are found without the parser having to understand nesting.
        request.scheduleIndex = RobotMotionCore::kWeaveScheduleInline;
        const int32_t shape = readIntField(message, "\"shape\"", 0);
        const int32_t rateMode = readIntField(message, "\"rate_mode\"", 0);
        if (shape < 0 || shape > static_cast<int32_t>(RobotMotionCore::WeaveShape::LShape) ||
            rateMode < 0 || rateMode > static_cast<int32_t>(RobotMotionCore::WeaveRateMode::Wavelength)) {
          if (rejectCode) *rejectCode = "weave_infeasible";
          return false;
        }
        request.shape = static_cast<RobotMotionCore::WeaveShape>(shape);
        request.rateMode = static_cast<RobotMotionCore::WeaveRateMode>(rateMode);
        request.frequencyHz = readFloatField(message, "\"freq_hz\"", 0.0);
        request.wavelengthMm = readFloatField(message, "\"wavelength_mm\"", 0.0);
        request.amplitudeLeftMm = readFloatField(message, "\"amp_left_mm\"", 0.0);
        request.amplitudeRightMm = readFloatField(message, "\"amp_right_mm\"", 0.0);
        request.elevationMm = readFloatField(message, "\"elevation_mm\"", 0.0);
        request.planeAngleDeg = readFloatField(message, "\"plane_angle_deg\"", 0.0);
        request.biasMm = readFloatField(message, "\"bias_mm\"", 0.0);
        request.dwellLeft = readFloatField(message, "\"dwell_left\"", 0.0);
        request.dwellCenter = readFloatField(message, "\"dwell_center\"", 0.0);
        request.dwellRight = readFloatField(message, "\"dwell_right\"", 0.0);
      }
      if (!RobotMotionCore::resolveWeaveParams(g_weaveSchedules, request, &command->weave)) {
        if (rejectCode) *rejectCode = "weave_schedule_missing";
        return false;
      }
    }
  } else if (commandIs(message, "movej")) {
    command->type = RobotMotionCore::MotionCommandType::MoveJ;
    if (!g_motionModelLoaded) {
      if (rejectCode) *rejectCode = "model_not_loaded";
      return false;
    }
    command->targetTcp = RobotMotionCore::toolPoseForJoints(g_motionModel, command->targetQ);
    command->jointSpeedDegPerSec = readFloatField(message, "\"speed_deg_s\"", kMoveJDefaultSpeedDegPerSec);
    command->linearSpeedMmPerSec = 25.0;
    command->blendMm = 0.0;
  } else {
    if (rejectCode) *rejectCode = "bad_target";
    return false;
  }

  if (rejectCode) *rejectCode = "ok";
  return true;
}

RobotMotionCore::MotionCommandType motionProgramCommandType(const RobotMotionCore::MotionProgram &program,
                                                            int32_t id) {
  for (uint8_t i = 0; i < program.count; ++i) {
    if (program.commands[i].id == id) {
      return program.commands[i].type;
    }
  }
  return RobotMotionCore::MotionCommandType::MoveJ;
}

uint32_t mmSecX1000(double value) {
  return RobotMotionCore::isFinite(value) && value > 0.0
      ? static_cast<uint32_t>(llround(value * 1000.0))
      : 0;
}

bool motionMessageIsFinalProgramCommand(const SerialMessage &message) {
  const int32_t programTotal = readIntField(message, "\"program_total\"", 0);
  const int32_t globalIndex = readIntField(message, "\"program_global_index\"", -1);
  if (programTotal > 0 && globalIndex >= 0) {
    return globalIndex + 1 >= programTotal;
  }
  return readIntField(message, "\"program_final\"", 1) != 0;
}

bool appendMotionMessageToRing(const SerialMessage &message, const char **rejectCode) {
  if (RobotMotionCore::motionCommandRingFree(g_motionCommandRing) == 0) {
    if (rejectCode) *rejectCode = "queue_full";
    return false;
  }
  RobotMotionCore::MotionCommand command = {};
  if (!buildMotionProgramCommand(message, &command, rejectCode)) {
    return false;
  }
  if (!RobotMotionCore::motionCommandRingPush(&g_motionCommandRing,
                                              command,
                                              motionMessageIsFinalProgramCommand(message))) {
    if (rejectCode) *rejectCode = "queue_full";
    return false;
  }
  if (rejectCode) *rejectCode = "ok";
  return true;
}

bool executeMotionProgramFromQueue(const SerialMessage &firstMessage) {
  const bool hasFirstMessage = firstMessage.text[0] != '\0';
  const RobotMotionCore::MotionCommand *firstRingCommand =
      !hasFirstMessage ? RobotMotionCore::motionCommandRingAt(g_motionCommandRing, 0) : nullptr;
  const int32_t firstId = hasFirstMessage
      ? motionCommandId(firstMessage)
      : (firstRingCommand ? firstRingCommand->id : -1);
  const bool firstLinear = hasFirstMessage
      ? commandIs(firstMessage, "movel")
      : (firstRingCommand && firstRingCommand->type == RobotMotionCore::MotionCommandType::MoveL);
  if (!g_jogArmed) {
    if (firstLinear) {
      enqueueMoveLResult("movel_rejected", "not_armed", 0, 0, firstId);
    } else {
      enqueueMoveJResult("movej_rejected", "not_armed", 0, firstId);
    }
    return false;
  }
  if (g_masteringActive) {
    if (firstLinear) {
      enqueueMoveLResult("movel_rejected", "busy", 0, 0, firstId);
    } else {
      enqueueMoveJResult("movej_rejected", "busy", 0, firstId);
    }
    return false;
  }
  if (!allAxesMastered()) {
    if (firstLinear) {
      enqueueMoveLResult("movel_rejected", "not_mastered", 0, 0, firstId);
    } else {
      enqueueMoveJResult("movej_rejected", "not_mastered", 0, firstId);
    }
    return false;
  }
  if (!g_motionModelLoaded) {
    if (firstLinear) {
      enqueueMoveLResult("movel_rejected", "model_not_loaded", 0, 0, firstId);
    } else {
      enqueueMoveJResult("movej_rejected", "model_not_loaded", 0, firstId);
    }
    return false;
  }

  const char *rejectCode = "unknown";
    // Only a new run resets state carried across planning windows.
  if (hasFirstMessage && g_motionCommandRing.count == 0 && !g_moveActive) {
    bumpMotionExecutionGeneration();
    resetMotionRunState();
    if (g_motionExecutionQueue != nullptr) {
      xQueueReset(g_motionExecutionQueue);
    }
  }
  const uint32_t planGeneration = readMotionExecutionGeneration();
  if (hasFirstMessage && !appendMotionMessageToRing(firstMessage, &rejectCode)) {
    if (firstLinear) {
      enqueueMoveLResult("movel_rejected", rejectCode, 0, 0, firstId);
    } else {
      enqueueMoveJResult("movej_rejected", rejectCode, 0, firstId);
    }
    return false;
  }

  const TickType_t fillDeadline = xTaskGetTickCount() + pdMS_TO_TICKS(2000);
  while (RobotMotionCore::motionCommandRingFree(g_motionCommandRing) > 0 && g_motionQueue != nullptr) {
    SerialMessage queued = {};
    const TickType_t waitTicks = RobotMotionCore::motionLookaheadExecutableCount(g_motionCommandRing) == 0 &&
                                 !g_motionCommandRing.finalCommandQueued
                                     ? pdMS_TO_TICKS(20)
                                     : 0;
    if (xQueueReceive(g_motionQueue, &queued, waitTicks) != pdPASS) {
      if (RobotMotionCore::motionLookaheadExecutableCount(g_motionCommandRing) == 0 &&
          !g_motionCommandRing.finalCommandQueued &&
          xTaskGetTickCount() < fillDeadline) {
        continue;
      }
      break;
    }
    if (!appendMotionMessageToRing(queued, &rejectCode)) {
      const int32_t rejectedId = motionCommandId(queued);
      if (commandIs(queued, "movel")) {
        enqueueMoveLResult("movel_rejected", rejectCode, 0, 0, rejectedId);
      } else {
        enqueueMoveJResult("movej_rejected", rejectCode, 0, rejectedId);
      }
      return false;
    }
  }

  g_motionWindowRunner.model = g_motionModel;
  g_motionWindowRunner.settings = g_motionSettingsLoaded ? g_motionSettings : defaultAr4MotionSettings();
  for (uint8_t i = 0; i < kAxisCount; ++i) {
    g_motionWindowRunner.settings.jointStepsPerDegree[i] = g_stepsPerDegree[i];
  }
  g_motionWindowRunner.settings.verifySampledDynamics = 0;
  // Where this window plans from. A run in flight carries its seed across the boundary; a run that has
  // just begun, or one resuming after the ring emptied, has none and takes the arm's real joints.
  if (!g_motionWindowRunner.replanSeedValid) {
    double currentQ[kAxisCount] = {};
    currentJointRadians(currentQ);
    RobotMotionCore::setMotionWindowRunnerSeedQ(&g_motionWindowRunner, currentQ);
  }

  const uint32_t planStartMs = millis();
  RobotMotionCore::RejectCode streamResult = RobotMotionCore::planMotionWindow(&g_motionWindowRunner);
  // Covers the ring copy and the sampler seek as well as the trajectory solve now. Both were already
  // inside the window's cost; only what this number is named after has widened slightly.
  const uint32_t planBuildMs = millis() - planStartMs;
  if (streamResult != RobotMotionCore::RejectCode::Ok) {
    if (firstLinear) {
      enqueueMoveLResult("movel_rejected", RobotMotionCore::rejectCodeName(streamResult), 0, 0, firstId);
    } else {
      enqueueMoveJResult("movej_rejected", RobotMotionCore::rejectCodeName(streamResult), 0, firstId);
    }
    return false;
  }
  // No window means the lookahead never released a prefix: the ring is short of
  // kMotionLookaheadRetainedCommands and no final command has closed it, so the wait above gave up.
  // Reported as it always was; "bad_ring" is gone with it, because inside the driver a program built
  // from the ring it just measured cannot fail.
  if (!g_motionWindowRunner.haveWindow) {
    if (firstLinear) {
      enqueueMoveLResult("movel_rejected", "lookahead_timeout", 0, 0, firstId);
    } else {
      enqueueMoveJResult("movej_rejected", "lookahead_timeout", 0, firstId);
    }
    return false;
  }

  RobotMotionCore::MotionProgram &program = g_motionProgramBuffer;
  RobotMotionCore::MotionSegmentProgram &segmentProgram = g_motionSegmentProgramBuffer;
  const uint8_t executeProgramCount = g_motionWindowRunner.executeCount;
  const int32_t lastExecuteCommandId = g_motionWindowRunner.lastExecutedCommandId;
  enqueueMotionLookaheadPlanned(program.count,
                                executeProgramCount,
                                firstId,
                                lastExecuteCommandId,
                                planBuildMs,
                                static_cast<uint32_t>(llround(segmentProgram.totalDurationSec * 1000.0)),
                                g_motionWindowRunner.settings.verifySampledDynamics);

  uint32_t sampleCounter = 0;
  bool aborted = false;
  const char *abortCode = "unknown";
  int32_t activeCommandId = firstId;

  int32_t zeroSteps[kAxisCount] = {};
  double stepsPerDegree[kAxisCount] = {};
  int32_t previousTargetSteps[kAxisCount] = {};
  taskENTER_CRITICAL();
  for (uint8_t i = 0; i < kAxisCount; ++i) {
    zeroSteps[i] = g_zeroSteps[i];
    stepsPerDegree[i] = g_stepsPerDegree[i];
    previousTargetSteps[i] = g_currentSteps[i];
  }
  taskEXIT_CRITICAL();

  RobotMotionCore::StepExecutorStats runStepStats = {};
  RobotMotionCore::StepExecutorStats segmentStepStats = {};
  RobotMotionCore::beginStepExecutorStats(previousTargetSteps, &runStepStats);
  RobotMotionCore::beginStepExecutorStats(previousTargetSteps, &segmentStepStats);

  // Per lookahead window, so a trigger tags exactly one sample. Commands not in this window have no
  // resolved time and are skipped regardless.
  uint8_t triggerFired[RobotMotionCore::kMaxMotionCommands] = {};

  auto enqueuePathSample = [&](const RobotMotionCore::PathSample &sample, bool finalSample) -> bool {
    int32_t targetSteps[kAxisCount] = {};
    double roundedQ[kAxisCount] = {};
    if (!RobotMotionCore::quantizeJointRadiansToSteps(sample.q, zeroSteps, stepsPerDegree, targetSteps, roundedQ)) {
      aborted = true;
      abortCode = "bad_steps_per_deg";
      return false;
    }
    RobotMotionCore::observeStepExecutorTarget(targetSteps, &runStepStats);
    RobotMotionCore::observeStepExecutorTarget(targetSteps, &segmentStepStats);

    MotionExecutionSample executionSample = {};
    executionSample.generation = planGeneration;
    executionSample.dueUs = static_cast<uint32_t>(
        llround((g_motionWindowRunner.timeOffsetSec + sample.timeSec) * 1000000.0));
    for (uint8_t i = 0; i < kAxisCount; ++i) executionSample.targetSteps[i] = targetSteps[i];
    executionSample.commandId = sample.commandId;
    executionSample.completedCommandId = sample.completedCommandId;
    executionSample.finalSample = finalSample ? 1 : 0;
    executionSample.sampleIndex = sampleCounter;

    const int32_t typeCommandId = sample.completedCommandId >= 0 ? sample.completedCommandId : sample.commandId;
    const RobotMotionCore::MotionCommandType type = motionProgramCommandType(program, typeCommandId);
    executionSample.commandType = type == RobotMotionCore::MotionCommandType::MoveL ? 1 : 2;
    executionSample.profileSpeedMmSecX1000 = mmSecX1000(sample.profileSpeed);

    // Emit at most one due trigger per sample, with carried triggers first.
    executionSample.triggerId = -1;
    const double absoluteSec = g_motionWindowRunner.timeOffsetSec + sample.timeSec;
    int32_t pendingId = -1;
    if (RobotMotionCore::takeDuePendingMotionTrigger(&g_pendingTriggers, absoluteSec, &pendingId)) {
      executionSample.triggerId = pendingId;
    } else {
      for (uint8_t c = 0; c < RobotMotionCore::kMaxMotionCommands && c < program.count; ++c) {
        if (triggerFired[c]) continue;
        const double when = segmentProgram.triggerTimeSec[c];
        if (when < 0.0 || sample.timeSec + 1.0e-9 < when) continue;
        if (!RobotMotionCore::motionTriggerIsActive(segmentProgram.commandTrigger[c])) continue;
        // Commands retained for the next window must not fire their resolved triggers here.
        if (program.commands[c].id > lastExecuteCommandId) continue;
        executionSample.triggerId = segmentProgram.commandTrigger[c].id;
        triggerFired[c] = 1;
        break;
      }
    }

    if (readMotionExecutionGeneration() != planGeneration) {
      aborted = true;
      abortCode = "stale_generation";
      return false;
    }
    if (g_motionExecutionQueue == nullptr) {
      aborted = true;
      abortCode = "execution_queue";
      return false;
    }
    while (xQueueSend(g_motionExecutionQueue, &executionSample, pdMS_TO_TICKS(10)) != pdPASS) {
      if (g_stopRequested || readMotionExecutionGeneration() != planGeneration) {
        aborted = true;
        abortCode = g_stopRequested ? "stopped" : "stale_generation";
        return false;
      }
    }
    for (uint8_t i = 0; i < kAxisCount; ++i) {
      previousTargetSteps[i] = targetSteps[i];
    }
    return true;
  };

  // Retire only completed windows; retirement removes the executed command prefix.
  const double controlPeriodSec = g_motionWindowRunner.settings.controlPeriodSec;
  double nextSampleTimeSec = controlPeriodSec;
  bool windowComplete = false;
  RobotMotionCore::PathSample executedPrefixSample = {};
  while (!windowComplete && !g_stopRequested && !aborted) {
    RobotMotionCore::PathSample sample = {};
    streamResult = RobotMotionCore::sampleMotionWindow(&g_motionWindowRunner, nextSampleTimeSec,
                                                      &sample, &windowComplete);
    if (streamResult != RobotMotionCore::RejectCode::Ok) {
      aborted = true;
      abortCode = RobotMotionCore::rejectCodeName(streamResult);
      break;
    }
    nextSampleTimeSec += controlPeriodSec;
    // A window with no duration hands out nothing, and is caught by the prefix check below.
    if (!sample.valid) continue;
    sampleCounter++;
    activeCommandId = sample.commandId;
    // prefixComplete is sticky across samples.
    const bool completesPrefix = g_motionWindowRunner.prefixComplete != 0;
    const bool finalSample = completesPrefix &&
                             g_motionCommandRing.finalCommandQueued &&
                             executeProgramCount >= g_motionCommandRing.count;
    if (!enqueuePathSample(sample, finalSample)) {
      break;
    }
    executedPrefixSample = sample;
  }
  const bool executedPrefixComplete = g_motionWindowRunner.prefixComplete != 0 && !aborted;

  if (!executedPrefixComplete && !aborted && !g_stopRequested) {
    aborted = true;
    abortCode = "segment_failed";
  }

  if (g_stopRequested && !aborted) {
    aborted = true;
    abortCode = "stopped";
  }

  if (aborted) {
    resetMotionRunState();
    const RobotMotionCore::MotionCommandType type = motionProgramCommandType(program, activeCommandId);
    if (type == RobotMotionCore::MotionCommandType::MoveL) {
      enqueueMoveLRunResult("movel_aborted",
                            abortCode,
                            sampleCounter,
                            0,
                            activeCommandId,
                            0,
                            0,
                            0,
                            sampleCounter,
                            0,
                            0,
                            0,
                            false,
                            0,
                            0,
                            "unknown",
                            0,
                            0,
                            0,
                            0,
                            0,
                            0,
                            0,
                            0,
                            0,
                            0,
                            0,
                            0,
                            0,
                            0,
                            0,
                            &segmentStepStats);
    } else {
      enqueueMoveJResult("movej_stopped", abortCode, 0, activeCommandId);
    }
  } else {
    // Read before retirement advances the prefix and time offset.
    const bool hasRemaining = executeProgramCount < g_motionCommandRing.count;

    // Carry unresolved triggers from the executed prefix in absolute run time.
    for (uint8_t c = 0; c < RobotMotionCore::kMaxMotionCommands && c < program.count; ++c) {
      if (triggerFired[c]) continue;
      const double when = segmentProgram.triggerTimeSec[c];
      if (when < 0.0) continue;
      if (!RobotMotionCore::motionTriggerIsActive(segmentProgram.commandTrigger[c])) continue;
      if (program.commands[c].id > lastExecuteCommandId) continue;
      if (!hasRemaining) {
        // A terminal trigger should have been clamped to and fired on the final sample.
        char note[96];
        snprintf(note, sizeof(note),
                 "{\"msg\":\"trigger_failed\",\"id\":%ld,\"code\":\"past_end_of_program\"}",
                 static_cast<long>(segmentProgram.commandTrigger[c].id));
        enqueueTx(note);
        continue;
      }
      RobotMotionCore::pushPendingMotionTrigger(&g_pendingTriggers,
                                                segmentProgram.commandTrigger[c].id,
                                                g_motionWindowRunner.timeOffsetSec + when);
    }

    // Retire from the planned sample; the helper removes weave excursion from the replan seed.
    RobotMotionCore::retireMotionWindow(&g_motionWindowRunner, executedPrefixSample,
                                       executedPrefixSample.q);
  }

  enqueueStatus();
  return !aborted;
}

uint32_t pulseJointUntilLimit(uint8_t joint, uint32_t maxSteps, int32_t requestedDir, uint16_t gapMs, bool *hitLimit) {
  if (hitLimit) {
    *hitLimit = false;
  }
  if (joint < 1 || joint > kAxisCount) {
    return 0;
  }

  const uint8_t index = static_cast<uint8_t>(joint - 1);
  const bool positiveDirection = requestedDir > 0;
  digitalWrite(kJogDirPins[index], positiveDirection != (g_jogDirInvert[index] != 0) ? HIGH : LOW);
  delayMicroseconds(10);

  uint32_t completed = 0;
  for (uint32_t step = 0; step < maxSteps; ++step) {
    if (jogAbortPending()) {
      break;
    }
    if (limitIsActive(joint)) {
      if (hitLimit) {
        *hitLimit = true;
      }
      break;
    }

    digitalWrite(kJogStepPins[index], LOW);
    delayMicroseconds(kJogPulseLowUs);
    digitalWrite(kJogStepPins[index], HIGH);
    completed++;
    applyLogicalStepDelta(joint, requestedDir, 1);
    vTaskDelay(pdMS_TO_TICKS(gapMs));
  }

  digitalWrite(kJogStepPins[index], HIGH);
  if (!jogAbortPending() && limitIsActive(joint) && hitLimit) {
    *hitLimit = true;
  }
  return completed;
}

uint32_t pulseJointUntilLimitReleased(uint8_t joint, uint32_t maxSteps, int32_t requestedDir, uint16_t gapMs, bool *released) {
  if (released) {
    *released = false;
  }
  if (joint < 1 || joint > kAxisCount) {
    return 0;
  }

  const uint8_t index = static_cast<uint8_t>(joint - 1);
  const bool positiveDirection = requestedDir > 0;
  digitalWrite(kJogDirPins[index], positiveDirection != (g_jogDirInvert[index] != 0) ? HIGH : LOW);
  delayMicroseconds(10);

  uint32_t completed = 0;
  for (uint32_t step = 0; step < maxSteps; ++step) {
    if (jogAbortPending()) {
      break;
    }
    if (!limitIsActive(joint)) {
      if (released) {
        *released = true;
      }
      break;
    }

    digitalWrite(kJogStepPins[index], LOW);
    delayMicroseconds(kJogPulseLowUs);
    digitalWrite(kJogStepPins[index], HIGH);
    completed++;
    applyLogicalStepDelta(joint, requestedDir, 1);
    vTaskDelay(pdMS_TO_TICKS(gapMs));
  }

  digitalWrite(kJogStepPins[index], HIGH);
  if (!jogAbortPending() && !limitIsActive(joint) && released) {
    *released = true;
  }
  return completed;
}

void enqueueMasterResult(const char *msg, uint8_t joint, const char *code, uint32_t steps) {
  SerialMessage response = {};
  snprintf(response.text,
           sizeof(response.text),
           "{\"msg\":\"%s\",\"joint\":%u,\"code\":\"%s\",\"steps\":%lu}",
           msg ? msg : "master_failed",
           joint,
           code ? code : "ok",
           static_cast<unsigned long>(steps));
  if (xQueueSend(g_txQueue, &response, 0) != pdPASS) {
    g_txDropped++;
  }
}

uint16_t readBackoffSteps(const SerialMessage &message) {
  int32_t steps = readIntField(message, "\"backoff_steps\"", kMasterBackoffStepsDefault);
  if (steps < kMasterBackoffStepsMin) {
    steps = kMasterBackoffStepsMin;
  }
  if (steps > kMasterBackoffStepsMax) {
    steps = kMasterBackoffStepsMax;
  }
  return static_cast<uint16_t>(steps);
}

double readStepsPerDegree(const SerialMessage &message, uint8_t index) {
  const double fallback = index < kAxisCount ? g_stepsPerDegree[index] : 1.0;
  double value = readFloatField(message, "\"steps_per_deg\"", fallback);
  if (value < 1.0f) {
    value = 1.0f;
  }
  if (value > 10000.0f) {
    value = 10000.0f;
  }
  return value;
}

double clampMasterOffsetDeg(double value) {
  if (value < -180.0) {
    return -180.0;
  }
  if (value > 180.0) {
    return 180.0;
  }
  return value;
}

bool masterJoint(uint8_t joint, double offsetDeg, uint16_t backoffSteps) {
  if (joint < 1 || joint > kAxisCount) {
    enqueueMasterResult("master_failed", joint, "bad_joint", 0);
    return false;
  }

  const uint8_t index = joint - 1;
  const int32_t masterDir = masterDirectionFor(index);
  const int32_t backoffDir = -masterDir;
  const uint32_t fastMaxSteps = static_cast<uint32_t>(axisStepLimit(index)) + static_cast<uint32_t>(backoffSteps);
  const uint32_t slowMaxSteps = static_cast<uint32_t>(backoffSteps) * 3UL;
  uint32_t totalSteps = 0;

  taskENTER_CRITICAL();
  g_masteringActive = true;
  g_masteringJoint = joint;
  taskEXIT_CRITICAL();

  bool released = true;
  if (limitIsActive(joint)) {
    totalSteps += pulseJointUntilLimitReleased(joint, static_cast<uint32_t>(backoffSteps) * 2UL, backoffDir, kMasterSlowStepGapMs, &released);
    if (jogAbortPending()) {
      enqueueMasterResult("master_stopped", joint, "stopped", totalSteps);
      return false;
    }
    if (!released) {
      enqueueMasterResult("master_failed", joint, "limit_stuck", totalSteps);
      return false;
    }
  }

  bool hitLimit = false;
  totalSteps += pulseJointUntilLimit(joint, fastMaxSteps, masterDir, kMasterFastStepGapMs, &hitLimit);
  if (jogAbortPending()) {
    enqueueMasterResult("master_stopped", joint, "stopped", totalSteps);
    return false;
  }
  if (!hitLimit) {
    enqueueMasterResult("master_failed", joint, "fast_no_limit", totalSteps);
    return false;
  }

  totalSteps += pulseJointUntilLimitReleased(joint, backoffSteps, backoffDir, kMasterFastStepGapMs, &released);
  if (jogAbortPending()) {
    enqueueMasterResult("master_stopped", joint, "stopped", totalSteps);
    return false;
  }
  if (!released) {
    enqueueMasterResult("master_failed", joint, "backoff_no_release", totalSteps);
    return false;
  }

  hitLimit = false;
  totalSteps += pulseJointUntilLimit(joint, slowMaxSteps, masterDir, kMasterSlowStepGapMs, &hitLimit);
  if (jogAbortPending()) {
    enqueueMasterResult("master_stopped", joint, "stopped", totalSteps);
    return false;
  }
  if (!hitLimit) {
    enqueueMasterResult("master_failed", joint, "slow_no_limit", totalSteps);
    return false;
  }

  const double limitPositionDeg = masterLimitPositionDeg(index, offsetDeg);
  taskENTER_CRITICAL();
  g_currentSteps[index] = 0;
  g_mastered[index] = 1;
  g_masterOffsetDeg[index] = clampMasterOffsetDeg(offsetDeg);
  updateMasterReference(index, limitPositionDeg);
  taskEXIT_CRITICAL();

  const int32_t targetZeroSteps = axisZeroStep(index);
  const int32_t returnDelta = targetZeroSteps - g_currentSteps[index];
  if (returnDelta != 0) {
    const int32_t returnDir = returnDelta > 0 ? 1 : -1;
    const uint32_t requestedReturnSteps = static_cast<uint32_t>(returnDelta > 0 ? returnDelta : -returnDelta);
    const uint16_t returnGapMs = stepGapMsForJointSpeed(index, kMasterReturnDegPerSec);
    const uint32_t completedReturnSteps = pulseJointStepsAtGap(joint, requestedReturnSteps, returnDir, returnGapMs);
    totalSteps += completedReturnSteps;
    if (jogAbortPending() || completedReturnSteps != requestedReturnSteps) {
      enqueueMasterResult("master_stopped", joint, "return_stopped", totalSteps);
      return false;
    }
  }

  enqueueMasterResult("master_done", joint, "ok", totalSteps);
  return true;
}

void finishMastering() {
  taskENTER_CRITICAL();
  g_masteringActive = false;
  g_masteringJoint = 0;
  taskEXIT_CRITICAL();
}

void handleMasterJoint(const SerialMessage &message) {
  const int32_t joint = readIntField(message, "\"joint\"", 0);
  if (joint < 1 || joint > kAxisCount) {
    enqueueMasterResult("master_failed", static_cast<uint8_t>(joint), "bad_joint", 0);
    return;
  }
  if (!g_jogArmed) {
    enqueueMasterResult("master_failed", static_cast<uint8_t>(joint), "not_armed", 0);
    return;
  }

  g_stopRequested = false;
  g_abortRequested = false;
  const double offsetDeg = readFloatField(message, "\"offset_deg\"", 0.0);
  const uint16_t backoffSteps = readBackoffSteps(message);
  const uint8_t index = static_cast<uint8_t>(joint - 1);
  g_stepsPerDegree[index] = readStepsPerDegree(message, index);
  // Scalar here, not an array: this command addresses one joint. Absent leaves the stored direction
  // alone, and only the sign is used.
  const int32_t requestedDir = readIntField(message, "\"master_dir\"", g_masterDirection[index]);
  g_masterDirection[index] = requestedDir >= 0 ? 1 : -1;
  masterJoint(static_cast<uint8_t>(joint), offsetDeg, backoffSteps);
  g_jogArmed = false;
  finishMastering();
}

void handleSetJointZero(const SerialMessage &message) {
  const int32_t joint = readIntField(message, "\"joint\"", 0);
  if (joint < 1 || joint > kAxisCount) {
    enqueueMasterResult("master_failed", static_cast<uint8_t>(joint), "bad_joint", 0);
    return;
  }
  if (!g_jogArmed) {
    enqueueMasterResult("master_failed", static_cast<uint8_t>(joint), "not_armed", 0);
    return;
  }

  const uint8_t index = static_cast<uint8_t>(joint - 1);
  if (g_mastered[index] == 0) {
    enqueueMasterResult("master_failed", static_cast<uint8_t>(joint), "not_mastered", 0);
    return;
  }

  const double currentOffsetDeg = readFloatField(message, "\"offset_deg\"", g_masterOffsetDeg[index]);
  const double limitDistanceDeg = fabs(masterLimitPositionDeg(index, currentOffsetDeg));
  const uint32_t measuredPulses = static_cast<uint32_t>(labs(g_currentSteps[index] - g_masterLimitSteps[index]));
  if (limitDistanceDeg <= 0.000001 || measuredPulses == 0) {
    enqueueMasterResult("master_failed", static_cast<uint8_t>(joint), "bad_zero_reference", 0);
    return;
  }

  const double solvedStepsPerDegree = static_cast<double>(measuredPulses) / limitDistanceDeg;
  taskENTER_CRITICAL();
  g_stepsPerDegree[index] = solvedStepsPerDegree;
  g_zeroSteps[index] = g_currentSteps[index];
  g_masterLimitDistanceDeg[index] = limitDistanceDeg;
  g_mastered[index] = 1;
  g_masterOffsetDeg[index] = clampMasterOffsetDeg(currentOffsetDeg);
  taskEXIT_CRITICAL();
  g_jogArmed = false;
  enqueueMasterResult("zero_done", static_cast<uint8_t>(joint), "ok", measuredPulses);
}

void handleLoadMastering(const SerialMessage &message) {
  if (g_masteringActive) {
    enqueueTx("{\"msg\":\"mastering_load_failed\",\"code\":\"mastering_active\"}");
    return;
  }

  uint8_t mastered[kAxisCount] = {};
  int32_t currentSteps[kAxisCount] = {};
  int32_t zeroSteps[kAxisCount] = {};
  int32_t masterLimitSteps[kAxisCount] = {};
  double stepsPerDegree[kAxisCount] = {};
  double offsetDeg[kAxisCount] = {};
  double masterLimitDistanceDeg[kAxisCount] = {};
  int8_t masterDirection[kAxisCount] = {};
  uint8_t jogDirInvert[kAxisCount] = {};

  for (uint8_t i = 0; i < kAxisCount; ++i) {
    mastered[i] = readArrayIntField(message, "\"mastered\"", i, g_mastered[i]) == 0 ? 0 : 1;
    currentSteps[i] = readArrayIntField(message, "\"current_steps\"", i, g_currentSteps[i]);
    zeroSteps[i] = readArrayIntField(message, "\"zero_steps\"", i, g_zeroSteps[i]);
    masterLimitSteps[i] = readArrayIntField(message, "\"master_limit_steps\"", i, g_masterLimitSteps[i]);
    const double parsedStepsPerDegree = readArrayDoubleField(message, "\"steps_per_deg\"", i, g_stepsPerDegree[i]);
    stepsPerDegree[i] = parsedStepsPerDegree < 1.0 ? 1.0 : (parsedStepsPerDegree > 10000.0 ? 10000.0 : parsedStepsPerDegree);
    offsetDeg[i] = clampMasterOffsetDeg(readArrayDoubleField(message, "\"offset_deg\"", i, g_masterOffsetDeg[i]));
    // Absent leaves the current direction alone, and only the sign is taken: a zero would mean
    // "do not move", which is never a valid sweep direction.
    masterDirection[i] = readArrayIntField(message, "\"master_dir\"", i, g_masterDirection[i]) >= 0 ? 1 : -1;
    jogDirInvert[i] = readArrayIntField(message, "\"jog_dir_invert\"", i, g_jogDirInvert[i]) != 0 ? 1 : 0;
    masterLimitDistanceDeg[i] = fabs(static_cast<double>(zeroSteps[i] - masterLimitSteps[i])) / stepsPerDegree[i];
  }

  taskENTER_CRITICAL();
  for (uint8_t i = 0; i < kAxisCount; ++i) {
    g_mastered[i] = mastered[i];
    g_currentSteps[i] = currentSteps[i];
    g_zeroSteps[i] = zeroSteps[i];
    g_masterLimitSteps[i] = masterLimitSteps[i];
    g_stepsPerDegree[i] = stepsPerDegree[i];
    g_masterOffsetDeg[i] = offsetDeg[i];
    g_masterLimitDistanceDeg[i] = masterLimitDistanceDeg[i];
    g_masterDirection[i] = masterDirection[i];
    g_jogDirInvert[i] = jogDirInvert[i];
  }
  g_jogArmed = false;
  taskEXIT_CRITICAL();

  enqueueTx("{\"msg\":\"mastering_load_done\"}");
  enqueueStatus();
}

void handleCommand(const SerialMessage &message) {
  if (message.text[0] != '{') {
    enqueueTx("{\"msg\":\"error\",\"code\":\"bad_json\"}");
    return;
  }

  if (commandIs(message, "hello")) {
    // Repeat the boot-time configuration result after the USB host connects.
    SerialMessage reply = {};
    snprintf(reply.text, sizeof(reply.text),
             "{\"msg\":\"hello\",\"fw\":\"ar4_rtos_status\",\"proto\":1,\"boot_config\":\"%s\",\"config_version\":%u}",
             g_configLoadResult == nullptr ? "loaded" : g_configLoadResult,
             static_cast<unsigned>(kConfigVersion));
    xQueueSend(g_txQueue, &reply, 0);
  } else if (commandIs(message, "stream_status")) {
    const bool enabled = readBoolField(message, "\"enabled\"", true);
    uint32_t periodMs = readPeriodMs(message);
    if (periodMs < kMinimumStatusPeriodMs) {
      periodMs = kMinimumStatusPeriodMs;
    }

    g_statusPeriodMs = periodMs;
    g_streamStatus = enabled;
    enqueueTx("{\"msg\":\"stream_status_ack\"}");
  } else if (commandIs(message, "get_status") || commandIs(message, "status")) {
    enqueueStatus();
  } else if (commandIs(message, "set_limit_poll")) {
    uint32_t periodMs = readPeriodMs(message);
    if (periodMs < kMinimumLimitPollPeriodMs) {
      periodMs = kMinimumLimitPollPeriodMs;
    }

    g_limitPollPeriodMs = periodMs;
    enqueueTx("{\"msg\":\"set_limit_poll_ack\"}");
  } else if (commandIs(message, "stop")) {
    requestStop(true);
  } else if (commandIs(message, "master_joint")) {
    handleMasterJoint(message);
  } else if (commandIs(message, "set_joint_zero")) {
    handleSetJointZero(message);
  } else if (commandIs(message, "load_mastering")) {
    handleLoadMastering(message);
  } else if (commandIs(message, "load_robot_model")) {
    handleLoadRobotModel(message);
  } else if (commandIs(message, "get_motion_settings")) {
    enqueueMotionSettings();
  } else if (commandIs(message, "set_motion_settings")) {
    handleSetMotionSettings(message);
  } else if (commandIs(message, "set_weave_schedule")) {
    handleSetWeaveSchedule(message);
  } else if (commandIs(message, "get_weave_schedules")) {
    enqueueWeaveSchedules();
  } else if (commandIs(message, "plan_movej")) {
    handlePlanMoveJ(message);
  } else if (commandIs(message, "movej")) {
    enqueueMotionCommand(message, false);
  } else if (commandIs(message, "plan_movel")) {
    handlePlanMoveL(message);
  } else if (commandIs(message, "movel")) {
    enqueueMotionCommand(message, true);
  } else if (commandIs(message, "save_config")) {
    // Refused while anything is moving: the flash write disables interrupts, and a sector erase
    // takes tens of milliseconds, which would wreck step timing mid-move.
    if (motionSettingsMutationBusy()) {
      enqueueTx("{\"msg\":\"config_save_failed\",\"code\":\"busy\"}");
    } else {
      savePersistentConfig();
      g_configStored = true;
      enqueueTx("{\"msg\":\"config_saved\"}");
      enqueueStatus();
    }
  } else if (commandIs(message, "load_config")) {
    if (motionSettingsMutationBusy()) {
      enqueueTx("{\"msg\":\"config_load_failed\",\"code\":\"busy\"}");
    } else {
      const char *result = loadPersistentConfig();
      if (result == nullptr) {
        enqueueTx("{\"msg\":\"config_loaded\",\"source\":\"eeprom\"}");
      } else {
        SerialMessage note = {};
        snprintf(note.text, sizeof(note.text), "{\"msg\":\"config_load_failed\",\"code\":\"%s\"}", result);
        xQueueSend(g_txQueue, &note, 0);
      }
      enqueueStatus();
    }
  } else if (commandIs(message, "clear_config")) {
    if (motionSettingsMutationBusy()) {
      enqueueTx("{\"msg\":\"config_clear_failed\",\"code\":\"busy\"}");
    } else {
      clearPersistentConfig();
      g_configStored = false;
      enqueueTx("{\"msg\":\"config_cleared\"}");
      enqueueStatus();
    }
  } else if (commandIs(message, "jog_arm")) {
    const bool enabled = readBoolField(message, "\"enabled\"", true);
    if (enabled) {
      g_stopRequested = false;
      g_abortRequested = false;
    }
    g_jogArmed = enabled;
    enqueueTx(enabled
                  ? "{\"msg\":\"jog_arm_ack\",\"armed\":true,\"allowed_joints\":[1,2,3,4,5,6],\"max_steps\":500}"
                  : "{\"msg\":\"jog_disarm_ack\",\"armed\":false}");
  } else if (commandIs(message, "jog_disarm")) {
    g_jogArmed = false;
    enqueueTx("{\"msg\":\"jog_disarm_ack\",\"armed\":false}");
  } else if (commandIs(message, "jog_joint")) {
    handleJogJoint(message);
  } else {
    enqueueTx("{\"msg\":\"error\",\"code\":\"unknown_cmd\"}");
  }
}

void serialIoTask(void *) {
  SerialMessage message = {};
  SerialMessage txMessage = {};
  size_t used = 0;

  while (true) {
    while (Serial.available() > 0) {
      const char c = static_cast<char>(Serial.read());
      if (c == '\r') {
        continue;
      }

      if (c == '\n') {
        if (used > 0) {
          message.text[used] = '\0';

          // Checked before anything acts on the line, stop included: a corrupted line is not
          // trustworthy enough to halt the arm on, and the host resends within milliseconds.
          size_t payloadLength = used;
          const SerialFraming::CheckResult check =
              SerialFraming::verifyLine(message.text, used, &payloadLength);
          if (check == SerialFraming::CheckResult::Invalid) {
            g_rxBadChecksum++;
            // No sequence number is quoted: the bytes that would carry it are the ones in doubt.
            // The host resends whatever it last sent, and the sequence number sorts it out there.
            enqueueTx("{\"msg\":\"nak\",\"seq\":-1,\"code\":\"bad_checksum\"}");
            memset(&message, 0, sizeof(message));
            used = 0;
            continue;
          }
          // Drop the suffix so the field parser and every handler below see only the JSON.
          message.text[payloadLength] = '\0';

          // A hello starts a new sequence-number session and must be handled before duplicate checks.
          if (commandIs(message, "hello")) {
            g_lastCommandSeq = -1;
          }

          const int32_t seq = readIntField(message, "\"seq\"", -1);
          if (seq >= 0 && seq == g_lastCommandSeq) {
            // Already executed. Acknowledged again so the host stops retrying, but deliberately
            // not run a second time.
            char ack[64];
            snprintf(ack, sizeof(ack), "{\"msg\":\"ack\",\"seq\":%ld,\"dup\":1}",
                     static_cast<long>(seq));
            enqueueTx(ack);
            memset(&message, 0, sizeof(message));
            used = 0;
            continue;
          }

          bool accepted = false;
          if (commandIs(message, "stop")) {
            requestStop(true);
            accepted = true;
          } else if (xQueueSend(g_commandQueue, &message, 0) == pdPASS) {
            accepted = true;
          } else {
            g_rxDropped++;
          }

          if (seq >= 0) {
            char reply[72];
            if (accepted) {
              g_lastCommandSeq = seq;
              snprintf(reply, sizeof(reply), "{\"msg\":\"ack\",\"seq\":%ld}", static_cast<long>(seq));
            } else {
              snprintf(reply, sizeof(reply), "{\"msg\":\"nak\",\"seq\":%ld,\"code\":\"queue_full\"}",
                       static_cast<long>(seq));
            }
            enqueueTx(reply);
          }

          memset(&message, 0, sizeof(message));
          used = 0;
        }
        continue;
      }

      if (used < sizeof(message.text) - 1) {
        message.text[used++] = c;
      } else {
        g_rxDropped++;
        memset(&message, 0, sizeof(message));
        used = 0;
        enqueueTx("{\"msg\":\"error\",\"code\":\"rx_line_too_long\"}");
      }
    }

    while (xQueueReceive(g_txQueue, &txMessage, 0) == pdPASS) {
      char suffix[SerialFraming::kChecksumTextLength + 2];
      SerialFraming::formatChecksumSuffix(txMessage.text, strlen(txMessage.text), suffix, sizeof(suffix));
      Serial.print(txMessage.text);
      Serial.println(suffix);
    }

    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

void commandTask(void *) {
  SerialMessage message = {};

  while (true) {
    if (xQueueReceive(g_commandQueue, &message, portMAX_DELAY) == pdPASS) {
      handleCommand(message);
    }
  }
}

void motionExecutionTask(void *) {
  MotionExecutionSample sample = {};
  bool active = false;
  uint32_t runStartUs = 0;
  uint32_t runStartMs = 0;
  uint32_t previousDeadlineUs = 0;
  uint32_t activeGeneration = 0;
  uint32_t lastStepUs[kAxisCount] = {};
  int32_t actualSteps[kAxisCount] = {};
  int32_t previousTargetSteps[kAxisCount] = {};
  uint32_t totalTicks = 0;
  RobotMotionCore::StepExecutorStats runStepStats = {};
  RobotMotionCore::StepExecutorStats segmentStepStats = {};

  auto finishActiveRun = [&]() {
    for (uint8_t i = 0; i < kAxisCount; ++i) {
      digitalWrite(kJogStepPins[i], HIGH);
    }
    taskENTER_CRITICAL();
    g_moveActive = false;
    taskEXIT_CRITICAL();
    activeGeneration = 0;
    active = false;
  };

  while (true) {
    if (xQueueReceive(g_motionExecutionQueue, &sample, portMAX_DELAY) != pdPASS) {
      continue;
    }

    const uint32_t currentGeneration = readMotionExecutionGeneration();
    if (sample.generation != currentGeneration) {
      if (active) finishActiveRun();
      continue;
    }
    if (active && sample.generation != activeGeneration) {
      finishActiveRun();
    }

    if (!active) {
      taskENTER_CRITICAL();
      for (uint8_t i = 0; i < kAxisCount; ++i) {
        actualSteps[i] = g_currentSteps[i];
        previousTargetSteps[i] = g_currentSteps[i];
      }
      g_moveActive = true;
      taskEXIT_CRITICAL();
      runStartUs = micros();
      runStartMs = millis();
      previousDeadlineUs = runStartUs;
      for (uint8_t i = 0; i < kAxisCount; ++i) lastStepUs[i] = runStartUs;
      totalTicks = 0;
      RobotMotionCore::beginStepExecutorStats(previousTargetSteps, &runStepStats);
      RobotMotionCore::beginStepExecutorStats(previousTargetSteps, &segmentStepStats);
      activeGeneration = sample.generation;
      active = true;
    }

    RobotMotionCore::observeStepExecutorTarget(sample.targetSteps, &runStepStats);
    RobotMotionCore::observeStepExecutorTarget(sample.targetSteps, &segmentStepStats);

    const uint32_t deadlineUs = runStartUs + sample.dueUs;
    const char *abortCode = "unknown";
    if (!followQueuedStepTarget(previousTargetSteps,
                                sample.targetSteps,
                                actualSteps,
                                lastStepUs,
                                previousDeadlineUs,
                                deadlineUs,
                                sample.generation,
                                &totalTicks,
                                &abortCode)) {
      if (sample.commandType == 1) {
        enqueueMoveLRunResult("movel_aborted",
                              abortCode,
                              sample.sampleIndex,
                              totalTicks,
                              sample.commandId,
                              millis() - runStartMs,
                              sample.dueUs / 1000,
                              0,
                              sample.sampleIndex,
                              0,
                              0,
                              runStepStats.zeroStepSamples,
                              false,
                              0,
                              0,
                              "queued_sample",
                              sample.profileSpeedMmSecX1000,
                              0,
                              0,
                              0,
                              0,
                              0,
                              0,
                              0,
                              0,
                              0,
                              0,
                              0,
                              0,
                              0,
                              0,
                              &segmentStepStats);
      } else {
        enqueueMoveJResult("movej_stopped", abortCode, totalTicks, sample.commandId);
      }
      if (g_motionExecutionQueue != nullptr) xQueueReset(g_motionExecutionQueue);
      finishActiveRun();
      g_stopRequested = false;
      continue;
    }

    for (uint8_t i = 0; i < kAxisCount; ++i) previousTargetSteps[i] = sample.targetSteps[i];
    previousDeadlineUs = deadlineUs;

    // Report triggers at execution time using the status frame's step units.
    if (sample.triggerId >= 0) {
      char line[240];
      const int written = snprintf(line, sizeof(line),
          "{\"msg\":\"trigger\",\"id\":%ld,\"millis\":%lu,\"estop\":%u,"
          "\"limits\":[%u,%u,%u,%u,%u,%u,%u,%u,%u],"
          "\"steps\":[%ld,%ld,%ld,%ld,%ld,%ld]}",
          static_cast<long>(sample.triggerId),
          static_cast<unsigned long>(millis()),
          // Read from the pin, the same way the status frame does; there is no cached global.
          digitalRead(kEstopPin) == LOW ? 1u : 0u,
          g_limitActive[0], g_limitActive[1], g_limitActive[2],
          g_limitActive[3], g_limitActive[4], g_limitActive[5],
          g_limitActive[6], g_limitActive[7], g_limitActive[8],
          static_cast<long>(actualSteps[0]), static_cast<long>(actualSteps[1]),
          static_cast<long>(actualSteps[2]), static_cast<long>(actualSteps[3]),
          static_cast<long>(actualSteps[4]), static_cast<long>(actualSteps[5]));
      if (written > 0 && static_cast<size_t>(written) < sizeof(line)) {
        enqueueTx(line);
      } else {
        enqueueTx("{\"msg\":\"trigger_failed\",\"code\":\"reply_too_long\"}");
      }
    }

    if (sample.completedCommandId >= 0) {
      if (sample.commandType == 1) {
        enqueueMoveLRunResult("movel_done",
                              "ok",
                              sample.sampleIndex,
                              totalTicks,
                              sample.completedCommandId,
                              millis() - runStartMs,
                              sample.dueUs / 1000,
                              0,
                              sample.sampleIndex,
                              0,
                              0,
                              runStepStats.zeroStepSamples,
                              false,
                              0,
                              0,
                              "queued_sample",
                              sample.profileSpeedMmSecX1000,
                              0,
                              0,
                              0,
                              0,
                              0,
                              0,
                              0,
                              0,
                              0,
                              0,
                              0,
                              0,
                              0,
                              0,
                              &segmentStepStats);
      } else {
        enqueueMoveJResult("movej_done", "ok", totalTicks, sample.completedCommandId);
      }
      RobotMotionCore::resetStepExecutorStats(&segmentStepStats);
    }

    if (sample.finalSample) {
      finishActiveRun();
    }
  }
}

void motionTask(void *) {
  SerialMessage message = {};

  while (true) {
    memset(&message, 0, sizeof(message));
    if (RobotMotionCore::motionLookaheadExecutableCount(g_motionCommandRing) == 0) {
      if (xQueueReceive(g_motionQueue, &message, portMAX_DELAY) != pdPASS) {
        continue;
      }
    }

    if (g_stopRequested) {
      bumpMotionExecutionGeneration();
      if (g_motionQueue != nullptr) {
        xQueueReset(g_motionQueue);
      }
      if (g_motionExecutionQueue != nullptr) {
        xQueueReset(g_motionExecutionQueue);
      }
      resetMotionRunState();
      g_stopRequested = false;
    }

    bool completed = true;
    if (message.text[0] == '\0' || commandIs(message, "movej") || commandIs(message, "movel")) {
      completed = executeMotionProgramFromQueue(message);
    }

    if ((!completed || g_stopRequested) && g_motionQueue != nullptr) {
      bumpMotionExecutionGeneration();
      xQueueReset(g_motionQueue);
      if (g_motionExecutionQueue != nullptr) {
        xQueueReset(g_motionExecutionQueue);
      }
      resetMotionRunState();
    }
    if (g_stopRequested) {
      g_stopRequested = false;
    }
  }
}

void statusTask(void *) {
  TickType_t lastWake = xTaskGetTickCount();

  while (true) {
    if (g_streamStatus && !g_moveActive) {
      enqueueStatus();
    }

    const uint32_t periodMs = g_streamStatus ? g_statusPeriodMs : kDefaultStatusPeriodMs;
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(periodMs));
  }
}

void limitPollTask(void *) {
  TickType_t lastWake = xTaskGetTickCount();
  uint32_t rateWindowStartMs = millis();
  uint32_t rateWindowStartCount = 0;
  bool initialized = false;

  while (true) {
    uint8_t raw[kJointCount] = {};
    uint8_t active[kJointCount] = {};
    const uint32_t now = millis();

    for (uint8_t i = 0; i < kJointCount; ++i) {
      const int rawState = digitalRead(kLimitPins[i]);
      raw[i] = rawState == HIGH ? 1 : 0;
      active[i] = rawState == kLimitActiveState[i] ? 1 : 0;
    }

    taskENTER_CRITICAL();
    for (uint8_t i = 0; i < kJointCount; ++i) {
      if (initialized && raw[i] != g_limitRaw[i]) {
        g_limitChangeCount[i]++;
        g_limitLastChangeMs[i] = now;
      }
      g_limitRaw[i] = raw[i];
      g_limitActive[i] = active[i];
    }
    initialized = true;
    g_limitPollCount++;

    const uint32_t elapsedMs = now - rateWindowStartMs;
    if (elapsedMs >= 1000) {
      const uint32_t polls = g_limitPollCount - rateWindowStartCount;
      g_limitPollMeasuredHzX100 = (polls * 100000UL) / elapsedMs;
      rateWindowStartMs = now;
      rateWindowStartCount = g_limitPollCount;
    }
    taskEXIT_CRITICAL();

    const uint32_t periodMs = g_limitPollPeriodMs < kMinimumLimitPollPeriodMs
                                  ? kMinimumLimitPollPeriodMs
                                  : g_limitPollPeriodMs;
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(periodMs));
  }
}

void heartbeatTask(void *) {
  pinMode(kHeartbeatPin, OUTPUT);
  bool on = false;

  while (true) {
    on = !on;
    digitalWrite(kHeartbeatPin, on ? HIGH : LOW);
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

void configurePins() {
  for (uint8_t i = 0; i < kJointCount; ++i) {
    pinMode(kLimitPins[i], kLimitPinModes[i]);
  }

  pinMode(kEstopPin, INPUT_PULLUP);
  for (uint8_t i = 0; i < kAxisCount; ++i) {
    pinMode(kJogStepPins[i], OUTPUT);
    pinMode(kJogDirPins[i], OUTPUT);
    digitalWrite(kJogStepPins[i], HIGH);
  }
}

void initializeMasteringState() {
  for (uint8_t i = 0; i < kAxisCount; ++i) {
    g_currentSteps[i] = 0;
    g_masterLimitSteps[i] = 0;
    g_masterLimitDistanceDeg[i] = fabs(masterLimitPositionDeg(i, g_masterOffsetDeg[i]));
    g_zeroSteps[i] = static_cast<int32_t>(llround(kAxisLimitNegDeg[i] * g_stepsPerDegree[i]));
  }
}

void startTask(TaskFunction_t task, const char *name, uint16_t stackWords, UBaseType_t priority) {
  if (xTaskCreate(task, name, stackWords, nullptr, priority, nullptr) != pdPASS) {
    Serial.print("{\"msg\":\"fatal\",\"code\":\"task_create_failed\",\"task\":\"");
    Serial.print(name);
    Serial.println("\"}");
    while (true) {
      delay(100);
    }
  }
}


// Persistent robot calibration, motion settings, and kinematics in emulated EEPROM.
// Flash writes disable interrupts, so saving is prohibited while motion is active.
struct PersistentConfig {
  uint32_t magic;
  uint16_t version;
  uint16_t length;
  uint32_t crc;  // over every byte that follows

  // Per-joint calibration.
  double stepsPerDegree[kAxisCount];
  double masterOffsetDeg[kAxisCount];
  int32_t zeroSteps[kAxisCount];
  int32_t currentSteps[kAxisCount];
  int32_t masterLimitSteps[kAxisCount];
  uint8_t mastered[kAxisCount];
  int8_t masterDirection[kAxisCount];
  uint8_t jogDirInvert[kAxisCount];
  uint8_t reservedA[2];

  // Planner limits and defaults.
  RobotMotionCore::MotionProgramSettings settings;

  // Kinematics. Stored whole, so the arm is drivable straight out of a power cycle.
  uint8_t modelValid;
  uint8_t reservedB[7];
  RobotMotionCore::RobotModel model;

  // Welding schedules. Stored for the same reason as the model: a program that says WeaveOn 3 is
  // refused with weave_schedule_missing until something supplies schedule 3, so a robot that has
  // been power cycled could not run a weaving program without a PC to re-send them.
  RobotMotionCore::WeaveScheduleTable weaveSchedules;
};

// The emulated EEPROM is 4284 bytes; this must fit with room for later fields.
static_assert(sizeof(PersistentConfig) <= 3072, "PersistentConfig outgrew its EEPROM budget");

// Bitwise CRC32, no table: this runs twice per save and once at boot, so the few microseconds cost
// nothing and 1 KB of table would be wasted RAM.
uint32_t configCrc32(const uint8_t *data, size_t length) {
  uint32_t crc = 0xFFFFFFFFUL;
  for (size_t i = 0; i < length; ++i) {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8; ++bit) {
      crc = (crc >> 1) ^ (0xEDB88320UL & (~(crc & 1) + 1));
    }
  }
  return ~crc;
}

// The CRC covers everything after the crc field itself, so the header's own bytes are checked by the
// magic, version and length instead.
uint32_t configPayloadCrc(const PersistentConfig &config) {
  const uint8_t *base = reinterpret_cast<const uint8_t *>(&config);
  const size_t offset = offsetof(PersistentConfig, crc) + sizeof(config.crc);
  return configCrc32(base + offset, sizeof(PersistentConfig) - offset);
}

void captureConfig(PersistentConfig *config) {
  if (config == nullptr) return;
  memset(config, 0, sizeof(*config));
  config->magic = kConfigMagic;
  config->version = kConfigVersion;
  config->length = static_cast<uint16_t>(sizeof(PersistentConfig));
  for (uint8_t i = 0; i < kAxisCount; ++i) {
    config->stepsPerDegree[i] = g_stepsPerDegree[i];
    config->masterOffsetDeg[i] = g_masterOffsetDeg[i];
    config->zeroSteps[i] = g_zeroSteps[i];
    config->currentSteps[i] = g_currentSteps[i];
    config->masterLimitSteps[i] = g_masterLimitSteps[i];
    config->mastered[i] = g_mastered[i];
    config->masterDirection[i] = static_cast<int8_t>(g_masterDirection[i] >= 0 ? 1 : -1);
    config->jogDirInvert[i] = g_jogDirInvert[i] ? 1 : 0;
  }
  config->settings = g_motionSettings;
  config->modelValid = g_motionModelLoaded && RobotMotionCore::modelIsValid(g_motionModel) ? 1 : 0;
  config->model = g_motionModel;
  config->weaveSchedules = g_weaveSchedules;
  config->crc = configPayloadCrc(*config);
}

// Applied through the same clamps the wire protocol uses, so a plausible-looking but out-of-range
// stored value cannot put the firmware somewhere the commands could not.
void applyConfig(const PersistentConfig &config) {
  for (uint8_t i = 0; i < kAxisCount; ++i) {
    const double stepsPerDeg = config.stepsPerDegree[i];
    g_stepsPerDegree[i] = stepsPerDeg < 1.0 ? 1.0 : (stepsPerDeg > 10000.0 ? 10000.0 : stepsPerDeg);
    g_masterOffsetDeg[i] = clampMasterOffsetDeg(config.masterOffsetDeg[i]);
    g_zeroSteps[i] = config.zeroSteps[i];
    g_currentSteps[i] = config.currentSteps[i];
    g_masterLimitSteps[i] = config.masterLimitSteps[i];
    g_mastered[i] = config.mastered[i] ? 1 : 0;
    g_masterDirection[i] = config.masterDirection[i] >= 0 ? 1 : -1;
    g_jogDirInvert[i] = config.jogDirInvert[i] ? 1 : 0;
    g_masterLimitDistanceDeg[i] =
        fabs(static_cast<double>(g_zeroSteps[i] - g_masterLimitSteps[i])) / g_stepsPerDegree[i];
  }

  g_motionSettings = config.settings;
  g_motionSettingsLoaded = true;
  g_weaveSchedules = config.weaveSchedules;
  if (config.modelValid && RobotMotionCore::modelIsValid(config.model)) {
    g_motionModel = config.model;
    g_motionModelLoaded = true;
    RobotMotionCore::applyControllerStepLimitsToMotionSettings(&g_motionSettings, g_stepsPerDegree,
                                                               kMoveJMinTickGapUs);
  }
}

// "not_stored" when the magic is absent, which is the state of a board that has never saved, and is
// not an error. Anything else means stored bytes that cannot be trusted, and the compiled defaults
// stand instead.
const char *loadPersistentConfig() {
  PersistentConfig config = {};
  eeprom_read_block(&config, reinterpret_cast<const void *>(kConfigEepromAddress), sizeof(config));
  if (config.magic != kConfigMagic) return "not_stored";

  // Discard incompatible or corrupt blocks so config_stored reflects usable data.
  const char *reason = nullptr;
  if (config.version != kConfigVersion) {
    reason = "bad_version";
  } else if (config.length != static_cast<uint16_t>(sizeof(PersistentConfig))) {
    reason = "bad_length";
  } else if (config.crc != configPayloadCrc(config)) {
    reason = "bad_crc";
  }
  if (reason != nullptr) {
    clearPersistentConfig();
    return reason;
  }

  applyConfig(config);
  return nullptr;
}

bool persistentConfigStored() {
  uint32_t magic = 0;
  eeprom_read_block(&magic, reinterpret_cast<const void *>(kConfigEepromAddress), sizeof(magic));
  return magic == kConfigMagic;
}

void savePersistentConfig() {
  PersistentConfig config = {};
  captureConfig(&config);
  eeprom_write_block(&config, reinterpret_cast<void *>(kConfigEepromAddress), sizeof(config));
}

// Only the magic is cleared. Rewriting the whole block with zeroes would cost an erase of every
// sector for no benefit; without a valid magic the rest is never read.
void clearPersistentConfig() {
  const uint32_t magic = 0;
  eeprom_write_block(&magic, reinterpret_cast<void *>(kConfigEepromAddress), sizeof(magic));
}

}  // namespace

void setup() {
  Serial.begin(kSerialBaud);
  configurePins();
  initializeMasteringState();
  g_motionSettings = defaultAr4MotionSettings();
  // Before the tasks start, so the first status frame already reflects the saved calibration and a
  // stored model makes the arm drivable without a PC having connected.
  g_configLoadResult = loadPersistentConfig();
  g_configStored = persistentConfigStored();
  g_commandQueue = xQueueCreate(kQueueDepth, sizeof(SerialMessage));
  g_motionQueue = xQueueCreate(kMotionQueueDepth, sizeof(SerialMessage));
  g_motionExecutionQueue = xQueueCreate(kMotionExecutionQueueDepth, sizeof(MotionExecutionSample));
  g_txQueue = xQueueCreate(kQueueDepth, sizeof(SerialMessage));
  if (g_commandQueue == nullptr || g_motionQueue == nullptr || g_motionExecutionQueue == nullptr || g_txQueue == nullptr) {
    Serial.println("{\"msg\":\"fatal\",\"code\":\"queue_create_failed\"}");
    while (true) {
      delay(100);
    }
  }

  startTask(serialIoTask, "serial_io", 2048, 5);
  startTask(commandTask, "command", 2048, 2);
  startTask(motionExecutionTask, "motion_exec", 4096, 4);
  startTask(motionTask, "motion_plan", 8192, 2);
  startTask(limitPollTask, "limit_poll", 512, 3);
  startTask(statusTask, "status", 2048, 1);
  startTask(heartbeatTask, "heartbeat", configMINIMAL_STACK_SIZE, 1);

  if (g_configLoadResult == nullptr) {
    enqueueTx("{\"msg\":\"config_loaded\",\"source\":\"eeprom\"}");
  } else if (strcmp(g_configLoadResult, "not_stored") != 0) {
    // Distinguished from a board that has simply never saved: stored-but-unusable means the
    // compiled defaults are in force and the operator needs to know the arm is not calibrated.
    SerialMessage note = {};
    snprintf(note.text, sizeof(note.text), "{\"msg\":\"config_load_failed\",\"code\":\"%s\"}",
             g_configLoadResult);
    xQueueSend(g_txQueue, &note, 0);
  }
  enqueueTx("{\"msg\":\"rtos_ready\",\"fw\":\"ar4_rtos_status\",\"proto\":1}");
  vTaskStartScheduler();

  Serial.println("{\"msg\":\"fatal\",\"code\":\"scheduler_failed\"}");
  while (true) {
    delay(100);
  }
}

void loop() {
}
