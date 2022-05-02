#include <Arduino.h>

#include <SPI.h>
#include <mcp2515.h>
#include <EEPROM.h>

//#define LEONARDO
//#define ID_CAN_FILTER 0x10C
//#define INTERRUPT_PIN 2

#define CS_PIN 10
#define LIGHT_PIN 4
#define REMOTE_PIN 5

#define MILLIS_TAPS 325

#define LIGHT_MODE_OFF 0
#define LIGHT_MODE_ALWAYS 1
#define LIGHT_MODE_HIGH 2

#define REMOTE_MODE_CONTINUOUS 0
#define REMOTE_MODE_BLINK 1

#define REMOTE_ON_MILLIS 7500

//#define STATUS
//#define TEST
//#define DEBUG
//#define INFO
#define VERBOSE

void initSerial();
void initCanBus();
void initStates();

void eepromRecovery();
void eepromSave(int address, int value);

void statusReport();
void processCanBusInfoStates(struct can_frame frame2process, unsigned long newMillis);
void processCanBus();
void processCanBusMessage(struct can_frame frame2process);
void processCanBusTurnStates(struct can_frame frame2process, unsigned long newMillis);
void processCanBusHighBeamStates(struct can_frame frame2process, unsigned long newMillis);
void processCanBusEnineRPM(struct can_frame frame2process, unsigned long newMillis);
void processActionsInfo(unsigned long newMillis);
void processActionsEngineStart(unsigned long newMillis);
void processActionsTurn(unsigned long newMillis);
void processActionsHighBeam(unsigned long newMillis);
bool checkCanBusDataMask(struct can_frame frameCheck, uint32_t can_id, int pos, byte mask, byte value_mask, bool &value);
bool getCanBusData(struct can_frame frameCheck, uint32_t can_id, int pos, byte &value);
void relay(int pin, bool on);

void irqHandler();
void processCanBusInterrupt();

void serialDataOutput(struct can_frame frameOutput);

enum class turn_states
{
  OFF = 'O',
  LEFT = 'L',
  RIGHT = 'R',
  BOTH = 'B'
};

enum class info_states
{
  OFF = 'O',
  SHORT = 'S',
  LONG = 'L'
};

enum class high_light_states
{
  OFF = 'O',
  ON = 'I'
};

struct turn_state
{
  unsigned long millis;
  unsigned long previous_interval_millis;
  turn_states state;
  turn_states previous_state;
  bool processed;
};

struct info_state
{
  unsigned long millis;
  unsigned long previous_interval_millis;
  info_states state;
  info_states previous_state;
  bool processed;
};

struct high_light_state
{
  unsigned long millis;
  unsigned long previous_interval_millis;
  high_light_states state;
  high_light_states previous_state;
  bool processed;
};

struct engine_state_values
{
  bool started;
  uint32_t rpm;
};

struct engine_state
{
  unsigned long millis;
  unsigned long previous_interval_millis;
  engine_state_values state;
  engine_state_values previous_state;
  bool processed;
};

// Message IDs
const uint32_t ID_CAN_LIGHTS = 0x130;
const uint32_t ID_CAN_CONTROLS = 0x2D0;
const uint32_t ID_CAN_ENGINE = 0x10C;

// A bitmasks for toggle bits.
const byte VALUE_CAN_TURN_OFF = 0b00001000;
const byte VALUE_CAN_TURN_LEFT = 0b00010000;
const byte VALUE_CAN_TURN_RIGHT = 0b00100000;
const byte VALUE_CAN_TURN_BOTHT = 0b00111000;
const byte VALUE_CAN_TURN_MASK = 0b00111000;

const byte VALUE_CAN_HIGH_LIGHT_ON = 0b00000001;
const byte VALUE_CAN_HIGH_LIGHT_OFF = 0b00000010;
const byte VALUE_CAN_HIGH_LIGHT_MASK = 0b00000011;

const byte VALUE_CAN_INFO_SHORT = 0b00000001;
const byte VALUE_CAN_INFO_LONG = 0b00000010;
const byte VALUE_CAN_INFO_OFF = 0b00000000;
const byte VALUE_CAN_INFO_MASK = 0b00000011;

turn_state turn_state_current;
info_state info_state_current;
high_light_state high_light_state_current;
engine_state engine_state_current;

unsigned int light_mode_address = 0;
byte light_mode = LIGHT_MODE_OFF;
bool light_on = false;

unsigned int remote_modes_address = 0;
byte remote_mode = REMOTE_MODE_CONTINUOUS;

bool turn_left_on = false;
bool turn_right_on = false;
bool turn_both_on = false;

unsigned long statusMillis = 0;
unsigned int totalInfoShorts = 0;
unsigned int totalHighBeamShorts = 0;

unsigned long startRemoteMillis = 0;

MCP2515 bus(CS_PIN);

void setup()
{
  initSerial();
  eepromRecovery();
  initCanBus();
  initStates();

  if (light_mode == LIGHT_MODE_ALWAYS)
    light_on = true;

  pinMode(LIGHT_PIN, OUTPUT);
  pinMode(REMOTE_PIN, OUTPUT);

  relay(REMOTE_PIN, false);
  relay(LIGHT_PIN, false);

}

void loop()
{
#ifdef STATUS
  statusReport();
#endif

#ifdef INTERRUPT_PIN
  processCanBusInterrupt();
#else
  processCanBus();
#endif

  if (light_on && engine_state_current.state.started)
    relay(LIGHT_PIN, true);
  else
    relay(LIGHT_PIN, false);


  if (startRemoteMillis > 0 && ((millis() - startRemoteMillis) <= REMOTE_ON_MILLIS))
  {
    if (remote_mode == REMOTE_MODE_CONTINUOUS)
      relay(REMOTE_PIN, true);
    else if (remote_mode == REMOTE_MODE_BLINK && ((int)(millis() - startRemoteMillis / 1000)) % 2 == 0)
      relay(REMOTE_PIN, true);
    else
      relay(REMOTE_PIN, false);
  }
  else
  {
    relay(REMOTE_PIN, false);
    startRemoteMillis = 0;
  }

#ifdef TEST
  writeTest();
#endif
}

void initSerial()
{
  Serial.begin(115200);
#ifdef LEONARDO
  while (!Serial);
#endif
}

void initCanBus()
{
  bus.reset();
  bus.setBitrate(CAN_500KBPS, MCP_8MHZ);

#ifdef TEST
  bus.setLoopbackMode();
#else
  //bus.setNormalMode();
  bus.setListenOnlyMode();
#endif

#ifdef INTERRUPT_PIN
  pinMode(INTERRUPT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), irqHandler, FALLING);
#endif
}

void initStates()
{
  // Init the structs for states of Can Bus Elements monitoring
  high_light_state_current.millis = 0;
  high_light_state_current.previous_interval_millis = 0;
  high_light_state_current.state = high_light_states::OFF;
  high_light_state_current.previous_state = high_light_states::OFF;
  high_light_state_current.processed = true;

  info_state_current.millis = 0;
  info_state_current.previous_interval_millis = 0;
  info_state_current.state = info_states::OFF;
  info_state_current.previous_state = info_states::OFF;
  info_state_current.processed = true;

  turn_state_current.millis = 0;
  turn_state_current.previous_interval_millis = 0;
  turn_state_current.state = turn_states::OFF;
  turn_state_current.previous_state = turn_states::OFF;
  info_state_current.processed = true;

  engine_state_current.millis = 0;
  engine_state_current.previous_interval_millis = 0;
  engine_state_current.state.started = false;
  engine_state_current.state.rpm = 0;
  engine_state_current.previous_state.started = false;
  engine_state_current.previous_state.rpm = 0;
  engine_state_current.processed = true;
}

void eepromRecovery()
{

  if (EEPROM.length() == 0)
  {
    EEPROM.update(light_mode_address, light_mode);
    remote_modes_address = light_mode_address + sizeof(light_mode);
    EEPROM.update(remote_modes_address, remote_mode);
  }
  else
  {
    EEPROM.get(light_mode_address, light_mode);
    remote_modes_address = light_mode_address + sizeof(light_mode);
    EEPROM.get(remote_modes_address, remote_mode);
  }
}

void eepromSave(int address, int value)
{
#ifdef INFO
  Serial.print("Save in EEPROM(0x");
  Serial.print(address, HEX);
  Serial.print(") -> ");
  Serial.println(value, DEC);
  Serial.println("------------------------------------------------------------------------");
#endif

  EEPROM.update(address, value);
  digitalWrite(LIGHT_PIN, LOW);
  delay(500);
  digitalWrite(LIGHT_PIN, HIGH);
  delay(250);
  digitalWrite(LIGHT_PIN, LOW);
  delay(500);
  digitalWrite(LIGHT_PIN, HIGH);
  delay(250);
}

void relay(int pin, bool on)
{
  if (on)
    digitalWrite(pin, LOW);
  else
    digitalWrite(pin, HIGH);
}

#ifdef STATUS
void statusReport()
{
  if (!((millis() - statusMillis) > 1000))
    return;

  statusMillis = millis();
  Serial.print("Turn: ");
  if (turn_left_on)
    Serial.println("left");
  else if (turn_right_on)
    Serial.println("right");
  else if (turn_both_on)
    Serial.println("both");
  else
    Serial.println("off");

  Serial.print("Aux lights mode: ");
  if (light_mode == LIGHT_MODE_OFF)
    Serial.println("off");
  else if (light_mode == LIGHT_MODE_ALWAYS)
    Serial.println("always");
  else if (light_mode == LIGHT_MODE_HIGH)
    Serial.println("high");
  else
    Serial.println(light_mode);

  Serial.print("Remote mode: ");
  if (remote_mode == REMOTE_MODE_CONTINUOUS)
    Serial.println("continous");
  else if (remote_mode == REMOTE_MODE_BLINK)
    Serial.println("blink");
  else
    Serial.println(remote_mode);

  Serial.print("High light: ");
  if (high_light_state_current.state == high_light_states::ON)
    Serial.println("on");
  else if (high_light_state_current.state == high_light_states::OFF)
    Serial.println("off");

  Serial.print("Engine started: ");
  if (engine_state_current.state.started)
    Serial.println("on");
  else
    Serial.println("off");

  Serial.print("Engine RPM: ");
  Serial.print(engine_state_current.state.rpm, DEC);
  Serial.println(" rpm");

  Serial.print("Aux lights: ");
  if (light_on)
    Serial.println("on");
  else if (!light_on)
    Serial.println("off");

  Serial.print("Remote: ");
  if (startRemoteMillis > 0)
    Serial.println("on");
  else
    Serial.println("off");

  Serial.println("------------------------------------------------------------------------");
}


#endif

#ifdef INTERRUPT_PIN

volatile bool interrupt = false;

void irqHandler()
{
  interrupt = true;
}

void processCanBusInterrupt()
{
  if (!interrupt)
    return;

  interrupt = false;

  struct can_frame frameInt;
  uint8_t irq = bus.getInterrupts();

  if (irq & MCP2515::CANINTF_RX0IF)
  {
    if (bus.readMessage(MCP2515::RXB0, &frameInt) == MCP2515::ERROR_OK)
    {
      processCanBusMessage(frameInt);
    }
  }

  if (irq & MCP2515::CANINTF_RX1IF)
  {
    if (bus.readMessage(MCP2515::RXB1, &frameInt) == MCP2515::ERROR_OK)
    {
      processCanBusMessage(frameInt);
    }
  }
}

#else

void processCanBus()
{

  struct can_frame frameInt;

  if (bus.readMessage(&frameInt) == MCP2515::ERROR_OK)
  {
    processCanBusMessage(frameInt);
  }
}

#endif

void processCanBusMessage(struct can_frame frame2process)
{

  unsigned long newMillis = millis();
#ifdef VERBOSE
  serialDataOutput(frame2process);
#endif

  processCanBusInfoStates(frame2process, newMillis);
  processCanBusTurnStates(frame2process, newMillis);
  processCanBusHighBeamStates(frame2process, newMillis);
  processCanBusEnineRPM(frame2process, newMillis);

  processActionsInfo(newMillis);
  processActionsTurn(newMillis);
  processActionsHighBeam(newMillis);
  processActionsEngineStart(newMillis);
}

void processCanBusInfoStates(struct can_frame frame2process, unsigned long currentMillis)
{
  bool info_short = false;
  bool info_long = false;
  bool info_off = false;

  bool info_short_checked = checkCanBusDataMask(frame2process, ID_CAN_CONTROLS, 5, VALUE_CAN_INFO_MASK, VALUE_CAN_INFO_SHORT, info_short);
  bool info_long_checked = checkCanBusDataMask(frame2process, ID_CAN_CONTROLS, 5, VALUE_CAN_INFO_MASK, VALUE_CAN_INFO_LONG, info_long);
  bool info_off_checked = checkCanBusDataMask(frame2process, ID_CAN_CONTROLS, 5, VALUE_CAN_INFO_MASK, VALUE_CAN_INFO_OFF, info_off);

  if (!(info_short_checked || info_long_checked || info_off_checked))
    return;

  if (info_short && info_state_current.state != info_states::SHORT)
  {
    info_state_current.previous_interval_millis = currentMillis - info_state_current.millis;
    info_state_current.millis = currentMillis;
    info_state_current.processed = false;
    info_state_current.previous_state = info_state_current.state;
    info_state_current.state = info_states::SHORT;
  }
  else if (info_long && info_state_current.state != info_states::LONG)
  {
    info_state_current.previous_interval_millis = currentMillis - info_state_current.millis;
    info_state_current.millis = currentMillis;
    info_state_current.processed = false;
    info_state_current.previous_state = info_state_current.state;
    info_state_current.state = info_states::LONG;
  }
  else if (info_off && info_state_current.state != info_states::OFF)
  {
    info_state_current.previous_interval_millis = currentMillis - info_state_current.millis;
    info_state_current.millis = currentMillis;
    info_state_current.processed = false;
    info_state_current.previous_state = info_state_current.state;
    info_state_current.state = info_states::OFF;
  }
#ifdef DEBUG
  serialCanBusStateOutput("INFO", (char)info_state_current.state, info_state_current.millis, newMillis);
#endif
}

void processCanBusTurnStates(struct can_frame frame2process, unsigned long newMillis)
{
  bool turn_off = false;
  bool turn_right = false;
  bool turn_left = false;
  bool turn_both = false;

  bool turn_off_checked = checkCanBusDataMask(frame2process, ID_CAN_LIGHTS, 7, VALUE_CAN_TURN_MASK, VALUE_CAN_TURN_OFF, turn_off);
  bool turn_right_checked = checkCanBusDataMask(frame2process, ID_CAN_LIGHTS, 7, VALUE_CAN_TURN_MASK, VALUE_CAN_TURN_RIGHT, turn_right);
  bool turn_left_checked = checkCanBusDataMask(frame2process, ID_CAN_LIGHTS, 7, VALUE_CAN_TURN_MASK, VALUE_CAN_TURN_LEFT, turn_left);
  bool turn_both_checked = checkCanBusDataMask(frame2process, ID_CAN_LIGHTS, 7, VALUE_CAN_TURN_MASK, VALUE_CAN_TURN_BOTHT, turn_both);

  if (!(turn_off_checked || turn_right_checked || turn_left_checked || turn_both_checked))
    return;

  if (turn_off && turn_state_current.state != turn_states::OFF)
  {
    turn_state_current.previous_interval_millis = newMillis - turn_state_current.millis;
    turn_state_current.millis = newMillis;
    turn_state_current.processed = false;
    turn_state_current.previous_state = turn_state_current.state;
    turn_state_current.state = turn_states::OFF;
  }
  else if (turn_left && turn_state_current.state != turn_states::LEFT)
  {
    turn_state_current.previous_interval_millis = newMillis - turn_state_current.millis;
    turn_state_current.millis = newMillis;
    turn_state_current.processed = false;
    turn_state_current.previous_state = turn_state_current.state;
    turn_state_current.state = turn_states::LEFT;
  }
  else if (turn_right && turn_state_current.state != turn_states::RIGHT)
  {
    turn_state_current.previous_interval_millis = newMillis - turn_state_current.millis;
    turn_state_current.millis = newMillis;
    turn_state_current.processed = false;
    turn_state_current.previous_state = turn_state_current.state;
    turn_state_current.state = turn_states::RIGHT;
  }
  else if (turn_both && turn_state_current.state != turn_states::BOTH)
  {
    turn_state_current.previous_interval_millis = newMillis - turn_state_current.millis;
    turn_state_current.millis = newMillis;
    turn_state_current.processed = false;
    turn_state_current.previous_state = turn_state_current.state;
    turn_state_current.state = turn_states::BOTH;
  }
#ifdef DEBUG
  serialCanBusStateOutput("TURN", (char)turn_state_current.state, turn_state_current.millis, newMillis);
#endif
}

void processCanBusHighBeamStates(struct can_frame frame2process, unsigned long newMillis)
{

  bool high_light_on = false;
  bool high_light_off = false;

  bool high_light_on_checked = checkCanBusDataMask(frame2process, ID_CAN_LIGHTS, 6, VALUE_CAN_HIGH_LIGHT_MASK, VALUE_CAN_HIGH_LIGHT_ON, high_light_on);
  bool high_light_off_checked = checkCanBusDataMask(frame2process, ID_CAN_LIGHTS, 6, VALUE_CAN_HIGH_LIGHT_MASK, VALUE_CAN_HIGH_LIGHT_OFF, high_light_off);

  if (!(high_light_on_checked || high_light_off_checked))
    return;

  if (high_light_on && high_light_state_current.state != high_light_states::ON)
  {
    high_light_state_current.previous_interval_millis = newMillis - high_light_state_current.millis;
    high_light_state_current.millis = newMillis;
    high_light_state_current.processed = false;
    high_light_state_current.previous_state = high_light_state_current.state;
    high_light_state_current.state = high_light_states::ON;
  }
  else if (high_light_off && high_light_state_current.state != high_light_states::OFF)
  {
    high_light_state_current.previous_interval_millis = newMillis - high_light_state_current.millis;
    high_light_state_current.millis = newMillis;
    high_light_state_current.processed = false;
    high_light_state_current.previous_state = high_light_state_current.state;
    high_light_state_current.state = high_light_states::OFF;
  }
#ifdef DEBUG
  serialCanBusStateOutput("HIGH BEAM", (char)high_light_state_current.state, high_light_state_current.millis, newMillis);
#endif
}

void processCanBusEnineRPM(struct can_frame frame2process, unsigned long newMillis)
{
  byte data2 = 0x00;
  byte data3 = 0x00;

  bool checked_data2 = getCanBusData(frame2process, ID_CAN_ENGINE, 2, data2);
  bool checked_data3 = getCanBusData(frame2process, ID_CAN_ENGINE, 3, data3);

  if (!(checked_data2 || checked_data3))
    return;

  uint32_t rpm = uint32_t((double(data3) * 256 + double(data2)) / 4);

  if (engine_state_current.state.rpm != rpm)
  {
    engine_state_current.previous_interval_millis = newMillis - engine_state_current.millis;
    engine_state_current.millis = newMillis;
    engine_state_current.processed = false;
    engine_state_current.previous_state = engine_state_current.state;
    engine_state_current.state.rpm = rpm;
    engine_state_current.state.started = (rpm > 0);
  }
#ifdef DEBUG
  serialCanBusNumberOutput("ENGIN RPM", engine_state_current.state.rpm, engine_state_current.millis, newMillis);
#endif
}

void processActionsInfo(unsigned long newMillis)
{

  bool processed = info_state_current.processed;
  bool executed_now = (info_state_current.millis == newMillis);

  unsigned long current_interval = newMillis - info_state_current.millis;
  unsigned long previous_interval_millis = info_state_current.previous_interval_millis;

  if (info_state_current.state == info_states::OFF && info_state_current.previous_state == info_states::SHORT && previous_interval_millis < MILLIS_TAPS && executed_now && !processed)
  {
    totalInfoShorts++;
  }
  else if (info_state_current.state == info_states::OFF && totalInfoShorts == 0 && current_interval >= MILLIS_TAPS && !processed)
  {
    totalInfoShorts = 0;
    info_state_current.processed = true;
#ifdef INFO
    Serial.println("##################### TAP INFO OFF ############################");
#endif
    // TODO: Action
  }
  else if (info_state_current.state == info_states::OFF && totalInfoShorts == 1 && current_interval >= MILLIS_TAPS && !processed)
  {
    totalInfoShorts = 0;
    info_state_current.processed = true;
#ifdef INFO
    Serial.println("##################### TAP INFO SHORT 1 ########################");
#endif
    // TODO: Action
  }
  else if (info_state_current.state == info_states::OFF && totalInfoShorts == 2 && current_interval >= MILLIS_TAPS && !processed)
  {
    totalInfoShorts = 0;
    info_state_current.processed = true;
#ifdef INFO
    Serial.println("##################### TAP INFO SHORT 2 ########################");
#endif

    startRemoteMillis = millis();
  }
  else if (info_state_current.state == info_states::OFF && totalInfoShorts == 3 && current_interval >= MILLIS_TAPS && !processed)
  {
    totalInfoShorts = 0;
    info_state_current.processed = true;
#ifdef INFO
    Serial.println("##################### TAP INFO SHORT 3 ########################");
#endif

    if (turn_left_on)
    {
#ifdef INFO
      Serial.println("******************** REMOTE MODE CONTINUOUS *******************");
#endif
      remote_mode = REMOTE_MODE_CONTINUOUS;
      eepromSave(remote_modes_address, remote_mode);
    }
    else if (turn_right_on)
    {
#ifdef INFO
      Serial.println("******************** REMOTE MODE BLINK ************************");
#endif
      remote_mode = REMOTE_MODE_BLINK;
      eepromSave(remote_modes_address, remote_mode);
    }
  }
  else if (info_state_current.state == info_states::OFF && totalInfoShorts > 3 && current_interval >= MILLIS_TAPS && !processed)
  {
    totalInfoShorts = 0;
    info_state_current.processed = true;
  }
  else if (info_state_current.state == info_states::LONG && executed_now && !processed)
  {
    totalInfoShorts = 0;
    info_state_current.processed = true;
#ifdef INFO
    Serial.println("##################### TAP INFO LONG ###########################");
#endif
    // TODO: Action
  }
}

void processActionsEngineStart(unsigned long newMillis)
{
  bool executed_now = (engine_state_current.millis == newMillis);
  bool processed = engine_state_current.processed;

  if (engine_state_current.state.started && !engine_state_current.previous_state.started && executed_now && !processed)
  {
    engine_state_current.processed = true;
#ifdef INFO
    Serial.println("##################### ENGINE START ###############################");
#endif
    // TODO: Action on engine start
  }
  else if (!engine_state_current.state.started && engine_state_current.previous_state.started && executed_now && !processed)
  {
    engine_state_current.processed = true;
#ifdef INFO
    Serial.println("##################### ENGINE STOP ###############################");
#endif
    // TODO: Action on engine stop
  }
}

void processActionsTurn(unsigned long newMillis)
{
  bool executed_now = (turn_state_current.millis == newMillis);
  bool processed = turn_state_current.processed;

  unsigned long current_interval = newMillis - turn_state_current.millis;
  unsigned long previous_interval_millis = turn_state_current.previous_interval_millis;

  if (turn_state_current.state == turn_states::LEFT && turn_state_current.previous_state != turn_states::LEFT && executed_now && !processed)
  {
    turn_state_current.processed = true;
#ifdef INFO
    Serial.println("##################### TURN LEFT ###############################");
#endif
    turn_left_on = true;
    turn_right_on = false;
    turn_both_on = false;
  }
  else if (turn_state_current.state == turn_states::RIGHT && turn_state_current.previous_state != turn_states::RIGHT && executed_now && !processed)
  {
    turn_state_current.processed = true;
#ifdef INFO
    Serial.println("##################### TURN RIGHT ##############################");
#endif
    turn_left_on = false;
    turn_right_on = true;
    turn_both_on = false;
  }
  else if (turn_state_current.state == turn_states::BOTH && turn_state_current.previous_state != turn_states::BOTH && executed_now && !processed)
  {
    turn_state_current.processed = true;
#ifdef INFO
    Serial.println("##################### TURN BOTH ###############################");
#endif
    turn_left_on = false;
    turn_right_on = false;
    turn_both_on = true;
  }
  else if (turn_state_current.state == turn_states::OFF && !processed && current_interval > previous_interval_millis)
  {
    turn_state_current.processed = true;
#ifdef INFO
    Serial.println("##################### TURN OFF ################################");
#endif
    turn_left_on = false;
    turn_right_on = false;
    turn_both_on = false;
  }
}

void processActionsHighBeam(unsigned long newMillis)
{
  bool processed = high_light_state_current.processed;
  bool executed_now = (high_light_state_current.millis == newMillis);

  unsigned long current_interval = newMillis - high_light_state_current.millis;
  unsigned long previous_interval_millis = high_light_state_current.previous_interval_millis;

  if (high_light_state_current.state == high_light_states::OFF && high_light_state_current.previous_state == high_light_states::ON && previous_interval_millis < MILLIS_TAPS && executed_now && !processed)
  {
    totalHighBeamShorts++;
  }
  else if (high_light_state_current.state == high_light_states::ON && current_interval >= MILLIS_TAPS && !processed)
  {
    totalHighBeamShorts = 0;
    high_light_state_current.processed = true;
#ifdef INFO
    Serial.println("##################### HIGH LIGHT ON ############################");
#endif
    if (light_mode == LIGHT_MODE_HIGH)
      light_on = true;
  }
  else if (high_light_state_current.state == high_light_states::OFF && totalHighBeamShorts == 0 && current_interval >= MILLIS_TAPS && !processed)
  {
    totalHighBeamShorts = 0;
    high_light_state_current.processed = true;
#ifdef INFO
    Serial.println("##################### HIGH LIGHT OFF ###########################");
#endif
    if (light_mode == LIGHT_MODE_HIGH)
      light_on = false;
  }
  else if (high_light_state_current.state == high_light_states::OFF && totalHighBeamShorts == 1 && current_interval >= MILLIS_TAPS && !processed)
  {
    totalHighBeamShorts = 0;
    high_light_state_current.processed = true;
#ifdef INFO
    Serial.println("##################### TAP HIGH LIGHT 1 #########################");
#endif
  }
  else if (high_light_state_current.state == high_light_states::OFF && totalHighBeamShorts == 2 && current_interval >= MILLIS_TAPS && !processed)
  {
    totalHighBeamShorts = 0;
    high_light_state_current.processed = true;
#ifdef INFO
    Serial.println("##################### TAP HIGH LIGHT 2 #########################");
#endif

    light_on = !light_on;
  }
  else if (high_light_state_current.state == high_light_states::OFF && totalHighBeamShorts == 3 && current_interval >= MILLIS_TAPS && !processed)
  {
    totalHighBeamShorts = 0;
    high_light_state_current.processed = true;
#ifdef INFO
    Serial.println("##################### TAP HIGH LIGHT 3 #########################");
#endif
    if (turn_left_on)
    {
#ifdef INFO
      Serial.println("******************** LIGHT MODE HIGH **************************");
#endif
      light_mode = LIGHT_MODE_HIGH;
      eepromSave(light_mode_address, light_mode);
    }
    else if (turn_right_on)
    {
#ifdef INFO
      Serial.println("******************** LIGHT MODE OFF ***************************");
#endif
      light_mode = LIGHT_MODE_OFF;
      eepromSave(light_mode_address, light_mode);
      light_on = false;
    }
    else if (turn_both_on)
    {
#ifdef INFO
      Serial.println("******************** LIGHT MODE ALWAYS *************************");
#endif
      light_mode = LIGHT_MODE_ALWAYS;
      eepromSave(light_mode_address, light_mode);
      light_on = true;
    }
  }
  else if (high_light_state_current.state == high_light_states::OFF && totalHighBeamShorts > 3 && current_interval >= MILLIS_TAPS && !processed)
  {
    totalHighBeamShorts = 0;
    high_light_state_current.processed = true;
  }
}


bool checkCanBusDataMask(struct can_frame frameCheck, uint32_t can_id, int pos, byte mask, byte value_mask, bool &value)
{
  if (frameCheck.can_id != can_id)
    return false;

  byte data = byte(frameCheck.data[pos]);
  byte data_mask = data & mask;

  if (data_mask == value_mask)
  {

#ifdef DEBUG
    serialDataMaskOutput(can_id, pos, mask, value_mask, data, data_mask);
#endif

    value = true;
  }
  else
  {
    value = false;
  }
  return true;
}

bool getCanBusData(struct can_frame frameCheck, uint32_t can_id, int pos, byte &value)
{

  if (frameCheck.can_id != can_id)
    return false;

  value = byte(frameCheck.data[pos]);

  return true;
}

#ifdef DEBUG
void serialCanBusStateOutput(String name, char state, unsigned long millis, unsigned long newMillis);
void serialCanBusStateOutput(String name, char state, unsigned long millis, unsigned long newMillis)
{

  Serial.print(name + ": ");
  Serial.print(state);
  Serial.print(" (Milis: ");
  Serial.print(millis, DEC);
  Serial.print(", Actual milis: ");
  Serial.print(newMillis, DEC);
  Serial.println(")");
}

void serialCanBusNumberOutput(String name, int value, unsigned long millis, unsigned long newMillis);
void serialCanBusNumberOutput(String name, int value, unsigned long millis, unsigned long newMillis)
{
  Serial.print(name + ": ");
  Serial.print(value, DEC);
  Serial.print(" (Milis: ");
  Serial.print(millis, DEC);
  Serial.print(", Actual milis: ");
  Serial.print(newMillis, DEC);
  Serial.println(")");
}

void serialDataMaskOutput(uint32_t can_id, int pos, byte mask, byte value_mask, byte data, byte data_mask);
void serialDataMaskOutput(uint32_t can_id, int pos, byte mask, byte value_mask, byte data, byte data_mask)
{
  Serial.print("checkBusData --> Match ID & Data[");
  Serial.print(pos, HEX);
  Serial.print("]: ");
  Serial.print(can_id, HEX);
  Serial.print(" --> Mask: ");
  Serial.print(mask, BIN);
  Serial.print(" --> Data: ");
  Serial.print(data, BIN);
  Serial.print(" --> Value mask: ");
  Serial.print(value_mask, BIN);
  Serial.print(" --> Data mask: ");
  Serial.print(data_mask, BIN);
  Serial.print(" --> Equals: ");
  Serial.println(data_mask == value_mask);
}
#endif

#ifdef VERBOSE
void serialDataOutput(struct can_frame frameOutput)
{

#ifdef ID_CAN_FILTER
  if (frameOutput.can_id != ID_CAN_FILTER)
    return; // A filter for this.
#endif

  Serial.print("0x");
  Serial.print(frameOutput.can_id, HEX); // print ID
  Serial.print("; ");
  Serial.print(frameOutput.can_dlc, DEC); // print DLC
  Serial.print("; ");

  for (int i = 0; i < frameOutput.can_dlc; i++)
  { // print the data
    Serial.print(frameOutput.data[i], HEX);
    Serial.print("; ");
  }
  Serial.println();
}
#endif

// ==================================================== TESTING ======================================================= //

#ifdef TEST
unsigned long testMilis;
unsigned int iteration;
void writeTest();
void writeTest()
{
  unsigned long newMilis = millis();
  if (newMilis - testMilis < 500)
  {
    return;
  }
  testMilis = newMilis;

  struct can_frame frame2write;
  frame2write.can_id = ID_CAN_CONTROLS;
  frame2write.can_dlc = 8;
  frame2write.data[0] = 0x00;
  frame2write.data[1] = 0x00;
  frame2write.data[2] = 0x00;
  frame2write.data[3] = 0x00;
  frame2write.data[4] = 0x00;
  frame2write.data[6] = 0x00;
  frame2write.data[5] = 0x00;
  frame2write.data[7] = 0x00;

  switch (iteration++)
  {
  case 1:
  case 2:
  case 3:
  case 4:
  case 5:
  case 6:
    frame2write.can_id = ID_CAN_ENGINE;
    frame2write.data[2] = 0x40;
    frame2write.data[3] = 0x19;
    break;
  case 7:
  case 8:
  case 9:
  case 10:
  case 11:
  case 12:
  case 13:
  case 14:
  default:
    frame2write.can_id = ID_CAN_ENGINE;
    frame2write.data[2] = 0x65;
    frame2write.data[3] = 0x15;
    break;
  }
  if (iteration >= 14)
  {
    iteration = 0;
  }
  bus.sendMessage(&frame2write);
}
#endif
