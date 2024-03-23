#include <Arduino.h>

#define H_BRIDGE_CURRENT_PIN A0
#define VOLTAGE_5V_PIN A1
#define VOLTAGE_12V_PIN A2
#define LIMIT_U1_PIN 4
#define LIMIT_U2_PIN 5
#define LIMIT_L1_PIN 6
#define LIMIT_L2_PIN 7

#define RAISE_1_PIN 2
#define RAISE_2_PIN 3

#define LOWER_1_PIN 10
#define LOWER_2_PIN 11

#define ENGAGE_LOCK_PIN 8
#define DISENGAGE_LOCK_PIN 9

#define CMD_RAISE_ROOF 0xAB
#define CMD_LOWER_ROOF 0xCD
#define CMD_STOP_ROOF 0xEF
#define CMD_ENGAGE_LOCK 0x12
#define CMD_DISENGAGE_LOCK 0x34
#define STOP_LOCK 0x56

#define uint unsigned int

#define SYNC_BYTE 0x50

#define NUM_ANALOG_SENSORS 3
#define NUM_DIGITAL_SENSORS 4

class AnalogSensor {
  public:
    AnalogSensor(int pin, float slope, float offset) {
      this->pin = pin;
      this->slope = slope;
      this->offset = offset;
    }

    float read_value()
    {
      last_reading_raw = analogRead(pin);
      last_reading_converted = ((float)last_reading_raw * slope) + offset;
      return last_reading_converted;
    }

    float get_last_reading()
    {
      return last_reading_converted;
    }

  private:
    int pin;
    float slope;
    float offset;
    int last_reading_raw;
    float last_reading_converted;
};
typedef enum {
    STATE_UNKNOWN,
    STATE_RAISING,
    STATE_LOWERING,
    STATE_RAISED,
    STATE_LOWERED,
} MotionState;

namespace Roof {
  MotionState roof_state = STATE_UNKNOWN;
}
namespace Lock {
  MotionState lock_state = STATE_UNKNOWN;
}

namespace Lock {

  void raise() {
    if (lock_state != STATE_RAISED && Roof::roof_state == STATE_RAISED) {
      digitalWrite(DISENGAGE_LOCK_PIN, LOW);
      digitalWrite(ENGAGE_LOCK_PIN, HIGH);
      lock_state = STATE_RAISING;
    }
  }
  void lower() {
    if (lock_state != STATE_RAISED && Roof::roof_state == STATE_RAISED) {
      digitalWrite(ENGAGE_LOCK_PIN, LOW);
      digitalWrite(DISENGAGE_LOCK_PIN, HIGH);
      lock_state = STATE_LOWERING;
    }
  }

}

namespace Roof {

  void raise() {
    if (roof_state != STATE_RAISED && (Lock::lock_state == STATE_LOWERED || Lock::lock_state == STATE_UNKNOWN)) {
      digitalWrite(RAISE_1_PIN, HIGH);
      digitalWrite(RAISE_2_PIN, HIGH);
      digitalWrite(LOWER_1_PIN, LOW);
      digitalWrite(LOWER_2_PIN, LOW);
      roof_state = STATE_RAISING;
    }
  }
  void lower() {
    if (roof_state != STATE_LOWERED && (Lock::lock_state == STATE_LOWERED || Lock::lock_state == STATE_UNKNOWN)) {
      digitalWrite(RAISE_1_PIN, LOW);
      digitalWrite(RAISE_2_PIN, LOW);
      digitalWrite(LOWER_1_PIN, HIGH);
      digitalWrite(LOWER_2_PIN, HIGH);
      roof_state = STATE_LOWERING;
    }
  }

}
typedef enum {
    STATE_IDLE,
    STATE_READING_SYNC,
    STATE_READING_LENGTH,
    STATE_READING_PAYLOAD,
} CommandReadState;



CommandReadState command_read_state = STATE_IDLE;
AnalogSensor h_bridge_current = AnalogSensor(H_BRIDGE_CURRENT_PIN, 0.00024438, 0.125);
AnalogSensor voltage_5v = AnalogSensor(VOLTAGE_5V_PIN, 0.009775171065494, 0);
AnalogSensor voltage_12v = AnalogSensor(VOLTAGE_12V_PIN, 0.02117953730857, 0);
AnalogSensor analog_sensors[NUM_ANALOG_SENSORS] = {h_bridge_current, voltage_5v, voltage_12v};

int limit_switches[NUM_DIGITAL_SENSORS] = {RAISE_1_PIN, RAISE_2_PIN, LOWER_1_PIN, LOWER_2_PIN};

byte serial_buffer[100] = {0};
unsigned int buff_idx = 0;
uint sync_bytes_read = 0;
uint payload_length = 0;

void setup() {
  pinMode(LIMIT_U1_PIN, INPUT);
  pinMode(LIMIT_U2_PIN, INPUT);
  pinMode(LIMIT_L1_PIN, INPUT);
  pinMode(LIMIT_L2_PIN, INPUT);

  pinMode(RAISE_1_PIN, OUTPUT);
  pinMode(RAISE_2_PIN, OUTPUT);
  pinMode(LOWER_1_PIN, OUTPUT);
  pinMode(LOWER_2_PIN, OUTPUT);

  pinMode(ENGAGE_LOCK_PIN, OUTPUT);
  pinMode(DISENGAGE_LOCK_PIN, OUTPUT);

  Serial.begin(57600);

}
void parse_command(byte* buffer, uint length) {
  if (length != 1) {
    return;
  }
  switch (buffer[0]) {
    case CMD_RAISE_ROOF:
      Roof::raise();
      break;
  }
}

void send_telemetry() {
  uint8_t header[3] = {0x50, 0x50, 0x50};
  Serial.write(header, 3);
  for (int i = 0; i < NUM_ANALOG_SENSORS; i++) {
    float value = analog_sensors[i].read_value();
    byte* value_ptr = (byte*)&value;
    for (int i = 0; i < sizeof(float); i++) {
      Serial.write(value_ptr[i]);
    }
  }
  for (int i = 0; i < NUM_DIGITAL_SENSORS; i++) {
    if (digitalRead(limit_switches[i]) == HIGH) {
      Serial.write(1);
    }
    else {
      Serial.write(0);
    }
  }
}

void loop() {

  //TODO: stop commands if commands are not sent
  send_telemetry();
  if (Serial.available() > 0) {
    // Read the incoming byte:
    byte data = Serial.read();
    switch (command_read_state) {
      case STATE_IDLE:
        if (data == SYNC_BYTE) {
          buff_idx = 0;
          sync_bytes_read = 1;
          command_read_state = STATE_READING_SYNC;
        }
        break;
      case STATE_READING_SYNC:
        if (data != SYNC_BYTE) {
          command_read_state = STATE_IDLE;
        }
        else {
          sync_bytes_read ++;
          if (sync_bytes_read == 3) {
            command_read_state = STATE_READING_LENGTH;
          }
        }
        break;
      case STATE_READING_LENGTH:
        payload_length = data;
        command_read_state = STATE_READING_PAYLOAD;
        break;
      case STATE_READING_PAYLOAD:
        serial_buffer[buff_idx]= data;
        buff_idx++;
        if (buff_idx == payload_length) {
          command_read_state = STATE_IDLE;
          parse_command(serial_buffer, payload_length);
        }
        
        break;
    }
  
    // Echo the received byte back to the serial   port
    //Serial.println(receivedChar);
  }
  delay(100);

}