// Generic stepper driver
#include <AccelStepper.h>
#include <MultiStepper.h>
#include <Encoder.h>

// Trinamic TMC2130 stepper driver
#include <TMC2130Stepper.h>
#include <TMC2130Stepper_REGDEFS.h>
#include <TMC2130Stepper_UTILITY.h>

// TMC2130 driver pins
#define EN_PIN    6
#define DIR_PIN   4
#define STEP_PIN  5

// SPI to TMC2130
#define CS_PIN    10
#define MOSI_PIN  11
#define MISO_PIN  12
#define SCK_PIN   13

// COBS serial packet handling
#include <FastCRC.h>
#include <PacketSerial.h>

// optical encoder pins
#define MOTOR_ENC_PINA 5
#define MOTOR_ENC_PINB 6

FastCRC16 CRC16;
PacketSerial packetSerial;

// interval Timer creation
IntervalTimer gatherTimer;
IntervalTimer motorTimer;
elapsedMicros current_micros;

// Encoders and Sensors
Encoder motorEncoder = Encoder(MOTOR_ENC_PINA, MOTOR_ENC_PINB);

//motors
TMC2130Stepper driver = TMC2130Stepper(EN_PIN, DIR_PIN, STEP_PIN, CS_PIN, MOSI_PIN, MISO_PIN, SCK_PIN);
AccelStepper stepper = AccelStepper(stepper.DRIVER, STEP_PIN, DIR_PIN);

// intermediate values
long encoderPosition = 0L;
long stepperPosition = 0L;
long velocity        = 0L;
long acceleration    = 0L;


// SERIAL PACKAGES
enum packetType: uint8_t {
  ptSTATUS,
  ptCOMMAND,
  ptERROR,
  ptOK,
  ptACK
};

enum Intructions: uint8_t {
  instMOVETO,
  instSTOP,
  instENABLE,
  instDISABLE
};

volatile unsigned long packetCount = 0;
struct dataPacket {
    uint8_t type;          // 1 B, packet type
    uint8_t length;        // 1 B, packet size
    uint16_t crc16;        // 2 B, CRC16
    
    unsigned long packetID;// 4 B, running packet count
    unsigned long us_start;// 4 B, gather start timestamp
    unsigned long us_end;  // 4 B, transmit timestamp
    long variables[8];     // 16 B, variables (position, speed)
    
    uint16_t digitalIn;    // 2 B, digital inputs
    uint8_t digitalOut;    // 1 B, digital outputs
    uint8_t padding[1];    // 1 B, align to 4B
    
    dataPacket() : type(ptSTATUS),
                   length(sizeof(dataPacket)),
                   crc16(0),
                   packetID(packetCount++),
                   digitalIn(0),
                   digitalOut(0) {}
};

struct errorPacket {
    uint8_t type;          // 1 B, packet type
    uint8_t length;        // 1 B, packet size
    uint16_t crc16;        // 2 B, CRC16
    unsigned long packetID;// 4 B, running packet count
    unsigned long us_start;// 4 B, gather start timestamp

    char message[16];      // 16 B, error message

    errorPacket() : type(ptERROR),
                   length(sizeof(errorPacket)),
                   crc16(0),
                   packetID(packetCount++) {}    
};

const int ledPin = LED_BUILTIN;

void setup() {
  pinMode(ledPin, OUTPUT);

  {
    pinMode(EN_PIN, OUTPUT);
    pinMode(DIR_PIN, OUTPUT);
    pinMode(STEP_PIN, OUTPUT);
    pinMode(CS_PIN, OUTPUT);
    digitalWrite(EN_PIN, HIGH); //deactivate driver (LOW active)
    digitalWrite(DIR_PIN, LOW);
    digitalWrite(STEP_PIN, LOW);
    digitalWrite(CS_PIN, HIGH);
    SPI.begin();
    pinMode(MISO_PIN, INPUT_PULLUP);
  }

//  { // stepper driver configuration
//    driver.push();
//    driver.toff(3);
//    driver.tbl(1);
//    driver.hysteresis_start(4);
//    driver.hysteresis_end(-2);
//    driver.rms_current(600); // mA
//    driver.microsteps(16);
//    driver.diag1_stall(1);
//    driver.diag1_active_high(1);
//    driver.coolstep_min_speed(0xFFFFF); // 20bit max
//    driver.THIGH(0);
//    driver.semin(5);
//    driver.semax(2);
//    driver.sedn(0b01);
//    driver.sg_stall_value(STALL_VALUE);
//    driver.stealthChop(1);
//  }
  
  { // motor setup
  stepper.setMaxSpeed(40000);
  stepper.setAcceleration(20000);
  stepper.setEnablePin(EN_PIN);
  stepper.setPinsInverted(false, false, true);
  stepper.enableOutputs();
  digitalWrite(EN_PIN, LOW); // redundant?

  stepper.moveTo(10000);
  }

  {
    packetSerial.begin(9600);
    packetSerial.setPacketHandler(&onPacketReceived);
  }

  // start motor update interval
  motorTimer.begin(step_motor, 10);

  // start telemetry transmission, 10 ms interval
  gatherTimer.begin(telemetry, 10000);
}

void loop() {
  digitalWrite(ledPin, !digitalRead(ledPin));
  delay(10);

  // check current serial status
  packetSerial.update();
  if (packetSerial.overflow()) {
    errorPacket ep;
    ep.us_start = current_micros;
    strcpy(ep.message, "serial_overflow");
    packetSerial.send((byte *) &ep, sizeof(ep));
  }
}

void step_motor() {
//  if (stepper.distanceToGo() == 0) {
//    stepper.moveTo(-stepper.targetPosition());
//  }
  stepper.run();
}

void telemetry() {
  volatile dataPacket packet;
  packet.us_start = current_micros;

  for (int i=0; i<16; i++) {
    packet.digitalIn |= digitalReadFast(i) << i;
  }

  for (int i=0; i<8; i++) {
    packet.digitalOut |= digitalReadFast(i) << i;
  }

  for (int p=0; p<8; p++) {
    packet.variables[p] = 0L;
  }
  
  packet.variables[0] = motorEncoder.read();
  packet.variables[1] = stepper.targetPosition();
  packet.variables[2] = stepper.currentPosition();
  packet.variables[3] = stepper.speed();

  packet.crc16 = CRC16.ccitt((byte*) &packet, sizeof(packet));

  // process state packet
  processState(&packet);
  
  packet.us_end = current_micros;
  packetSerial.send((byte*) &packet, sizeof(packet));

}

void onPacketReceived(const uint8_t* buffer, size_t size) {
  // if we receive a command, do what it tells us to do...
  if (buffer[0] == ptCOMMAND) {
    processCommand(buffer, size);
  }
}

void processState(volatile dataPacket* packet) {
  // apply finite state machine updates here
}

void processCommand(const uint8_t* buffer, size_t size) {
  stepper.moveTo(stepper.targetPosition() + (int) (int8_t) buffer[2]);
}
