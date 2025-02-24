#include "Robot.h"

void Robot::init(void (*serialWriteChannelFunction)(char c, int32_t v)) {
  uint8_t i;

  // Serial write channels function
  serialWriteChannel = serialWriteChannelFunction;

  // Encoders
  initEnc();
  updateEncodersState();
  for (i = 0; i < 4; i++) {
    encoders[i].delta = 0;
  }
  Timer1.attachInterrupt(updateEncodersState);
  Timer1.initialize(20);  // calls every X us

  // Motors
  Timer3.initialize(20);
  for (i = 0; i < 4; i++) {
    mot[i].init(kMotDirPin[i], kMotPwmPin[i]);
  }

  // Controllers
  for (i = 0; i < 4; i++) {
    initCtrlPID(i);
  }
}

void Robot::update(uint32_t &delta) {
  uint8_t i;
  dt = delta;

  // Encoders
  for (i = 0; i < 4; i++) {
    enc[i].updateTick();
  }

  // Controllers
  for (i = 0; i < 4; i++) {
    pid[i].update(enc[i].odo * kEncImp2MotW);
  }

  // Actuators
  for (i = 0; i < 4; i++) {
    if (pid[i].active) {
      mot[i].setPWM( round( kMotV2MotPWM * pid[i].m ) );
    }
  }
}

void Robot::send(void) {
  (*serialWriteChannel)('g', enc[0].tick);
  (*serialWriteChannel)('h', enc[1].tick);
  (*serialWriteChannel)('i', enc[2].tick);
  (*serialWriteChannel)('j', enc[3].tick);

  (*serialWriteChannel)('k', dt);
}

void Robot::stop(void) {
  uint8_t i;

  for (i = 0; i < 4; i++) {
    setMotorPWM(i, 0);
  }
}

void Robot::setMotorWref(uint8_t index, float new_w_r) {
  pid[index].enable(true);
  pid[index].w_ref = new_w_r;
}

void Robot::setMotorPWM(uint8_t index, int16_t pwm) {
  pid[index].enable(false);
  mot[index].setPWM(pwm);
}

void Robot::initEnc() {
  pinMode(kMotEncPin0A, INPUT);
  pinMode(kMotEncPin0B, INPUT);
  pinMode(kMotEncPin1A, INPUT);
  pinMode(kMotEncPin1B, INPUT);
  pinMode(kMotEncPin2A, INPUT);
  pinMode(kMotEncPin2B, INPUT);
  pinMode(kMotEncPin3A, INPUT);
  pinMode(kMotEncPin3B, INPUT);
}

void Robot::initCtrlPID(uint8_t index) {
  pid[index].active = false;
  pid[index].kp = kMotCtrlKc;
  if (kMotCtrlTi == 0) {
    pid[index].ki = 0;
  } else {
    pid[index].ki = kMotCtrlKc / kMotCtrlTi;
  }
  pid[index].kd = 0;
  pid[index].kf = 0;
  pid[index].dt = kMotCtrlTime;

  pid[index].m_max = kMotVmax;

  pid[index].hamm_vd = 0;
  pid[index].hamm_v0 = 0;

  pid[index].reset();
}
