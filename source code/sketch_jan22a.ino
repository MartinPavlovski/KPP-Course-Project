// Adafruit Motor shield library
// copyright Adafruit Industries LLC, 2009
// this code is public domain, enjoy!

/*
 * Usage Notes::
 * For PIC32, all features work properly with the following two exceptions:
 *
 * 1) Because the PIC32 only has 5 PWM outputs, and the AFMotor shield needs 6
 *    to completely operate (four for motor outputs and two for RC servos), the
 *    M1 motor output will not have PWM ability when used with a PIC32 board.
 *    However, there is a very simple workaround. If you need to drive a stepper
 *    or DC motor with PWM on motor output M1, you can use the PWM output on pin
 *    9 or pin 10 (normally use for RC servo outputs on Arduino, not needed for 
 *    RC servo outputs on PIC32) to drive the PWM input for M1 by simply putting
 *    a jumber from pin 9 to pin 11 or pin 10 to pin 11. Then uncomment one of the
 *    two #defines below to activate the PWM on either pin 9 or pin 10. You will
 *    then have a fully functional microstepping for 2 stepper motors, or four
 *    DC motor outputs with PWM.
 *
 * 2) There is a conflict between RC Servo outputs on pins 9 and pins 10 and 
 *    the operation of DC motors and stepper motors as of 9/2012. This issue
 *    will get fixed in future MPIDE releases, but at the present time it means
 *    that the Motor Party example will NOT work properly. Any time you attach
 *    an RC servo to pins 9 or pins 10, ALL PWM outputs on the whole board will
 *    stop working. Thus no steppers or DC motors.
 * 
 */
// <BPS> 09/15/2012 Modified for use with chipKIT boards


#ifndef _AFMotor_h_
#define _AFMotor_h_

#include <inttypes.h>
#if defined(__AVR__)
    #include <avr/io.h>

    //#define MOTORDEBUG 1

    #define MICROSTEPS 16                       // 8 or 16

    #define MOTOR12_64KHZ _BV(CS20)             // no prescale
    #define MOTOR12_8KHZ _BV(CS21)              // divide by 8
    #define MOTOR12_2KHZ _BV(CS21) | _BV(CS20)  // divide by 32
    #define MOTOR12_1KHZ _BV(CS22)              // divide by 64

    #define MOTOR34_64KHZ _BV(CS00)             // no prescale
    #define MOTOR34_8KHZ _BV(CS01)              // divide by 8
    #define MOTOR34_1KHZ _BV(CS01) | _BV(CS00)  // divide by 64
    
    #define DC_MOTOR_PWM_RATE   MOTOR34_8KHZ    // PWM rate for DC motors
    #define STEPPER1_PWM_RATE   MOTOR12_64KHZ   // PWM rate for stepper 1
    #define STEPPER2_PWM_RATE   MOTOR34_64KHZ   // PWM rate for stepper 2
    
#elif defined(__PIC32MX__)
    //#define MOTORDEBUG 1
    
    // Uncomment the one of following lines if you have put a jumper from 
    // either pin 9 to pin 11 or pin 10 to pin 11 on your Motor Shield.
    // Either will enable PWM for M1
    //#define PIC32_USE_PIN9_FOR_M1_PWM
    //#define PIC32_USE_PIN10_FOR_M1_PWM

    #define MICROSTEPS 16       // 8 or 16

    // For PIC32 Timers, define prescale settings by PWM frequency
    #define MOTOR12_312KHZ  0   // 1:1, actual frequency 312KHz
    #define MOTOR12_156KHZ  1   // 1:2, actual frequency 156KHz
    #define MOTOR12_64KHZ   2   // 1:4, actual frequency 78KHz
    #define MOTOR12_39KHZ   3   // 1:8, acutal frequency 39KHz
    #define MOTOR12_19KHZ   4   // 1:16, actual frequency 19KHz
    #define MOTOR12_8KHZ    5   // 1:32, actual frequency 9.7KHz
    #define MOTOR12_4_8KHZ  6   // 1:64, actual frequency 4.8KHz
    #define MOTOR12_2KHZ    7   // 1:256, actual frequency 1.2KHz
    #define MOTOR12_1KHZ    7   // 1:256, actual frequency 1.2KHz

    #define MOTOR34_312KHZ  0   // 1:1, actual frequency 312KHz
    #define MOTOR34_156KHZ  1   // 1:2, actual frequency 156KHz
    #define MOTOR34_64KHZ   2   // 1:4, actual frequency 78KHz
    #define MOTOR34_39KHZ   3   // 1:8, acutal frequency 39KHz
    #define MOTOR34_19KHZ   4   // 1:16, actual frequency 19KHz
    #define MOTOR34_8KHZ    5   // 1:32, actual frequency 9.7KHz
    #define MOTOR34_4_8KHZ  6   // 1:64, actual frequency 4.8KHz
    #define MOTOR34_2KHZ    7   // 1:256, actual frequency 1.2KHz
    #define MOTOR34_1KHZ    7   // 1:256, actual frequency 1.2KHz
    
    // PWM rate for DC motors.
    #define DC_MOTOR_PWM_RATE   MOTOR34_39KHZ
    // Note: for PIC32, both of these must be set to the same value
    // since there's only one timebase for all 4 PWM outputs
    #define STEPPER1_PWM_RATE   MOTOR12_39KHZ
    #define STEPPER2_PWM_RATE   MOTOR34_39KHZ
    
#endif

// Bit positions in the 74HCT595 shift register output
#define MOTOR1_A 2
#define MOTOR1_B 3
#define MOTOR2_A 1
#define MOTOR2_B 4
#define MOTOR4_A 0
#define MOTOR4_B 6
#define MOTOR3_A 5
#define MOTOR3_B 7

// Constants that the user passes in to the motor calls
#define FORWARD 1
#define BACKWARD 2
#define BRAKE 3
#define RELEASE 4

// Constants that the user passes in to the stepper calls
#define SINGLE 1
#define DOUBLE 2
#define INTERLEAVE 3
#define MICROSTEP 4

/*
#define LATCH 4
#define LATCH_DDR DDRB
#define LATCH_PORT PORTB

#define CLK_PORT PORTD
#define CLK_DDR DDRD
#define CLK 4

#define ENABLE_PORT PORTD
#define ENABLE_DDR DDRD
#define ENABLE 7

#define SER 0
#define SER_DDR DDRB
#define SER_PORT PORTB
*/

// Arduino pin names for interface to 74HCT595 latch
#define MOTORLATCH 12
#define MOTORCLK 4
#define MOTORENABLE 7
#define MOTORDATA 8

class AFMotorController
{
  public:
    AFMotorController(void);
    void enable(void);
    friend class AF_DCMotor;
    void latch_tx(void);
    uint8_t TimerInitalized;
};

class AF_DCMotor
{
 public:
  AF_DCMotor(uint8_t motornum, uint8_t freq = DC_MOTOR_PWM_RATE);
  void run(uint8_t);
  void setSpeed(uint8_t);

 private:
  uint8_t motornum, pwmfreq;
};

class AF_Stepper {
 public:
  AF_Stepper(uint16_t, uint8_t);
  void step(uint16_t steps, uint8_t dir,  uint8_t style = SINGLE);
  void setSpeed(uint16_t);
  uint8_t onestep(uint8_t dir, uint8_t style);
  void release(void);
  uint16_t revsteps; // # steps per revolution
  uint8_t steppernum;
  uint32_t usperstep, steppingcounter;
 private:
  uint8_t currentstep;

};

uint8_t getlatchstate(void);

#endif



// Adafruit Motor shield library
// copyright Adafruit Industries LLC, 2009
// this code is public domain, enjoy!


#if (ARDUINO >= 100)
  #include "Arduino.h"
#else
  #if defined(__AVR__)
    #include <avr/io.h>
  #endif
  #include "WProgram.h"
#endif



static uint8_t latch_state;

#if (MICROSTEPS == 8)
uint8_t microstepcurve[] = {0, 50, 98, 142, 180, 212, 236, 250, 255};
#elif (MICROSTEPS == 16)
uint8_t microstepcurve[] = {0, 25, 50, 74, 98, 120, 141, 162, 180, 197, 212, 225, 236, 244, 250, 253, 255};
#endif

AFMotorController::AFMotorController(void) {
    TimerInitalized = false;
}

void AFMotorController::enable(void) {
  // setup the latch
  /*
  LATCH_DDR |= _BV(LATCH);
  ENABLE_DDR |= _BV(ENABLE);
  CLK_DDR |= _BV(CLK);
  SER_DDR |= _BV(SER);
  */
  pinMode(MOTORLATCH, OUTPUT);
  pinMode(MOTORENABLE, OUTPUT);
  pinMode(MOTORDATA, OUTPUT);
  pinMode(MOTORCLK, OUTPUT);

  latch_state = 0;

  latch_tx();  // "reset"

  //ENABLE_PORT &= ~_BV(ENABLE); // enable the chip outputs!
  digitalWrite(MOTORENABLE, LOW);
}


void AFMotorController::latch_tx(void) {
  uint8_t i;

  //LATCH_PORT &= ~_BV(LATCH);
  digitalWrite(MOTORLATCH, LOW);

  //SER_PORT &= ~_BV(SER);
  digitalWrite(MOTORDATA, LOW);

  for (i=0; i<8; i++) {
    //CLK_PORT &= ~_BV(CLK);
    digitalWrite(MOTORCLK, LOW);

    if (latch_state & _BV(7-i)) {
      //SER_PORT |= _BV(SER);
      digitalWrite(MOTORDATA, HIGH);
    } else {
      //SER_PORT &= ~_BV(SER);
      digitalWrite(MOTORDATA, LOW);
    }
    //CLK_PORT |= _BV(CLK);
    digitalWrite(MOTORCLK, HIGH);
  }
  //LATCH_PORT |= _BV(LATCH);
  digitalWrite(MOTORLATCH, HIGH);
}

static AFMotorController MC;

/******************************************
               MOTORS
******************************************/
inline void initPWM1(uint8_t freq) {
#if defined(__AVR_ATmega8__) || \
    defined(__AVR_ATmega48__) || \
    defined(__AVR_ATmega88__) || \
    defined(__AVR_ATmega168__) || \
    defined(__AVR_ATmega328P__)
    // use PWM from timer2A on PB3 (Arduino pin #11)
    TCCR2A |= _BV(COM2A1) | _BV(WGM20) | _BV(WGM21); // fast PWM, turn on oc2a
    TCCR2B = freq & 0x7;
    OCR2A = 0;
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    // on arduino mega, pin 11 is now PB5 (OC1A)
    TCCR1A |= _BV(COM1A1) | _BV(WGM10); // fast PWM, turn on oc1a
    TCCR1B = (freq & 0x7) | _BV(WGM12);
    OCR1A = 0;
#elif defined(__PIC32MX__)
    #if defined(PIC32_USE_PIN9_FOR_M1_PWM)
        // Make sure that pin 11 is an input, since we have tied together 9 and 11
        pinMode(9, OUTPUT);
        pinMode(11, INPUT);
        if (!MC.TimerInitalized)
        {   // Set up Timer2 for 80MHz counting fro 0 to 256
            T2CON = 0x8000 | ((freq & 0x07) << 4); // ON=1, FRZ=0, SIDL=0, TGATE=0, TCKPS=<freq>, T32=0, TCS=0; // ON=1, FRZ=0, SIDL=0, TGATE=0, TCKPS=0, T32=0, TCS=0
            TMR2 = 0x0000;
            PR2 = 0x0100;
            MC.TimerInitalized = true;
        }
         // Setup OC4 (pin 9) in PWM mode, with Timer2 as timebase
        OC4CON = 0x8006;    // OC32 = 0, OCTSEL=0, OCM=6
        OC4RS = 0x0000;
        OC4R = 0x0000;
    #elif defined(PIC32_USE_PIN10_FOR_M1_PWM)
        // Make sure that pin 11 is an input, since we have tied together 9 and 11
        pinMode(10, OUTPUT);
        pinMode(11, INPUT);
        if (!MC.TimerInitalized)
        {   // Set up Timer2 for 80MHz counting fro 0 to 256
            T2CON = 0x8000 | ((freq & 0x07) << 4); // ON=1, FRZ=0, SIDL=0, TGATE=0, TCKPS=<freq>, T32=0, TCS=0; // ON=1, FRZ=0, SIDL=0, TGATE=0, TCKPS=0, T32=0, TCS=0
            TMR2 = 0x0000;
            PR2 = 0x0100;
            MC.TimerInitalized = true;
        }
         // Setup OC5 (pin 10) in PWM mode, with Timer2 as timebase
        OC5CON = 0x8006;    // OC32 = 0, OCTSEL=0, OCM=6
        OC5RS = 0x0000;
        OC5R = 0x0000;
    #else
        // If we are not using PWM for pin 11, then just do digital
        digitalWrite(11, LOW);
    #endif
#else
   #error "This chip is not supported!"
#endif
    #if !defined(PIC32_USE_PIN9_FOR_M1_PWM) && !defined(PIC32_USE_PIN10_FOR_M1_PWM)
        pinMode(11, OUTPUT);
    #endif
}

inline void setPWM1(uint8_t s) {
#if defined(__AVR_ATmega8__) || \
    defined(__AVR_ATmega48__) || \
    defined(__AVR_ATmega88__) || \
    defined(__AVR_ATmega168__) || \
    defined(__AVR_ATmega328P__)
    // use PWM from timer2A on PB3 (Arduino pin #11)
    OCR2A = s;
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    // on arduino mega, pin 11 is now PB5 (OC1A)
    OCR1A = s;
#elif defined(__PIC32MX__)
    #if defined(PIC32_USE_PIN9_FOR_M1_PWM)
        // Set the OC4 (pin 9) PMW duty cycle from 0 to 255
        OC4RS = s;
    #elif defined(PIC32_USE_PIN10_FOR_M1_PWM)
        // Set the OC5 (pin 10) PMW duty cycle from 0 to 255
        OC5RS = s;
    #else
        // If we are not doing PWM output for M1, then just use on/off
        if (s > 127)
        {
            digitalWrite(11, HIGH);
        }
        else
        {
            digitalWrite(11, LOW);
        }
    #endif
#else
   #error "This chip is not supported!"
#endif
}

inline void initPWM2(uint8_t freq) {
#if defined(__AVR_ATmega8__) || \
    defined(__AVR_ATmega48__) || \
    defined(__AVR_ATmega88__) || \
    defined(__AVR_ATmega168__) || \
    defined(__AVR_ATmega328P__)
    // use PWM from timer2B (pin 3)
    TCCR2A |= _BV(COM2B1) | _BV(WGM20) | _BV(WGM21); // fast PWM, turn on oc2b
    TCCR2B = freq & 0x7;
    OCR2B = 0;
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    // on arduino mega, pin 3 is now PE5 (OC3C)
    TCCR3A |= _BV(COM1C1) | _BV(WGM10); // fast PWM, turn on oc3c
    TCCR3B = (freq & 0x7) | _BV(WGM12);
    OCR3C = 0;
#elif defined(__PIC32MX__)
    if (!MC.TimerInitalized)
    {   // Set up Timer2 for 80MHz counting fro 0 to 256
        T2CON = 0x8000 | ((freq & 0x07) << 4); // ON=1, FRZ=0, SIDL=0, TGATE=0, TCKPS=<freq>, T32=0, TCS=0; // ON=1, FRZ=0, SIDL=0, TGATE=0, TCKPS=0, T32=0, TCS=0
        TMR2 = 0x0000;
        PR2 = 0x0100;
        MC.TimerInitalized = true;
    }
    // Setup OC1 (pin3) in PWM mode, with Timer2 as timebase
    OC1CON = 0x8006;    // OC32 = 0, OCTSEL=0, OCM=6
    OC1RS = 0x0000;
    OC1R = 0x0000;
#else
   #error "This chip is not supported!"
#endif

    pinMode(3, OUTPUT);
}

inline void setPWM2(uint8_t s) {
#if defined(__AVR_ATmega8__) || \
    defined(__AVR_ATmega48__) || \
    defined(__AVR_ATmega88__) || \
    defined(__AVR_ATmega168__) || \
    defined(__AVR_ATmega328P__)
    // use PWM from timer2A on PB3 (Arduino pin #11)
    OCR2B = s;
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    // on arduino mega, pin 11 is now PB5 (OC1A)
    OCR3C = s;
#elif defined(__PIC32MX__)
    // Set the OC1 (pin3) PMW duty cycle from 0 to 255
    OC1RS = s;
#else
   #error "This chip is not supported!"
#endif
}

inline void initPWM3(uint8_t freq) {
#if defined(__AVR_ATmega8__) || \
    defined(__AVR_ATmega48__) || \
    defined(__AVR_ATmega88__) || \
    defined(__AVR_ATmega168__) || \
    defined(__AVR_ATmega328P__)
    // use PWM from timer0A / PD6 (pin 6)
    TCCR0A |= _BV(COM0A1) | _BV(WGM00) | _BV(WGM01); // fast PWM, turn on OC0A
    //TCCR0B = freq & 0x7;
    OCR0A = 0;
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    // on arduino mega, pin 6 is now PH3 (OC4A)
    TCCR4A |= _BV(COM1A1) | _BV(WGM10); // fast PWM, turn on oc4a
    TCCR4B = (freq & 0x7) | _BV(WGM12);
    //TCCR4B = 1 | _BV(WGM12);
    OCR4A = 0;
#elif defined(__PIC32MX__)
    if (!MC.TimerInitalized)
    {   // Set up Timer2 for 80MHz counting fro 0 to 256
        T2CON = 0x8000 | ((freq & 0x07) << 4); // ON=1, FRZ=0, SIDL=0, TGATE=0, TCKPS=<freq>, T32=0, TCS=0; // ON=1, FRZ=0, SIDL=0, TGATE=0, TCKPS=0, T32=0, TCS=0
        TMR2 = 0x0000;
        PR2 = 0x0100;
        MC.TimerInitalized = true;
    }
    // Setup OC3 (pin 6) in PWM mode, with Timer2 as timebase
    OC3CON = 0x8006;    // OC32 = 0, OCTSEL=0, OCM=6
    OC3RS = 0x0000;
    OC3R = 0x0000;
#else
   #error "This chip is not supported!"
#endif
    pinMode(6, OUTPUT);
}

inline void setPWM3(uint8_t s) {
#if defined(__AVR_ATmega8__) || \
    defined(__AVR_ATmega48__) || \
    defined(__AVR_ATmega88__) || \
    defined(__AVR_ATmega168__) || \
    defined(__AVR_ATmega328P__)
    // use PWM from timer0A on PB3 (Arduino pin #6)
    OCR0A = s;
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    // on arduino mega, pin 6 is now PH3 (OC4A)
    OCR4A = s;
#elif defined(__PIC32MX__)
    // Set the OC3 (pin 6) PMW duty cycle from 0 to 255
    OC3RS = s;
#else
   #error "This chip is not supported!"
#endif
}



inline void initPWM4(uint8_t freq) {
#if defined(__AVR_ATmega8__) || \
    defined(__AVR_ATmega48__) || \
    defined(__AVR_ATmega88__) || \
    defined(__AVR_ATmega168__) || \
    defined(__AVR_ATmega328P__)
    // use PWM from timer0B / PD5 (pin 5)
    TCCR0A |= _BV(COM0B1) | _BV(WGM00) | _BV(WGM01); // fast PWM, turn on oc0a
    //TCCR0B = freq & 0x7;
    OCR0B = 0;
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    // on arduino mega, pin 5 is now PE3 (OC3A)
    TCCR3A |= _BV(COM1A1) | _BV(WGM10); // fast PWM, turn on oc3a
    TCCR3B = (freq & 0x7) | _BV(WGM12);
    //TCCR4B = 1 | _BV(WGM12);
    OCR3A = 0;
#elif defined(__PIC32MX__)
    if (!MC.TimerInitalized)
    {   // Set up Timer2 for 80MHz counting fro 0 to 256
        T2CON = 0x8000 | ((freq & 0x07) << 4); // ON=1, FRZ=0, SIDL=0, TGATE=0, TCKPS=<freq>, T32=0, TCS=0; // ON=1, FRZ=0, SIDL=0, TGATE=0, TCKPS=0, T32=0, TCS=0
        TMR2 = 0x0000;
        PR2 = 0x0100;
        MC.TimerInitalized = true;
    }
    // Setup OC2 (pin 5) in PWM mode, with Timer2 as timebase
    OC2CON = 0x8006;    // OC32 = 0, OCTSEL=0, OCM=6
    OC2RS = 0x0000;
    OC2R = 0x0000;
#else
   #error "This chip is not supported!"
#endif
    pinMode(5, OUTPUT);
}

inline void setPWM4(uint8_t s) {
#if defined(__AVR_ATmega8__) || \
    defined(__AVR_ATmega48__) || \
    defined(__AVR_ATmega88__) || \
    defined(__AVR_ATmega168__) || \
    defined(__AVR_ATmega328P__)
    // use PWM from timer0A on PB3 (Arduino pin #6)
    OCR0B = s;
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    // on arduino mega, pin 6 is now PH3 (OC4A)
    OCR3A = s;
#elif defined(__PIC32MX__)
    // Set the OC2 (pin 5) PMW duty cycle from 0 to 255
    OC2RS = s;
#else
   #error "This chip is not supported!"
#endif
}

AF_DCMotor::AF_DCMotor(uint8_t num, uint8_t freq) {
  motornum = num;
  pwmfreq = freq;

  MC.enable();

  switch (num) {
  case 1:
    latch_state &= ~_BV(MOTOR1_A) & ~_BV(MOTOR1_B); // set both motor pins to 0
    MC.latch_tx();
    initPWM1(freq);
    break;
  case 2:
    latch_state &= ~_BV(MOTOR2_A) & ~_BV(MOTOR2_B); // set both motor pins to 0
    MC.latch_tx();
    initPWM2(freq);
    break;
  case 3:
    latch_state &= ~_BV(MOTOR3_A) & ~_BV(MOTOR3_B); // set both motor pins to 0
    MC.latch_tx();
    initPWM3(freq);
    break;
  case 4:
    latch_state &= ~_BV(MOTOR4_A) & ~_BV(MOTOR4_B); // set both motor pins to 0
    MC.latch_tx();
    initPWM4(freq);
    break;
  }
}

void AF_DCMotor::run(uint8_t cmd) {
  uint8_t a, b;
  switch (motornum) {
  case 1:
    a = MOTOR1_A; b = MOTOR1_B; break;
  case 2:
    a = MOTOR2_A; b = MOTOR2_B; break;
  case 3:
    a = MOTOR3_A; b = MOTOR3_B; break;
  case 4:
    a = MOTOR4_A; b = MOTOR4_B; break;
  default:
    return;
  }
  
  switch (cmd) {
  case FORWARD:
    latch_state |= _BV(a);
    latch_state &= ~_BV(b); 
    MC.latch_tx();
    break;
  case BACKWARD:
    latch_state &= ~_BV(a);
    latch_state |= _BV(b); 
    MC.latch_tx();
    break;
  case RELEASE:
    latch_state &= ~_BV(a);     // A and B both low
    latch_state &= ~_BV(b); 
    MC.latch_tx();
    break;
  }
}

void AF_DCMotor::setSpeed(uint8_t speed) {
  switch (motornum) {
  case 1:
    setPWM1(speed); break;
  case 2:
    setPWM2(speed); break;
  case 3:
    setPWM3(speed); break;
  case 4:
    setPWM4(speed); break;
  }
}

/******************************************
               STEPPERS
******************************************/

AF_Stepper::AF_Stepper(uint16_t steps, uint8_t num) {
  MC.enable();

  revsteps = steps;
  steppernum = num;
  currentstep = 0;

  if (steppernum == 1) {
    latch_state &= ~_BV(MOTOR1_A) & ~_BV(MOTOR1_B) &
      ~_BV(MOTOR2_A) & ~_BV(MOTOR2_B); // all motor pins to 0
    MC.latch_tx();
    
    // enable both H bridges
    pinMode(11, OUTPUT);
    pinMode(3, OUTPUT);
    digitalWrite(11, HIGH);
    digitalWrite(3, HIGH);

    // use PWM for microstepping support
    initPWM1(STEPPER1_PWM_RATE);
    initPWM2(STEPPER1_PWM_RATE);
    setPWM1(255);
    setPWM2(255);

  } else if (steppernum == 2) {
    latch_state &= ~_BV(MOTOR3_A) & ~_BV(MOTOR3_B) &
      ~_BV(MOTOR4_A) & ~_BV(MOTOR4_B); // all motor pins to 0
    MC.latch_tx();

    // enable both H bridges
    pinMode(5, OUTPUT);
    pinMode(6, OUTPUT);
    digitalWrite(5, HIGH);
    digitalWrite(6, HIGH);

    // use PWM for microstepping support
    // use PWM for microstepping support
    initPWM3(STEPPER2_PWM_RATE);
    initPWM4(STEPPER2_PWM_RATE);
    setPWM3(255);
    setPWM4(255);
  }
}

void AF_Stepper::setSpeed(uint16_t rpm) {
  usperstep = 60000000 / ((uint32_t)revsteps * (uint32_t)rpm);
  steppingcounter = 0;
}

void AF_Stepper::release(void) {
  if (steppernum == 1) {
    latch_state &= ~_BV(MOTOR1_A) & ~_BV(MOTOR1_B) &
      ~_BV(MOTOR2_A) & ~_BV(MOTOR2_B); // all motor pins to 0
    MC.latch_tx();
  } else if (steppernum == 2) {
    latch_state &= ~_BV(MOTOR3_A) & ~_BV(MOTOR3_B) &
      ~_BV(MOTOR4_A) & ~_BV(MOTOR4_B); // all motor pins to 0
    MC.latch_tx();
  }
}

void AF_Stepper::step(uint16_t steps, uint8_t dir,  uint8_t style) {
  uint32_t uspers = usperstep;
  uint8_t ret = 0;

  if (style == INTERLEAVE) {
    uspers /= 2;
  }
 else if (style == MICROSTEP) {
    uspers /= MICROSTEPS;
    steps *= MICROSTEPS;
#ifdef MOTORDEBUG
    Serial.print("steps = "); Serial.println(steps, DEC);
#endif
  }

  while (steps--) {
    ret = onestep(dir, style);
    delay(uspers/1000); // in ms
    steppingcounter += (uspers % 1000);
    if (steppingcounter >= 1000) {
      delay(1);
      steppingcounter -= 1000;
    }
  }
  if (style == MICROSTEP) {
    while ((ret != 0) && (ret != MICROSTEPS)) {
      ret = onestep(dir, style);
      delay(uspers/1000); // in ms
      steppingcounter += (uspers % 1000);
      if (steppingcounter >= 1000) {
	delay(1);
	steppingcounter -= 1000;
      } 
    }
  }
}

uint8_t AF_Stepper::onestep(uint8_t dir, uint8_t style) {
  uint8_t a, b, c, d;
  uint8_t ocrb, ocra;

  ocra = ocrb = 255;

  if (steppernum == 1) {
    a = _BV(MOTOR1_A);
    b = _BV(MOTOR2_A);
    c = _BV(MOTOR1_B);
    d = _BV(MOTOR2_B);
  } else if (steppernum == 2) {
    a = _BV(MOTOR3_A);
    b = _BV(MOTOR4_A);
    c = _BV(MOTOR3_B);
    d = _BV(MOTOR4_B);
  } else {
    return 0;
  }

  // next determine what sort of stepping procedure we're up to
  if (style == SINGLE) {
    if ((currentstep/(MICROSTEPS/2)) % 2) { // we're at an odd step, weird
      if (dir == FORWARD) {
	currentstep += MICROSTEPS/2;
      }
      else {
	currentstep -= MICROSTEPS/2;
      }
    } else {           // go to the next even step
      if (dir == FORWARD) {
	currentstep += MICROSTEPS;
      }
      else {
	currentstep -= MICROSTEPS;
      }
    }
  } else if (style == DOUBLE) {
    if (! (currentstep/(MICROSTEPS/2) % 2)) { // we're at an even step, weird
      if (dir == FORWARD) {
	currentstep += MICROSTEPS/2;
      } else {
	currentstep -= MICROSTEPS/2;
      }
    } else {           // go to the next odd step
      if (dir == FORWARD) {
	currentstep += MICROSTEPS;
      } else {
	currentstep -= MICROSTEPS;
      }
    }
  } else if (style == INTERLEAVE) {
    if (dir == FORWARD) {
       currentstep += MICROSTEPS/2;
    } else {
       currentstep -= MICROSTEPS/2;
    }
  } 

  if (style == MICROSTEP) {
    if (dir == FORWARD) {
      currentstep++;
    } else {
      // BACKWARDS
      currentstep--;
    }

    currentstep += MICROSTEPS*4;
    currentstep %= MICROSTEPS*4;

    ocra = ocrb = 0;
    if ( (currentstep >= 0) && (currentstep < MICROSTEPS)) {
      ocra = microstepcurve[MICROSTEPS - currentstep];
      ocrb = microstepcurve[currentstep];
    } else if  ( (currentstep >= MICROSTEPS) && (currentstep < MICROSTEPS*2)) {
      ocra = microstepcurve[currentstep - MICROSTEPS];
      ocrb = microstepcurve[MICROSTEPS*2 - currentstep];
    } else if  ( (currentstep >= MICROSTEPS*2) && (currentstep < MICROSTEPS*3)) {
      ocra = microstepcurve[MICROSTEPS*3 - currentstep];
      ocrb = microstepcurve[currentstep - MICROSTEPS*2];
    } else if  ( (currentstep >= MICROSTEPS*3) && (currentstep < MICROSTEPS*4)) {
      ocra = microstepcurve[currentstep - MICROSTEPS*3];
      ocrb = microstepcurve[MICROSTEPS*4 - currentstep];
    }
  }

  currentstep += MICROSTEPS*4;
  currentstep %= MICROSTEPS*4;

#ifdef MOTORDEBUG
  Serial.print("current step: "); Serial.println(currentstep, DEC);
  Serial.print(" pwmA = "); Serial.print(ocra, DEC); 
  Serial.print(" pwmB = "); Serial.println(ocrb, DEC); 
#endif

  if (steppernum == 1) {
    setPWM1(ocra);
    setPWM2(ocrb);
  } else if (steppernum == 2) {
    setPWM3(ocra);
    setPWM4(ocrb);
  }


  // release all
  latch_state &= ~a & ~b & ~c & ~d; // all motor pins to 0

  //Serial.println(step, DEC);
  if (style == MICROSTEP) {
    if ((currentstep >= 0) && (currentstep < MICROSTEPS))
      latch_state |= a | b;
    if ((currentstep >= MICROSTEPS) && (currentstep < MICROSTEPS*2))
      latch_state |= b | c;
    if ((currentstep >= MICROSTEPS*2) && (currentstep < MICROSTEPS*3))
      latch_state |= c | d;
    if ((currentstep >= MICROSTEPS*3) && (currentstep < MICROSTEPS*4))
      latch_state |= d | a;
  } else {
    switch (currentstep/(MICROSTEPS/2)) {
    case 0:
      latch_state |= a; // energize coil 1 only
      break;
    case 1:
      latch_state |= a | b; // energize coil 1+2
      break;
    case 2:
      latch_state |= b; // energize coil 2 only
      break;
    case 3:
      latch_state |= b | c; // energize coil 2+3
      break;
    case 4:
      latch_state |= c; // energize coil 3 only
      break; 
    case 5:
      latch_state |= c | d; // energize coil 3+4
      break;
    case 6:
      latch_state |= d; // energize coil 4 only
      break;
    case 7:
      latch_state |= d | a; // energize coil 1+4
      break;
    }
  }

 
  MC.latch_tx();
  return currentstep;
}

#include <Servo.h>

Servo myservo;
const int LED_PIN = 13;
const int analogPin = 0;
int speed = 60; // percent of maximum speed
AF_DCMotor Motor_Left_Rear(1, MOTOR12_1KHZ); // Motor 1
AF_DCMotor Motor_Right_Rear(2, MOTOR12_1KHZ); // Motor 2
AF_DCMotor Motor_Left_Front(4, MOTOR12_1KHZ); // Motor 3
AF_DCMotor Motor_Right_Front(3, MOTOR12_1KHZ); // Motor 4

int pwm, pos, val = 0;
int TopSonarPin = 10;
int topSonarVal;
int PIRSensorVal, PIRSensorState = LOW;


int PIRSensorDetectMotion() 
{
  /*pinMode(A5, INPUT);
  PIRSensorVal = digitalRead(A5);
  Serial.println(PIRSensorVal);
  return PIRSensorVal;*/
 /* pinMode(A5, INPUT);
  PIRSensorVal = digitalRead(A5);
  if(PIRSensorVal == HIGH) {
    if(PIRSensorState == LOW) {
      Serial.println("Motion detected");
      PIRSensorState = HIGH;
    }
  } else {
    if(PIRSensorState == HIGH) {
      Serial.println("Motion ended");
      PIRSensorState = LOW;
    }
  }*/
  
  PIRSensorVal = digitalRead(A5);
  if(PIRSensorVal == LOW) {
   Serial.println("Motion detected");
  } else {
    Serial.println("Motion ended");
  }
  delay(2000);
}

int pingGetDistance(int pingPin)
{
// establish variables for duration of the ping,
// and the distance result in inches and centimeters:
long duration, cm;
// The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
// Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
pinMode(pingPin, OUTPUT);
digitalWrite(pingPin, LOW);
delayMicroseconds(2);
digitalWrite(pingPin, HIGH);
delayMicroseconds(5);
digitalWrite(pingPin, LOW);
pinMode(pingPin, INPUT);
duration = pulseIn(pingPin, HIGH, 20000); // if a pulse does not arrive
// in 20 ms then the ping sensor
// is not connected
if(duration >=20000)
return 0;
// convert the time into a distance
cm = microsecondsToCentimeters(duration);
return cm;
}

long microsecondsToCentimeters(long microseconds)
{
// The speed of sound is 340 m/s or 29 microseconds per centimeter
// The ping travels out and back, so to find the distance of the
// object we take half of the distance travelled.
return microseconds / 29 / 2;
}


//#include "RobotMotor.h" // 2wd or 4wd motor library
//#include "robotDefines.h" // these were the global defines from myRobot
/// Setup runs at startup and is used configure pins and init system variables

// RobotMotor.h - START
// defines for left and right motors
const int MOTOR_LEFT = 0;
const int MOTOR_RIGHT = 1;
extern const int MIN_SPEED;
extern int speedTable[];
extern int rotationTime[];
extern const int SPEED_TABLE_INTERVAL;
extern const int NBR_SPEEDS;


void motorBegin(int motor);
// speed range is 0 to 100 percent
void motorSetSpeed(int motor, int speed);
void motorForward(int motor, int speed);
void motorReverse(int motor, int speed);
void motorStop(int motor);
void motorBrake(int motor);


//#include <Arduino.h>
//#include <AFMotor.h> // adafruit motor shield library
//#include "RobotMotor.h"
const int differential = 0; // % faster left motor turns compared to right
const int MIN_SPEED = 60; // first table entry is 60% speed
const int SPEED_TABLE_INTERVAL = 10; // each table entry is 10% faster speed
const int NBR_SPEEDS = 1 + (100 - MIN_SPEED)/ SPEED_TABLE_INTERVAL;
int speedTable[NBR_SPEEDS] = {60, 70, 80, 90, 100}; // speeds
int rotationTime[NBR_SPEEDS] = {5500, 3300, 2400, 2000, 1750}; // time


AF_DCMotor motors[] = {
AF_DCMotor(4, MOTOR34_1KHZ), // left front is Motor #4
AF_DCMotor(3, MOTOR34_1KHZ), // right front is Motor #3
AF_DCMotor(1, MOTOR12_1KHZ), // left rear is Motor #1
AF_DCMotor(2, MOTOR12_1KHZ) // right rear is Motor #2
};

int motorSpeed[2] = {0,0}; // left and right motor speeds stored here (0-100%)

void motorBegin(int motor)
{
	motorStop(motor); // stop the front motor
	motorStop(motor+2); // stop the rear motor
}

void motorSetSpeed(int motor, int speed)
{
	if( motor == MOTOR_LEFT && speed > differential)
	speed -= differential;
	motorSpeed[motor] = speed; // save the value
	int pwm = map(speed, 0,100, 0,255); // scale to PWM range
	motors[motor].setSpeed(pwm) ;
	motors[motor+2].setSpeed(pwm) ;
}


void motorForward(int motor, int speed)
{
	motorSetSpeed(motor, speed);
	motors[motor].run(FORWARD);
	motors[motor+2].run(FORWARD);
}


void motorReverse(int motor, int speed)
{
	motorSetSpeed(motor, speed);
	motors[motor].run(BACKWARD);
	motors[motor+2].run(BACKWARD);
}


void motorStop(int motor)
{
	// todo set speed to 0 ???
	motors[motor].run(RELEASE); // stopped
	motors[motor+2].run(RELEASE);
}


void motorBrake(int motor)
{
	motors[motor].run(BRAKE); // stopped
	motors[motor+2].run(BRAKE);
}
// RobotMotor.h - END


// ===============================================================================================================================



// The Hello, Robot sketch - START
/***** Global Defines ****/
// defines to identify sensors
const int SENSE_IR_LEFT = 0;
const int SENSE_IR_RIGHT = 1;
const int SENSE_IR_CENTER = 2;
// defines for directions
const int DIR_LEFT = 0;
const int DIR_RIGHT = 1;
const int DIR_CENTER = 2;
const char* locationString[] = {"Left", "Right", "Center"};
const int OBST_NONE = 0; // no obstacle detected
const int OBST_LEFT_EDGE = 1; // left edge detected
const int OBST_RIGHT_EDGE = 2; // right edge detected
const int OBST_FRONT_EDGE = 3; // edge detect at both left and right sensors

//const int LED_PIN = 13;
/**** End of Global Defines ****************/
// Setup runs at startup and is used configure pins and init system variables
/*void setup()
{
  Serial.begin(9600);
  blinkNumber(8); // open port while flashing. Needed for Leonardo only
  motorBegin(MOTOR_LEFT);
  motorBegin(MOTOR_RIGHT);
  irSensorBegin(); // initialize sensors
  pinMode(LED_PIN, OUTPUT); // enable the LED pin for output
  Serial.println("Waiting for a sensor to detect blocked reflection");
}

void loop()
{
  // call a function when reflection blocked on left side
  if(lookForObstacle(OBST_LEFT_EDGE) == true) {
    calibrateRotationRate(DIR_LEFT,360); // calibrate CCW rotation
  }
  // as above for right sensor
  if(lookForObstacle(OBST_RIGHT_EDGE) == true) {
    calibrateRotationRate(DIR_RIGHT, 360); // calibrate CW rotation
  }
}

// function to indicate numbers by flashing the built-in LED
void blinkNumber( byte number) {
  pinMode(LED_PIN, OUTPUT); // enable the LED pin for output
  while(number--) {
    digitalWrite(LED_PIN, HIGH); delay(100);
    digitalWrite(LED_PIN, LOW); delay(400);
  }
}*/





/****************************
ir reflectance sensor code
****************************/
const byte NBR_SENSORS = 3; // this version only has left and right sensors
const byte IR_SENSOR[NBR_SENSORS] = {0, 1, 2}; // analog pins for sensors
int irSensorAmbient[NBR_SENSORS]; // sensor value with no reflection
int irSensorReflect[NBR_SENSORS]; // value considered detecting an object
int irSensorEdge[NBR_SENSORS]; // value considered detecting an edge
boolean isDetected[NBR_SENSORS] = {false,false}; // set true if object detected
const int irReflectThreshold = 10; // % level below ambient to trigger reflection
const int irEdgeThreshold = 90; // % level above ambient to trigger edge

void irSensorBegin()
{
  for(int sensor = 0; sensor < NBR_SENSORS; sensor++)
  irSensorCalibrate(sensor);
}
// calibrate thresholds for ambient light
void irSensorCalibrate(byte sensor)
{
  int ambient = analogRead(IR_SENSOR[sensor]); // get ambient level
  irSensorAmbient[sensor] = ambient;
  // precalculate the levels for object and edge detection
  irSensorReflect[sensor] = (ambient * (long)(100-irReflectThreshold)) / 100;
  irSensorEdge[sensor] = (ambient * (long)(100+irEdgeThreshold)) / 100;
}
// returns true if an object reflection detected on the given sensor
// the sensor parameter is the index into the sensor array
boolean irSensorDetect(int sensor)
{
  boolean result = false; // default value
  int value = analogRead(IR_SENSOR[sensor]); // get IR light level
  
  if( value <= irSensorReflect[sensor]) {
    result = true; // object detected (lower value means more reflection)
    if( isDetected[sensor] == false) { // only print on initial detection
      Serial.print(locationString[sensor]);
      Serial.println(" object detected");
    }
  }
  isDetected[sensor] = result;
  return result;
}

boolean irEdgeDetect(int sensor)
{
  boolean result = false; // default value
  int value = analogRead(IR_SENSOR[sensor]); // get IR light level
  if( value >= irSensorEdge[sensor]) {
    result = true; // edge detected (higher value means less reflection)
    if( isDetected[sensor] == false) { // only print on initial detection
      Serial.print(locationString[sensor]);
      Serial.println(" edge detected");
    }
  }
  isDetected[sensor] = result;
  return result;
}
// The Hello, Robot sketch - END



// The core movement functions - START
const int MOV_LEFT = 0;
const int MOV_RIGHT = 1;
const int MOV_FORWARD = 2;
const int MOV_BACK = 3;
const int MOV_ROTATE = 4;
const int MOV_STOP = 5;

int moveState = MOV_STOP; // what robot is doing
int moveSpeed = 0; // move speed stored here (0-100%)
int speedIncrement = 10; // percent to increase or decrease speed

void moveBegin()
{
  motorBegin(MOTOR_LEFT);
  motorBegin(MOTOR_RIGHT);
  moveStop();
}

void moveLeft()
{
  motorForward(MOTOR_LEFT, 0);
  motorForward(MOTOR_RIGHT, moveSpeed);
  changeMoveState(MOV_LEFT);
}

void moveRight()
{
  motorForward(MOTOR_LEFT, moveSpeed);
  motorForward(MOTOR_RIGHT, 0);
  changeMoveState(MOV_RIGHT);
}

void moveStop()
{
  motorStop(MOTOR_LEFT);
  motorStop(MOTOR_RIGHT);
  changeMoveState(MOV_STOP);
}

void moveBrake()
{
  motorBrake(MOTOR_LEFT);
  motorBrake(MOTOR_RIGHT);
  changeMoveState(MOV_STOP);
}

void moveBackward()
{
  motorReverse(MOTOR_LEFT, moveSpeed);
  motorReverse(MOTOR_RIGHT, moveSpeed);
  changeMoveState(MOV_BACK);
}

void moveForward()
{
  motorForward(MOTOR_LEFT, moveSpeed);
  motorForward(MOTOR_RIGHT, moveSpeed);
  changeMoveState(MOV_FORWARD);
}

void moveSetSpeed(int speed)
{
  motorSetSpeed(MOTOR_LEFT, speed) ;
  motorSetSpeed(MOTOR_RIGHT, speed) ;
  moveSpeed = speed; // save the value
}
// The core movement functions - END




// Additional Core Functions - START
void moveSlower(int decrement)
{
  Serial.print(" Slower: ");
  if( moveSpeed >= speedIncrement + MIN_SPEED)
    moveSpeed -= speedIncrement;
  else
    moveSpeed = MIN_SPEED;
  
  moveSetSpeed(moveSpeed);
}

void moveFaster(int increment)
{
  Serial.print(" Faster: ");
  moveSpeed += speedIncrement;
  if(moveSpeed > 100)
    moveSpeed = 100;
  
  moveSetSpeed(moveSpeed);
}

int moveGetState()
{
  return moveState;
}

const char* states[] = {"Left", "Right", "Forward", "Back", "Rotate", "Stop"};

// this is the low level movement state.
// it will differ from the command state when the robot is avoiding obstacles
void changeMoveState(int newState)
{
  if(newState != moveState)
  {
    Serial.print("Changing move state from ");
    Serial.print( states[moveState]);
    Serial.print(" to ");
    Serial.println(states[newState]);
    moveState = newState;
  }
}

void moveRotate(int angle)
{
  //Serial.print("Rotating "); Serial.println(angle);
  if(angle < 0)
  {
    Serial.println(" (left)");
    motorReverse(MOTOR_LEFT, moveSpeed);
    motorForward(MOTOR_RIGHT, moveSpeed);
    angle = -angle; changeMoveState(MOV_ROTATE);
  }
  else if(angle > 0)
  {
    Serial.println(" (right)");
    motorForward(MOTOR_LEFT, moveSpeed);
    motorReverse(MOTOR_RIGHT, moveSpeed);
    changeMoveState(MOV_ROTATE);
  }
  int ms = rotationAngleToTime(angle, moveSpeed);
  movingDelay(ms);
  moveBrake();
}

// return the time in milliseconds to turn the given angle at the given speed
long rotationAngleToTime( int angle, int speed)
{
  int fullRotationTime; // time to rotate 360 degrees at given speed
  if(speed < MIN_SPEED)
    return 0; // ignore speeds slower then the first table entry
    
  angle = abs(angle);
  if(speed >= 100)
    fullRotationTime = rotationTime[NBR_SPEEDS-1]; // the last entry is 100%
  else
  {
    int index = (speed - MIN_SPEED) / SPEED_TABLE_INTERVAL; // index into speed and time tables
    int t0 = rotationTime[index];
    int t1 = rotationTime[index+1]; // time of the next higher speed
    fullRotationTime = map(speed, speedTable[index], speedTable[index+1], t0, t1);
    // Serial.print("index= "); Serial.print(index); Serial.print(", t0 = "); Serial.print(t0);
    // Serial.print(", t1 = "); Serial.print(t1);
  }
  // Serial.print(" full rotation time = "); Serial.println(fullRotationTime);
  long result = map(angle, 0,360, 0, fullRotationTime);
  return result;
}

// rotate the robot from MIN_SPEED to 100% increasing by SPEED_TABLE_INTERVAL
void calibrateRotationRate(int direction, int angle)
{
  Serial.print(locationString[direction]);
  Serial.println(" calibration" );
  for(int speed = MIN_SPEED; speed <= 100; speed += SPEED_TABLE_INTERVAL)
  {
    delay(1000);
    //blinkNumber(speed/10);
    if( direction == DIR_LEFT)
    { // rotate left
      motorReverse(MOTOR_LEFT, speed);
      motorForward(MOTOR_RIGHT, speed);
    }
    else if( direction == DIR_RIGHT)
    { // rotate right
      motorForward(MOTOR_LEFT, speed);
      motorReverse(MOTOR_RIGHT, speed);
    }
    else
      Serial.println("Invalid direction");
    
    int time = rotationAngleToTime(angle, speed);
    Serial.print(locationString[direction]);
    Serial.print(": rotate "); Serial.print(angle);
    Serial.print(" degrees at speed "); Serial.print(speed);
    Serial.print(" for "); Serial.print(time);
    Serial.println("ms");
    delay(time);
    motorStop(MOTOR_LEFT);
    motorStop(MOTOR_RIGHT);
    delay(2000); // two second delay between speeds
  }
}
// Additional Core Functions - END




// Higher level movement functions - START
//moves in the given direction at the current speed for the given duration in milliseconds
void timedMove(int direction, int duration)
{
  Serial.print("Timed move ");
  if(direction == MOV_FORWARD) {
    Serial.println("forward");
    moveForward();
  }
  else if(direction == MOV_BACK) {
    Serial.println("back");
    moveBackward();
  }
  else
    Serial.println("?");
  
  movingDelay(duration);
  moveStop();
}

// check for obstacles while delaying the given duration in ms
void movingDelay(long duration)
{
  long startTime = millis();
  while(millis() - startTime < duration) {
    // function in Look module checks for obstacle in direction of movement
    if(checkMovement() == false) {
      if( moveState != MOV_ROTATE) // rotate is only valid movement
      {
        Serial.println("Stopping in moving Delay()");
        moveBrake();
      }
    }
  }
}

/**********************
code to look for obstacles
**********************/
const int MIN_DISTANCE = 20; // robot stops when object is nearer (in inches)
const int CLEAR_DISTANCE = 24; // distance in inches considered attractive to move
const int MAX_DISTANCE = 150; // the maximum range of the distance sensor

// angles left, right, center
const int lookAngles[] = { -30, 30, 0};
const byte pingPin = 10; // digital pin 10

const int OBST_FRONT = 4; // obstacle in front
const int OBST_REAR = 5; // obstacle behind


void lookBegin()
{
  irSensorBegin(); // initialize sensors
}

// returns true if the given obstacle is detected
boolean lookForObstacle(int obstacle)
{
  switch(obstacle) {
    case OBST_FRONT_EDGE: return irEdgeDetect(DIR_LEFT) && irEdgeDetect(DIR_RIGHT);
    case OBST_LEFT_EDGE: return irEdgeDetect(DIR_LEFT);
    case OBST_RIGHT_EDGE: return irEdgeDetect(DIR_RIGHT);
    case OBST_FRONT: return lookAt(lookAngles[DIR_CENTER]) <= MIN_DISTANCE;
  }
  return false;
}

// returns the distance of objects at the given angle
// this version rotates the robot
int lookAt(int angle)
{
  moveRotate(angle); // rotate the robot
  int distance, samples;
  long cume;
  distance = samples = cume = 0;
  for(int i =0; i < 4; i++)
  {
    distance = pingGetDistance(pingPin);
    if(distance > 0)
    {
      // printlnValue(" D= ",distance);
      samples++;
      cume+= distance;
    }
  }
  if(samples > 0)
    distance = cume / samples;
  else
    distance = 0;
  
  moveRotate(-angle); // rotate back to original direction
  return distance;
}

// function to check if robot can continue moving when taking evasive action
// returns true if robot is not blocked when moving to avoid obstacles
// this 'placeholder' version always returns true
boolean checkMovement()
{
  boolean isClear = true; // default return value if no obstacles
  if(moveGetState() == MOV_FORWARD)
  {
    if(lookForObstacle(OBST_FRONT) == true)
    {
      isClear = false;
    }
  }
  else if(moveGetState() == MOV_BACK)
  {
    if(lookForObstacle(OBST_REAR) == true)
    {
      isClear = false;
    }
  }
  return isClear;
}

void clearMotors()
{
    motorForward(MOTOR_LEFT, 0);
    motorForward(MOTOR_RIGHT, 0);
    motorReverse(MOTOR_LEFT, 0);
    motorReverse(MOTOR_RIGHT, 0);
}

// Look for and avoid obstacles by rotating robot
void roam(int servoDegree)
{
    int CRIT_VERY_CLOSE_DISTANCE = 10;
    
    int distance = lookAt(lookAngles[DIR_CENTER]);
    if(distance == 0)
    {
        moveStop();
        Serial.println("No front sensor");
        return; // no sensor
    }
    else if(distance <= CRIT_VERY_CLOSE_DISTANCE)
    {
        Serial.println("od strana");
        //moveStop();
        //timedMove(MOV_BACK, 2000);
        
    }
    else if(distance <= MIN_DISTANCE && pos == 90)
    {
        moveStop();
        Serial.print("Scanning:");
        myservo.write(180);
        delay(400);
        int leftDistance = pingGetDistance(pingPin);
        myservo.write(pos);
        if(leftDistance > CLEAR_DISTANCE) {
            Serial.println("LEFT CLEARENCE");
          
            for(int steps=0; steps < 250; steps++)
            {
                motorForward(MOTOR_RIGHT, 90);
                motorReverse(MOTOR_LEFT, 60);
                delay(10);
            }
            
            clearMotors();
            for(int steps=0; steps < 180; steps++)
            {
                motorForward(MOTOR_RIGHT, 90);
                motorForward(MOTOR_LEFT, 90);
                delay(10);
            }
            
            clearMotors();
            for(int steps=0; steps < 270; steps++)
            {
                motorForward(MOTOR_LEFT, 90);
                motorReverse(MOTOR_RIGHT, 60);
                delay(10);
            }
            
            clearMotors();
            for(int steps=0; steps < 220; steps++)
            {
                motorForward(MOTOR_RIGHT, 90);
                motorForward(MOTOR_LEFT, 90);
                delay(10);
            }
            clearMotors();
            delay(100);
        }
        else {
            Serial.println("RIGHT CLEARENCE");  
          
            //delay(500);
            moveStop();
            //myservo.write(0);
            delay(400);
            //int rightDistance = pingGetDistance(pingPin);
            //myservo.write(pos);
            
            //if(rightDistance > CLEAR_DISTANCE) {
                for(int steps=0; steps < 350; steps++)
                {
                    motorForward(MOTOR_LEFT, 90);
                    motorReverse(MOTOR_RIGHT, 60);
                    delay(10);
                }
                
                clearMotors();
                for(int steps=0; steps < 265; steps++)
                {
                    motorForward(MOTOR_RIGHT, 90);
                    motorForward(MOTOR_LEFT, 90);
                    delay(10);
                }
                
                clearMotors();
                for(int steps=0; steps < 400; steps++)
                {
                    motorForward(MOTOR_RIGHT, 90);
                    motorReverse(MOTOR_LEFT, 60);
                    delay(20);
                }
                
                clearMotors();
                for(int steps=0; steps < 395; steps++)
                {
                    motorForward(MOTOR_RIGHT, 90);
                    motorForward(MOTOR_LEFT, 90);
                    delay(10);
                }
                clearMotors();
                delay(100);
                
            //}
            /*else {
                Serial.print(" no clearence : ");
                distance = max( leftDistance, rightDistance);
                if(distance < CLEAR_DISTANCE/2) {
                    timedMove(MOV_BACK, 1000); // back up for one second
                    moveRotate(-180); // turn around
                }
                else {
                    if(leftDistance > rightDistance)
                    moveRotate(-90);
                    else
                    moveRotate(90);
                }
            }*/
        }
        
    }
}

// the following is based on loop code from myRobotEdge
// robot checks for edge and moves to avoid
void avoidEdge()
{
    if( lookForObstacle(OBST_FRONT_EDGE) == true)
    {
        Serial.println("left and right sensors detected edge");
        timedMove(MOV_BACK, 300);
        moveRotate(120);
        while(lookForObstacle(OBST_FRONT_EDGE) == true )
        moveStop(); // stop motors if still over cliff
    }
    else if(lookForObstacle(OBST_LEFT_EDGE) == true)
    {
        Serial.println("left sensor detected edge");
        timedMove(MOV_BACK, 100);
        moveRotate(30);
    }
    else if(lookForObstacle(OBST_RIGHT_EDGE) == true)
    {
        Serial.println("right sensor detected edge");
        timedMove(MOV_BACK, 100);
        moveRotate(-30);
    }
}


// Higher level movement functions - END



// robotDefines.h - START
// robotDefines.h - END

// function to indicate numbers by flashing the built-in LED
void blinkNumber( byte number) {
    pinMode(LED_PIN, OUTPUT); // enable the LED pin for output
    while(number--) {
        digitalWrite(LED_PIN, HIGH); delay(100);
        digitalWrite(LED_PIN, LOW); delay(400);
    }
}


/****************************
Line Sensor code
****************************/
int damping = 5; //1 is most sensitive, range 1 to 1023)
void lineSenseBegin()
{
}

//returns drift - 0 if over line, minus value if left, plus if right
int lineSense()
{
    int leftVal = analogRead(SENSE_IR_LEFT);
    int centerVal = analogRead(SENSE_IR_CENTER);
    int rightVal = analogRead(SENSE_IR_RIGHT);
    int leftSense = centerVal - leftVal;
    int rightSense = rightVal - centerVal;
    int drift = leftVal - rightVal;
    return drift;
}

int lineFollow(int drift, int speed)
{
    int leftSpeed = constrain(speed - (drift / damping), 0, 100);
    int rightSpeed = constrain(speed + (drift / damping), 0, 100);
    motorForward(MOTOR_LEFT, leftSpeed);
    motorForward(MOTOR_RIGHT, rightSpeed);
}


void setup()
{
    Serial.begin(9600);
    blinkNumber(8); // open port while flashing. Needed for Leonardo only
    lookBegin(); /// added Look tab
    moveBegin(); /// added Move tab
    
    irSensorBegin(); // initialize sensors
    pinMode(LED_PIN, OUTPUT); // enable the LED pin for output
    
    lineSenseBegin(); // initialize sensors
    myservo.attach(9);
    Serial.println("Ready");
}




void loop()
{
    speed = 45; //50
    for(pos = 0; pos < 180; pos += 15)  // goes from 0 degrees to 180 degrees
    {                                  // in steps of 1 degree
        myservo.write(pos);              // tell servo to go to position in variable 'pos'
        delay(10);                      // waits 15ms for the servo to reach the position
        int drift = lineSense();
        lineFollow(drift, speed);
        roam(pos); // look around
    }
    for(pos = 180; pos >= 0; pos -= 15)  // goes from 0 degrees to 180 degrees
    {                                  // in steps of 1 degree
        myservo.write(pos);              // tell servo to go to position in variable 'pos'
        delay(10);                      // waits 15ms for the servo to reach the position
        int drift = lineSense();
        lineFollow(drift, speed);
        roam(pos); // look around
    }
}
