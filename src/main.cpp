#include <Arduino.h>

/**
 * Author Teemu Mäntykallio
 *
 * Plot TMC2130 or TMC2660 motor load using the stallGuard value.
 * You can finetune the reading by changing the STALL_VALUE.
 * This will let you control at which load the value will read 0
 * and the stall flag will be triggered. This will also set pin DIAG1 high.
 * A higher STALL_VALUE will make the reading less sensitive and
 * a lower STALL_VALUE will make it more sensitive.
 *
 * You can control the rotation speed with
 * 0 Stop
 * 1 Resume
 * + Speed up
 * - Slow down
 */
#include <TMCStepper.h>

#define MAX_SPEED        40 // In timer value
#define MIN_SPEED      1000

#define STALL_VALUE     100 // [0..255]

#define EN_PIN           19 // Enable
#define DIR_PIN          5 // Direction
#define STEP_PIN         18 // Step
#define SW_RX            16 // TMC2208/TMC2224 SoftwareSerial receive pin
#define SW_TX            17 // TMC2208/TMC2224 SoftwareSerial transmit pin
#define SERIAL_PORT Serial1 // TMC2208/TMC2224 HardwareSerial port
#define DRIVER_ADDRESS 0b00 // TMC2209 Driver address according to MS1 and MS2

#define homeEnd_PIN     39 //Right Side
#define farEnd_PIN      36 //Left Side
#define case_PIN        34 //Front Glass switch
#define stall_PIN       35 //Stall Guard Interrupt high when 
#define spindle_PIN     23 //Spindle PWM pin 
#define spindleDIR_PIN  22 //Spindle DIR pin

#define R_SENSE 0.11f // Match to your driver
                      // SilentStepStick series use 0.11
                      // UltiMachine Einsy and Archim2 boards use 0.2
                      // Panucatt BSD2660 uses 0.1
                      // Watterott TMC5160 uses 0.075

// Select your stepper driver type
TMC2209Stepper driver(&SERIAL_PORT, R_SENSE, DRIVER_ADDRESS);
//TMC2209Stepper driver(SW_RX, SW_TX, R_SENSE, DRIVER_ADDRESS);

//using namespace TMC2208_n;

// Using direct register manipulation can reach faster stepping times
#define STEP_PORT     PORTF // Match with STEP_PIN
#define STEP_BIT_POS      0 // Match with STEP_PIN

hw_timer_t * timer = NULL;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

int passCount = 0;
int wheelCount = 0;
int passMax = 4;
int debugLevel = 1;

void IRAM_ATTR handleInterrupt() {
  portENTER_CRITICAL_ISR(&mux);
  digitalWrite(STEP_PIN, !digitalRead(STEP_PIN));
  portEXIT_CRITICAL_ISR(&mux);
}
 
void IRAM_ATTR handleFarEnd() {
  portENTER_CRITICAL_ISR(&mux);
  digitalWrite(DIR_PIN, !digitalRead(DIR_PIN)); //Change Direction of Steper
  portEXIT_CRITICAL_ISR(&mux);
}

void IRAM_ATTR handleHomeEnd() {
  portENTER_CRITICAL_ISR(&mux);
  digitalWrite(DIR_PIN, !digitalRead(DIR_PIN));
  passCount++;
  if(passCount >= passMax){
    digitalWrite(EN_PIN, HIGH);
  }
  portEXIT_CRITICAL_ISR(&mux);
}

void IRAM_ATTR handleCase() {
  portENTER_CRITICAL_ISR(&mux);
  wheelCount++;
  portEXIT_CRITICAL_ISR(&mux);
}

void IRAM_ATTR handleStall() {
  portENTER_CRITICAL_ISR(&mux);
  digitalWrite(spindle_PIN, HIGH); //Turn off Spindle
  //return home
  portEXIT_CRITICAL_ISR(&mux);
}

void setup() {
  Serial.begin(250000);         // Init serial port and set baudrate
  while(!Serial);               // Wait for serial port to connect
  Serial.println("\nStart...");

  SERIAL_PORT.begin(115200);
  //driver.beginSerial(115200);
  

  pinMode(EN_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW);
  pinMode(farEnd_PIN,INPUT_PULLUP);
  pinMode(homeEnd_PIN,INPUT_PULLUP);
  pinMode(case_PIN,INPUT_PULLUP);
  pinMode(stall_PIN, INPUT);
  pinMode(spindle_PIN, OUTPUT);
  pinMode(spindleDIR_PIN, OUTPUT);

  

  driver.begin();
  driver.toff(4);
  driver.blank_time(24);
  driver.rms_current(400); // mA
  driver.microsteps(16);
  driver.TCOOLTHRS(0xFFFFF); // 20bit max
  driver.semin(5);
  driver.semax(2);
  driver.sedn(0b01);
  driver.SGTHRS(STALL_VALUE);


//Default the stepper to off
  digitalWrite( EN_PIN, HIGH );

  // Set stepper interrupt

 int matchregister = 400;
  timer = timerBegin(0, 8, true);
  timerAttachInterrupt(timer, &handleInterrupt, true);
  timerAlarmWrite(timer, matchregister, true);
  timerAlarmEnable(timer);


}

void loop() {
  if(debugLevel > 0){
    if(digitalRead(homeEnd_PIN) == 1){
      Serial.println("Home Endstop Triggered");
    }
    
    if(digitalRead(farEnd_PIN) == 1){
      Serial.println("Far Endstop Triggered");
    }
    
    if(digitalRead(case_PIN) == 1){
      Serial.println("Case Switch Triggered");
    }
  }
  static uint32_t last_time=0;
  uint32_t ms = millis();

  while(Serial.available() > 0) {
    int8_t read_byte = Serial.read();
    if (read_byte == '0')      { digitalWrite( EN_PIN, HIGH ); }
    else if (read_byte == '1') { digitalWrite( EN_PIN,  LOW ); }
  }

  if((ms-last_time) > 100) { //run every 0.1s
    last_time = ms;

    Serial.print("0 ");
    Serial.print(driver.SG_RESULT(), DEC);
    Serial.print(" ");
    Serial.println(driver.cs2rms(driver.cs_actual()), DEC);
  }
}
