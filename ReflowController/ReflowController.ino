/*
 ESTechnical Reflow Oven Controller

 Ed Simmons 2012-2015
 MOD by Akimitsu Sadoi (aki@theledart.com)

 http://www.estechnical.co.uk

 http://www.estechnical.co.uk/reflow-controllers/t962a-reflow-oven-controller-upgrade

 http://www.estechnical.co.uk/reflow-ovens/estechnical-reflow-oven
 */

#include <MAX31855.h>
#include <EEPROM.h>
#include <PID_v1.h>
#include <LiquidCrystal.h>
#include <ParLCD.h>
#include <LCDMenu.h>
#include <Encoder.h> // needed by the menu
#include <MenuItemSelect.h>
#include <MenuItemInteger.h>
#include <MenuItemDouble.h>
#include <MenuItemAction.h>
#include <MenuItemIntegerAction.h>
#include <MenuBase.h>
#include <MenuItemSubMenu.h>

//#define DEBUG

#define VERSION "2.83"  // bump minor version number on small changes, major on large changes, eg when eeprom layout changes

//--- default parameters -------------------------

#define WINDOW_SIZE 250           // control loop duration in milliseconds
                                  // this value controls PWM frequency as well
                                  // slower fan PWM requires lower idle fan speed

#define IDLE_TEMP     60          // the temperature at which to consider the oven safe to leave to cool naturally
#define PREHEAT_TEMP  75          // preheat temperature
#define FANSPEED_DEFAULT 25       // default fan speed
#define MINIMUM_FAN      10       // minimum fan speed allowed

#define OFFSET_FANSPEED 481       // 30 * 16 + 1 one byte wide
#define OFFSET_PROFNUM 482        // 30 * 16 + 2 one byte wide

#define DEBOUNCE_TIME   50        // button debounce time in milliseconds

#define MIN_PREHEAT_TIME  45      // minimum time to hold at preheat temp
#define SOAKSTART_DIFF  2         // start soak this degree lower than set soak temp
#define SOAK_RAMPUP     (0.1)     // ramp-up rate during soak phase

#define DRYING_TEMP     75        // drying temperature
#define DRYING_TIME     (60*5)    // drying duration in seconds

//--- PID tuning parameters ------------
#define Kp    (30.0*WINDOW_SIZE/100) // normalize to percent
#define Ki    (0.2)  // will be normalized to per second
#define Kd    (12.0)  // (was 18) will be normalized to per second
#define fanKp (18.0*WINDOW_SIZE/100) // normalize to percent
#define fanKi (0.1)  // will be normalized to per second
#define fanKd (1.5)  // will be normalized to per second


//--- IO pins ------------------------------------
#define stopKeyInputPin 7
#define fanOutPin 8
#define heaterOutPin 9

//--- thermo couples -----------------------------
#define TC1_InputPin 10          // TC1 used for control
#define TC2_InputPin 2           // TC2 is just for display
#define tcUpdateInterval WINDOW_SIZE/10  // sample temperture more often than PID update

MAX31855 tc1(TC1_InputPin, tcUpdateInterval);
MAX31855 tc2(TC2_InputPin, tcUpdateInterval);

//------------------------------------------------
// data type for the values used in the reflow profile

int profileNumber = 0;  // active profile number

struct profileValues {
  int soakTemp;
  int soakDuration;
  int peakTemp;
  int peakDuration;
  double rampUpRate;
  double rampDownRate;
} activeProfile;

// fan speed* to be included in the profile setting
int fanAssistSpeed = FANSPEED_DEFAULT;

//--- global variables -------------------------------------

// a handful of timer variables
unsigned long startTime, stateChangedTime = 0;

// PID
double Setpoint, Input, Output = 0;

//Specify the links and initial tuning parameters
PID PID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// heater & fan control
unsigned int fanValue, heaterValue;

// PWM control time window
unsigned long windowStartTime;

// vars for keeping track of the temperature ramp (for 1 second)
#define NUMREADINGS (1000/WINDOW_SIZE)   // 10
double airTemp[NUMREADINGS];
double rampRate = 0;

// state machine bits
enum state {
  idle,
  preHeat,
  preHeatforever,
  rampToSoak,
  soak,
  rampToPeak,
  peak,
  rampDown,
  coolDown,
  drying
} currentState = idle, lastState = idle;


//--- initialize LCD -----------------------------
ParLCD lcd(19, 18, 17, 16, 15, 14);

bool redrawDisplay = true;
LCDMenu myMenu;

// reflow profile menu items

MenuItemAction control;
MenuItemAction preheat_start;
MenuItemAction dryer_start;

MenuItemSubMenu profile;
MenuItemDouble rampUp_rate;
MenuItemInteger soak_temp;
MenuItemInteger soak_duration;
MenuItemInteger peak_temp;
MenuItemInteger peak_duration;
MenuItemDouble rampDown_rate;

MenuItemSubMenu profileLoadSave;
MenuItemIntegerAction profileLoad;
MenuItemIntegerAction profileSave;
MenuItemInteger profile_number;
MenuItemAction save_profile;
MenuItemAction load_profile;

MenuItemIntegerAction fan_control;
//MenuItemIntegerAction idle_speed;
//MenuItemAction save_fan_speed;
// Factory Reset is not needed and rather dangerous
//MenuItemAction factory_reset;


//==============================================================================

void abortWithError(int error)
{
  // turn off heater and cool down for safety.
  digitalWrite(heaterOutPin, LOW);
  digitalWrite(fanOutPin, HIGH);

  lcd.clear();

  switch (error) {
    case 1:
      lcd.print(F("Temperature"));
      lcd.setCursor(0, 1);
      lcd.print(F("following error"));
      lcd.setCursor(0, 2);
      lcd.print(F("during heating"));
      break;
      /*
      case 2:
       lcd.print("Temperature");
       lcd.setCursor(0,1);
       lcd.print("following error");
       lcd.setCursor(0,2);
       lcd.print("during cooling");
       break;
       */
    case 3:
      lcd.print(F("Thermocouple input"));
      lcd.setCursor(0, 1);
      lcd.print(F("open circuit"));
      lcd.setCursor(0, 2);
      lcd.print(F("Power off &"));
      lcd.setCursor(0, 3);
      lcd.print(F("check connections"));
      break;
  }
  while (1) { // and stop forever...
  }
}

void displayTemperature(double val)
{
  char tempStr[7];
  dtostrf(val, 5, 1, tempStr);
  displayPaddedString(tempStr, 5);
  lcd.print((char)223);// degrees symbol!
  lcd.print(F("C"));
}

void displayOutputPower(unsigned int val)
{
  val = val*100/WINDOW_SIZE;
  if (val <= 99) lcd.print(" ");
  if (val <= 9) lcd.print(" ");
  lcd.print(val);
  lcd.print(F("%"));
}

void displayRampRate(double val)
{
  char tempStr[6];
  dtostrf(val, 4, 1, tempStr);
  lcd.write((uint8_t)0); // delta symbol
//  lcd.print(F("Temp "));
  displayPaddedString(tempStr, 5);
  lcd.print((char)223);// degrees symbol!
  lcd.print(F("C/S"));
}

void displayTempError(double val)
{
  char tempStr[6];
  dtostrf(val, 4, 1, tempStr);
  lcd.print(F(" "));
  displayPaddedString(tempStr, 5);
  lcd.print((char)223);// degrees symbol!
  lcd.print(F("C"));
}

// prints a string of the defined length
// any space left after the string has been added to the string to print is filled with spaces
void displayPaddedString(char *str, uint8_t length)
{
  uint8_t srcLength = strlen(str);
  uint8_t spaces = length - srcLength;
  lcd.print(str);
  for(uint8_t i=0; i< spaces; i++)
  {
    lcd.print(" ");
  }
}

void displayState()
{
  switch (currentState) {
    case idle:
      displayPaddedString("Idle", 9);
      break;
    case preHeat:
      displayPaddedString("Preheat", 9);
      break;
    case rampToSoak:
      displayPaddedString("Ramp Up 1", 9);
      break;
    case soak:
      displayPaddedString("Soak", 9);
      break;
    case rampToPeak:
      displayPaddedString("Ramp Up 2", 9);
      break;
    case peak:
      displayPaddedString("Peak", 9);
      break;
    case rampDown:
      displayPaddedString("Ramp Down", 9);
      break;
    case coolDown:
      displayPaddedString("Cool Down", 9);
      break;
    case drying:
      displayPaddedString("Drying", 9);
      break;
  }
}

void displayCycleDuration()
{
  char buf[5];
  sprintf(buf, "%dS", (millis() - startTime) / 1000);
  lcd.print(buf);
}

void updateDisplay()
{
  if (redrawDisplay)
  {
    lcd.clear();
    redrawDisplay = false;

    // draw all the static bits now
    lcd.setCursor(10, 1);
    lcd.print(F("Sp"));

    lcd.setCursor(0, 2);
    lcd.print(F("Heat"));

    lcd.setCursor(10, 2);
    lcd.print(F("Fan"));
  }
  lcd.setCursor(0, 0);
  displayTemperature(tc1.getTemperature());

  lcd.setCursor(8, 0);
  if (tc2.getStatus() == 0)
  {
    displayTemperature(tc2.getTemperature());
  }
  else
  {
    lcd.print(F(" ---"));
  }

  lcd.setCursor(16, 0);
  displayCycleDuration();

  lcd.setCursor(0, 1);
  displayState();

  lcd.setCursor(12, 1);
  displayTemperature(Setpoint);

  lcd.setCursor(5, 2);
  displayOutputPower((unsigned int)heaterValue);

  lcd.setCursor(14, 2);
  displayOutputPower((unsigned int)fanValue);

  lcd.setCursor(0, 3);
  displayRampRate(rampRate);
  displayTempError(Input - Setpoint);
}

boolean getJumperState()
{
  boolean result = false; // jumper open
  unsigned int val = analogRead(7);
  if (val < 500) result = true;
  return result;
}

bool isStopKeyPressed()
{
  static boolean lastStopPinState = HIGH;
  static unsigned long lastButtonPush = 0;
  boolean pressed = false;
  boolean stopPin = digitalRead(stopKeyInputPin);
  if (lastButtonPush + DEBOUNCE_TIME > millis()
   && lastStopPinState == HIGH
   && stopPin == LOW) { // if the state has changed
    pressed = true;
  }
  lastStopPinState = stopPin;
  lastButtonPush = millis();

  return pressed;
}

void setupMenu()
{
  boolean jumperState = getJumperState(); // open for T962(A/C) use, closed for toaster conversion kit keypad
  myMenu.init(&control, &lcd, jumperState);

  // initialise the menu strings (stored in the progmem), min and max values, pointers to variables etc
  control.init (F("Reflow Cycle start"),  &cycleStart);
  preheat_start.init (F("Preheat start"),  &preheatStart);
  dryer_start.init (F("Drying start"),  &dryingStart);
  profile.init (F("Edit Profile"));
  rampUp_rate.init(F("Ramp up rate (C/S)"), &activeProfile.rampUpRate, 0.1, 5.0);
  soak_temp.init(F("Soak temp (C)"),  &activeProfile.soakTemp, 50, 180, false);
  soak_duration.init(F("Soak time (S)"), &activeProfile.soakDuration, 10, 300, false);
  peak_temp.init(F("Peak temp (C)"), &activeProfile.peakTemp, 100, 300, false);
  peak_duration.init(F("Peak time (S)"), &activeProfile.peakDuration, 5, 60, false);
  rampDown_rate.init(F("Ramp down rate (C/S)"), &activeProfile.rampDownRate, 0.1, 10);
  profileLoad.init(F("Load Profile"), &loadProfile, &profileNumber, F("Select Profile"), 0, 29, true);
  profileSave.init(F("Save Profile"), &saveProfile, &profileNumber, F("Select Profile"), 0, 29, true);
  fan_control.init(F("Fan settings"),  &saveFanSpeed, &fanAssistSpeed, F("Set Idle Speed"), 0, 70, false);
//  factory_reset.init(F("Factory Reset"),  &factoryReset);

  // initialise the menu structure
  control.addItem(&preheat_start);
  control.addItem(&dryer_start);
  control.addItem(&profile);
  profile.addChild(&rampUp_rate);
  rampUp_rate.addItem(&soak_temp);
  soak_temp.addItem(&soak_duration);
  soak_duration.addItem(&peak_temp);
  peak_temp.addItem(&peak_duration);
  peak_duration.addItem(&rampDown_rate);


  control.addItem(&profileLoad);
  control.addItem(&profileSave);

  // fan speed control
  control.addItem(&fan_control);

  //factory reset function
//  control.addItem(&factory_reset);

  // set up the LCD's number of columns and rows:
  lcd.begin(20, 4);

  byte deltaChar[8] = {
    0b00000,
    0b00100,
    0b00100,
    0b01010,
    0b01010,
    0b10001,
    0b11111,
    0b00000
  };
  lcd.createChar(0, deltaChar);
}

void displaySplash()
{
  lcd.print(" ESTechnical.co.uk");
  lcd.setCursor(0, 1);
  lcd.print(" Reflow controller");
  lcd.setCursor(7, 2);
  lcd.print("v");
  lcd.print(VERSION);
}

void setup()
{
  Serial.begin(115200);
  Serial.println("--- Controller Started. ---");

  setupMenu();

  lcd.clear();

  if (firstRun()) {
    factoryReset();
    loadParameters(0);
  }
  else {
    loadLastUsedProfile();
    loadFanSpeed();
  }

  displaySplash(); // nothing else modifies the LCD display until the loop runs...

  tc1.setup();
  tc2.setup();
  tc1.update();
  tc2.update();

  pinMode(fanOutPin, OUTPUT);
  pinMode(heaterOutPin, OUTPUT);

  PID.SetSampleTime(WINDOW_SIZE);  // adjust for the actual sample time
  PID.SetOutputLimits(0, WINDOW_SIZE);
  //turn the PID on
  PID.SetMode(AUTOMATIC);

  if (tc1.getStatus() != 0) {
    abortWithError(3);
  }

  // not sure this is needed in this form any more...
  for (int i = 0; i < NUMREADINGS; i++) {
    airTemp[i] = tc1.getTemperature();
  }

  while ((millis() < 5000) && (isStopKeyPressed() == false)); // interruptible delay to show the splash screen

  startTime = millis();
  myMenu.showCurrent();

}


void loop()
{
  static unsigned long nextSerialOutput = 0, nextDisplayUpdate = 0;
  static boolean stateChanged = true;

  // check for the stop key being pressed ----------------------------

  if (currentState == idle) {
    myMenu.poll();
  } else
  if (isStopKeyPressed()) { // if the key pressed
#ifdef DEBUG
    Serial.println("-stop key-");
#endif
    if (currentState == coolDown) {
      currentState = idle;
    } else {
      currentState = coolDown;
    }
  }

  // update temperature at independent interval from PID -------------
  tc1.update();
  tc2.update();

  if (tc1.getStatus() != 0) {
    abortWithError(3);
  }

  Input = tc1.getTemperature(); // update the variable the PID reads

  // compute PID and update output -----------------------------------
  if (PID.Compute()) {

    // calculate the temperature delta per second --------------------
    rampRate = Input - airTemp[0]; // subtract earliest reading from the current one
    // this gives us the rate of rise in degrees per polling cycle time/ num readings

    // need to keep track of a few past readings in order to work out rate of rise
    for (int i = 1; i < NUMREADINGS; i++) { // iterate over all previous entries, moving them backwards one index
      airTemp[i - 1] = airTemp[i];
    }
    airTemp[NUMREADINGS - 1] = Input; // update the last index with the newest average

    //----------------------------------------------------------------

    // if the state has changed, set the flags and update the time of state change
    if (currentState != lastState) {
      lastState = currentState;
      stateChanged = true;
      stateChangedTime = millis();
    }

    if (currentState != idle) {
      if (millis() >= nextDisplayUpdate) { // 2hz display during reflow cycle
        nextDisplayUpdate = millis() + 500;
        updateDisplay();
      }
    }

    // profile cycle state machine -----------------------------------

    switch (currentState) {
      case idle:
        if (stateChanged) {
          Serial.print("-Idle-");
          Serial.print(" Kp: ");
          Serial.print((double)fanKp);
          Serial.print(" Ki: ");
          Serial.print((double)fanKi);
          Serial.print(" Kd: ");
          Serial.print((double)fanKd);
          Serial.println("");
          myMenu.showCurrent();
          PID.SetMode(MANUAL);
          PID.SetControllerDirection(REVERSE);
          PID.SetTunings(fanKp, fanKi, fanKd);
          Output = 0;
          Setpoint = IDLE_TEMP;
          PID.SetMode(AUTOMATIC);
        } else
        {
          heaterValue = 0;
          fanValue = Output;
        }
        break;

      case preHeatforever:
      case preHeat:
        if (stateChanged) {
          Serial.print("-preHeat-");
          Serial.println("");
          PID.SetMode(MANUAL);
          Output = 0;
          PID.SetControllerDirection(DIRECT);
          PID.SetTunings(Kp, Ki, Kd);
          Setpoint = PREHEAT_TEMP;
          PID.SetMode(AUTOMATIC);
        }
        else
        {
          heaterValue = Output;
          fanValue = fanAssistSpeed*WINDOW_SIZE/100;

          if ( (currentState == preHeat)
            && (Input >= Setpoint)
            && (millis() - stateChangedTime >= (unsigned long) MIN_PREHEAT_TIME * 1000) ) {
            currentState = rampToSoak;
          }
        }
        break;

      case rampToSoak:
        if (stateChanged) {
          Serial.print("-rampToSoak-");
          Serial.println("");
          PID.SetControllerDirection(DIRECT);
          PID.SetTunings(Kp, Ki, Kd);
          Setpoint = Input;
        }
        else
        {
          heaterValue = Output;
          fanValue = fanAssistSpeed*WINDOW_SIZE/100;

          // adjust the set point
          Setpoint += (activeProfile.rampUpRate / (1000/WINDOW_SIZE)); // target set ramp up rate

          if (Setpoint >= activeProfile.soakTemp - SOAKSTART_DIFF) {
            currentState = soak;
          }
        }
        break;

      case soak:
        if (stateChanged) {
          Serial.println("-soak-");
          Setpoint = activeProfile.soakTemp;
        } else
        {
          heaterValue = Output;
          fanValue = fanAssistSpeed*WINDOW_SIZE/100;
        }

        // adjust the set point
        Setpoint += (SOAK_RAMPUP / (1000/WINDOW_SIZE)); //

        if (millis() - stateChangedTime >= (unsigned long) activeProfile.soakDuration * 1000) {
          currentState = rampToPeak;
        }
        break;

      case rampToPeak:
        if (stateChanged) {
          Serial.println("-rampToPeak-");
        } else
        {
          heaterValue = Output;
          fanValue = fanAssistSpeed*WINDOW_SIZE/100;
        }

        // adjust the set point
        Setpoint += (activeProfile.rampUpRate / (1000/WINDOW_SIZE)); // target set ramp up rate

        if (Setpoint >= activeProfile.peakTemp - 1) { // seems to take arodun 8 degrees rise to tail off to 0 rise
          Setpoint = activeProfile.peakTemp;
          currentState = peak;
        }
        break;

      case peak:
        if (stateChanged) {
          Serial.println("-peak-");
          Setpoint = activeProfile.peakTemp;
        } else
        {
          heaterValue = Output;
          fanValue = fanAssistSpeed*WINDOW_SIZE/100;
        }

        if (millis() - stateChangedTime >= (unsigned long) activeProfile.peakDuration * 1000) {
          currentState = rampDown;
        }
        break;

      case rampDown:
        if (stateChanged) {
          Serial.println("-rampDown-");
          PID.SetControllerDirection(REVERSE);
          PID.SetTunings(fanKp, fanKi, fanKd);
          Setpoint = activeProfile.peakTemp;
        } else
        {
          heaterValue = 0;
          fanValue = Output;
        }

        if (Setpoint > IDLE_TEMP)
        {
          // adjust the set point
          Setpoint -= (activeProfile.rampDownRate / (1000/WINDOW_SIZE));
        } else
        if (Input <= IDLE_TEMP)
        {
          currentState = idle;
        }
        break;

      case coolDown:
        if (stateChanged) {
          Serial.println("-coolDown-");
          PID.SetControllerDirection(REVERSE);
          PID.SetTunings(fanKp, fanKi, fanKd);
          Setpoint = 0;   // rapid cool down
        }
        heaterValue = 0;
        fanValue = Output;

        if (Input <= IDLE_TEMP) {
          currentState = idle;
        }
        break;

      case drying:
        if (stateChanged) {
          Serial.print("-drying-");
          PID.SetControllerDirection(DIRECT);
          PID.SetTunings(Kp, Ki, Kd);
          Setpoint = DRYING_TEMP;
        }
        else
        {
          heaterValue = Output;
          fanValue = WINDOW_SIZE;  // fan at full speed

          if (millis() - stateChangedTime >= (unsigned long) DRYING_TIME * 1000) {
            currentState = coolDown;
          }
        }
        break;

    }
    stateChanged = false;
  }

  // safety check that we're not doing something stupid.
  // if the thermocouple is wired backwards, temp goes DOWN when it increases
  // during cooling, the t962a lags a long way behind, hence the hugely lenient cooling allowance.

//  if (Setpoint > Input + 50) abortWithError(1); // if we're 50 degree cooler than setpoint, abort

  // keep fan value above the minimum to make sure the fan actually runs
  if (fanValue > 0 && fanValue < MINIMUM_FAN*WINDOW_SIZE/100) {
    fanValue = MINIMUM_FAN*WINDOW_SIZE/100;
  }

  // PWM control heater & fan ----------------------------------------

  if (millis() - windowStartTime > WINDOW_SIZE)
  { //time to shift the Relay Window
    windowStartTime += WINDOW_SIZE;
  }

  if (heaterValue < millis() - windowStartTime) {
    digitalWrite(heaterOutPin, LOW);
  }
  else {
    digitalWrite(heaterOutPin, HIGH);
  }

  if (fanValue < millis() - windowStartTime) {
    digitalWrite(fanOutPin, LOW);
  }
  else {
    digitalWrite(fanOutPin, HIGH);
  }

  // report status via serial ----------------------------------------
  if (millis() >= nextSerialOutput) {
    nextSerialOutput = millis() + 500;
    sendSerialUpdate();
  }

}



void cycleStart()
{
  startTime = millis();
  currentState = preHeat; //rampToSoak;
  lcd.clear();
  lcd.print("Starting cycle ");
  lcd.print(profileNumber);
  delay(1000);
  redrawDisplay = true;
}

void preheatStart()
{
  startTime = millis();
  currentState = preHeatforever;
  lcd.clear();
  lcd.print("Starting preheat");
  delay(1000);
  redrawDisplay = true;
}

void dryingStart()
{
  startTime = millis();
  currentState = drying;
  lcd.clear();
  lcd.print("Starting dryer");
  delay(1000);
  redrawDisplay = true;
}

void saveProfile(unsigned int targetProfile)
{
  profileNumber = targetProfile;
  lcd.clear();
  lcd.print("Saving profile ");
  lcd.print(profileNumber);

#ifdef DEBUG
  Serial.println("Check parameters:");
  Serial.print("idleTemp ");
  Serial.println(IDLE_TEMP);
  Serial.print("ramp Up rate ");
  Serial.println(activeProfile.rampUpRate);
  Serial.print("soakTemp ");
  Serial.println(activeProfile.soakTemp);
  Serial.print("soakDuration ");
  Serial.println(activeProfile.soakDuration);
  Serial.print("peakTemp ");
  Serial.println(activeProfile.peakTemp);
  Serial.print("peakDuration ");
  Serial.println(activeProfile.peakDuration);
  Serial.print("rampDownRate ");
  Serial.println(activeProfile.rampDownRate);
  Serial.println("About to save parameters");
#endif

  saveParameters(profileNumber); // profileNumber is modified by the menu code directly, this method is called by a menu action

  delay(500);
}

void loadProfile(unsigned int targetProfile)
{
  // We may be able to do-away with profileNumber entirely now the selection is done in-function.
  profileNumber = targetProfile;
  lcd.clear();
  lcd.print("Loading profile ");
  lcd.print(profileNumber);
  saveLastUsedProfile();

#ifdef DEBUG
  Serial.println("Check parameters:");
  Serial.print("idleTemp ");
  Serial.println(IDLE_TEMP);
  Serial.print("ramp Up rate ");
  Serial.println(activeProfile.rampUpRate);
  Serial.print("soakTemp ");
  Serial.println(activeProfile.soakTemp);
  Serial.print("soakDuration ");
  Serial.println(activeProfile.soakDuration);
  Serial.print("peakTemp ");
  Serial.println(activeProfile.peakTemp);
  Serial.print("peakDuration ");
  Serial.println(activeProfile.peakDuration);
  Serial.print("rampDownRate ");
  Serial.println(activeProfile.rampDownRate);
  Serial.println("About to load parameters");
#endif

  loadParameters(profileNumber);

#ifdef DEBUG
  Serial.println("Check parameters:");
  Serial.print("idleTemp ");
  Serial.println(IDLE_TEMP);
  Serial.print("ramp Up rate ");
  Serial.println(activeProfile.rampUpRate);
  Serial.print("soakTemp ");
  Serial.println(activeProfile.soakTemp);
  Serial.print("soakDuration ");
  Serial.println(activeProfile.soakDuration);
  Serial.print("peakTemp ");
  Serial.println(activeProfile.peakTemp);
  Serial.print("peakDuration ");
  Serial.println(activeProfile.peakDuration);
  Serial.print("rampDownRate ");
  Serial.println(activeProfile.rampDownRate);
  Serial.println("after loading parameters");
#endif

  delay(500);
}


void saveParameters(unsigned int profile)
{

  unsigned int offset = 0;
  if (profile != 0) offset = profile * 16;


  EEPROM.write(offset, lowByte(activeProfile.soakTemp));
  offset++;
  EEPROM.write(offset, highByte(activeProfile.soakTemp));
  offset++;

  EEPROM.write(offset, lowByte(activeProfile.soakDuration));
  offset++;
  EEPROM.write(offset, highByte(activeProfile.soakDuration));
  offset++;

  EEPROM.write(offset, lowByte(activeProfile.peakTemp));
  offset++;
  EEPROM.write(offset, highByte(activeProfile.peakTemp));
  offset++;

  EEPROM.write(offset, lowByte(activeProfile.peakDuration));
  offset++;
  EEPROM.write(offset, highByte(activeProfile.peakDuration));
  offset++;

  int temp = activeProfile.rampUpRate * 10;
  EEPROM.write(offset, (temp & 255));
  offset++;
  EEPROM.write(offset, (temp >> 8) & 255);
  offset++;

  temp = activeProfile.rampDownRate * 10;
  EEPROM.write(offset, (temp & 255));
  offset++;
  EEPROM.write(offset, (temp >> 8) & 255);
  offset++;

}

void loadParameters(unsigned int profile) {
  unsigned int offset = 0;
  if (profile != 0) offset = profile * 16;


  activeProfile.soakTemp = EEPROM.read(offset);
  offset++;
  activeProfile.soakTemp |= EEPROM.read(offset) << 8;
  offset++;

  activeProfile.soakDuration = EEPROM.read(offset);
  offset++;
  activeProfile.soakDuration |= EEPROM.read(offset) << 8;
  offset++;

  activeProfile.peakTemp = EEPROM.read(offset);
  offset++;
  activeProfile.peakTemp |= EEPROM.read(offset) << 8;
  offset++;

  activeProfile.peakDuration = EEPROM.read(offset);
  offset++;
  activeProfile.peakDuration |= EEPROM.read(offset) << 8;
  offset++;

  int temp = EEPROM.read(offset);
  offset++;
  temp |= EEPROM.read(offset) << 8;
  offset++;
  activeProfile.rampUpRate = ((double)temp / 10);

  temp = EEPROM.read(offset);
  offset++;
  temp |= EEPROM.read(offset) << 8;
  offset++;
  activeProfile.rampDownRate = ((double)temp / 10);

}


boolean firstRun()
{
  // we check the whole of the space of the 16th profile, if all bytes are 255, we are doing the very first run
  unsigned int offset = 16;
  for (unsigned int i = offset * 15; i < (offset * 15) + 16; i++) {
    if (EEPROM.read(i) != 255) return false;
  }
  lcd.clear();
  lcd.print("First run...");
  delay(500);
  return true;
}

void factoryReset()
{
  // clear any adjusted settings first, just to be sure...
  activeProfile.soakTemp = 130;
  activeProfile.soakDuration = 30;
  activeProfile.peakTemp = 220;
  activeProfile.peakDuration = 10;

  activeProfile.rampUpRate = 0.80;
  activeProfile.rampDownRate = 3.0;
  lcd.clear();
  lcd.print("Resetting...");

  // then save the same profile settings into all slots
  for (int i = 0; i < 30; i++) {
    saveParameters(i);
  }
  fanAssistSpeed = FANSPEED_DEFAULT;
  saveFanSpeed(0);
  profileNumber = 0;
  saveLastUsedProfile();
  delay(500);
}

void saveFanSpeed(unsigned int var)
{
  unsigned int temp = (unsigned int) fanAssistSpeed;
  EEPROM.write(OFFSET_FANSPEED, (temp & 255));
  lcd.clear();
  lcd.print("Saving...");
  delay(250);

}

void loadFanSpeed()
{
  unsigned int temp = 0;
  temp = EEPROM.read(OFFSET_FANSPEED);
  fanAssistSpeed = (int) temp;
}

void saveLastUsedProfile()
{
  EEPROM.write(OFFSET_PROFNUM, (profileNumber & 255));
}

void loadLastUsedProfile()
{
  profileNumber = (int) EEPROM.read(OFFSET_PROFNUM);
  loadParameters(profileNumber);
}

void sendSerialUpdate()
{
#if 0
  if (currentState == idle)
  {
    Serial.print(millis() - startTime);
    Serial.print(", ");
    Serial.print((int)currentState);
    Serial.print(", ");
    Serial.print("0, 0, 0, ");
    Serial.print(tc1.getTemperature());
    Serial.print(", ");
    if (tc2.getStatus() == 0)
    {
      Serial.print(tc2.getTemperature());
    }
    else
    {
      Serial.print("999");
    }
    Serial.println();
  }
  else
#endif
  {
    Serial.print(millis() - startTime);
    Serial.print(", ");
    Serial.print((int)currentState);
    Serial.print(", ");
    Serial.print(Setpoint);
    Serial.print(", ");
    Serial.print(heaterValue*100/WINDOW_SIZE);
    Serial.print("%, ");
    Serial.print(fanValue*100/WINDOW_SIZE);
    Serial.print("%, ");
    Serial.print(tc1.getTemperature());
    Serial.print(", ");
    if (tc2.getStatus() == 0)
    {
      Serial.print(tc2.getTemperature());
    }
    else
    {
      Serial.print("999");
    }
    Serial.print(", ");
    Serial.print(Input-Setpoint);
    Serial.println();
  }
}
