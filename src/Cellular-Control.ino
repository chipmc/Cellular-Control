/*
* Project Cellular-Montoring - converged software for Monitoring Control Systems
* Description: Cellular Connected Data Logger for Utility and Solar powered installations
* Author: Chip McClelland chip@mcclellands.org
* Date:19 Jan 2018
*/

/*  This is the start of an update to the well head controllers we move toward taking control over the pumps
    In this release, we will no longer listen for the signals over the wire but will, instead take commands 
    via webhook from Ubidots and by subscribing to a Publish channel that the water storage faility will call out on.
      - First step, water storage faility calls "pumps on" and Ubidots has an event that calls the Particle.function - Pump Control
      - Next step, rewite the storage controller software to move to Publish, this device subscribes and no Ubidots Event needed 
          - This is better in the long run as it can ensure that the pump will continue pumping if there is a reset

    The mode will be set and recoded in the CONTROLREGISTER so resets will not change the mode
    Control Register - bits 7-4, 3 - Verbose Mode, 2- Solar Power Mode, 1 - Pumping, 0 - Low Power Mode
    We won't use Solar or Low Power modes at this time but will keep control register structure
*/

// v1.48 - Removing the old system of control by wire.  Particularly fixeing issue where webhook turns on pubm and wire turns it off
// v1.49 - Ripped out the old FRAM stuff and moved to Pub / Sub for control
// v1.50 - Relase Candidate - Fixed a few bugs
// v1.51 - Fixed sampling during pumping - streamlined program flow and reduced lines of code
// v1.54 - Added Time Zone and DST calculations.  Added more messaging to understand WDT issues.
// v1.55 - Still having issues with the watchdog - added petting at Setup and Reporting.
// v1.56 - Moved to all 32-bit values in FRAM
// v1.57 - Added a pumping control variable
// v1.58 - Added a persistent pumping lockout feature
// v1.59 - Fixed issue with pump lockout value not being saved to FRAM
// v1.60 - Moved failsafe from 60 to 95 mins, moved to deviceOS@2.0.1

// Namespace for the FRAM storage
namespace FRAM {                                    // Moved to namespace instead of #define to limit scope
  enum Addresses {
    versionAddr           = 0x00,                   // 8- bits - Where we store the memory map version number
    controlRegisterAddr   = 0x04,                   // 8- bits - The control register for the device
    timeZoneAddr          = 0x08,                   // 8- bits - The time zone for the device (Note, assumes US based Device)
    resetCountAddr        = 0x0C,                   // 32-bits - How many resets today
    pumpingLockoutAddr    = 0x10,                   // Adding a function to lock out pumping
    dailyPumpingMinsAddr  = 0x20,                   // 32-bits - How many minutes have we pumped today
    pumpingStartAddr      = 0x24,                   // 32-bits - Unix Time
    lastHookResponseAddr  = 0x28                    // 32-bits - Unix Time 
  };
};


// Finally, here are the variables I want to change often and pull them all together here
#define FRAMMEMORYMAPVERSION 1
#define SOFTWARERELEASENUMBER "1.60"
#define PUMPCHANNEL "FallsLakeBeaverDamn-FallsLake3-PumpControl"
#define DSTRULES isDSTusa

// Included Libraries
#include "MB85RC256V-FRAM-RK.h"
#include "electrondoc.h"                                              // Documents pinout

// Prototypes and System Mode calls
SYSTEM_MODE(SEMI_AUTOMATIC);                                          // These devices are always connected
SYSTEM_THREAD(ENABLED);                                               // Means my code will not be held up by Particle processes.
STARTUP(System.enableFeature(FEATURE_RESET_INFO));
FuelGauge batteryMonitor;                                             // Prototype for the fuel gauge (included in Particle core library)
PMIC power;                                                           // Enables us to monitor the power supply to the board
MB85RC64 fram(Wire, 0);

// State Machine Variables
enum State { INITIALIZATION_STATE, ERROR_STATE, IDLE_STATE, PUMPING_STATE, LOW_BATTERY_STATE, REPORTING_STATE, RESP_WAIT_STATE };
char stateNames[8][14] = {"Initialize", "Error", "Idle", "Pumping", "Low Battery", "Reporting", "Response Wait" };
State state = INITIALIZATION_STATE;
State oldState = INITIALIZATION_STATE;

// Pin Constants
const int tmp36Pin =          A0;               // Simple Analog temperature sensor
const int pumpCurrentPin =    A2;               // Pin for Current Sensor - Brown-White
const int pumpControlPin =    A4;               // Pin for Turning on the pump - Brown
const int wakeUpPin =         A7;               // This is the Particle Electron WKP pin
const int controlPowerPin =   B1;               // Pin for Voltage Sensor V-SNS-0 - Orange-White
const int pumpOnPin =         B2;               // Pin for Voltage Sensor V-SNS-1 - Green-White  -- This is only for pumps with wired Pump control like Falls Lake 2
const int lowLevelPin =       B3;               // Pin for Voltage Sensor V-SNS-2 - Green
const int anyOnDetectPin =    B4;               // Pin for anyOnDetectPin interrupt - Blue
const int tmp36Shutdwn =      B5;               // Can turn off the TMP-36 to save energy
const int hardResetPin =      D4;               // Power Cycles the Electron and the Carrier Board
const int userSwitch =        D5;               // User switch with a pull-up resistor
const int donePin =           D6;               // Pin the Electron uses to "pet" the watchdog
const int blueLED =           D7;               // This LED is on the Electron itself

// On the headers, GND is Blue-White and 3.3V is Orange

// Timing Variables
unsigned long webhookWait = 45000;                                    // How long we will wair for a webhook response
unsigned long resetWait = 30000;                                      // Honw long we will wait before resetting on an error
unsigned long sampleFrequency = 2000;                                 // How often will we take a sample
unsigned long webhookTimeStamp = 0;
unsigned long resetTimeStamp = 0;
volatile bool watchdogFlag = false;

// Program Variables
int temperatureF;                                                     // Global variable so we can monitor via cloud variable
int resetCount;                                                       // Counts the number of times the Electron has had a pin reset
const char* releaseNumber = SOFTWARERELEASENUMBER;                    // Displays the release on the menu
byte controlRegister;                                                 // Stores the control register values
int lowBattLimit = 30;                                                // Trigger for Low Batt State
bool verboseMode;                                                     // Enables more active communications for configutation and setup
char SignalString[64];                                                // Used to communicate Wireless RSSI and Description
const char* radioTech[10] = {"Unknown","None","WiFi","GSM","UMTS","CDMA","LTE","IEEE802154","LTE_CAT_M1","LTE_CAT_NB1"};
char currentOffsetStr[10];                                            // What is our offset from UTC


// FRAM and Unix time variables
time_t t;
int alertValue = 0;                                                   // Current Active Alerts
bool dataInFlight = false;                                            // Tracks if we have sent data but not yet cleared it from counts until we get confirmation
byte currentHourlyPeriod;                                             // This is where we will know if the period changed

// Battery monitor
int stateOfCharge = 0;                                                // stores battery charge level value

// Pump control and monitoriing
int pumpAmps = 0;
int pumpCurrentRaw = 0;
time_t pumpingStart = 0;
int dailyPumpingMins = 0;
bool pumpCalled = false;
bool pumpLockOut = false;


Timer pumpBackupTimer(5700000, pumpTimerCallback, true);              // This sets a limit on how long we can pump - set to 95 minutes at Vinny's Request - email 3/3/21

void setup()                                                          // Note: Disconnected Setup()
{
  pinMode(pumpControlPin,OUTPUT);                                     // Turns on the pump
  pinMode(pumpCurrentPin,INPUT);                                      // Senses the pump current
  pinMode(controlPowerPin,INPUT);                                     // Voltage Sensor Interrupt pin
  pinMode(lowLevelPin,INPUT);                                         // Voltage Sensor Interrupt pin
  pinMode(userSwitch,INPUT);                                          // Momentary contact button on board for direct user input
  pinMode(blueLED, OUTPUT);                                           // declare the Blue LED Pin as an output
  pinMode(tmp36Shutdwn,OUTPUT);                                       // Supports shutting down the TMP-36 to save juice
  digitalWrite(tmp36Shutdwn, HIGH);                                   // Turns on the temp sensor
  pinMode(hardResetPin,OUTPUT);                                       // For a hard reset active HIGH

  pinMode(wakeUpPin,INPUT);                                           // This pin is active HIGH
  pinResetFast(donePin);
  pinMode(donePin,OUTPUT);                                            // Allows us to pet the watchdog
  petWatchdog();                                                      // Proactively pet the watchdog
  attachInterrupt(wakeUpPin, watchdogISR, RISING);                    // The watchdog timer will signal us and we have to response

  char responseTopic[125];
  String deviceID = System.deviceID();                                // Multiple Electrons share the same hook - keeps things straight
  deviceID.toCharArray(responseTopic,125);
  Particle.subscribe(responseTopic, UbidotsHandler, MY_DEVICES);      // Subscribe to the integration response event

  if (Particle.subscribe(PUMPCHANNEL, pumpControlHandler, MY_DEVICES)) {
    Particle.publish("PubSub", "Subscribe successful",PRIVATE);
  }
  else Particle.publish("PubSub", "Subscribe Not successful",PRIVATE);

  Particle.variable("AlertValue", alertValue);
  Particle.variable("Signal", SignalString);
  Particle.variable("ResetCount", resetCount);
  Particle.variable("Temperature",temperatureF);
  Particle.variable("Release",releaseNumber);
  Particle.variable("stateOfChg", stateOfCharge);
  Particle.variable("pumpAmps",pumpAmps);
  Particle.variable("pumpMinutes",dailyPumpingMins);
  Particle.variable("TimeOffset",currentOffsetStr);
  Particle.variable("PumpCalled", pumpCalled);
  Particle.variable("PumpLockOut", pumpLockOut);

  Particle.function("Reset-FRAM", resetFRAM);
  Particle.function("PumpCalled",pumpControl);
  Particle.function("Reset-Counts",resetCounts);
  Particle.function("Hard-Reset",hardResetNow);
  Particle.function("Send-Now",sendNow);
  Particle.function("Verbose-Mode",setVerboseMode);
  Particle.function("Set-Timezone",setTimeZone);
  Particle.function("PumpLockout",setPumpLockout);


  Particle.connect();

  fram.begin();                                                         // Initializes Wire but does not return a boolean on successful initialization

  fram.get(FRAM::resetCountAddr, resetCount);                           // Retrive system recount data from FRAM
  if (System.resetReason() == RESET_REASON_PIN_RESET) {                 // Check to see if we are starting from a pin reset
    resetCount++;
    fram.put(FRAM::resetCountAddr,resetCount);                          // If so, store incremented number - watchdog must have done This
  }

  fram.get(FRAM::controlRegisterAddr, controlRegister);
  verboseMode = (0b00001000 & controlRegister);                         // verboseMode
  fram.get(FRAM::dailyPumpingMinsAddr,dailyPumpingMins);                // Reload so we don't loose track
  if (controlRegister & 0b00000010) {                                   // This means we reset while pumpting
    fram.get(FRAM::pumpingStartAddr,pumpingStart);                      // Reload the pumping start time
  }
  // Get Time Squared Away
  int8_t tempTimeZoneValue;
  fram.get(FRAM::timeZoneAddr,tempTimeZoneValue);
  if (tempTimeZoneValue > 12 || tempTimeZoneValue < -12) {
    tempTimeZoneValue = -5;
    fram.put(FRAM::timeZoneAddr,tempTimeZoneValue);                     // Load the default value into FRAM for next time
  }
  Time.zone((float)tempTimeZoneValue);                                  // Implement the local time Zone value
  snprintf(currentOffsetStr,sizeof(currentOffsetStr),"%2.1f UTC",(Time.local() - Time.now()) / 3600.0);

  stateOfCharge = int(batteryMonitor.getSoC());                         // Percentage of full charge
  if (stateOfCharge > lowBattLimit) connectToParticle();                // If not low battery, we can connect

  pumpBackupTimer.stop();

  fram.get(FRAM::pumpingLockoutAddr,pumpLockOut);                       // Retreive the value from memory so it persists

  if (state != ERROR_STATE) state = IDLE_STATE;                         // IDLE unless error from above code
}

void loop()
{
  switch(state) {
  case IDLE_STATE:
    if (verboseMode && state != oldState) publishStateTransition();
    if (watchdogFlag)  {
      waitUntil(meterParticlePublish);
      Particle.publish("State","Watchdog flag",PRIVATE);
      petWatchdog();
    }
    if (Time.hour() != currentHourlyPeriod) state = REPORTING_STATE;              // We want to report on the hour
    if (stateOfCharge <= lowBattLimit) state = LOW_BATTERY_STATE;                 // The battery is low - sleep
    if (pumpCalled || digitalRead(pumpControlPin)) state = PUMPING_STATE;     // If we are pumping, we need to report
    if (meterSampleRate()) takeMeasurements();                                    // Take measurements every couple seconds
    break;

  case PUMPING_STATE: {
    if (verboseMode && state != oldState) publishStateTransition();

    if (pumpCalled && !pumpLockOut && !digitalRead(pumpControlPin)) {               // First time to this state we will turn on the pump and report
      digitalWrite(pumpControlPin,HIGH);
      digitalWrite(blueLED,HIGH);
      pumpBackupTimer.start();
    }
    else if (!pumpCalled && digitalRead(pumpControlPin)) {
      digitalWrite(pumpControlPin,LOW);
      digitalWrite(blueLED,LOW);
      pumpBackupTimer.stop();
    }
    state = IDLE_STATE;                                                 // Go back to IDLE to make sure housekeeping is done
  } break;

  case LOW_BATTERY_STATE: {
    if (verboseMode && state != oldState) publishStateTransition();
      if (Particle.connected()) disconnectFromParticle();               // If connected, we need to disconned and power down the modem
      digitalWrite(blueLED,LOW);                                        // Turn off the LED
      digitalWrite(pumpControlPin,LOW);                                 // Turn off the pump as we cannot monitor in our sleep
      digitalWrite(tmp36Shutdwn, LOW);                                  // Turns off the temp sensor
      int secondsToHour = (60*(60 - Time.minute()));                    // Time till the top of the hour
      System.sleep(SLEEP_MODE_DEEP,secondsToHour);                      // Very deep sleep till the next hour - then resets
  } break;

  case REPORTING_STATE: 
    if (verboseMode && state != oldState) publishStateTransition();
    if (Particle.connected()) {
      if (alertValue != 0)  resolveAlert();
      petWatchdog();                                                    // Proactively pet the watchdog
      sendEvent();                                                      // Send data to Ubidots
      state = RESP_WAIT_STATE;                                          // Wait for Response
    }
    else state = ERROR_STATE;
    break;

  case RESP_WAIT_STATE:
    if (verboseMode && state != oldState) publishStateTransition();
    if (!dataInFlight) {
      if (Time.hour() == 2 && Time.isValid()) DSTRULES() ? Time.beginDST() : Time.endDST();    // Each day, at 2am we will check to see if we need a DST offset
      if (Time.hour() == 0) dailyCleanup();                                                    // Each day at midnight, we need to clean up and zero counts
      state = IDLE_STATE;                                               // Response received
    }
    else if (millis() - webhookTimeStamp >= webhookWait) {              // If it takes too long - will need to reset
      resetTimeStamp = millis();
      if (verboseMode) {
        waitUntil(meterParticlePublish);
        Particle.publish("State","Response Timeout Error",PRIVATE);
      }
      state = ERROR_STATE;                                              // Response timed out
    }
    break;

    case ERROR_STATE: {                                                 // Here is where we deal with errors
      if (verboseMode && state != oldState) publishStateTransition();
      unsigned long lastWebHookResponse;
      fram.get(FRAM::lastHookResponseAddr,lastWebHookResponse);
      if (millis() - resetTimeStamp >= resetWait)
      {
        if (resetCount <= 3) {                                          // First try simple reset
          waitUntil(meterParticlePublish);
          if (Particle.connected()) Particle.publish("State","Error State - Reset", PRIVATE);    // Brodcast Reset Action
          delay(2000);
          System.reset();
        }
        else if (Time.now() - lastWebHookResponse > 7200L) {            //It has been more than two hours since a sucessful hook response
          waitUntil(meterParticlePublish);
          if (Particle.connected()) Particle.publish("State","Error State - Power Cycle", PRIVATE);  // Broadcast Reset Action
          delay(2000);
          fram.put(FRAM::resetCountAddr,0);                             // Zero the ResetCount
          digitalWrite(hardResetPin,HIGH);                              // This will cut all power to the Electron AND the carrier board
        }
        else {                                                          // If we have had 3 resets - time to do something more
          waitUntil(meterParticlePublish);
          if (Particle.connected()) Particle.publish("State","Error State - Full Modem Reset", PRIVATE);            // Brodcase Reset Action
          delay(2000);
          fram.put(FRAM::resetCountAddr,0);                             // Zero the ResetCount
          fullModemReset();                                             // Full Modem reset and reboots
        }
      }
    } break;
  }
}

void pumpTimerCallback() { pumpCalled = false; }

void resolveAlert() {                                                   // This function takes the AlertValue and publishes a message
  char data[128];
  if (alertValue & 0b00000001) strcat(data,"Control Power - ");
  if (alertValue & 0b00000010) strcat(data,"Low Level - ");
  if (alertValue & 0b00000100) strcat(data,"Pump On - ");
  if (alertValue & 0b10000000) strcat(data,"Particle Power");
  waitUntil(meterParticlePublish);
  if(Particle.connected()) Particle.publish("Alerts",data,PRIVATE);
}

void sendEvent() {
  char data[256];                                                       // Store the date in this character array - not global
  snprintf(data, sizeof(data), "{\"alertValue\":%i, \"pumpAmps\":%i, \"pumpMins\":%i, \"battery\":%i, \"temp\":%i, \"resets\":%i}",alertValue, pumpAmps, dailyPumpingMins, stateOfCharge, temperatureF,resetCount);
  waitUntil(meterParticlePublish);
  Particle.publish("Monitoring_Event", data, PRIVATE);
  webhookTimeStamp = millis();
  currentHourlyPeriod = Time.hour();                                    // Change the time period since we have reported for this one 
  dataInFlight = true;                                                  // set the data inflight flag
}

void UbidotsHandler(const char *event, const char *data) { // Looks at the response from Ubidots - Will reset Photon if no successful response
  // Response Template: "{{hourly.0.status_code}}"
  if (!data) {                                                          // First check to see if there is any data
    waitUntil(meterParticlePublish);
    Particle.publish("Ubidots Hook", "No Data",PRIVATE);
    return;
  }
  int responseCode = atoi(data);                                        // Response is only a single number thanks to Template
  if ((responseCode == 200) || (responseCode == 201)) {
    if (verboseMode) {
      waitUntil(meterParticlePublish);
      Particle.publish("State","Response Received",PRIVATE);
    }
    fram.put(FRAM::lastHookResponseAddr,Time.now());                    // Keep track of last hook response
    dataInFlight = false;                                               // Data has been received
  }
  else {
    waitUntil(meterParticlePublish);
    Particle.publish("Ubidots Hook", data, PRIVATE);                    // Publish the response code
  }
}


void getSignalStrength() {
  // New Signal Strength capability - https://community.particle.io/t/boron-lte-and-cellular-rssi-funny-values/45299/8
  CellularSignal sig = Cellular.RSSI();
  auto rat = sig.getAccessTechnology();
 
  //float strengthVal = sig.getStrengthValue();
  float strengthPercentage = sig.getStrength();

  //float qualityVal = sig.getQualityValue();
  float qualityPercentage = sig.getQuality();

  snprintf(SignalString,sizeof(SignalString), "%s S:%2.0f%%, Q:%2.0f%% ", radioTech[rat], strengthPercentage, qualityPercentage);
}

int getTemperature() {
  int reading = analogRead(tmp36Pin);                                   //getting the voltage reading from the temperature sensor
  float voltage = reading * 3.3;                                        // converting that reading to voltage, for 3.3v arduino use 3.3
  voltage /= 4096.0;                                                    // Electron is different than the Arduino where there are only 1024 steps
  int temperatureC = int(((voltage - 0.5) * 100));                      //converting from 10 mv per degree with 500 mV offset to degrees ((voltage - 500mV) times 100) - 5 degree calibration
  temperatureF = int((temperatureC * 9.0 / 5.0) + 32.0);                // now convert to Fahrenheit
  return temperatureF;
}

// Here is were we will put the timer and other ISRs
void watchdogISR() {                                                    // The watchdog ISR
  watchdogFlag = true;
}

void petWatchdog() {
  digitalWriteFast(donePin, HIGH);                                      // Pet the watchdog not done in the ISR so we can ensure that we are transiting the main loop - petting occurs in IDLE
  digitalWriteFast(donePin, LOW);
  watchdogFlag = false;
}

// These functions manage our connecion to Particle
bool connectToParticle() {
  if (!Cellular.ready())
  {
    Cellular.on();                                                      // turn on the Modem
    Cellular.connect();                                                 // Connect to the cellular network
    if(!waitFor(Cellular.ready,90000)) return false;                    // Connect to cellular - give it 90 seconds
  }
  Particle.process();
  Particle.connect();                                                   // Connect to Particle
  if(!waitFor(Particle.connected,30000)) return false;                  // Connect to Particle - give it 30 seconds
  Particle.process();
  return true;
}

bool disconnectFromParticle() {
  Particle.disconnect();                                                // Disconnect from Particle in prep for sleep
  waitFor(notConnected,10000);
  Cellular.disconnect();                                                // Disconnect from the cellular network
  delay(3000);
  Cellular.off();                                                       // Turn off the cellular modem
  return true;
}

bool notConnected() {
  return !Particle.connected();                                         // This is a requirement to use waitFor
}


// Take measurements
void takeMeasurements() {
  static byte lastAlertValue = 0;                                       // Last value - so we can detect a change
  static int lastPumpAmps = 0;                                          // Need to make sure we are reporting signficant changes here
  fram.get(FRAM::controlRegisterAddr, controlRegister);                 // Check out the control register
  bool pumpAmpsSignificantChange = false;                               // Don't want to waste bandwidth reporting small changes

  // Gather the measurements
  if (Cellular.ready()) getSignalStrength();                            // Test signal strength if the cellular modem is on and ready
  getTemperature();                                                     // Get Temperature at startup as well
  stateOfCharge = int(batteryMonitor.getSoC());                         // Percentage of full charge
  pumpCurrentRaw = analogRead(pumpCurrentPin);                          // Current sensor is fairly linear from 1 to 32 Amps
  pumpAmps = map(pumpCurrentRaw,0,4095,0,32);                           // Map analog voltage to current
  if (pumpAmps >= lastPumpAmps + 2 || pumpAmps <= lastPumpAmps - 2) pumpAmpsSignificantChange = true;

  // Build the Alert Value
  alertValue = 0b00000000;                                              // Reset for each run through
  if (!pinReadFast(controlPowerPin)) alertValue = alertValue | 0b00000001;      // Set the value for alertValue - This is opposite - power is good
  if (!pinReadFast(lowLevelPin)) alertValue = alertValue | 0b00000010;  // Set the value for alertValue
  if (pumpCalled)                                                   // If the pump is on
  {
    alertValue = alertValue | 0b00000100;                               // Set the value for alertValue
    if (!(controlRegister & 0b00000010)) {                              // This is a new pumping session
      pumpingStart = Time.now();
      fram.put(FRAM::pumpingStartAddr,pumpingStart);                    // Write to FRAM in case of a reset
      fram.put(FRAM::controlRegisterAddr, controlRegister | 0b00000010);// Turn on the pumping bit
    }
  }
  else if (controlRegister & 0b00000010) {                              // If the pump is off but the pumping flag is set
    fram.put(FRAM::controlRegisterAddr, controlRegister ^ 0b00000010);  // It is on and I want to turn the pumping bit off with an xor
    time_t pumpingStop = Time.now();
    dailyPumpingMins += int(difftime(pumpingStop,pumpingStart)/60);     // Add to the total for the day
    fram.put(FRAM::dailyPumpingMinsAddr,dailyPumpingMins);              // Store it in FRAM in case of a reset
  }
  if (System.powerSource() == 5) alertValue = alertValue | 0b10000000;  // Set the value for alertValue if we are on battery power

  if (alertValue != lastAlertValue || pumpAmpsSignificantChange) state = REPORTING_STATE;
  lastAlertValue = alertValue;
  lastPumpAmps = pumpAmps;
}


/* These are the particle functions that allow you to configure and run the device
 * They are intended to allow for customization and control during installations
 * and to allow for management.
*/

int pumpControl(String command)                                         // This is the API end point to turn the pump on and off
{
  if (command == "1") {
    pumpCalled = true;
    return 1;
  }
  else if (command == "0") {
    pumpCalled = false;
    return 1;
  }
  else return 0;
}

int setPumpLockout(String command) {                                        // This is a way to esnure the pump will not pump even when called
  if (command == "1") {
    Particle.publish("Lockout","True",PRIVATE);
    pumpLockOut = true;
    fram.put(FRAM::pumpingLockoutAddr,pumpLockOut);
    return 1;
  }
  else if (command == "0") {
    Particle.publish("Lockout","False",PRIVATE);
    pumpLockOut = false;
    fram.put(FRAM::pumpingLockoutAddr,pumpLockOut);
    return 1;
  }
  else return 0;
}


int resetFRAM(String command)                                           // Will reset the local counts
{
  if (command == "1") {
    fram.erase();
    return 1;
  }
  else return 0;
}

int resetCounts(String command)                                         // Resets the current hourly and daily counts
{
  if (command == "1") {
    fram.put(FRAM::resetCountAddr,0);                                   // If so, store incremented number - watchdog must have done This
    resetCount = 0;
    dataInFlight = false;
    dailyPumpingMins = 0;
    fram.put(FRAM::dailyPumpingMinsAddr,0);
    alertValue = 0;
    return 1;
  }
  else return 0;
}

int hardResetNow(String command)                                        // Will perform a hard reset on the Electron
{
  if (command == "1") {
    digitalWrite(hardResetPin,HIGH);                                    // This will cut all power to the Electron AND the carrir board
    return 1;                                                           // Unfortunately, this will never be sent
  }
  else return 0;
}

int sendNow(String command)                                             // Function to force sending data in current hour
{
  if (command == "1") {
    state = REPORTING_STATE;
    return 1;
  }
  else return 0;
}

int setVerboseMode(String command)                                      // Function to force sending data in current hour
{
  if (command == "1") {
    verboseMode = true;
    fram.get(FRAM::controlRegisterAddr,controlRegister);
    controlRegister = (0b00001000 | controlRegister);                   // Turn on verboseMode
    fram.put(FRAM::controlRegisterAddr, controlRegister);               // Write it to the register
    waitUntil(meterParticlePublish);
    Particle.publish("Mode","Set Verbose Mode",PRIVATE);
    return 1;
  }
  else if (command == "0") {
    verboseMode = false;
    fram.get(FRAM::controlRegisterAddr,controlRegister);
    controlRegister = (0b11110111 & controlRegister);                   // Turn off verboseMode
    fram.put(FRAM::controlRegisterAddr, controlRegister);               // Write it to the register
    waitUntil(meterParticlePublish);
    Particle.publish("Mode","Cleared Verbose Mode",PRIVATE);
    return 1;
  }
  else return 0;
}

bool meterParticlePublish(void) {
  static unsigned long lastPublish = 0;                                 // Keep track of when we publish a webhook
  if(millis() - lastPublish >= 1000) {
    lastPublish = millis();
    return 1;
  }
  else return 0;
}

bool meterSampleRate(void) {
  static unsigned long lastSample = 0;
  if(millis() - lastSample >= sampleFrequency) {
    lastSample = millis();
    return 1;
  }
  else return 0;
}

void fullModemReset() {  // Adapted form Rikkas7's https://github.com/rickkas7/electronsample
	Particle.disconnect(); 	                                              // Disconnect from the cloud
	unsigned long startTime = millis();  	                                // Wait up to 15 seconds to disconnect
	while(Particle.connected() && millis() - startTime < 15000) {
		delay(100);
	}
	// Reset the modem and SIM card
	// 16:MT silent reset (with detach from network and saving of NVM parameters), with reset of the SIM card
	Cellular.command(30000, "AT+CFUN=16\r\n");
	delay(1000);
	// Go into deep sleep for 10 seconds to try to reset everything. This turns off the modem as well.
	System.sleep(SLEEP_MODE_DEEP, 10);
}

void pumpControlHandler(const char *event, const char *data)
{
  char * pEND;
  int onOrOff = strtol(data,&pEND,10);
  if (onOrOff == 1) {
    pumpCalled = true;
    waitUntil(meterParticlePublish);
    Particle.publish("Status", "Pump On Received",PRIVATE);
  }
  else if (onOrOff == 0) {
    pumpCalled = false;
    waitUntil(meterParticlePublish);
    Particle.publish("Status", "Pump Off Received",PRIVATE);
  }
}


void dailyCleanup() {                                                   // Function to clean house at the end of the day
  fram.get(FRAM::controlRegisterAddr,controlRegister);

  waitUntil(meterParticlePublish);
  Particle.publish("Daily Cleanup","Running", PRIVATE);                 // Make sure this is being run

  verboseMode = false;
  controlRegister = (0b11110111 & controlRegister);                     // Turn off verboseMode

  dailyPumpingMins = 0;                                                 // Zero for the day
  fram.put(FRAM::dailyPumpingMinsAddr,0);

  Particle.syncTime();                                                  // Set the clock each day
  waitFor(Particle.syncTimeDone,30000);                                 // Wait for up to 30 seconds for the SyncTime to complete

  fram.put(FRAM::controlRegisterAddr, controlRegister);
}

void publishStateTransition(void) {                                     // Mainly for troubleshooting - publishes the transition between states
  char stateTransitionString[40];
  snprintf(stateTransitionString, sizeof(stateTransitionString), "From %s to %s", stateNames[oldState],stateNames[state]);
  if(Particle.connected()) {
    waitUntil(meterParticlePublish);
    Particle.publish("State Transition",stateTransitionString, PRIVATE);
  }
  oldState = state;
}

int setTimeZone(String command) {                                       // Set the Base Time Zone vs GMT - not Daylight savings time
  char * pEND;
  char data[256];
  time_t t = Time.now();
  int8_t tempTimeZoneValue = strtol(command,&pEND,10);                  // Looks for the first integer and interprets it
  if ((tempTimeZoneValue < -12) || (tempTimeZoneValue > 12)) return 0; // Make sure it falls in a valid range or send a "fail" result
  Time.zone((float)tempTimeZoneValue);
  fram.put(FRAM::timeZoneAddr,tempTimeZoneValue);                       // Load the default value into FRAM for next time
  snprintf(data, sizeof(data), "Time base time zone is %i",tempTimeZoneValue);
  waitUntil(meterParticlePublish);
  if (Time.isValid()) DSTRULES() ? Time.beginDST() : Time.endDST();     // Perform the DST calculation here 
  snprintf(currentOffsetStr,sizeof(currentOffsetStr),"%2.1f UTC",(Time.local() - Time.now()) / 3600.0);
  if (Particle.connected()) Particle.publish("Time",data, PRIVATE);
  waitUntil(meterParticlePublish);
  if (Particle.connected()) Particle.publish("Time",Time.timeStr(t), PRIVATE);
  return 1;
}

bool isDSTusa() { 
  // United States of America Summer Timer calculation (2am Local Time - 2nd Sunday in March/ 1st Sunday in November)
  // Adapted from @ScruffR's code posted here https://community.particle.io/t/daylight-savings-problem/38424/4
  // The code works in from months, days and hours in succession toward the two transitions
  int dayOfMonth = Time.day();
  int month = Time.month();
  int dayOfWeek = Time.weekday() - 1; // make Sunday 0 .. Saturday 6

  // By Month - inside or outside the DST window
  if (month >= 4 && month <= 10)  
  { // April to October definetly DST
    return true;
  }
  else if (month < 3 || month > 11)
  { // before March or after October is definetly standard time
    return false;
  }

  boolean beforeFirstSunday = (dayOfMonth - dayOfWeek < 0);
  boolean secondSundayOrAfter = (dayOfMonth - dayOfWeek > 7);

  if (beforeFirstSunday && !secondSundayOrAfter) return (month == 11);
  else if (!beforeFirstSunday && !secondSundayOrAfter) return false;
  else if (!beforeFirstSunday && secondSundayOrAfter) return (month == 3);

  int secSinceMidnightLocal = Time.now() % 86400;
  boolean dayStartedAs = (month == 10); // DST in October, in March not
  // on switching Sunday we need to consider the time
  if (secSinceMidnightLocal >= 2*3600)
  { //  In the US, Daylight Time is based on local time 
    return !dayStartedAs;
  }
  return dayStartedAs;
}