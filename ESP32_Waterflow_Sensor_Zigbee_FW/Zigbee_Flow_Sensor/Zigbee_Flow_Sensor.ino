#ifndef ZIGBEE_MODE_ED
#error "Zigbee end device mode is not selected in Tools->Zigbee mode"
#endif

#include "Zigbee.h"
#include <Wire.h>
 
volatile int pulseCount = 0;   // Variable to count the number of pulses in the last second
unsigned long lastTime = 0;    // Variable to track the last time the flow rate was calculated
double totalLiters = 0;        // Total volume of water passed in liters
 
const int flowMeterPin = D10;   // Define the pin connected to the flow meter
const double pulsesPerLiter = 5880.0; // Number of pulses per liter (adjust as needed)
float counter = 0;
 
void IRAM_ATTR pulse() {
  pulseCount++;               // Increment pulse count for each pulse detected
}

/* Zigbee sensor configuration */
#define FLOW_SENSOR_ENDPOINT_NUMBER 11

#define uS_TO_S_FACTOR 1000000ULL  /* Conversion factor for micro seconds to seconds */

/*Define sleeptimer*/
#define TIME_TO_SLEEP  895         /* Sleep for 55s will + 5s delay for establishing connection => data reported every 1 minute */


uint8_t button = BOOT_PIN;

RTC_DATA_ATTR int bootCount = 0;

ZigbeeFlowSensor zbFlowSensor = ZigbeeFlowSensor(FLOW_SENSOR_ENDPOINT_NUMBER);

//------------------------------------------------------------//
//------------------------------ Functions ------------------------------//
//------------------------------------------------------------//

void calcFlowRate(){
  unsigned long currentTime = millis(); // Get the current time
  //------------------------------ Flow Sensor ------------------------------//
  // Calculate flow rate in liters per minute
  double flowRate = (pulseCount / pulsesPerLiter) * 60.0;

  // Calculate total volume in liters
  totalLiters += (pulseCount / pulsesPerLiter);

  // Reset pulse count for the next second
  pulseCount = 0;

  // Update last time
  lastTime = currentTime;
  Serial1.printf("Updating flow sensor value to %.3f L/min\r\n", flowRate);
  //zbFlowSensor.setFlow(flowRate);
  if (counter > 10){
    counter = 0;
  }
  else{
    counter = counter + 1.234;
  }
  zbFlowSensor.setFlow(bootCount);
  Serial1.printf("Flow-Sensor (DEMO): %d \n", bootCount);
}

void calcBattery(){
  // Battery voltage readout
  uint32_t Vbatt = 0;
  for(int i = 0; i < 16; i++) {
    Vbatt += analogReadMilliVolts(A0); // Read and accumulate ADC voltage
    delay(50); // Small delay to allow ADC to stabilize
  }
  float Vbattf = 2 * Vbatt / 16 / 1000.0;     // Adjust for 1:2 divider and convert to volts
  Serial1.printf("Battery: %0.3f V\r\n", Vbattf);                  // Output voltage to 3 decimal places

  int battPercentage = round(123-123/pow((1+pow(Vbattf/3.7,80)),0.165));
  Serial1.printf("Battery: %d %% \r\n", battPercentage);                  // Output voltage to 3 decimal places
  zbFlowSensor.setBatteryPercentage(battPercentage); 
}

// Internal Led flash
void flashLED() {
  // Turn on LED for 100ms
  digitalWrite(LED_BUILTIN, LOW);
  delay(100);
  digitalWrite(LED_BUILTIN, HIGH);
}

void measureAndSleep() {
  
  calcFlowRate();
  calcBattery();

  // Report values
  zbFlowSensor.report();
  zbFlowSensor.reportBatteryPercentage();

  // Turn on the builtin LED for a very short time
  flashLED();

  // Add small delay to allow the data to be sent before going to sleep
  delay(500);

  // Put device to deep sleep
  Serial1.println("Going to sleep now");
  Serial1.flush(); 
  esp_deep_sleep_start();
}

/*
Method to print the reason by which ESP32
has been awaken from sleep
*/
void print_wakeup_reason(){
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch(wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 : Serial1.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : Serial1.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : Serial1.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial1.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP : Serial1.println("Wakeup caused by ULP program"); break;
    default : Serial1.printf("Wakeup was not caused by deep sleep: %d\n",wakeup_reason); break;
  }
}

//------------------------------------------------------------//
//------------------------------ setup() ------------------------------//
//------------------------------------------------------------//
void setup() {
  //Serial1.begin(115200, SERIAL_8N1, RX1PIN, TX1PIN);
  Serial1.begin(115200, SERIAL_8N1, D7, D6);

  //Increment boot number and print it every reboot
  ++bootCount;
  Serial1.println("Boot number: " + String(bootCount));
  //Print the wakeup reason for ESP32
  print_wakeup_reason();

  // Configure the wake up source and set to wake up every 5 seconds
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  Serial1.println("Setup ESP32 to sleep for every " + String(TIME_TO_SLEEP) +
  " Seconds");

  // Battery voltage readout
  pinMode(A0, INPUT);         // Configure A0 as ADC input
  // Init button switch
  pinMode(button, INPUT_PULLUP);
  // Optional: set Zigbee device name and model
  zbFlowSensor.setManufacturerAndModel("Espressif", "ZigbeeFlowSensor_voltage");
  // Set minimum and maximum flow measurement value in 0,1 m3/h
  zbFlowSensor.setMinMaxValue(0.001, 100.000);
  // Optional: Set tolerance for flow measurement in 0,1 m3/h
  zbFlowSensor.setTolerance(0.1);
  // Set power source to battery and set battery percentage to measured value (now 100% for demonstration)
  // The value can be also updated by calling zbTempSensor.setBatteryPercentage(percentage) anytime
  zbFlowSensor.setPowerSource(ZB_POWER_SOURCE_BATTERY, 100);
  // Add endpoints to Zigbee Core
  Zigbee.addEndpoint(&zbFlowSensor);
  // Create a custom Zigbee configuration for End Device wvith keep alive 10s to aoid interference with reporting data
  esp_zb_cfg_t zigbeeConfig = ZIGBEE_DEFAULT_ED_CONFIG();
  zigbeeConfig.nwk_cfg.zed_cfg.keep_alive = 10000;
  // For battery powered devices, it can be better to set timeout for Zigbee Begin to lower value to save battery
  // If the timeout has been reached, the network channel mask will be reset and the device will try to connect again after reset (scanning all channels)
  Zigbee.setTimeout(10000);  // Set timeout for Zigbee Begin to 10s (default is 30s)
  // When all EPs are registered, start Zigbee in End Device mode
  if (!Zigbee.begin(&zigbeeConfig, false)) {
    Serial1.println("Zigbee failed to start!");
    Serial1.println("Rebooting...");
    ESP.restart();  // If Zigbee failed to start, reboot the device and try again
  }
  Serial1.println("Connecting to network");
  while (!Zigbee.connected()) {
    Serial1.print(".");
    delay(100);
  }
  Serial1.println();
  Serial1.println("Successfully connected to Zigbee network");
  // Delay approx 1s (may be adjusted) to allow establishing proper connection with coordinator, needed for sleepy devices
  delay(1000);
  // Set reporting interval for flow and pressure measurement in seconds, must be called after Zigbee.begin()
  // min_interval and max_interval in seconds, delta (pressure change in hPa, flow change in 0,1 m3/h)
  // if min = 1 and max = 0, reporting is sent only when temperature changes by delta
  // if min = 0 and max = 10, reporting is sent every 10 seconds or temperature changes by delta
  // if min = 0, max = 10 and delta = 0, reporting is sent every 10 seconds regardless of delta change
  zbFlowSensor.setReporting(0, 10, 0.01);

  // Attach interrupt to the flow meter pin
  pinMode(flowMeterPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(flowMeterPin), pulse, RISING);

  // Init LED and turn it OFF (if LED_PIN == RGB_BUILTIN, the rgbLedWrite() will be used under the hood)
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
}

//------------------------------------------------------------//
//------------------------------ loop() ------------------------------//
//------------------------------------------------------------//
void loop() {
  //------------------------------ Factory Reset ------------------------------//
  // Checking button for factory reset
  if (digitalRead(button) == LOW) {  // Push button pressed
    // Key debounce handling
    delay(100);
    int startTime = millis();
    while (digitalRead(button) == LOW) {
      delay(50);
      if ((millis() - startTime) > 10000) {
        // If key pressed for more than 10secs, factory reset Zigbee and reboot
        Serial1.println("Resetting Zigbee to factory and rebooting in 1s.");
        delay(1000);
        // Optional set reset in factoryReset to false, to not restart device after erasing nvram, but set it to endless sleep manually instead
        Zigbee.factoryReset(false);
        Serial1.println("Going to endless sleep, press RESET button or power off/on the device to wake up");
        esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TIMER);
        esp_deep_sleep_start();
      }
    }
  }
  
  //------------------------------ meausureAndSleep ------------------------------//
  measureAndSleep();
}
