/**
 * @file PenthuAura_Health_Monitor.ino
 * @author PenthuAura Team
 * @brief Firmware for an ESP32-based health monitoring device named PenthuAura.
 * @version 2.3.3
 * @date 2025-06-24
 *
 * @details
 * This firmware operates a comprehensive health monitoring system. It interfaces with
 * a MAX30102 sensor for SpO2 and Heart Rate data, and a MAX30205 sensor for temperature.
 * The device features a state machine to manage measurement cycles, which involve:
 * 1. Acquiring a stable temperature reading.
 * 2. Sending raw PPG (Photoplethysmography) data in chunks to a Raspberry Pi server via a fetched ngrok URL.
 * 3. Fetching processed results (SpO2, HR, Blood Pressure, Stress, Anomaly, Health State) from the RPi via ThingSpeak.
 * 4. Displaying results on a 16x2 LCD screen and providing local alerts via an RGB LED and buzzer.
 *
 * It includes a web server for configuration, WiFi management (STA and AP modes), and viewing
 * real-time/historical data. Power management is handled via light and deep sleep modes,
 * with wake-up triggers from a timer or a physical button. Data persistence across
 * deep sleeps is achieved using RTC memory.
 *
 * Core Components:
 * - ESP32 Microcontroller
 * - MAX30102 SpO2/HR Sensor
 * - MAX30205 Temperature Sensor
 * - 16x2 I2C Liquid Crystal Display (LCD)
 * - Push Button for user interaction
 * - RGB LED and Buzzer for alerts
 * - LittleFS for storing configuration files.
 * - ThingSpeak for data exchange between ESP32 and a remote server (RPi).
 */

// =================================================================================================
// ===================================== LIBRARIES =================================================
// =================================================================================================
#include <WiFi.h>                   // For WiFi connectivity (STA and AP modes)
#include <ESPAsyncWebServer.h>      // For creating a non-blocking web server for configuration
#include <LittleFS.h>               // For onboard file system to store configurations
#include <HTTPClient.h>             // To make HTTP requests (e.g., to ThingSpeak and the RPi server)
#include <ArduinoJson.h>            // For parsing and creating JSON data for the web server and APIs
#include <Wire.h>                   // For I2C communication with sensors (MAX30102, MAX30205)
#include <cmath>                    // For mathematical functions
#include <LiquidCrystal.h>          // For controlling the 16x2 character LCD
#include "MAX30105.h"               // Library for the MAX30102/MAX30105 particle sensor
#include "esp_sleep.h"              // For controlling light and deep sleep functionality

// =================================================================================================
// =============================== CONFIGURATION CONSTANTS =========================================
// =================================================================================================

// --- Sleep and Inactivity Timers ---
const unsigned long ACTIVE_IDLE_DURATION_MS = 10000;      // Time (ms) to stay awake after user interaction before entering light sleep.
const unsigned long DEEP_SLEEP_INACTIVITY_MS = 3 * 60 * 1000; // Inactivity time (ms) before entering deep sleep (3 minutes).

// --- Measurement Cycle Timers ---
const unsigned long REVIEW_PERIOD_MS = 10000;             // Time (ms) to display results on the LCD after a cycle completes.
unsigned long reviewPeriodStartTime = 0;                  // Tracks the start time of the review period.
const int CALCULATION_BUFFER_SIZE = 100;                  // Buffer size for short-term sensor readings (e.g., 1 second of data at 100Hz).

// --- Hardware Pin Definitions ---
const int BUZZER_PIN = 25;           // GPIO pin connected to the buzzer for audio alerts.
const int RGB_LED_R_PIN = 32;        // GPIO pin for the Red component of the RGB LED.
const int RGB_LED_G_PIN = 33;        // GPIO pin for the Green component of the RGB LED.
const int RGB_LED_B_PIN = 26;        // GPIO pin for the Blue component of the RGB LED.
const int LCD_RS_PIN = 19;           // LCD Register Select pin.
const int LCD_E_PIN = 18;            // LCD Enable pin.
const int LCD_D4_PIN = 5;            // LCD Data 4 pin.
const int LCD_D5_PIN = 17;           // LCD Data 5 pin.
const int LCD_D6_PIN = 16;           // LCD Data 6 pin.
const int LCD_D7_PIN = 4;            // LCD Data 7 pin.
const int BUTTON_PIN = 23;           // GPIO pin for the user input button.

// --- Button Behavior Constants ---
const unsigned long DEBOUNCE_DELAY = 50;                  // Debounce delay (ms) to prevent multiple reads from a single button press.
const unsigned long MULTI_PRESS_TIMEOUT = 400;            // Timeout (ms) to distinguish between single, double, or triple presses.
const unsigned long LONG_PRESS_DURATION = 2000;           // Duration (ms) a button must be held to register a long press.

// --- WiFi and Network Configuration ---
const char* default_ssid = "Holaco";                      // Default WiFi SSID if no credentials are saved.
const char* default_password = "diaa123456789";          // Default WiFi password.
const char* ap_ssid = "PenthuAuraSetup";                  // SSID for the Access Point mode.
const char* ap_password = "123456789";                  // Password for the Access Point mode.
const char* wifi_credentials_path = "/wifi_credentials.txt"; // Path in LittleFS to store WiFi credentials.
const char* config_cycle_path = "/config_cycle.txt";      // Path in LittleFS to store the measurement cycle interval.

// --- ThingSpeak Channel Configuration ---
// Channel 1: Used by the ESP32 to log its own data (e.g., temperature) for the RPi to see.
unsigned long logChannelId = 2957151;
String logWriteApiKey = "EWHN1MFXMLB5YTYH";

// Channel 2: Used by the ESP32 to fetch all results calculated and uploaded by the RPi.
unsigned long fetchChannelId = 2995240;
String fetchReadApiKey = "UW8OCQ8U24K25OCZ";

// --- MAX30102 (SpO2/HR Sensor) Configuration ---
const byte MAX30102_LED_BRIGHTNESS = 0xFF;     // LED brightness (0x00-0xFF).
const byte MAX30102_RED_LED_CURRENT = 0xFF;    // Red LED current setting.
const byte MAX30102_IR_LED_CURRENT = 0xFF;     // IR LED current setting.
const byte MAX30102_SAMPLE_AVERAGE = 1;        // Number of samples to average (1, 2, 4, 8, 16, 32).
const byte MAX30102_LED_MODE = 2;              // LED mode (1=Red only, 2=Red+IR, 3=Multi-LED).
const int MAX30102_SAMPLE_RATE = 100;          // Samples per second (50, 100, 200, 400, 800, 1000, 1600, 3200).
const int MAX30102_PULSE_WIDTH = 411;          // Pulse width in uS (69, 118, 215, 411).
const int MAX30102_ADC_RANGE = 16384;          // ADC range (2048, 4096, 8192, 16384).

// --- Finger Detection & Data Stabilization Constants ---
const long FINGER_DETECT_THRESHOLD_IR = 150000;      // IR signal value threshold to detect a finger.
const unsigned long FINGER_LOST_TIMEOUT_MS = 2000;    // Timeout (ms) before deciding the finger has been removed.
const int STABLE_SPO2_WINDOW_SIZE = 10;               // Number of readings to average for a stable SpO2 value.
const float STABILITY_THRESHOLD = 1.0;                // Maximum deviation allowed for SpO2 to be considered stable.
const int SPO2_READ_INTERVAL_MS = 100;                // Interval (ms) between SpO2 readings.
const long SENSOR_INITIAL_BOOT_DELAY_MS = 2000;       // Delay (ms) after boot for sensors to stabilize.

// --- MAX30205 (Temperature Sensor) Configuration ---
#define MAX30205_ADDRESS 0x48                         // I2C address of the temperature sensor.
#define TEMPERATURE_REGISTER 0x00                     // Register to read temperature data from.
const int STABLE_TEMP_WINDOW_SIZE = 5;                // Number of readings to average for a stable temperature.
const float TEMP_STABILITY_THRESHOLD = 0.2;           // Maximum deviation (°C) for temperature to be considered stable.
const int TEMP_READ_INTERVAL_MS = 250;                // Interval (ms) between temperature readings during stabilization.
const unsigned long TEMP_READ_INTERVAL_BUFFER_MS = 1000; // Interval (ms) for general temperature updates.
const unsigned long STABILIZATION_OVERALL_TIMEOUT_MS = 15000;

// --- Error and Special Value Constants ---
const float NO_FINGER_VALUE = -1.0;                   // Value indicating no finger is detected.
const float CALCULATION_ERROR_VALUE = -2.0;           // Value indicating a calculation error occurred.
const float STABILIZATION_TIMEOUT_VALUE = -5.0;       // Value indicating sensor stabilization timed out.
const float SENSOR_INIT_FAILED_VALUE = -6.0;          // Value indicating a sensor failed to initialize.
const float INVALID_READING_VALUE = -7.0;             // Value indicating an invalid reading from a sensor.

// --- RTC (Real-Time Clock) Data ---
// `RTC_DATA_ATTR` ensures these variables retain their values during deep sleep.
RTC_DATA_ATTR int bootCount = 0; // Counts the number of boots (survives deep sleep).

// =================================================================================================
// ================================= INACTIVITY SLEEP GLOBALS ======================================
// =================================================================================================
unsigned long lastInteractionTime = 0; // Tracks the timestamp of the last user interaction (e.g., button press).

// =================================================================================================
// ================================== CHUNK CONFIGURATION ==========================================
// =================================================================================================
// These parameters control how raw sensor data is collected and sent to the RPi.
const int CHUNK_SIZE = 1000;    // Number of samples to collect in each data chunk (1000 samples @ 100Hz = 10 seconds).
const int TOTAL_CHUNKS = 3;     // Total number of chunks to send for one full measurement (3 chunks = 30 seconds of data).
const unsigned long FINGER_SETTLE_DURATION_MS = 0; // Time (ms) to wait after finger detection before starting collection.
bool isFingerSettling = false;
unsigned long fingerSettleStartTime = 0;

// Buffers to store a full chunk of raw PPG data before sending.
long irChunkBuffer[CHUNK_SIZE];
long redChunkBuffer[CHUNK_SIZE];

// State tracking for the chunk collection and sending process.
int chunkSampleCounter = 0;     // Counts samples collected for the current chunk.
int chunksSent = 0;             // Counts how many chunks have been successfully sent.
bool settled = false;           // Flag to indicate if the sensor has had its initial warm-up time.
unsigned long collectionStartTime; // Timestamp when the data collection process began.

// =================================================================================================
// =================================== GLOBAL VARIABLES ============================================
// =================================================================================================
String gHealthState = "Initializing"; // Global health state string, determined by the RPi ("Normal", "Moderate", "Critical").
char fetchedNgrokUrl[100];            // Buffer to store the ngrok URL fetched from ThingSpeak.
volatile bool isPerformingManualScan = false; // Flag to prevent main loop interference during a manual WiFi scan.

// --- Web Server and WiFi ---
AsyncWebServer server(80);      // The asynchronous web server object.
String saved_ssid = "";         // SSID loaded from LittleFS.
String saved_password = "";     // Password loaded from LittleFS.
bool ap_mode = false;           // Flag indicating if the device is in Access Point mode.

// --- LCD Display ---
LiquidCrystal lcd(LCD_RS_PIN, LCD_E_PIN, LCD_D4_PIN, LCD_D5_PIN, LCD_D6_PIN, LCD_D7_PIN); // LCD object.

// --- Button State ---
volatile bool buttonPressedFlag = false; // A flag set by the ISR (not currently used, but good practice).
int lastButtonState = HIGH;     // Last known state of the button (used for debouncing).
int buttonState = HIGH;         // Current debounced state of thebutton.
unsigned long lastDebounceTime = 0; // Timestamp of the last button state change.
unsigned long buttonDownTime = 0;   // Timestamp when the button was pressed down.
int pressCount = 0;             // Counter for multi-press detection.
unsigned long lastPressTime = 0;    // Timestamp of the last registered press.
bool longPressTriggered = false;    // Flag to indicate a long press has been detected.

// --- Display Management ---
enum DisplayScreen {
    SCREEN_VITALS,        // Main screen showing latest vital signs.
    SCREEN_WIFI_STATUS,   // Shows WiFi connection status, IP, etc.
    SCREEN_ALERT_STATUS,  // Shows health state and stress status.
    SCREEN_RP_STATUS,     // Shows status of the connection to the RPi server.
    SCREEN_SYS_INFO       // Shows firmware version and boot count.
};
DisplayScreen currentScreen = SCREEN_VITALS; // The currently active LCD screen.
const int NUM_SCREENS = 5;                   // Total number of screens to cycle through.
unsigned long lastLCDUpdateTime = 0;         // Timestamp of the last LCD update to control refresh rate.
const int LCD_UPDATE_INTERVAL = 500;         // LCD refresh rate in milliseconds.

// --- Measurement and Alert Data ---
int cycle_interval_minutes = 10; // Default interval in minutes for automatic measurements.
float gCurrentSPO2 = NO_FINGER_VALUE; // Global for latest SpO2 value.
int gCurrentHR = 0;                   // Global for latest Heart Rate value.
float gCurrentTemp = INVALID_READING_VALUE; // Global for latest Temperature value.
// New globals for detailed predictions from the RPi
float gSBP = 0;                       // Systolic Blood Pressure.
float gDBP = 0;                       // Diastolic Blood Pressure.
String gStressStatus = "N/A";         // Stress level status.
String gAnomalyStatus = "N/A";        // Anomaly detection status.

bool gIsModerateAlertActive = false;  // Becomes true if gHealthState is "Moderate".
bool gIsCriticalAlertActive = false;  // Becomes true if gHealthState is "Critical".
bool gAlertsSilenced = false;         // Flag to silence audio/visual alerts for the current cycle.

// --- Historical Data Logging ---
struct SensorDataLog {
    float spo2;
    float hr;
    float temp;
    float sbp;
    float dbp;
    char stress[16];
    char anomaly[16];
    char state[16];
    time_t timestamp;
    int cycle;
};

const int MAX_HISTORY_LOGS = 10; // Maximum number of historical measurement cycles to store.

// These variables are stored in RTC memory to survive deep sleep.
RTC_DATA_ATTR SensorDataLog historicalData[MAX_HISTORY_LOGS]; // Array to store historical data logs.
RTC_DATA_ATTR int historyLogIndex = 0;                        // Index for the circular buffer of historical data.
RTC_DATA_ATTR int historyLogCount = 0;                        // Total number of logs currently stored.
RTC_DATA_ATTR unsigned long measurementCycleCounter = 0;      // Counter for the total number of cycles performed.

// --- Sensor Objects ---
MAX30105 particleSensor; // Object for the MAX30102/5 sensor.

// --- Manual Calculation Windows (Stabilization) ---
static float tempReadingsWindow[STABLE_TEMP_WINDOW_SIZE];
static int tempCurrentReadingIndex = 0;
static int tempReadingsInWindowCount = 0;

// --- Automatic Cycle Scheduling ---
unsigned long nextCycleStartTime = 0; // Timestamp for when the next automatic measurement cycle should start.

// =================================================================================================
// ============================= MEASUREMENT CYCLE STATE MACHINE ===================================
// =================================================================================================
enum CycleState {
    IDLE,                           // Device is waiting for user input or for the next scheduled cycle.
    ACQUIRE_TEMPERATURE,            // Step 1: Measure a stable skin temperature.
    SENDING_TEMPERATURE_TO_TS,      // Step 2: Upload the temperature to ThingSpeak for the RPi.
    FETCHING_URL_FROM_TS,           // Step 3: Fetch the RPi server's ngrok URL from ThingSpeak.
    COLLECTING_AND_SENDING_CHUNKS,  // Step 4: Collect and send raw PPG data chunks to the RPi.
    WAITING_FOR_RPI,                // Step 5: Wait for the RPi to process the data and upload results.
    FETCHING_VITALS_FROM_TS,        // Step 6: Fetch all processed results from ThingSpeak.
    CHECKING_ALERTS,                // Step 7: Check the fetched health state and trigger alerts if necessary.
    REVIEWING_RESULTS,              // Step 8: Display results on the LCD for a fixed review period.
    CYCLE_COMPLETE                  // Final state: Clean up and schedule the next cycle.
};

CycleState currentCycleState = IDLE; // The current state of the measurement cycle.
bool isCycleRunning = false;         // Flag to indicate if a measurement cycle is currently in progress.


// =================================================================================================
// ================================== FUNCTION PROTOTYPES ==========================================
// =================================================================================================
// --- LCD Functions ---
void initLCD();
void updateLCD();
void displaySensorDataOnLCD();
void displayWiFiStatusOnLCD();
void displayAlertStatusOnLCD();
void displayRPiStatusOnLCD();
void displaySysInfoOnLCD();
void displaySystemMessageOnLCD(String l1, String l2 = "");

// --- Button and Interaction Functions ---
void setupButton();
void handleButton();
void triggerManualMeasurement();
void forceAPMode();
void cycleDisplayScreen();
void acknowledgeAlerts();

// --- Configuration and File System Functions ---
void loadAllConfigs();
void loadCycleConfig();
bool saveCycleConfig(int interval);
bool loadWiFiCredentials();
bool saveWiFiCredentials(const char* ssid, const char* pwd);

// --- WiFi, Web Server, and API Handlers ---
void startFallbackAP();
void setupWebServer();
void handleScanWifi(AsyncWebServerRequest* request);
void handleDataRealtime(AsyncWebServerRequest* request);
void handleDataHistory(AsyncWebServerRequest* request);
void handleSaveConfig(AsyncWebServerRequest* request);
void handleDeviceInfo(AsyncWebServerRequest* request);
void connectToWiFi();

// --- Sensor Initialization and Reading ---
bool initMAX30102();
bool initMAX30205();
float waitForStableTemperature_impl();
float getRawTemperatureValue();
bool isValueStable(float window[], int numReadings, int maxWindowSize, float threshold);

// --- Alerting and Data Logging ---
void addHistoricalData(float spo2, float hr, float temp, float sbp, float dbp, String stress, String anomaly, String state);
void setRgbLed(int r, int g, int b);
void soundModerateAlert();
void soundCriticalAlert();

// --- Sleep and Power Management ---
void enterDeepSleep();
void print_wakeup_reason();

// --- Data Communication Functions ---
void sendDataChunk();
bool fetchNgrokUrlFromThingSpeak();
void sendTemperatureToThingSpeak();
bool fetchAllResultsFromThingSpeak();

// --- Main Cycle and System Functions ---
void runMeasurementCycle();
void syncTimeWithNTP();
void warmReboot();


// =================================================================================================
// ==================================== SETUP FUNCTION =============================================
// =================================================================================================
/**
 * @brief Initializes the device on startup.
 * @details This function runs once after boot. It initializes serial communication, hardware
 * (LCD, I2C, Button, LED, Buzzer), the file system (LittleFS), sensors (MAX30102, MAX30205),
 * and connects to WiFi. It displays boot progress on the LCD. If sensor initialization
 * fails, it enters deep sleep.
 */
void setup() {
    // Start serial communication for debugging purposes.
    Serial.begin(115200);
    unsigned long setupStartTime = millis(); // Record start time to measure setup duration.
    Serial.println(F("\n\nESP32 PenthuAura Health Monitor Starting..."));
    Serial.print(F("Version: 2.3.3\n"));

    // Increment boot count and print the reason for waking up (e.g., timer, button press).
    bootCount++;
    Serial.print(F("Boot count: "));
    Serial.println(bootCount);
    print_wakeup_reason();

    // Initialize the LCD and display a startup message.
    initLCD();
    displaySystemMessageOnLCD("Booting Up...", "PenthuAura v2.3.3");
    delay(1000);

    // Initialize the I2C communication bus.
    Wire.begin();
    Serial.println(F("I2C Initialized."));
    displaySystemMessageOnLCD("I2C Initialized", "LittleFS Next...");
    delay(500);

    // Initialize the LittleFS file system. `true` formats it if it fails to mount.
    if (!LittleFS.begin(true)) {
        Serial.println(F("LittleFS Mount Failed."));
        displaySystemMessageOnLCD("LittleFS Error!", "Check Filesystem");
        delay(2000);
    } else {
        Serial.println(F("LittleFS Mounted."));
        displaySystemMessageOnLCD("LittleFS Mounted", "Loading Configs");
        delay(500);
        // Load all saved configurations from the file system.
        loadAllConfigs();
    }

    // Initialize the MAX30102 SpO2/HR sensor.
    displaySystemMessageOnLCD("Sensor Init...", "MAX30102 SpO2/HR");
    bool max30102_ok = initMAX30102();
    if (!max30102_ok) {
        Serial.println(F("MAX30102 FAILED!"));
        displaySystemMessageOnLCD("MAX30102 Error!", "Check Connection");
        delay(2000);
    }

    // Initialize the MAX30205 Temperature sensor.
    displaySystemMessageOnLCD("Sensor Init...", "MAX30205 Temp");
    bool max30205_ok = initMAX30205();
    if (!max30205_ok) {
        Serial.println(F("MAX30205 FAILED!"));
        displaySystemMessageOnLCD("MAX30205 Error!", "Check Connection");
        delay(2000);
    }

    // If either sensor fails to initialize, enter deep sleep to save power and prevent errors.
    if (!max30102_ok || !max30205_ok) {
        Serial.println(F("Sensor init failed. Deep sleeping..."));
        displaySystemMessageOnLCD("Sensor Init FAIL", "Deep Sleep Soon");
        delay(3000);
        enterDeepSleep();
    }

    // Allow a short delay for sensors to stabilize after power-on.
    Serial.println(F("Sensors Initialized. Settling..."));
    displaySystemMessageOnLCD("Sensors OK!", "Settling...");
    delay(SENSOR_INITIAL_BOOT_DELAY_MS);

    // Set up the button pin and enable it as a GPIO wakeup source from light sleep.
    setupButton();
    gpio_wakeup_enable(GPIO_NUM_23, GPIO_INTR_LOW_LEVEL);
    displaySystemMessageOnLCD("Button Ready", "WiFi Next...");
    delay(500);

    // Initialize output pins for the buzzer and RGB LED.
    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(RGB_LED_R_PIN, OUTPUT);
    pinMode(RGB_LED_G_PIN, OUTPUT);
    pinMode(RGB_LED_B_PIN, OUTPUT);
    setRgbLed(255, 255, 255); // Set LED to white during boot.

    // Set up the web server endpoints and attempt to connect to WiFi.
    setupWebServer();
    connectToWiFi();

    // Final setup steps.
    updateLCD(); // Perform an initial update of the LCD screen.
    Serial.print(F("Setup completed in: "));
    Serial.print(millis() - setupStartTime);
    Serial.println(F(" ms"));

    collectionStartTime = millis(); // Initialize the collection timer.
    Serial.println(F("Setup complete. Place finger on sensor to begin collection..."));
}


// =================================================================================================
// ===================================== MAIN LOOP =================================================
// =================================================================================================
/**
 * @brief The main execution loop of the program.
 * @details This function runs continuously. It is responsible for handling button presses,
 * running the measurement cycle state machine, and managing power-saving sleep modes.
 * If a cycle is running, it calls `runMeasurementCycle()`. If idle, it checks for
 * inactivity to enter light or deep sleep, or checks if a scheduled cycle is due.
 */
void loop() {
    handleButton(); // Always check for user input first.

    if (isCycleRunning) {
        // If a measurement cycle is active, dedicate processing to it.
        runMeasurementCycle();
    } else {
        // --- IDLE STATE LOGIC ---
        // If no cycle is running, the device is in an idle state.

        // 1. Check if it's time to start the next scheduled measurement cycle.
        if (nextCycleStartTime > 0 && millis() >= nextCycleStartTime) {
            Serial.println("Cycle interval reached. Starting a new measurement cycle.");
            triggerManualMeasurement(); // Start a new cycle.
            return; // Exit the loop iteration to start the cycle immediately.
        }

        // 2. Check for long-term inactivity to enter deep sleep for maximum power saving.
        if (millis() - lastInteractionTime > DEEP_SLEEP_INACTIVITY_MS) {
            Serial.println("Long-term inactivity. Entering DEEP sleep.");
            enterDeepSleep();
        }

        // 3. Check for short-term inactivity to enter light sleep.
        if (millis() - lastInteractionTime > ACTIVE_IDLE_DURATION_MS) {
            // The device has been idle but not long enough for deep sleep.
            displaySystemMessageOnLCD("Idle. Sleeping...", "Press to wake");
            Serial.println("Entering LIGHT sleep...");

            // Enable GPIO wakeup and enter light sleep. Processor is halted until wakeup.
            esp_sleep_enable_gpio_wakeup();
            esp_light_sleep_start();

            // --- Code execution resumes here after waking up from light sleep ---
            Serial.println("Woke up from light sleep.");
            // Reset the interaction timer immediately to provide a brief active window.
            lastInteractionTime = millis();
        } else {
            // If we are here, we are in the "active idle" window after an interaction.
            // Update the LCD periodically during this time.
            if (millis() - lastLCDUpdateTime > LCD_UPDATE_INTERVAL) {
                updateLCD();
            }
        }
    }
}


// =================================================================================================
// ========================= CONFIGURATION LOADING/SAVING FUNCTIONS ================================
// =================================================================================================

/**
 * @brief Loads all configurations from LittleFS.
 * @details A wrapper function that calls individual loading functions for each config file.
 */
void loadAllConfigs() {
    loadWiFiCredentials();
    loadCycleConfig();
}

/**
 * @brief Loads the measurement cycle interval from LittleFS.
 * @details Reads the `config_cycle.txt` file. If the file exists and contains a valid
 * interval (1-1440 mins), it updates the `cycle_interval_minutes` variable.
 * If the file doesn't exist or is invalid, it saves the default value.
 */
void loadCycleConfig() {
    if (LittleFS.exists(config_cycle_path)) {
        File f = LittleFS.open(config_cycle_path, "r");
        if (f && f.size() > 0) {
            int v = f.readStringUntil('\n').toInt();
            // Validate the loaded value.
            if (v >= 1 && v <= 1440) { // 1 minute to 24 hours
                cycle_interval_minutes = v;
            } else {
                cycle_interval_minutes = 10; // Use default if value is out of range.
            }
            Serial.println("Loaded Cycle Interval: " + String(cycle_interval_minutes) + " mins");
        }
        if (f) f.close();
    } else {
        // If the file doesn't exist, create it with the default value.
        saveCycleConfig(cycle_interval_minutes);
    }
}

/**
 * @brief Saves the measurement cycle interval to LittleFS.
 * @param interval The interval in minutes to save.
 * @return `true` if saving was successful, `false` otherwise.
 */
bool saveCycleConfig(int interval) {
    if (interval < 1 || interval > 1440) return false; // Validate interval before saving.
    File f = LittleFS.open(config_cycle_path, "w");
    if (!f) return false;
    f.println(interval);
    f.close();
    cycle_interval_minutes = interval; // Update the global variable.
    Serial.println("Saved Cycle Interval: " + String(cycle_interval_minutes) + " mins");
    return true;
}


// =================================================================================================
// =============================== BUTTON HANDLING FUNCTIONS =======================================
// =================================================================================================

/**
 * @brief Initializes the button pin.
 * @details Sets the button pin as an input with an internal pull-up resistor.
 */
void setupButton() {
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    Serial.println("Button Initialized: Pin " + String(BUTTON_PIN));
}

/**
 * @brief Handles all button press logic.
 * @details This function implements debouncing, and detects single, double, triple, and long presses.
 * It is called continuously from the main loop. A press of any kind resets the inactivity timer.
 * - Single Press: Cycles through LCD screens.
 * - Double Press: Triggers a manual measurement cycle.
 * - Triple Press: Acknowledges/silences active alerts.
 * - Long Press: Forces the device into Access Point (AP) mode for configuration.
 */
void handleButton() {
    lastInteractionTime = millis(); // Any interaction resets the inactivity timer.
    int r = digitalRead(BUTTON_PIN); // Read the current state of the button.
    unsigned long cT = millis();     // Get the current time.

    // Debouncing logic: only register a state change if it's stable for DEBOUNCE_DELAY.
    if (r != lastButtonState) {
        lastDebounceTime = cT;
    }

    if ((cT - lastDebounceTime) > DEBOUNCE_DELAY) {
        // If the state has been stable for long enough, process the change.
        if (r != buttonState) {
            buttonState = r;
            if (buttonState == LOW) { // Button was just pressed down.
                buttonDownTime = cT;
                longPressTriggered = false; // Reset long press flag.

                // Multi-press detection logic.
                if (pressCount == 0 || (cT - lastPressTime) > MULTI_PRESS_TIMEOUT) {
                    // If it's the first press or the timeout has passed, start a new count.
                    pressCount = 1;
                } else {
                    // If pressed again within the timeout, increment the count.
                    pressCount++;
                }
                lastPressTime = cT;
            }
        }
    }
    lastButtonState = r; // Update the last raw state.

    // Long press detection: check if the button is held down long enough.
    if (buttonState == LOW && !longPressTriggered && (cT - buttonDownTime) > LONG_PRESS_DURATION) {
        longPressTriggered = true;
        forceAPMode(); // Trigger the long press action.
        pressCount = 0; // Reset multi-press count to avoid conflicts.
    }

    // Multi-press action trigger: executes when the button is released and timeout is met.
    if (pressCount > 0 && buttonState == HIGH && (cT - lastPressTime) > MULTI_PRESS_TIMEOUT) {
        if (!longPressTriggered) { // Only act if a long press didn't happen.
            if (pressCount == 1) cycleDisplayScreen();
            else if (pressCount == 2) triggerManualMeasurement();
            else if (pressCount == 3) acknowledgeAlerts();
        }
        pressCount = 0; // Reset the counter for the next press.
    }
}

/**
 * @brief Cycles to the next LCD screen.
 * @details Increments the `currentScreen` enum and wraps around using the modulo operator.
 * Calls `updateLCD()` to refresh the display immediately.
 */
void cycleDisplayScreen() {
    currentScreen = static_cast<DisplayScreen>((static_cast<int>(currentScreen) + 1) % NUM_SCREENS);
    updateLCD();
}

/**
 * @brief Starts a manual measurement cycle.
 * @details Checks if a cycle is already running or if the device is in AP mode.
 * If conditions are met, it sets the `isCycleRunning` flag and transitions the
 * state machine to its first step.
 */
void triggerManualMeasurement() {
    // A measurement cycle requires a WiFi connection.
    if (ap_mode) {
        displaySystemMessageOnLCD("Connect to WiFi", "to start cycle.");
        delay(2000);
        return;
    }
    // Prevent starting a new cycle if one is already in progress.
    if (isCycleRunning) {
        Serial.println("Cannot start manual cycle, one is already running.");
        displaySystemMessageOnLCD("Cycle in Progress", "Please wait...");
        delay(1500);
        return;
    }

    gAlertsSilenced = false; // Re-enable alerts for the new cycle.
    Serial.println("Manual Measurement Cycle Triggered!");
    isCycleRunning = true;
    currentCycleState = ACQUIRE_TEMPERATURE; // Set the state machine to the first state.
}

/**
 * @brief Acknowledges or toggles alerts.
 * @details If the health state is "Moderate" or "Critical", this function will
 * toggle the `gAlertsSilenced` flag, which stops audible/visual alerts for the
 * current cycle.
 */
void acknowledgeAlerts() {
    // Check if there is an active alert based on the RPi's health state assessment.
    if (gHealthState == "Critical" || gHealthState == "Moderate") {
        if (!gAlertsSilenced) {
            gAlertsSilenced = true;
            displaySystemMessageOnLCD("Alerts Silenced", "For this cycle.");
            Serial.println("Alerts have been silenced for the current cycle.");
        } else {
            gAlertsSilenced = false;
            displaySystemMessageOnLCD("Alerts Re-enabled", "");
        }
    } else {
        // Inform the user if there are no active alerts to silence.
        displaySystemMessageOnLCD("No Active Alerts", "To Silence");
    }
    delay(1500); // Show the message on the LCD for a moment.
    updateLCD(); // Return to the previous screen.
}

/**
 * @brief Forces the device into Access Point (AP) mode.
 * @details Disconnects from any existing WiFi network, ends the web server,
 * and starts a new AP with a pre-defined SSID and password, allowing for
 * direct connection and configuration.
 */
void forceAPMode() {
    Serial.println(F("Forcing AP Mode..."));
    displaySystemMessageOnLCD("Force AP Mode...", "Restarting WiFi");
    delay(1500);
    if (!ap_mode) {
        WiFi.disconnect(true, true); // Disconnect and erase credentials from memory.
        delay(100);
        server.end(); // Stop the web server.
        startFallbackAP(); // Start the AP.
    } else {
        displaySystemMessageOnLCD("Already in AP", "Mode");
        delay(1000);
    }
    updateLCD();
}


// =================================================================================================
// ============================= ALERTING & DATA LOGGING FUNCTIONS =================================
// =================================================================================================
/**
 * @brief Adds a complete set of cycle data to the historical log.
 * @details This function takes all vital signs and predictions from a completed cycle
 * and stores them in the `historicalData` circular buffer in RTC memory.
 * @param spo2 The SpO2 value.
 * @param hr The heart rate value.
 * @param temp The temperature value.
 * @param sbp The systolic blood pressure value.
 * @param dbp The diastolic blood pressure value.
 * @param stress The stress status string.
 * @param anomaly The anomaly status string.
 * @param state The overall health state string.
 */
void addHistoricalData(float spo2, float hr, float temp, float sbp, float dbp, String stress, String anomaly, String state) {
    int index_to_write = historyLogIndex; // Get the current index to write to.

    // Assign all the numerical and string data to the struct at the current index.
    historicalData[index_to_write].spo2 = spo2;
    historicalData[index_to_write].hr = hr;
    historicalData[index_to_write].temp = temp;
    historicalData[index_to_write].sbp = sbp;
    historicalData[index_to_write].dbp = dbp;
    historicalData[index_to_write].timestamp = time(nullptr); // Get the current time from NTP.
    historicalData[index_to_write].cycle = ++measurementCycleCounter;

    // Safely copy strings into the fixed-size char arrays, ensuring null termination.
    strncpy(historicalData[index_to_write].stress, stress.c_str(), 15);
    historicalData[index_to_write].stress[15] = '\0';
    strncpy(historicalData[index_to_write].anomaly, anomaly.c_str(), 15);
    historicalData[index_to_write].anomaly[15] = '\0';
    strncpy(historicalData[index_to_write].state, state.c_str(), 15);
    historicalData[index_to_write].state[15] = '\0';

    // Advance the circular buffer index for the next write.
    historyLogIndex = (historyLogIndex + 1) % MAX_HISTORY_LOGS;
    // Increment the count of total logs, capped at the maximum size.
    if (historyLogCount < MAX_HISTORY_LOGS) {
        historyLogCount++;
    }

    Serial.println("Logged complete cycle data to RTC history. Count: " + String(historyLogCount));
}


// =================================================================================================
// =================================== LCD DISPLAY FUNCTIONS =======================================
// =================================================================================================

/**
 * @brief Initializes the LCD.
 * @details Begins communication with the 16x2 LCD, clears it, and prints an initial message.
 */
void initLCD() {
    lcd.begin(16, 2);
    lcd.clear();
    lcd.print(F("PenthuAura Init"));
    Serial.println(F("LCD Initialized."));
}

/**
 * @brief Main LCD update router.
 * @details Clears the screen and calls the appropriate display function based on the
 * `currentScreen` state variable.
 */
void updateLCD() {
    lcd.clear();
    switch (currentScreen) {
        case SCREEN_VITALS:        displaySensorDataOnLCD(); break;
        case SCREEN_WIFI_STATUS:   displayWiFiStatusOnLCD(); break;
        case SCREEN_ALERT_STATUS:  displayAlertStatusOnLCD(); break;
        case SCREEN_RP_STATUS:     displayRPiStatusOnLCD(); break;
        case SCREEN_SYS_INFO:      displaySysInfoOnLCD(); break;
        default:                   displaySystemMessageOnLCD("Unknown Screen", "Error!"); break;
    }
    lastLCDUpdateTime = millis(); // Record the time of this update.
}

/**
 * @brief Displays the latest vital signs on the LCD.
 * @details Shows the SpO2, Heart Rate, and overall Health State from the most recent
 * entry in the historical log. If no history exists, it displays a prompt.
 */
void displaySensorDataOnLCD() {
    // Calculate the index of the most recently added log entry.
    int lastEntryIndex = (historyLogIndex == 0) ? MAX_HISTORY_LOGS - 1 : historyLogIndex - 1;

    if (historyLogCount > 0) {
        // Line 1: Show SpO2 and Heart Rate.
        String l0 = "S:" + String(historicalData[lastEntryIndex].spo2, 1) + "%";
        l0 += " H:" + String((int)historicalData[lastEntryIndex].hr);
        lcd.setCursor(0, 0);
        lcd.print(l0.substring(0, 16)); // Truncate to fit screen.

        // Line 2: Show the overall Health State.
        String l1 = "State: " + String(historicalData[lastEntryIndex].state);
        lcd.setCursor(0, 1);
        lcd.print(l1.substring(0, 16));
    } else {
        // If no data has been logged yet, show a prompt to the user.
        lcd.setCursor(0, 0);
        lcd.print("Run cycle to get");
        lcd.setCursor(0, 1);
        lcd.print("vitals...");
    }
}

/**
 * @brief Displays the current WiFi status on the LCD.
 * @details Shows whether the device is in AP mode or STA (station) mode. If connected,
 * it displays "CNXN" and the local IP. If disconnected, it shows "Offline".
 */
void displayWiFiStatusOnLCD() {
    String l1 = "WiFi: ", l2 = "";
    if (ap_mode) {
        l1 += "AP Mode";
        l2 = WiFi.softAPIP().toString(); // Show the AP's IP address.
    } else if (WiFi.status() == WL_CONNECTED) {
        l1 += "CNXN"; // "Connected"
        l2 = WiFi.localIP().toString(); // Show the device's IP on the network.
    } else {
        l1 += "Offline";
        if (WiFi.getMode() == WIFI_STA && saved_ssid.length() > 0) {
            l2 = "Connecting...";
        } else {
            l2 = "No Connection";
        }
    }
    lcd.setCursor(0, 0);
    lcd.print(l1.substring(0, 16));
    lcd.setCursor(0, 1);
    lcd.print(l2.substring(0, 16));
}

/**
 * @brief Displays the current alert status on the LCD.
 * @details Shows the overall Health State and the Stress status as determined by the RPi.
 */
void displayAlertStatusOnLCD() {
    lcd.setCursor(0, 0);
    lcd.print("State: " + gHealthState);

    lcd.setCursor(0, 1);
    lcd.print("Stress: " + gStressStatus);
}

/**
 * @brief Displays the status of the connection to the Raspberry Pi server.
 * @details Indicates whether the ngrok URL for the RPi server has been successfully
 * fetched. This is a prerequisite for sending data.
 */
void displayRPiStatusOnLCD() {
    lcd.setCursor(0, 0);
    lcd.print("Server Status:");

    lcd.setCursor(0, 1);
    if (strlen(fetchedNgrokUrl) > 0) {
        // If the URL has been fetched, the server is considered ready.
        lcd.print("URL OK - Ready");
    } else {
        // If no URL, indicate whether we are trying to fetch it or if it has failed.
        if(currentCycleState == FETCHING_URL_FROM_TS) {
            lcd.print("Fetching URL...");
        } else {
            lcd.print("URL Not Fetched");
        }
    }
}

/**
 * @brief Displays system information on the LCD.
 * @details Shows the device name, firmware version, and the current boot count.
 */
void displaySysInfoOnLCD() {
    String l1 = "Sys: PenthuAura";
    String l2 = "Ver:2.3.3 B:" + String(bootCount); // 'B' for Boot Count.
    lcd.setCursor(0, 0);
    lcd.print(l1.substring(0, 16));
    lcd.setCursor(0, 1);
    lcd.print(l2.substring(0, 16));
}

/**
 * @brief Displays a temporary system message on the LCD.
 * @details A utility function to show a two-line message. Useful for status updates
 * during blocking operations.
 * @param l1 The string to display on the first line.
 * @param l2 The string to display on the second line (optional).
 */
void displaySystemMessageOnLCD(String l1, String l2) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(l1.substring(0, 16));
    if (l2.length() > 0) {
        lcd.setCursor(0, 1);
        lcd.print(l2.substring(0, 16));
    }
    // Also print the message to the serial monitor for debugging.
    Serial.println("LCD SysMsg: " + l1 + " | " + l2);
}


// =================================================================================================
// ============================== WIFI CREDENTIAL HANDLING =========================================
// =================================================================================================

/**
 * @brief Loads WiFi credentials from LittleFS.
 * @details Reads the `wifi_credentials.txt` file and populates the `saved_ssid`
 * and `saved_password` global variables.
 * @return `true` if credentials were loaded successfully, `false` otherwise.
 */
bool loadWiFiCredentials() {
    if (LittleFS.exists(wifi_credentials_path)) {
        File f = LittleFS.open(wifi_credentials_path, "r");
        if (!f || f.size() == 0) {
            if (f) f.close();
            Serial.println(F("Bad cred file"));
            return false;
        }
        // Read SSID and password, separated by newlines.
        saved_ssid = f.readStringUntil('\n');
        saved_ssid.trim();
        saved_password = f.readStringUntil('\n');
        saved_password.trim();
        f.close();
        Serial.println("Loaded WiFi: " + saved_ssid);
        return saved_ssid.length() > 0;
    }
    Serial.println(F("No cred file"));
    return false;
}

/**
 * @brief Saves WiFi credentials to LittleFS.
 * @details Overwrites the `wifi_credentials.txt` file with the provided SSID and password.
 * @param ssid The SSID to save.
 * @param pwd The password to save.
 * @return `true` if saving was successful, `false` otherwise.
 */
bool saveWiFiCredentials(const char* ssid, const char* pwd) {
    File f = LittleFS.open(wifi_credentials_path, "w");
    if (!f) {
        Serial.println(F("Fail save cred"));
        return false;
    }
    f.println(ssid);
    f.println(pwd);
    f.close();
    Serial.println(F("Saved WiFi Credentials."));
    // Update the global variables immediately.
    saved_ssid = String(ssid);
    saved_password = String(pwd);
    return true;
}


// =================================================================================================
// =================================== WIFI CONNECTION LOGIC =======================================
// =================================================================================================

/**
 * @brief Connects the device to a WiFi network.
 * @details This function orchestrates the WiFi connection process:
 * 1. It first tries to connect using saved credentials from LittleFS.
 * 2. If that fails, it tries to connect using hardcoded default credentials.
 * 3. If both attempts fail, it starts the device in Access Point (AP) mode.
 * On successful connection, it syncs the time via NTP and starts the first measurement cycle.
 */
void connectToWiFi() {
    // If already connected, just ensure the server is running and exit.
    if (WiFi.status() == WL_CONNECTED && !ap_mode) {
        server.begin();
        return;
    }

    bool connected = false;
    WiFi.mode(WIFI_STA); // Set mode to Station (client).
    currentScreen = SCREEN_WIFI_STATUS;
    updateLCD();

    // --- Attempt 1: Connect using saved credentials ---
    if (saved_ssid.length() > 0) {
        displaySystemMessageOnLCD("Connecting Saved", saved_ssid.substring(0, 10));
        WiFi.begin(saved_ssid.c_str(), saved_password.c_str());
        unsigned long startAttemptTime = millis();
        int dots = 0;
        // Wait up to 15 seconds for a connection.
        while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 15000) {
            String msg = "Connecting";
            for (int i = 0; i < dots; ++i) msg += ".";
            displaySystemMessageOnLCD(msg, saved_ssid.substring(0, 10));
            dots = (dots + 1) % 4; // Animate dots on LCD.
            delay(500);
        }
        if (WiFi.status() == WL_CONNECTED) {
            Serial.println(F("Connected to saved WiFi!"));
            Serial.println("IP: " + WiFi.localIP().toString());
            connected = true;
        } else {
            Serial.println(F("Failed to connect to saved WiFi."));
            WiFi.disconnect(false);
            delay(100);
        }
    }

    // --- Attempt 2: Connect using default credentials (if first attempt failed) ---
    if (!connected && strlen(default_ssid) > 0) {
        displaySystemMessageOnLCD("Connecting Def.", String(default_ssid).substring(0, 10));
        WiFi.begin(default_ssid, default_password);
        unsigned long startAttemptTime = millis();
        int dots = 0;
        // Wait up to 15 seconds.
        while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 15000) {
            String msg = "Connecting";
            for (int i = 0; i < dots; ++i) msg += ".";
            displaySystemMessageOnLCD(msg, String(default_ssid).substring(0, 10));
            dots = (dots + 1) % 4;
            delay(500);
        }
        if (WiFi.status() == WL_CONNECTED) {
            Serial.println(F("Connected to default WiFi!"));
            Serial.println("IP: " + WiFi.localIP().toString());
            connected = true;
        } else {
            Serial.println(F("Failed to connect to default WiFi."));
            WiFi.disconnect(false);
            delay(100);
        }
    }

    // --- Handle Connection Outcome ---
    if (connected) {
        ap_mode = false; // We are in Station mode.
        server.begin(); // Start the web server.
        displaySystemMessageOnLCD("WiFi Connected!", WiFi.localIP().toString());
        syncTimeWithNTP(); // Sync time now that we have internet.
        delay(1500);

        // Automatically start the first measurement cycle after a successful connection.
        gAlertsSilenced = false;
        isCycleRunning = true;
        currentCycleState = ACQUIRE_TEMPERATURE;
    } else {
        // --- Fallback: Start Access Point Mode ---
        Serial.println(F("All WiFi connection attempts failed. Starting AP."));
        displaySystemMessageOnLCD("No WiFi Avail.", "Starting AP...");
        delay(1500);
        startFallbackAP();
    }
    updateLCD();
}


// =================================================================================================
// ================================= WEB SERVER & API HANDLERS =====================================
// =================================================================================================

/**
 * @brief Sets up all web server routes (endpoints).
 * @details This function defines how the server responds to different HTTP requests.
 * It serves the HTML/CSS frontend, provides API endpoints for data and configuration,
 * and includes handlers for status, reboot, and not-found errors.
 */
void setupWebServer() {
    // Serve the main HTML page from LittleFS.
    server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
        request->send(LittleFS, "/index.html", "text/html");
    });
    server.on("/index.html", HTTP_GET, [](AsyncWebServerRequest* request) {
        request->send(LittleFS, "/index.html", "text/html");
    });
    // Serve the CSS stylesheet.
    server.on("/style.css", HTTP_GET, [](AsyncWebServerRequest* request) {
        request->send(LittleFS, "/style.css", "text/css");
    });

    // API endpoint to get the latest (real-time) data.
    server.on("/data/realtime", HTTP_GET, handleDataRealtime);
    // API endpoint to get the full history of logged data.
    server.on("/data/history", HTTP_GET, handleDataHistory);
    // API endpoint to scan for available WiFi networks.
    server.on("/scanwifi", HTTP_GET, handleScanWifi);
    // API endpoint to save new configuration (WiFi, cycle interval).
    server.on("/saveconfig", HTTP_POST, handleSaveConfig);
    // API endpoint to get basic device info (firmware, IP).
    server.on("/api/device-info", HTTP_GET, handleDeviceInfo);

    // Simple status endpoint for quick checks.
    server.on("/status", HTTP_GET, [](AsyncWebServerRequest* request) {
        String statusMsg = "Mode: " + String(ap_mode ? "AP" : "STA")
                         + "\nIP: " + (ap_mode ? WiFi.softAPIP().toString() : WiFi.localIP().toString())
                         + "\nSSID: " + (ap_mode ? String(ap_ssid) : WiFi.SSID())
                         + "\nBootCount: " + String(bootCount) + "\n";
        request->send(200, "text/plain", statusMsg);
    });

    // Endpoint to trigger a software reboot.
    server.on("/reboot", HTTP_GET, [](AsyncWebServerRequest* request) {
        request->send(200, "text/plain", "Rebooting...");
        warmReboot(); // Perform a warm reboot to preserve RTC data.
    });

    // Handle 404 Not Found errors.
    server.onNotFound([](AsyncWebServerRequest* request) {
        Serial.printf("NOT_FOUND: HTTP %s: %s\n", request->methodToString(), request->url().c_str());
        request->send(404, "text/plain", "Not found");
    });
}

/**
 * @brief Starts the ESP32 in Access Point (AP) + Station (STA) mode.
 * @details This function is the fallback when a WiFi connection fails. It creates a WiFi
 * network with a predefined SSID and password, allowing a user to connect directly to the
* ESP32 to configure it via its web page. `WIFI_AP_STA` mode is used so it can
* simultaneously host an AP and scan for other networks.
 */
void startFallbackAP() {
    WiFi.mode(WIFI_AP_STA); // Set dual mode.

    ap_mode = true; // Set the global AP mode flag.
    Serial.print(F("Starting AP in dual mode (AP_STA): "));
    Serial.println(ap_ssid);

    WiFi.softAP(ap_ssid, ap_password); // Start the Access Point.

    Serial.print(F("AP IP address: "));
    Serial.println(WiFi.softAPIP());

    // CRUCIAL: The web server must be started *after* the AP is configured.
    server.begin();
    Serial.println(F("HTTP server started for AP mode."));

    // Update the display to show AP information.
    currentScreen = SCREEN_WIFI_STATUS;
    updateLCD();
    displaySystemMessageOnLCD(ap_ssid, WiFi.softAPIP().toString());
    delay(1500);
}

/**
 * @brief Handles requests to the `/data/realtime` endpoint.
 * @details Creates a JSON object containing the most recent set of vital signs from the
 * historical log, as well as the current temperature and WiFi status. This is used by
 * the web interface to display live data.
 * @param request The web server request object.
 */
void handleDataRealtime(AsyncWebServerRequest* request) {
    DynamicJsonDocument jsonDoc(1024);
    // Find the index of the last entry in the circular buffer.
    int lastEntryIndex = (historyLogCount == 0) ? MAX_HISTORY_LOGS - 1 : historyLogIndex - 1;

    if (historyLogCount > 0) {
        // If history exists, populate JSON with the latest data.
        jsonDoc["spo2"] = historicalData[lastEntryIndex].spo2;
        jsonDoc["hr"] = historicalData[lastEntryIndex].hr;
        jsonDoc["sbp"] = historicalData[lastEntryIndex].sbp;
        jsonDoc["dbp"] = historicalData[lastEntryIndex].dbp;
        jsonDoc["stress"] = historicalData[lastEntryIndex].stress;
        jsonDoc["anomaly"] = historicalData[lastEntryIndex].anomaly;
        jsonDoc["healthState"] = historicalData[lastEntryIndex].state;
    } else {
        // If no history, send default/error values.
        jsonDoc["spo2"] = -999.0; jsonDoc["hr"] = 0; jsonDoc["sbp"] = 0;
        jsonDoc["dbp"] = 0; jsonDoc["stress"] = "N/A"; jsonDoc["anomaly"] = "No Data";
        jsonDoc["healthState"] = "Initializing";
    }

    // Add other real-time information.
    jsonDoc["temp"] = getRawTemperatureValue();
    jsonDoc["wifiStatus"] = ap_mode ? "AP Mode (" + String(ap_ssid) + ")" : (WiFi.status() == WL_CONNECTED ? "Connected (" + WiFi.SSID() + ")" : "Disconnected");

    // Serialize the JSON object to a string and send it as the response.
    String output;
    serializeJson(jsonDoc, output);
    request->send(200, "application/json", output);
}

/**
 * @brief Handles requests for device information.
 * @details Creates a JSON object with static device info like firmware version,
 * IP address, and boot count.
 * @param request The web server request object.
 */
void handleDeviceInfo(AsyncWebServerRequest* request) {
    DynamicJsonDocument jsonDoc(256);

    // Add key-value pairs to the JSON object.
    jsonDoc["firmwareVersion"] = "2.3.3 Debug";
    jsonDoc["deviceIP"] = ap_mode ? WiFi.softAPIP().toString() : WiFi.localIP().toString();
    jsonDoc["bootCount"] = bootCount;

    String output;
    serializeJson(jsonDoc, output);
    request->send(200, "application/json", output);
}

/**
 * @brief Handles requests to the `/data/history` endpoint.
 * @details Creates a JSON array containing all the logged measurement cycles stored in
 * RTC memory. This is used by the web interface to display a history table.
 * @param request The web server request object.
 */
void handleDataHistory(AsyncWebServerRequest* request) {
    DynamicJsonDocument jsonDoc(2048);
    JsonArray dataArray = jsonDoc.to<JsonArray>();

    // Iterate through all stored logs and add them to the JSON array.
    for (int i = 0; i < historyLogCount; i++) {
        // Calculate the actual index in the circular buffer, starting from the newest.
        int actualIndex = (historyLogIndex - 1 - i + MAX_HISTORY_LOGS) % MAX_HISTORY_LOGS;
        JsonObject entry = dataArray.createNestedObject();
        entry["spo2"] = historicalData[actualIndex].spo2;
        entry["hr"] = historicalData[actualIndex].hr;
        entry["temp"] = historicalData[actualIndex].temp;
        entry["sbp"] = historicalData[actualIndex].sbp;
        entry["dbp"] = historicalData[actualIndex].dbp;
        entry["stress"] = historicalData[actualIndex].stress;
        entry["anomaly"] = historicalData[actualIndex].anomaly;
        entry["state"] = historicalData[actualIndex].state;
        entry["timestamp"] = historicalData[actualIndex].timestamp;
    }

    String output;
    serializeJson(jsonDoc, output);
    request->send(200, "application/json", output);
}

/**
 * @brief Handles POST requests to `/saveconfig`.
 * @details Parses incoming form data from the web interface to update settings.
 * It can update WiFi credentials and the measurement cycle interval. If WiFi settings
 * are changed, it triggers a reboot to apply them.
 * @param request The web server request object.
 */
void handleSaveConfig(AsyncWebServerRequest* request) {
    Serial.println(F("Handling /saveconfig"));
    bool settingsChanged = false;
    bool needsRestart = false; // A reboot is needed for WiFi changes.
    String responseMsg = "Settings Update:\n";

    // --- Handle WiFi Credentials ---
    if (request->hasParam("ssid", true) && request->hasParam("password", true)) {
        String newSsid = request->getParam("ssid", true)->value();
        String newPass = request->getParam("password", true)->value();
        if (newSsid.length() > 0) {
            // Only save if the credentials have actually changed.
            if (!newSsid.equals(saved_ssid) || !newPass.equals(saved_password)) {
                if (saveWiFiCredentials(newSsid.c_str(), newPass.c_str())) {
                    responseMsg += "- WiFi credentials saved.\n";
                    settingsChanged = true;
                    needsRestart = true; // Set flag to reboot.
                } else {
                    responseMsg += "- WiFi credentials save FAILED.\n";
                }
            } else {
                responseMsg += "- WiFi credentials unchanged.\n";
            }
        } else {
            responseMsg += "- WiFi SSID cannot be empty.\n";
        }
    }

    // --- Handle Cycle Interval ---
    if (request->hasParam("cycle_interval", true)) {
        int newInterval = request->getParam("cycle_interval", true)->value().toInt();
        // Validate the input.
        if (newInterval >= 1 && newInterval <= 1440) {
            if (newInterval != cycle_interval_minutes) {
                if (saveCycleConfig(newInterval)) {
                    responseMsg += "- Cycle interval saved: " + String(newInterval) + " mins.\n";
                    settingsChanged = true;
                } else {
                    responseMsg += "- Cycle interval save FAILED.\n";
                }
            } else {
                responseMsg += "- Cycle interval unchanged.\n";
            }
        } else {
            responseMsg += "- Invalid cycle interval value.\n";
        }
    }

    // --- Finalize and Respond ---
    if (needsRestart) {
        responseMsg += "Device will RESTART to apply WiFi changes.";
        request->send(200, "text/plain", responseMsg);
        displaySystemMessageOnLCD("Config Saved!", "Restarting...");
        delay(3000);
        warmReboot();
    } else {
        if (settingsChanged) {
            responseMsg = "Settings saved successfully!\n" + responseMsg;
            displaySystemMessageOnLCD("Config Saved!", "Check Web/LCD");
        } else {
            responseMsg = "No settings were changed.\n";
            displaySystemMessageOnLCD("Config", "No Changes");
        }
        request->send(200, "text/plain", responseMsg);
        delay(2000);
    }
    updateLCD();
}

/**
 * @brief Handles requests to `/scanwifi`.
 * @details Performs a WiFi scan for nearby networks and returns them as a JSON array.
 * This is used by the web interface to populate a dropdown of available networks.
 * It temporarily disables main loop functions to prevent interference during the scan.
 * @param request The web server request object.
 */
void handleScanWifi(AsyncWebServerRequest* request) {
    isPerformingManualScan = true; // Set flag to pause main loop logic.

    Serial.println(F("========================================="));
    Serial.println(F("Handling /scanwifi request..."));
    displaySystemMessageOnLCD("Scanning WiFi...", "Please Wait");

    // Prepare for a clean scan.
    WiFi.scanDelete();
    delay(100);
    WiFi.disconnect();
    delay(100);

    Serial.println(F("Starting new network scan..."));
    int n = WiFi.scanNetworks(); // Perform the scan.
    Serial.print(F("===> Raw result from scanNetworks(): "));
    Serial.println(n);

    if (n < 0) n = 0; // Handle potential error codes.

    // Create a JSON response with the scan results.
    DynamicJsonDocument jsonDoc(1024 + n * 120);
    JsonArray networksArray = jsonDoc.to<JsonArray>();
    if (n > 0) {
        for (int i = 0; i < n; ++i) {
            // Filter out invalid or hidden SSIDs.
            if (WiFi.SSID(i).length() > 0 && WiFi.SSID(i).length() < 33) {
                JsonObject net = networksArray.createNestedObject();
                net["ssid"] = WiFi.SSID(i);
                net["rssi"] = WiFi.RSSI(i);
                // Get encryption type (ESP32 specific).
                #ifndef ESP8266
                switch (WiFi.encryptionType(i)) {
                    case WIFI_AUTH_OPEN:           net["encryption"] = "OPEN"; break;
                    case WIFI_AUTH_WEP:            net["encryption"] = "WEP"; break;
                    case WIFI_AUTH_WPA_PSK:        net["encryption"] = "WPA_PSK"; break;
                    case WIFI_AUTH_WPA2_PSK:       net["encryption"] = "WPA2_PSK"; break;
                    case WIFI_AUTH_WPA_WPA2_PSK:   net["encryption"] = "WPA/2_PSK"; break;
                    case WIFI_AUTH_WPA2_ENTERPRISE:net["encryption"] = "WPA2_ENT"; break;
                    case WIFI_AUTH_WPA3_PSK:       net["encryption"] = "WPA3_PSK"; break;
                    case WIFI_AUTH_WPA2_WPA3_PSK:  net["encryption"] = "WPA2/3_PSK"; break;
                    default:                       net["encryption"] = "UNKNOWN";
                }
                #else
                net["encryption"] = "?"; // Placeholder for other platforms.
                #endif
            }
        }
    }

    String jsonResponse;
    serializeJson(jsonDoc, jsonResponse);
    request->send(200, "application/json", jsonResponse);
    Serial.println("Sent WiFi scan results to browser.");
    Serial.println(F("=========================================\n"));

    displaySystemMessageOnLCD("Scan Complete!", String(n) + " Nets");
    delay(1500);
    updateLCD();

    isPerformingManualScan = false; // Unset flag to resume normal loop operation.
}


// =================================================================================================
// ========================== SENSOR INITIALIZATION & READING ======================================
// =================================================================================================

/**
 * @brief Initializes the MAX30102 sensor.
 * @details Checks for the sensor on the I2C bus and configures its operating parameters
 * like sample rate, LED currents, and pulse width.
 * @return `true` if initialization is successful, `false` otherwise.
 */
bool initMAX30102() {
    Serial.println(F("Initializing MAX30102/MAX30105..."));
    if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
        Serial.println(F("MAX30102 sensor not found. Check wiring."));
        return false;
    }
    Serial.println(F("MAX30102 OK. Configuring sensor..."));
    // Set up the sensor with predefined constants.
    particleSensor.setup(MAX30102_LED_BRIGHTNESS, MAX30102_SAMPLE_AVERAGE, MAX30102_LED_MODE, MAX30102_SAMPLE_RATE, MAX30102_PULSE_WIDTH, MAX30102_ADC_RANGE);
    particleSensor.setPulseAmplitudeRed(MAX30102_RED_LED_CURRENT);
    particleSensor.setPulseAmplitudeIR(MAX30102_IR_LED_CURRENT);
    particleSensor.clearFIFO(); // Clear any old data from the sensor's buffer.
    return true;
}

/**
 * @brief Initializes the MAX30205 temperature sensor.
 * @details Checks for the presence of the sensor at its I2C address.
 * @return `true` if the sensor is found, `false` otherwise.
 */
bool initMAX30205() {
    Serial.println(F("Initializing MAX30205 Temperature Sensor..."));
    Wire.beginTransmission(MAX30205_ADDRESS);
    // `endTransmission` returns 0 on success (ACK received from device).
    if (Wire.endTransmission() == 0) {
        Serial.println(F("MAX30205 Found."));
        return true;
    }
    Serial.println(F("MAX30205 not found. Check wiring."));
    return false;
}

/**
 * @brief Waits for a stable temperature reading.
 * @details Reads the temperature sensor repeatedly, storing values in a sliding window.
 * It returns an averaged value once the difference between the min and max in the window
 * falls below a stability threshold (`TEMP_STABILITY_THRESHOLD`).
 * @return The stable temperature in Celsius, or an error value on timeout/failure.
 */
float waitForStableTemperature_impl() {
    Serial.println(F("waitForStableTemperature_impl"));
    // Reset state variables for the stabilization window.
    tempCurrentReadingIndex = 0;
    tempReadingsInWindowCount = 0;
    unsigned long tempStartTime = millis();
    int tempAttempts = 0;
    float lastValidTempValue = INVALID_READING_VALUE;

    // Loop until timeout.
    while (millis() - tempStartTime < STABILIZATION_OVERALL_TIMEOUT_MS / 2) {
        float currentTemp = getRawTemperatureValue();
        // Ignore invalid readings.
        if (currentTemp <= INVALID_READING_VALUE + 0.1) {
            Serial.println(F("Invalid temp reading during stabilization."));
            lastValidTempValue = INVALID_READING_VALUE;
            delay(TEMP_READ_INTERVAL_MS * 2);
            continue;
        }

        Serial.print(F("Raw Temp for stabilization: "));
        Serial.println(currentTemp);
        lastValidTempValue = currentTemp;

        // Add the reading to the circular window buffer.
        tempReadingsWindow[tempCurrentReadingIndex] = currentTemp;
        tempCurrentReadingIndex = (tempCurrentReadingIndex + 1) % STABLE_TEMP_WINDOW_SIZE;
        if (tempReadingsInWindowCount < STABLE_TEMP_WINDOW_SIZE) tempReadingsInWindowCount++;

        // Check if the values in the window are stable.
        if (isValueStable(tempReadingsWindow, tempReadingsInWindowCount, STABLE_TEMP_WINDOW_SIZE, TEMP_STABILITY_THRESHOLD)) {
            float sum = 0;
            for (int i = 0; i < STABLE_TEMP_WINDOW_SIZE; ++i) sum += tempReadingsWindow[i];
            float stableTemp = sum / STABLE_TEMP_WINDOW_SIZE;
            Serial.print(F("Stable Temperature: "));
            Serial.println(stableTemp);
            return stableTemp; // Return the averaged stable temperature.
        }

        tempAttempts++;
        if (tempAttempts > 50) {
            Serial.println(F("Max Temp stabilization attempts."));
            // Return the last good reading if we fail, otherwise return timeout error.
            return (lastValidTempValue > INVALID_READING_VALUE + 0.1) ? lastValidTempValue : STABILIZATION_TIMEOUT_VALUE;
        }
        delay(TEMP_READ_INTERVAL_MS);
    }

    Serial.println(F("Temperature stabilization timeout."));
    return (lastValidTempValue > INVALID_READING_VALUE + 0.1) ? lastValidTempValue : STABILIZATION_TIMEOUT_VALUE;
}

/**
 * @brief Gets a single raw temperature reading from the MAX30205.
 * @details Performs the I2C communication to read the two-byte temperature register
 * and converts the raw value to degrees Celsius.
 * @return The temperature in Celsius, or `INVALID_READING_VALUE` on failure.
 */
float getRawTemperatureValue() {
    Wire.beginTransmission(MAX30205_ADDRESS);
    Wire.write(TEMPERATURE_REGISTER); // Point to the temperature register.
    if (Wire.endTransmission(false) != 0) { // Send command, `false` keeps connection active.
        return INVALID_READING_VALUE; // Failed to communicate.
    }

    // Request 2 bytes of data.
    if (Wire.requestFrom(MAX30205_ADDRESS, 2) == 2) {
        byte msb = Wire.read();
        byte lsb = Wire.read();
        int16_t rawTemp = (msb << 8) | lsb; // Combine bytes into a 16-bit integer.
        // Convert raw value to Celsius using the formula from the datasheet.
        float temp = (float)rawTemp * 0.00390625;
        // Basic sanity check on the reading.
        if (temp < 10.0 || temp > 50.0) { return INVALID_READING_VALUE; }
        return temp;
    }
    return INVALID_READING_VALUE; // Failed to read data.
}

/**
 * @brief Checks if a window of readings is stable.
 * @details Calculates the difference between the maximum and minimum values in a given
 * array (window) of readings.
 * @param window The array of float values.
 * @param numReadings The number of valid readings currently in the window.
 * @param maxWindowSize The total size of the window array.
 * @param threshold The maximum allowed difference for the values to be considered stable.
 * @return `true` if the window is full and the range is within the threshold, `false` otherwise.
 */
bool isValueStable(float window[], int numReadings, int maxWindowSize, float threshold) {
    // Cannot be stable if the window isn't full yet.
    if (numReadings < maxWindowSize) return false;

    // Find the min and max values in the window.
    float minVal = window[0], maxVal = window[0];
    for (int i = 1; i < maxWindowSize; ++i) {
        if (window[i] < minVal) minVal = window[i];
        if (window[i] > maxVal) maxVal = window[i];
    }
    // Return true if the difference (range) is within the allowed threshold.
    return (maxVal - minVal) <= threshold;
}


// =================================================================================================
// ==================================== DEEP SLEEP LOGIC ===========================================
// =================================================================================================

/**
 * @brief Puts the ESP32 into deep sleep mode.
 * @details This is the lowest power mode. The CPU is powered down. The device can be
 * woken up by either an external signal on a specific pin (the button) or by an
 * internal RTC timer. The timer is set based on the `cycle_interval_minutes`.
 */
void enterDeepSleep() {
    Serial.println("Entering DEEP sleep for maximum power savings.");
    displaySystemMessageOnLCD("Deep Sleeping...", "Button to wake");
    delay(1000);
    lcd.clear();
    lcd.noDisplay(); // Turn off the LCD backlight/display.

    // Set up the RTC timer as a wakeup source.
    long long sleep_duration_us = (long long)cycle_interval_minutes * 60 * 1000000;
    esp_sleep_enable_timer_wakeup(sleep_duration_us);

    // Set up the button (GPIO 23) as an external wakeup source (wake on LOW level).
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_23, 0);

    Serial.println("Going to sleep now.");
    Serial.flush(); // Ensure all serial messages are sent before sleeping.
    esp_deep_sleep_start(); // Enter deep sleep.
}

/**
 * @brief Prints the reason for the ESP32 waking up from sleep.
 * @details Called at the beginning of `setup()` to determine if the boot is a cold start
 * or a wakeup from sleep, and if so, what caused it.
 */
void print_wakeup_reason() {
    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
    Serial.print(F("Wakeup cause: "));
    switch (wakeup_reason) {
        case ESP_SLEEP_WAKEUP_EXT0:     Serial.println(F("External signal using RTC_IO")); break;
        case ESP_SLEEP_WAKEUP_EXT1:     Serial.println(F("External signal using RTC_CNTL")); break;
        case ESP_SLEEP_WAKEUP_TIMER:    Serial.println(F("Timer")); break;
        case ESP_SLEEP_WAKEUP_TOUCHPAD: Serial.println(F("Touchpad")); break;
        case ESP_SLEEP_WAKEUP_ULP:      Serial.println(F("ULP program")); break;
        default:                        Serial.printf("Other reason (%d)\n", wakeup_reason); break;
    }
}


// =================================================================================================
// ============================= DATA COMMUNICATION FUNCTIONS ======================================
// =================================================================================================

/**
 * @brief Sends a chunk of raw PPG data to the RPi server.
 * @details Serializes the `irChunkBuffer` and `redChunkBuffer` into a JSON payload
 * and sends it via an HTTP POST request to the ngrok URL. Implements a retry
 * mechanism to handle transient network errors.
 */
void sendDataChunk() {
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("Cannot send chunk, WiFi is not connected.");
        return;
    }
    if (strlen(fetchedNgrokUrl) == 0) {
        Serial.println("Cannot send chunk, ngrok URL is empty.");
        displaySystemMessageOnLCD("Send Chunk FAIL", "No URL Fetched");
        delay(1500);
        return;
    }

    // Create a large JSON document to hold the chunk data.
    DynamicJsonDocument doc(30000);
    JsonArray irData = doc.createNestedArray("ir_data");
    JsonArray redData = doc.createNestedArray("red_data");
    for (int i = 0; i < CHUNK_SIZE; i++) {
        irData.add(irChunkBuffer[i]);
        redData.add(redChunkBuffer[i]);
    }
    String jsonPayload;
    serializeJson(doc, jsonPayload);

    bool sentSuccessfully = false;
    int retryCount = 0;
    const int MAX_RETRIES = 5;

    // Retry loop: keep trying to send until successful or max retries are reached.
    while (!sentSuccessfully && retryCount < MAX_RETRIES) {
        HTTPClient http;
        String fullUrl = String(fetchedNgrokUrl) + "/data"; // Append the API endpoint to the base URL.
        http.begin(fullUrl);
        http.addHeader("Content-Type", "application/json");

        Serial.printf("Attempt %d: Sending POST request to: %s\n", retryCount + 1, fullUrl.c_str());
        displaySystemMessageOnLCD("Sending Chunk " + String(chunksSent + 1), "Attempt " + String(retryCount + 1));
        int httpResponseCode = http.POST(jsonPayload);

        if (httpResponseCode == HTTP_CODE_OK) {
            String response = http.getString();
            Serial.printf("HTTP Response code: %d\n", httpResponseCode);
            Serial.printf("Response from server: %s\n", response.c_str());
            displaySystemMessageOnLCD("Chunk Sent OK", "Code: " + String(httpResponseCode));
            sentSuccessfully = true; // Set flag to exit the retry loop.
        } else {
            Serial.printf("Error on sending POST: %s\n", http.errorToString(httpResponseCode).c_str());
            retryCount++;
            if (retryCount < MAX_RETRIES) {
                displaySystemMessageOnLCD("Send FAIL. Retry", "in 1s...");
                delay(1000); // Wait before the next attempt.
            } else {
                displaySystemMessageOnLCD("Send FAILED.", "Max retries.");
            }
        }
        http.end();
    }
}

/**
 * @brief Fetches the ngrok URL from ThingSpeak.
 * @details The RPi server is expected to periodically update a specific ThingSpeak field
 * with its current public ngrok URL. This function reads that field to discover
 * the server's address.
 * @return `true` if a valid URL was fetched, `false` otherwise.
 */
bool fetchNgrokUrlFromThingSpeak() {
    if (WiFi.status() != WL_CONNECTED) return false;
    HTTPClient http;
    // Construct the URL to read the last entry of Field 7 from the 'fetch' channel.
    String url = "https://api.thingspeak.com/channels/" + String(fetchChannelId) + "/fields/7/last.txt?api_key=" + fetchReadApiKey;
    http.begin(url);
    int httpCode = http.GET();
    if (httpCode == HTTP_CODE_OK) {
        String payload = http.getString();
        payload.trim(); // Remove any whitespace.
        // Basic validation of the fetched URL.
        if (payload.length() > 0 && payload.startsWith("http")) {
            strncpy(fetchedNgrokUrl, payload.c_str(), sizeof(fetchedNgrokUrl) - 1);
            return true;
        }
    }
    return false;
}

/**
 * @brief Sends the measured temperature to the logging ThingSpeak channel.
 * @details This function allows the RPi to know the temperature measured by the ESP32,
 * which might be used as an input for its models.
 */
void sendTemperatureToThingSpeak() {
    if (WiFi.status() != WL_CONNECTED) return;
    HTTPClient http;
    // Construct the ThingSpeak update URL for the 'log' channel.
    String url = "https://api.thingspeak.com/update?api_key=" + logWriteApiKey + "&channel_id=" + String(logChannelId) + "&field1=" + String(gCurrentTemp, 2);
    http.begin(url);
    http.GET(); // Send the request.
    http.end();
    Serial.println("Temperature sent to LOGGING channel.");
}

/**
 * @brief Fetches all processed results from the RPi's ThingSpeak channel.
 * @details After the RPi has processed the data, it uploads the results to different
 * fields on a ThingSpeak channel. This function performs individual HTTP GET requests
 * to read the latest value from each field (SpO2, HR, BP, etc.) and updates the
 * corresponding global variables.
 * @return `true` if all fields were fetched successfully, `false` if any fetch failed.
 */
bool fetchAllResultsFromThingSpeak() {
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("Cannot fetch results, no WiFi.");
        return false;
    }

    HTTPClient http;
    bool all_ok = true;
    int httpCode;

    Serial.println("--- Starting to fetch individual fields from RPi Results Channel ---");

    // Fetch SpO2 (Field 1)
    http.begin("https://api.thingspeak.com/channels/" + String(fetchChannelId) + "/fields/1/last.txt?api_key=" + fetchReadApiKey);
    http.setTimeout(10000); // 10-second timeout per request.
    httpCode = http.GET();
    if (httpCode == HTTP_CODE_OK) { gCurrentSPO2 = http.getString().toFloat(); }
    else { all_ok = false; Serial.printf("[FETCH FAIL] SpO2 (Field 1) - HTTP Code: %d\n", httpCode); }
    http.end();

    // Fetch HR (Field 2)
    http.begin("https://api.thingspeak.com/channels/" + String(fetchChannelId) + "/fields/2/last.txt?api_key=" + fetchReadApiKey);
    http.setTimeout(10000);
    httpCode = http.GET();
    if (httpCode == HTTP_CODE_OK) { gCurrentHR = http.getString().toInt(); }
    else { all_ok = false; Serial.printf("[FETCH FAIL] HR (Field 2) - HTTP Code: %d\n", httpCode); }
    http.end();

    // Fetch SBP (Field 3)
    http.begin("https://api.thingspeak.com/channels/" + String(fetchChannelId) + "/fields/3/last.txt?api_key=" + fetchReadApiKey);
    http.setTimeout(10000);
    httpCode = http.GET();
    if (httpCode == HTTP_CODE_OK) { gSBP = http.getString().toFloat(); }
    else { all_ok = false; Serial.printf("[FETCH FAIL] SBP (Field 3) - HTTP Code: %d\n", httpCode); }
    http.end();

    // Fetch DBP (Field 4)
    http.begin("https://api.thingspeak.com/channels/" + String(fetchChannelId) + "/fields/4/last.txt?api_key=" + fetchReadApiKey);
    http.setTimeout(10000);
    httpCode = http.GET();
    if (httpCode == HTTP_CODE_OK) { gDBP = http.getString().toFloat(); }
    else { all_ok = false; Serial.printf("[FETCH FAIL] DBP (Field 4) - HTTP Code: %d\n", httpCode); }
    http.end();

    // Fetch Stress (Field 5), converting 0/1 to a string.
    http.begin("https://api.thingspeak.com/channels/" + String(fetchChannelId) + "/fields/5/last.txt?api_key=" + fetchReadApiKey);
    http.setTimeout(10000);
    httpCode = http.GET();
    if (httpCode == HTTP_CODE_OK) { gStressStatus = http.getString().toInt() == 1 ? "Stressed" : "Not Stressed"; }
    else { all_ok = false; Serial.printf("[FETCH FAIL] Stress (Field 5) - HTTP Code: %d\n", httpCode); }
    http.end();

    // Fetch Anomaly (Field 6), converting 0/1 to a string.
    http.begin("https://api.thingspeak.com/channels/" + String(fetchChannelId) + "/fields/6/last.txt?api_key=" + fetchReadApiKey);
    http.setTimeout(10000);
    httpCode = http.GET();
    if (httpCode == HTTP_CODE_OK) { gAnomalyStatus = http.getString().toInt() == 1 ? "Abnormal" : "Normal"; }
    else { all_ok = false; Serial.printf("[FETCH FAIL] Anomaly (Field 6) - HTTP Code: %d\n", httpCode); }
    http.end();

    // Fetch ngrok URL (Field 7)
    http.begin("https://api.thingspeak.com/channels/" + String(fetchChannelId) + "/fields/7/last.txt?api_key=" + fetchReadApiKey);
    http.setTimeout(10000);
    httpCode = http.GET();
    if (httpCode == HTTP_CODE_OK) { String p = http.getString(); p.trim(); strncpy(fetchedNgrokUrl, p.c_str(), sizeof(fetchedNgrokUrl) - 1); }
    else { all_ok = false; Serial.printf("[FETCH FAIL] ngrok URL (Field 7) - HTTP Code: %d\n", httpCode); }
    http.end();

    // Fetch Health State (Field 8), converting 0/1/2 to a string.
    http.begin("https://api.thingspeak.com/channels/" + String(fetchChannelId) + "/fields/8/last.txt?api_key=" + fetchReadApiKey);
    http.setTimeout(10000);
    httpCode = http.GET();
    if (httpCode == HTTP_CODE_OK) { String s = http.getString(); s.trim(); if(s=="2") gHealthState="Critical"; else if(s=="1") gHealthState="Moderate"; else gHealthState="Normal"; }
    else { all_ok = false; Serial.printf("[FETCH FAIL] Health State (Field 8) - HTTP Code: %d\n", httpCode); }
    http.end();

    Serial.println("--- Finished fetching individual fields ---");
    if(all_ok) { Serial.println("Successfully fetched all results from RPi channel."); return true; }
    else { Serial.println("-> Overall fetch failed because one or more fields returned an error."); return false; }
}


// =================================================================================================
// ============================= HARDWARE CONTROL FUNCTIONS ========================================
// =================================================================================================
/**
 * @brief Controls the RGB LED.
 * @param r Red component (0 for off, >0 for on).
 * @param g Green component (0 for off, >0 for on).
 * @param b Blue component (0 for off, >0 for on).
 */
void setRgbLed(int r, int g, int b) {
    digitalWrite(RGB_LED_R_PIN, r > 0 ? HIGH : LOW);
    digitalWrite(RGB_LED_G_PIN, g > 0 ? HIGH : LOW);
    digitalWrite(RGB_LED_B_PIN, b > 0 ? HIGH : LOW);
}

/**
 * @brief Sounds a "moderate" alert on the buzzer.
 * @details Two short beeps.
 */
void soundModerateAlert() {
    digitalWrite(BUZZER_PIN, HIGH); delay(150); digitalWrite(BUZZER_PIN, LOW);
    delay(100);
    digitalWrite(BUZZER_PIN, HIGH); delay(150); digitalWrite(BUZZER_PIN, LOW);
}

/**
 * @brief Sounds a "critical" alert on the buzzer.
 * @details One long, continuous beep.
 */
void soundCriticalAlert() {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(1000);
    digitalWrite(BUZZER_PIN, LOW);
}


// =================================================================================================
// ============================= MAIN MEASUREMENT CYCLE FUNCTION ===================================
// =================================================================================================
/**
 * @brief The core state machine for a measurement cycle.
 * @details This function is called repeatedly from `loop()` when `isCycleRunning` is true.
 * It uses a `switch` statement to execute the code for the `currentCycleState`. After
 * each step, it transitions to the next state, orchestrating the entire measurement process
 * from temperature acquisition to final review of the results.
 */
void runMeasurementCycle() {
    if (!isCycleRunning) {
        return; // Safety check.
    }

    switch (currentCycleState) {
        case IDLE:
            // This case should ideally not be reached while isCycleRunning is true, but as a fallback, it stops the cycle.
            isCycleRunning = false;
            break;

        case ACQUIRE_TEMPERATURE:
            Serial.println("Cycle Step 1: Acquiring Temperature...");
            displaySystemMessageOnLCD("Measuring Temp", "Please wait...");
            gCurrentTemp = waitForStableTemperature_impl(); // This is a blocking call.
            Serial.printf("Temp Acquired: %.1f C\n", gCurrentTemp);
            currentCycleState = SENDING_TEMPERATURE_TO_TS; // Transition to the next state.
            break;

        case SENDING_TEMPERATURE_TO_TS:
            Serial.println("Cycle Step 2: Sending Temperature to ThingSpeak...");
            displaySystemMessageOnLCD("Sending Temp", "To ThingSpeak...");
            sendTemperatureToThingSpeak();
            currentCycleState = FETCHING_URL_FROM_TS; // Transition.
            break;

        case FETCHING_URL_FROM_TS:
            Serial.println("Cycle Step 3: Fetching RPi URL...");
            if (fetchNgrokUrlFromThingSpeak()) {
                // If successful, proceed to data collection.
                currentCycleState = COLLECTING_AND_SENDING_CHUNKS;
                // Reset all chunk collection variables.
                chunksSent = 0; chunkSampleCounter = 0; settled = false;
                isFingerSettling = false; collectionStartTime = millis();
            } else {
                // If failed, wait and retry this state in the next loop iteration.
                Serial.println("Failed to fetch URL. Retrying in 5 seconds...");
                displaySystemMessageOnLCD("URL Fetch Fail", "Retrying...");
                delay(5000); // Blocking delay before retry.
            }
            break;

        case COLLECTING_AND_SENDING_CHUNKS:
            { // Use a block to create a local scope for variables if needed.
                // --- Sensor warm-up period ---
                if (!settled) {
                    if (millis() - collectionStartTime >= 5000) {
                        settled = true;
                        Serial.println("Warm-up complete. Place finger firmly...");
                        displaySystemMessageOnLCD("Ready", "Place Finger");
                    } else {
                        Serial.println("Sensor warming up...");
                        displaySystemMessageOnLCD("Sensor Warm-up", "Please Wait...");
                        delay(500);
                    }
                    break; // Exit switch and re-enter on next loop to continue checking time.
                }

                // --- Finger detection ---
                long irValue = particleSensor.getIR();
                particleSensor.nextSample(); // Advance FIFO to get the next sample ready.
                if (irValue < FINGER_DETECT_THRESHOLD_IR) {
                    isFingerSettling = false; // Finger is not present or lifted.
                    break; // Stay in this state, waiting for a finger.
                }

                // --- Finger settling ---
                if (!isFingerSettling) {
                    isFingerSettling = true;
                    fingerSettleStartTime = millis();
                    displaySystemMessageOnLCD("Finger Detected", "Settling...");
                }

                // --- Data collection and sending ---
                if (millis() - fingerSettleStartTime >= FINGER_SETTLE_DURATION_MS) {
                    if (chunkSampleCounter == 0 && chunksSent < TOTAL_CHUNKS) {
                        displaySystemMessageOnLCD("Collecting...", "Chunk " + String(chunksSent + 1));
                    }
                    if (chunkSampleCounter < CHUNK_SIZE) {
                        // Store IR and Red PPG values in the buffers.
                        irChunkBuffer[chunkSampleCounter] = irValue;
                        redChunkBuffer[chunkSampleCounter] = particleSensor.getRed();
                        chunkSampleCounter++;
                    }
                    if (chunkSampleCounter >= CHUNK_SIZE) {
                        // Buffer is full, send the chunk.
                        sendDataChunk();
                        chunksSent++;
                        chunkSampleCounter = 0; // Reset for the next chunk.
                        isFingerSettling = false; // Require finger to resettle.
                    }
                }
            }
            // Check if all chunks have been sent.
            if (chunksSent >= TOTAL_CHUNKS) {
                Serial.println("All chunks sent to RPi. Waiting for processing...");
                currentCycleState = WAITING_FOR_RPI; // Transition.
            }
            break;

        case WAITING_FOR_RPI:
            Serial.println("Cycle Step 5: Waiting for RPi to calculate and upload...");
            displaySystemMessageOnLCD("Waiting for RPi", "10 seconds...");
            delay(10000); // A fixed delay to give the RPi time to work.
            currentCycleState = FETCHING_VITALS_FROM_TS; // Transition.
            break;

        case FETCHING_VITALS_FROM_TS:
            Serial.println("Cycle Step 6: Fetching All RPi Results...");
            displaySystemMessageOnLCD("Fetching Results", "From ThingSpeak");

            if (fetchAllResultsFromThingSpeak()) {
                // If fetch is successful, log the data and check for alerts.
                addHistoricalData(gCurrentSPO2, gCurrentHR, gCurrentTemp, gSBP, gDBP, gStressStatus, gAnomalyStatus, gHealthState);
                updateLCD(); // Update the vitals screen with new data.
                currentCycleState = CHECKING_ALERTS;
            } else {
                // If fetch fails, end the cycle prematurely.
                Serial.println("Could not fetch all RPi results. Ending cycle.");
                currentCycleState = CYCLE_COMPLETE;
            }
            break;

        case CHECKING_ALERTS:
            Serial.println("Cycle Step 7: Checking for RPi-based Alerts...");

            // Trigger alerts based on the fetched health state.
            if (gHealthState == "Critical") {
                displaySystemMessageOnLCD("! CRITICAL !", gAnomalyStatus);
                soundCriticalAlert();
            } else if (gHealthState == "Moderate") {
                displaySystemMessageOnLCD("! MODERATE !", gStressStatus);
                soundModerateAlert();
            } else { // Normal
                displaySystemMessageOnLCD("State: Normal", "All systems OK");
            }

            delay(2000); // Hold the alert message on the screen.
            currentCycleState = REVIEWING_RESULTS; // Transition to the final review period.
            break;

        case REVIEWING_RESULTS:
            // This state provides a timed window for the user to view results on the LCD.
            if (reviewPeriodStartTime == 0) {
                // This is the first entry into this state for this cycle.
                Serial.println("Cycle Complete. Entering review period...");
                displaySystemMessageOnLCD("Cycle Complete", "Review Results");
                setRgbLed(0, 0, 255); // Blue LED indicates review period.
                reviewPeriodStartTime = millis(); // Start the timer.
            }

            // Allow the LCD to be updated if the user cycles through screens.
            if (millis() - lastLCDUpdateTime > LCD_UPDATE_INTERVAL) {
                updateLCD();
            }

            // Check if the review period has ended.
            if (millis() - reviewPeriodStartTime >= REVIEW_PERIOD_MS) {
                Serial.println("Review period ended.");
                reviewPeriodStartTime = 0; // Reset timer for the next cycle.
                currentCycleState = CYCLE_COMPLETE; // Transition.
            }
            break;

        case CYCLE_COMPLETE:
            Serial.println("Measurement Cycle Complete. Scheduling next cycle and returning to Idle.");

            // Schedule the start time for the next automatic cycle.
            nextCycleStartTime = millis() + (long)cycle_interval_minutes * 60 * 1000;

            // Reset flags to return to a clean idle state.
            lastInteractionTime = millis(); // Reset inactivity timer.
            isCycleRunning = false;
            currentCycleState = IDLE;
            break;
    }
}


// =================================================================================================
// =================================== UTILITY FUNCTIONS ===========================================
// =================================================================================================

/**
 * @brief Synchronizes the internal clock with an NTP server.
 * @details Configures the time using a predefined time zone (Egypt, UTC+2) and NTP
 * servers. This is necessary for accurate timestamps in the historical data log.
 */
void syncTimeWithNTP() {
    Serial.println("Syncing time with NTP server...");
    // Configure time: timezone UTC+2, no daylight saving, primary and secondary NTP servers.
    configTime(2 * 3600, 0, "pool.ntp.org", "time.windows.com");

    struct tm timeinfo;
    if (!getLocalTime(&timeinfo)) {
        Serial.println("Failed to obtain time");
        return;
    }
    Serial.println("Time synchronized successfully.");
    // Print the synchronized time to the serial monitor.
    Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
}

/**
 * @brief Performs a "warm" reboot.
 * @details A warm reboot is a software reset that does not clear data stored in RTC
 * memory (like boot count and historical data). This is achieved by setting a
 * deep sleep timer for a minuscule duration, which causes an immediate wakeup and reset.
 */
void warmReboot() {
    Serial.println("Performing a warm reboot to preserve RTC data...");
    displaySystemMessageOnLCD("Rebooting...", "Please wait");
    delay(1000);

    // Deep sleep for 1 microsecond to trigger an instant warm reboot.
    esp_sleep_enable_timer_wakeup(1);
    esp_deep_sleep_start();
}
