// ============================================================
//  ACCIDENT DETECTION & EMERGENCY ALERT SYSTEM
//  Hardware : ESP32 + MPU6050 + NEO-6M GPS + SIM800L
//  Author   : Integrated System
//  Purpose  : Detects vehicle collision via MPU6050 linear
//             acceleration, fetches GPS coordinates via NEO-6M,
//             and places an emergency call + sends an SMS with
//             a Google Maps link via SIM800L.
// ============================================================

// ─────────────────────────────────────────
//  LIBRARY INCLUDES
// ─────────────────────────────────────────
#include <Arduino.h>
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>

// ─────────────────────────────────────────
//  PIN DEFINITIONS
// ─────────────────────────────────────────
// I2C (MPU6050)
#define I2C_SDA          21
#define I2C_SCL          22

// GPS  (UART2)
#define GPS_RX           16    // ESP32 RX ← GPS TX
#define GPS_TX           17    // ESP32 TX → GPS RX
#define GPS_BAUD         9600

// SIM800L (UART1)
#define SIM_RX           25    // ESP32 RX ← SIM800 TX
#define SIM_TX           26    // ESP32 TX → SIM800 RX
#define SIM_BAUD         9600

// ─────────────────────────────────────────
//  EMERGENCY CONTACTS
// ─────────────────────────────────────────
// 112 is India's national emergency number and accepts SMS.
// SMS is sent here first as it works even on weak signal.
#define EMERGENCY_SMS_NUMBER   "112"             // National emergency — SMS (India)

// Registered police / emergency contact for the voice call.
// Replace with the actual number (local police station, etc.)
#define EMERGENCY_CALL_NUMBER  "+91XXXXXXXXXX"   // ← Replace with registered number

// ─────────────────────────────────────────
//  ACCIDENT DETECTION THRESHOLDS
// ─────────────────────────────────────────
// Linear acceleration magnitude threshold (m/s²).
// A normal hard brake ≈ 8–10 m/s².  A collision ≈ 20–50+ m/s².
// Tune IMPACT_THRESHOLD to your vehicle / sensitivity needs.
#define IMPACT_THRESHOLD      20.0f   // m/s²  — trigger above this

// After an alert fires, ignore further triggers for this long (ms).
#define COOLDOWN_MS           30000UL  // 30 seconds

// GPS fix wait timeout after impact (ms)
#define GPS_FIX_TIMEOUT_MS    15000UL  // 15 seconds

// ─────────────────────────────────────────
//  OBJECT & VARIABLE DECLARATIONS
// ─────────────────────────────────────────

// — MPU6050 —
MPU6050 mpu;
int16_t ax_raw, ay_raw, az_raw;
int16_t gx_raw, gy_raw, gz_raw;

float ax, ay, az;          // total acceleration (m/s²)
float ax_lin, ay_lin, az_lin;  // linear (gravity-removed) acceleration

// Low-pass gravity estimator
float g_est_x = 0.0f, g_est_y = 0.0f, g_est_z = 0.0f;
const float GRAVITY_ALPHA = 0.995f;

// — GPS —
TinyGPSPlus gps;
HardwareSerial gpsSerial(2);   // UART2

// — SIM800L —
HardwareSerial sim800(1);      // UART1

// — State —
bool     accidentDetected  = false;
unsigned long lastAlertTime = 0;

// ─────────────────────────────────────────
//  FUNCTION PROTOTYPES
// ─────────────────────────────────────────
void     initMPU();
void     initGPS();
void     initSIM800();
bool     readMPU();
void     feedGPS(unsigned long timeoutMs);
void     handleAccident();
void     sendEmergencyCall();
void     sendEmergencySMS(const char* number, float lat, float lng, bool hasFix);
bool     simSendAT(const char* cmd, const char* expected, unsigned long timeoutMs);
void     logSerial(const char* msg);

// ============================================================
//  SETUP
// ============================================================
void setup()
{
    Serial.begin(115200);
    delay(500);

    logSerial("=== Accident Detection System Booting ===");

    Wire.begin(I2C_SDA, I2C_SCL);
    initMPU();
    initGPS();
    initSIM800();

    logSerial("=== System Ready — Monitoring for Impact ===");
}

// ============================================================
//  MAIN LOOP
// ============================================================
void loop()
{
    // 1. Feed GPS parser continuously (non-blocking)
    while (gpsSerial.available())
        gps.encode(gpsSerial.read());

    // 2. Read MPU6050 and check for impact
    if (readMPU())
    {
        unsigned long now = millis();

        // Cooldown guard — don't spam calls
        if (now - lastAlertTime > COOLDOWN_MS)
        {
            lastAlertTime = now;
            accidentDetected = true;
            handleAccident();
            accidentDetected = false;
        }
        else
        {
            logSerial("[WARN] Impact detected but still in cooldown period.");
        }
    }

    delay(4);   // ~250 Hz loop (MPU runs at ~250 Hz with 4 ms delay)
}

// ============================================================
//  INITIALISERS
// ============================================================

void initMPU()
{
    logSerial("[MPU] Initialising MPU6050...");
    mpu.initialize();
    mpu.setSleepEnabled(false);
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
    mpu.setDLPFMode(0);

    if (mpu.testConnection())
        logSerial("[MPU] MPU6050 connected OK.");
    else
        logSerial("[MPU] WARNING: MPU6050 connection FAILED!");
}

void initGPS()
{
    logSerial("[GPS] Initialising NEO-6M on UART2...");
    gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);
    logSerial("[GPS] NEO-6M ready. Waiting for satellite fix...");
}

void initSIM800()
{
    logSerial("[SIM] Initialising SIM800L on UART1...");
    sim800.begin(SIM_BAUD, SERIAL_8N1, SIM_RX, SIM_TX);
    delay(3000);   // Give SIM800L time to register on network

    simSendAT("AT",       "OK",   2000);
    simSendAT("AT+CPIN?", "READY",3000);   // Check SIM present
    simSendAT("AT+CREG?", "OK",   2000);   // Network registration
    simSendAT("AT+CSQ",   "OK",   2000);   // Signal quality

    logSerial("[SIM] SIM800L initialisation complete.");
}

// ============================================================
//  MPU6050 — READ & IMPACT DETECTION
//  Returns true if a sudden impact is detected.
// ============================================================
bool readMPU()
{
    mpu.getMotion6(&ax_raw, &ay_raw, &az_raw,
                   &gx_raw, &gy_raw, &gz_raw);

    // Convert raw → m/s²  (±2g range → 16384 LSB/g)
    ax = (ax_raw / 16384.0f) * 9.81f;
    ay = (ay_raw / 16384.0f) * 9.81f;
    az = (az_raw / 16384.0f) * 9.81f;

    // Gravity estimation via low-pass filter
    g_est_x = GRAVITY_ALPHA * g_est_x + (1.0f - GRAVITY_ALPHA) * ax;
    g_est_y = GRAVITY_ALPHA * g_est_y + (1.0f - GRAVITY_ALPHA) * ay;
    g_est_z = GRAVITY_ALPHA * g_est_z + (1.0f - GRAVITY_ALPHA) * az;

    // Linear (gravity-subtracted) acceleration
    ax_lin = ax - g_est_x;
    ay_lin = ay - g_est_y;
    az_lin = az - g_est_z;

    // Magnitude of linear acceleration vector
    float magnitude = sqrtf(ax_lin * ax_lin +
                            ay_lin * ay_lin +
                            az_lin * az_lin);

    // Debug output every loop (comment out in production to reduce noise)
    // Serial.printf("[MPU] Lin Accel Mag: %.3f m/s²\n", magnitude);

    return (magnitude >= IMPACT_THRESHOLD);
}

// ============================================================
//  GPS — FEED PARSER FOR A FIXED WINDOW
// ============================================================
void feedGPS(unsigned long timeoutMs)
{
    unsigned long start = millis();
    while (millis() - start < timeoutMs)
    {
        while (gpsSerial.available())
            gps.encode(gpsSerial.read());
    }
}

// ============================================================
//  ACCIDENT HANDLER — ORCHESTRATES EVERYTHING
// ============================================================
void handleAccident()
{
    logSerial("!!! ACCIDENT DETECTED — initiating emergency protocol !!!");

    // Step 1: Try to get a fresh GPS fix
    logSerial("[GPS] Attempting to get GPS fix...");
    feedGPS(GPS_FIX_TIMEOUT_MS);

    bool hasFix = gps.location.isValid() && gps.location.age() < 5000;

    if (hasFix)
    {
        Serial.printf("[GPS] Fix obtained → Lat: %.6f, Lng: %.6f\n",
                      gps.location.lat(), gps.location.lng());
    }
    else
    {
        logSerial("[GPS] No valid fix — SMS will indicate location unavailable.");
    }

    float lat = hasFix ? (float)gps.location.lat() : 0.0f;
    float lng = hasFix ? (float)gps.location.lng() : 0.0f;

    // Step 2: SMS → 112  (sent first; works even on weak signal)
    logSerial("[SIM] Step 1 — Sending SMS to 112 (national emergency)...");
    sendEmergencySMS(EMERGENCY_SMS_NUMBER, lat, lng, hasFix);

    // Step 3: Voice call → registered police/emergency number
    logSerial("[SIM] Step 2 — Placing voice call to registered number...");
    sendEmergencyCall();

    logSerial("[SYSTEM] Emergency protocol complete.");
}

// ============================================================
//  SIM800L — PLACE EMERGENCY VOICE CALL
// ============================================================
void sendEmergencyCall()
{
    logSerial("[SIM] Placing emergency voice call...");

    String dialCmd = "ATD";
    dialCmd += EMERGENCY_CALL_NUMBER;
    dialCmd += ";";   // Semicolon = voice call (not data)

    sim800.println(dialCmd);
    logSerial("[SIM] Dial command sent. Keeping call active for 20 seconds...");
    delay(20000);   // Keep call active for 20 seconds

    // Hang up
    simSendAT("ATH", "OK", 3000);
    logSerial("[SIM] Call ended.");
}

// ============================================================
//  SIM800L — SEND EMERGENCY SMS
//  number  : destination number string (e.g. "112" or "+91XXXXXXXXXX")
// ============================================================
void sendEmergencySMS(const char* number, float lat, float lng, bool hasFix)
{
    logSerial("[SIM] Sending emergency SMS...");

    // Set SMS text mode
    simSendAT("AT+CMGF=1", "OK", 2000);

    // Set recipient
    String cmgsCmd = "AT+CMGS=\"";
    cmgsCmd += number;
    cmgsCmd += "\"";
    sim800.println(cmgsCmd);
    delay(1000);

    // Build message body
    String message = "EMERGENCY ALERT: Vehicle accident detected!\n";

    if (hasFix)
    {
        message += "Location: https://maps.google.com/?q=";
        message += String(lat, 6);
        message += ",";
        message += String(lng, 6);
        message += "\n";
        message += "Speed: ";
        message += String(gps.speed.kmph(), 1);
        message += " km/h\n";
    }
    else
    {
        message += "GPS location unavailable.\n";
        message += "Please trace via cell tower.\n";
    }

    message += "Please send immediate assistance.";

    sim800.print(message);
    delay(500);
    sim800.write(26);   // Ctrl+Z = send SMS
    delay(5000);

    logSerial("[SIM] SMS sent.");
}

// ============================================================
//  SIM800L — SEND AT COMMAND & WAIT FOR EXPECTED RESPONSE
// ============================================================
bool simSendAT(const char* cmd, const char* expected, unsigned long timeoutMs)
{
    sim800.println(cmd);
    unsigned long start = millis();
    String response = "";

    while (millis() - start < timeoutMs)
    {
        while (sim800.available())
        {
            char c = sim800.read();
            response += c;
        }
        if (response.indexOf(expected) != -1)
        {
            Serial.printf("[SIM] CMD: %-20s → OK (%s)\n", cmd, expected);
            return true;
        }
    }

    Serial.printf("[SIM] CMD: %-20s → TIMEOUT (got: %s)\n",
                  cmd, response.c_str());
    return false;
}

// ============================================================
//  UTILITY — TAGGED SERIAL LOG
// ============================================================
void logSerial(const char* msg)
{
    Serial.print("[");
    Serial.print(millis());
    Serial.print("ms] ");
    Serial.println(msg);
}




// #include <Arduino.h>
// #include <Wire.h>
// #include "I2Cdev.h"
// #include "MPU6050.h"

// MPU6050 mpu;

// // raw values
// int16_t ax_raw, ay_raw, az_raw;
// int16_t gx_raw, gy_raw, gz_raw;

// // converted values
// float ax, ay, az;

// // linear acceleration
// float ax_lin, ay_lin, az_lin;

// // gravity estimator
// float g_est_x = 0.0f;
// float g_est_y = 0.0f;
// float g_est_z = 0.0f;

// const float gravity_alpha = 0.995f;

// void setup()
// {
//     Wire.begin(21, 22);

//     Serial.begin(115200);   // Keep 115200 for now

//     mpu.initialize();
//     mpu.setSleepEnabled(false);

//     mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
//     mpu.setDLPFMode(0);
// }

// void loop()
// {
//     // -------- READ SENSOR --------
//     mpu.getMotion6(&ax_raw, &ay_raw, &az_raw,
//                    &gx_raw, &gy_raw, &gz_raw);

//     ax = (ax_raw / 16384.0f) * 9.81f;
//     ay = (ay_raw / 16384.0f) * 9.81f;
//     az = (az_raw / 16384.0f) * 9.81f;

//     // -------- GRAVITY ESTIMATION --------
//     g_est_x = gravity_alpha * g_est_x + (1.0f - gravity_alpha) * ax;
//     g_est_y = gravity_alpha * g_est_y + (1.0f - gravity_alpha) * ay;
//     g_est_z = gravity_alpha * g_est_z + (1.0f - gravity_alpha) * az;

//     ax_lin = ax - g_est_x;
//     ay_lin = ay - g_est_y;
//     az_lin = az - g_est_z;

//     // -------- SERIAL OUTPUT --------
//     Serial.print(ax_lin, 4);
//     Serial.print(",");
//     Serial.print(ay_lin, 4);
//     Serial.print(",");
//     Serial.println(az_lin, 4);

//     // -------- 4ms delay (~25 Hz target) --------
//     delay(4);
// }

// // #include <TinyGPSPlus.h>
// // #include <HardwareSerial.h>

// // TinyGPSPlus gps;
// // HardwareSerial gpsSerial(2);

// // #define GPS_RX 16   // ESP32 RX pin connected to GPS TX
// // #define GPS_TX 17   // ESP32 TX pin connected to GPS RX

// // void setup() {
// //   Serial.begin(115200);
// //   delay(1000);

// //   gpsSerial.begin(115200, SERIAL_8N1, GPS_RX, GPS_TX);

// //   Serial.println("NEO-6M GPS Test Started");
// //   Serial.println("Waiting for GPS data...");
// // }

// // void loop() {
// //   while (gpsSerial.available() > 0) {
// //     char c = gpsSerial.read();

// //     // This prints raw GPS data like $GPGGA, $GPRMC
// //     Serial.write(c);

// //     gps.encode(c);

// //     if (gps.location.isUpdated()) {
// //       Serial.println();
// //       Serial.println("===== GPS LOCATION FOUND =====");

// //       Serial.print("Latitude: ");
// //       Serial.println(gps.location.lat(), 6);

// //       Serial.print("Longitude: ");
// //       Serial.println(gps.location.lng(), 6);

// //       Serial.print("Speed km/h: ");
// //       Serial.println(gps.speed.kmph());

// //       Serial.print("Satellites: ");
// //       Serial.println(gps.satellites.value());

// //       Serial.print("Altitude meters: ");
// //       Serial.println(gps.altitude.meters());

// //       Serial.println("------------------------------");
// //     }
// //   }

// //   // If GPS is connected but no signal, this helps check status
// //   static unsigned long lastPrint = 0;
// //   if (millis() - lastPrint > 5000) {
// //     lastPrint = millis();

// //     Serial.println();
// //     Serial.println("Still waiting for GPS fix...");
// //     Serial.print("Satellites: ");
// //     Serial.println(gps.satellites.value());
// //   }
// // }




// // #include <HardwareSerial.h>

// // // Use UART0 pins
// // // GPIO3 = RX0
// // // GPIO1 = TX0

// // #define SIM800_RX 16// ESP32 RX0 connected to SIM800L TXD
// // #define SIM800_TX 17//ESP32 TX0 connected to SIM800L RXD

// // HardwareSerial sim800(2);

// // void setup() {
// //   Serial.begin(115200);
// //   delay(1000);

// //   sim800.begin(115200, SERIAL_8N1, SIM800_RX, SIM800_TX);

// //   Serial.println("SIM800L Test Started");
// //   Serial.println("Testing SIM800L...");

// //   delay(2000);

// //   sim800.println("AT");
// //   delay(1000);

// //   sim800.println("AT+CPIN?");
// //   delay(1000);

// //   sim800.println("AT+CSQ");
// //   delay(1000);

// //   sim800.println("AT+CREG?");
// //   delay(1000);
// // }

// // void loop() {
// //   // Send data from Serial Monitor to SIM800L
// //   if (Serial.available()) {
// //     sim800.write(Serial.read());
// //   }

// //   // Show SIM800L reply on Serial Monitor
// //   if (sim800.available()) {
// //     Serial.write(sim800.read());
// //   }
// // }