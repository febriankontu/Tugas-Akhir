#include <WiFi.h>
#include <Firebase_ESP_Client.h>
#include <TinyGPSPlus.h>
#include "addons/TokenHelper.h"

// ==========================================
// 1. KONFIGURASI WIFI & FIREBASE
// ==========================================
#define WIFI_SSID       "feb"            // Ganti dengan nama WiFi kamu
#define WIFI_PASSWORD   "passwordapa"    // Ganti dengan password WiFi kamu
#define API_KEY         "AIzaSyAzSGDRNwlLvcSCPomMYcfihT5pNdaYXKA"
#define FIREBASE_PROJECT_ID "npkmonitoring-b0d7c"

// ==========================================
// 2. KONFIGURASI PIN HARDWARE
// ==========================================
// GPS -> UART 2 (Pin 16 & 17)
#define GPS_RX_PIN 16
#define GPS_TX_PIN 17

// NPK -> UART 1 (Pin 33 & 32)
#define NPK_RX_PIN 33
#define NPK_TX_PIN 32

// ==========================================
// 3. OBJEK & VARIABEL
// ==========================================
TinyGPSPlus gps;
HardwareSerial gpsSerial(2);
HardwareSerial npkSerial(1);

FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

bool signupOK = false;
unsigned long lastSendTimeNPK = 0;
unsigned long lastSendTimeGPS = 0;

// Command Baca NPK (Alamat 0x1E)
const byte npkQuery[] = {0x01, 0x03, 0x00, 0x1E, 0x00, 0x03, 0x65, 0xCD};

// Variabel Data
float lat = 0.0;
float lng = 0.0;
int val_N = 0; // mg/kg
int val_P = 0; // mg/kg
int val_K = 0; // mg/kg

// ==========================================
// 4. SETUP
// ==========================================
void setup() {
  Serial.begin(115200);
  
  // Init Serial Sensor
  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  npkSerial.begin(9600, SERIAL_8N1, NPK_RX_PIN, NPK_TX_PIN);

  Serial.println("Sistem Monitoring Dimulai...");

  // Koneksi WiFi
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Menghubungkan WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print("."); delay(300);
  }
  Serial.println("\nWiFi Terhubung!");

  // Koneksi Firebase
  config.api_key = API_KEY;
  config.service_account.data.project_id = FIREBASE_PROJECT_ID;

  if (Firebase.signUp(&config, &auth, "", "")) {
    Serial.println("Firebase Auth Berhasil");
    signupOK = true;
  } else {
    Serial.printf("Firebase Error: %s\n", config.signer.signupError.message.c_str());
  }

  config.token_status_callback = tokenStatusCallback;
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);
}

// ==========================================
// 5. BACA NPK
// ==========================================
void readNPK() {
  // Bersihkan buffer
  while (npkSerial.available()) npkSerial.read();

  // Kirim Perintah
  npkSerial.write(npkQuery, sizeof(npkQuery));
  delay(200); // Tunggu respon sensor sebentar

  if (npkSerial.available()) {
    byte values[11];
    int bytesRead = npkSerial.readBytes(values, 11);

    // Validasi Header (ID=1, Func=3)
    if (bytesRead >= 7 && values[0] == 0x01 && values[1] == 0x03) {
      // Parsing Data (High Byte + Low Byte)
      val_N = (values[3] << 8) | values[4];
      val_P = (values[5] << 8) | values[6];
      val_K = (values[7] << 8) | values[8];
      
      Serial.printf("[SENSOR NPK] N:%d P:%d K:%d\n", val_N, val_P, val_K);
    } 
  } else {
    Serial.println("[SENSOR NPK] Timeout/Error.");
  }
}

// ==========================================
// 6. LOOP UTAMA
// ==========================================
void loop() {
  // 1. Feed GPS Terus Menerus (Wajib agar data GPS update)
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }

  // Cek Status Firebase & WiFi
  if (Firebase.ready() && signupOK) {

    // ==================================================
    // LOGIKA 1: KIRIM GPS SETIAP 1 DETIK
    // ==================================================
    if (millis() - lastSendTimeGPS > 1000) {
      lastSendTimeGPS = millis();

      // Cek apakah ada lokasi valid
      if (gps.location.isValid()) {
        lat = gps.location.lat();
        lng = gps.location.lng();
      } else {
        // Jika GPS belum lock, bisa pakai data dummy atau biarkan 0
        // lat = 0.0; lng = 0.0; 
        Serial.println("GPS Searching...");
      }

      // Upload KHUSUS GPS
      FirebaseJson content;
      content.set("fields/latitude/doubleValue", lat);
      content.set("fields/longitude/doubleValue", lng);

      Serial.printf("[UPLOAD 1s] GPS -> Lat:%.6f Lng:%.6f\n", lat, lng);

      // Kita pakai updateMask hanya untuk latitude & longitude agar tidak menimpa NPK
      if (Firebase.Firestore.patchDocument(&fbdo, FIREBASE_PROJECT_ID, "", "Data_testing/live_data", content.raw(), "latitude,longitude")) {
         // Sukses GPS
      } else {
         Serial.print("Gagal Upload GPS: "); Serial.println(fbdo.errorReason());
      }
    }

    // ==================================================
    // LOGIKA 2: KIRIM NPK SETIAP 3 DETIK
    // ==================================================
    if (millis() - lastSendTimeNPK > 3000) {
      lastSendTimeNPK = millis();

      // Baca Sensor
      readNPK();

      // Upload KHUSUS NPK
      FirebaseJson content;
      content.set("fields/nitrogen/integerValue", val_N);
      content.set("fields/phosphorus/integerValue", val_P);
      content.set("fields/potassium/integerValue", val_K);

      Serial.printf("[UPLOAD 3s] NPK -> N:%d P:%d K:%d\n", val_N, val_P, val_K);

      // Kita pakai updateMask hanya untuk field NPK
      if (Firebase.Firestore.patchDocument(&fbdo, FIREBASE_PROJECT_ID, "", "Data_testing/live_data", content.raw(), "nitrogen,phosphorus,potassium")) {
         // Sukses NPK
      } else {
         Serial.print("Gagal Upload NPK: "); Serial.println(fbdo.errorReason());
      }
    }
  }
}