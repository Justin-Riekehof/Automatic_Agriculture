#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>

// -------------------------------
// WLAN Zugangsdaten
// -------------------------------
// const char* ssid = "FRITZ!Powerline 1260";                  // WLAN-Name
// const char* password = "70323227422421195243"; // WLAN-Passwort
// Test-Verbindung im Haus-Netzwerk
const char* ssid = "Pichler";                  // WLAN-Name
const char* password = "10031584918620092960"; // WLAN-Passwort

// -------------------------------
// HiveMQ MQTT Cloud Broker
// -------------------------------
const char* mqtt_server = "9f842ff8cdfa4626bbff7520495845c8.s1.eu.hivemq.cloud";
const int mqtt_port = 8883; // TLS-Port

// MQTT Zugangsdaten
const char* mqtt_user = "Pichler";
const char* mqtt_pass = "DPadgGLWnbdJ2025e!";
const char* clientID = "klappe1";

// MQTT Topics
const char* cmd_topic    = "huehnerklappe/klappe1/cmd";
const char* status_topic = "huehnerklappe/klappe1/status";

// -------------------------------
// WLAN + MQTT Client Objekte
// -------------------------------
WiFiClientSecure espClient;
PubSubClient client(espClient);

// -------------------------------
// BTS7960 Pins
// -------------------------------
const int RPWM = 5;   // PWM Vorwärts
const int LPWM = 18;  // PWM Rückwärts

const int pwmFreq = 5000;      // PWM Frequenz
const int pwmResolution = 8;   // PWM Auflösung: 8 Bit (0–255)

// BTS7960 Current-Sense (IS) -> analog an ESP32
const int R_IS_PIN = 32;  // IS rechts (Vorwärts)
const int L_IS_PIN = 33;  // IS links  (Rückwärts)

// -------------------------------
// Status-Variablen
// -------------------------------
String motorState = "stopped";     // Aktueller Motorstatus
int currentPWM = 200;              // Letzter PWM Wert
unsigned long motorStartTime = 0;  // Zeitpunkt des letzten Motorstarts (Inrush-Ignore)

// --- Analogstrom-Überwachung: Filter & Grenzwerte (RAW 0..4095) ---
const uint16_t IS_THRESHOLD_RAW = 2200;  // <-- Kalibrieren!
const uint16_t IS_HYSTERESIS    = 150;
const float    IS_ALPHA         = 0.20f; // EMA-Faktor
uint16_t isFiltR = 0, isFiltL = 0;
bool isInitialized = false;

// -------------------------------
// WLAN verbinden
// -------------------------------
void setup_wifi() {
  delay(10);
  Serial.println("Verbinde mit WLAN...");
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWLAN verbunden.");
  Serial.println("IP-Adresse: " + WiFi.localIP().toString());
}

void stopMotor(bool dueToError = false);

// -------------------------------
// Eingehende MQTT-Nachricht verarbeiten
// -------------------------------
void mqtt_callback(char* topic, byte* payload, unsigned int length) {
  String msg;
  for (unsigned int i = 0; i < length; i++) msg += (char)payload[i];

  Serial.println("Empfangen: " + msg);

  int sep = msg.indexOf(':');
  String cmd = msg;
  int pwmValue = 200; // Standard PWM

  if (sep >= 0) {
    cmd = msg.substring(0, sep);
    pwmValue = msg.substring(sep + 1).toInt();
    pwmValue = constrain(pwmValue, 0, 255);
  }

  if (cmd == "open") {
    // Startzeit setzen, dann fahren
    openDoor(pwmValue);
  } else if (cmd == "close") {
    closeDoor(pwmValue);
  } else if (cmd == "stop") {
    stopMotor();
  }
}

// -------------------------------
// MQTT-Verbindung wiederherstellen
// -------------------------------
void reconnect() {
  while (!client.connected()) {
    Serial.print("Verbindung zum MQTT-Broker...");
    if (client.connect(clientID, mqtt_user, mqtt_pass)) {
      Serial.println("Verbunden.");
      client.subscribe(cmd_topic);
    } else {
      Serial.print("Fehlgeschlagen, rc=");
      Serial.print(client.state());
      delay(5000);
    }
  }
}

// -------------------------------
// Klappe öffnen (dauerhaft)
// -------------------------------
void openDoor(int pwmValue) {
  currentPWM = pwmValue;
  motorState = "geöffnet";
  motorStartTime = millis();  // Inrush-Ignore starten

  Serial.printf("Öffne Klappe mit PWM %d...\n", pwmValue);

  ledcWrite(RPWM, pwmValue);
  ledcWrite(LPWM, 0);

  client.publish(status_topic, "geöffnet");
}

// -------------------------------
// Klappe schließen (dauerhaft)
// -------------------------------
void closeDoor(int pwmValue) {
  currentPWM = pwmValue;
  motorState = "geschlossen";
  motorStartTime = millis();  // Inrush-Ignore starten

  Serial.printf("Schließe Klappe mit PWM %d...\n", pwmValue);

  ledcWrite(RPWM, 0);
  ledcWrite(LPWM, pwmValue);

  client.publish(status_topic, "geschlossen");
}

// -------------------------------
// Motor sofort stoppen
// -------------------------------
void stopMotor(bool dueToError) {
  motorState = "stopped";
  Serial.println("Motor gestoppt.");

  ledcWrite(RPWM, 0);
  ledcWrite(LPWM, 0);

  if (dueToError) {
    client.publish(status_topic, "FEHLER: ZU HOHER MOTORSTROM");
  } else {
    client.publish(status_topic, "gestoppt");
  }
}

// -------------------------------
// Überstrom prüfen (analog, gefiltert)
//  - 1 s Inrush-Ignore nach Start
//  - EMA-Filter
//  - Schwellwert + Hysterese
// -------------------------------
void checkOvercurrent() {
  if (millis() - motorStartTime < 1000) return; // Einschaltstrom ignorieren

  uint16_t rRaw = analogRead(R_IS_PIN);
  uint16_t lRaw = analogRead(L_IS_PIN);

  if (!isInitialized) { // Initialwert für Filter
    isFiltR = rRaw;
    isFiltL = lRaw;
    isInitialized = true;
  } else {
    isFiltR = (uint16_t)(IS_ALPHA * rRaw + (1.0f - IS_ALPHA) * isFiltR);
    isFiltL = (uint16_t)(IS_ALPHA * lRaw + (1.0f - IS_ALPHA) * isFiltL);
  }

  // Debug für Kalibrierung:
  // Serial.printf("IS R=%u (f=%u)  L=%u (f=%u)\n", rRaw, isFiltR, lRaw, isFiltL);

  static bool tripped = false;
  const uint16_t hi = IS_THRESHOLD_RAW;
  const uint16_t lo = (IS_THRESHOLD_RAW > IS_HYSTERESIS) ? (IS_THRESHOLD_RAW - IS_HYSTERESIS) : 0;

  bool over = (isFiltR >= hi) || (isFiltL >= hi);
  bool safe = (isFiltR <= lo)  && (isFiltL <= lo);

  if (!tripped && over) {
    Serial.println("FEHLER: Überstrom (analog) erkannt!");
    stopMotor(true);
    tripped = true;
  } else if (tripped && safe) {
    tripped = false; // Freigabe, wenn Strom zurückgeht
  }
}

// -------------------------------
// Initialisierung
// -------------------------------
void setup() {
  Serial.begin(115200);
  setup_wifi();

  // PWM-Kanäle initialisieren (Pin-basierte LEDC-API)
  ledcAttach(RPWM, pwmFreq, pwmResolution);
  ledcAttach(LPWM, pwmFreq, pwmResolution);

  // IS-Pins als ANALOGE Eingänge (keine Pullups/-downs!)
  pinMode(R_IS_PIN, INPUT);
  pinMode(L_IS_PIN, INPUT);
  analogReadResolution(12);                 // 0..4095
  analogSetPinAttenuation(R_IS_PIN, ADC_11db); // ~0..3.3V
  analogSetPinAttenuation(L_IS_PIN, ADC_11db);

  // MQTT-Client starten
  espClient.setInsecure();  // TLS ohne Zertifikatsprüfung
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(mqtt_callback);
}

// -------------------------------
// Hauptloop: MQTT aktiv halten + Überstrom überwachen
// -------------------------------
void loop() {
  if (!client.connected()) reconnect();
  client.loop();

  checkOvercurrent();
}
