#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>

// -------------------------------
// WLAN Zugangsdaten
// -------------------------------
const char* ssid     = "Pichler";
const char* password = "10031584918620092960";

// -------------------------------
// HiveMQ MQTT Cloud Broker (TLS)
// -------------------------------
const char* mqtt_server = "9f842ff8cdfa4626bbff7520495845c8.s1.eu.hivemq.cloud";
const int   mqtt_port   = 8883; // TLS-Port

// MQTT Zugangsdaten
const char* mqtt_user  = "Pichler";
const char* mqtt_pass  = "DPadgGLWnbdJ2025e!";
const char* clientID   = "klappe1";

// MQTT Topics (beibehalten)
const char* cmd_topic    = "huehnerklappe/klappe1/cmd";
const char* status_topic = "huehnerklappe/klappe1/status";

// -------------------------------
// WLAN + MQTT Client Objekte
// -------------------------------
WiFiClientSecure espClient;
PubSubClient     client(espClient);

// -------------------------------
// BTS7960 wiring (digital HIGH/LOW + Enables)
// -------------------------------
const int RPWM = 25;      // forward input
const int LPWM = 26;      // reverse input
const int R_EN_PIN = 27;  // enable right side  -> HIGH
const int L_EN_PIN = 14;  // enable left side   -> HIGH
const uint16_t DEADTIME_MS = 50; // dead-time beim Richtungswechsel

// -------------------------------
// Status-Variablen
// -------------------------------
String motorState = "stopped";

// -------------------------------
// WLAN verbinden
// -------------------------------
void setup_wifi() {
  Serial.println("Verbinde mit WLAN...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.printf("\nWLAN verbunden. IP: %s\n", WiFi.localIP().toString().c_str());
}

// -------------------------------
// Motor-Helfer (volle Geschwindigkeit)
// -------------------------------
void motorStop() {
  digitalWrite(RPWM, LOW);
  digitalWrite(LPWM, LOW);
}

void motorForward() {
  // Dead-time erzwingen, um Shoot-Through zu vermeiden
  motorStop();
  delay(DEADTIME_MS);
  digitalWrite(RPWM, HIGH);
  digitalWrite(LPWM, LOW);
}

void motorReverse() {
  motorStop();
  delay(DEADTIME_MS);
  digitalWrite(RPWM, LOW);
  digitalWrite(LPWM, HIGH);
}

// Vorwärts-Deklarationen
void openDoor();
void closeDoor();
void stopMotor(bool dueToError = false);

// -------------------------------
// Eingehende MQTT-Nachricht verarbeiten
// -------------------------------
void mqtt_callback(char* topic, byte* payload, unsigned int length) {
  String msg; msg.reserve(length);
  for (unsigned int i = 0; i < length; i++) msg += (char)payload[i];

  Serial.printf("Empfangen auf %s: %s\n", topic, msg.c_str());

  // optionales "cmd:NNN" akzeptieren, NNN wird hier ignoriert (volle Geschwindigkeit)
  int sep = msg.indexOf(':');
  String cmd = (sep >= 0) ? msg.substring(0, sep) : msg;

  if      (cmd == "open")  openDoor();
  else if (cmd == "close") closeDoor();
  else if (cmd == "stop")  stopMotor();
}

// -------------------------------
// MQTT-Verbindung (mit LWT)
// -------------------------------
void reconnect() {
  while (!client.connected()) {
    Serial.print("Verbindung zum MQTT-Broker...");

    // LWT: Broker publiziert "Offline" (QoS1, retained), wenn Verbindung unerwartet endet
    bool ok = client.connect(
      clientID,
      mqtt_user, mqtt_pass,
      status_topic, /*willQos=*/1, /*willRetain=*/true, /*willMessage=*/"Offline"
    );

    if (ok) {
      Serial.println("Verbunden.");
      client.subscribe(cmd_topic, 1);
      // einmalig "Online" (retained) nach erfolgreichem Connect
      client.publish(status_topic, "Online", true);
    } else {
      Serial.printf("Fehlgeschlagen, rc=%d. Neuer Versuch in 5s...\n", client.state());
      delay(5000);
    }
  }
}

// -------------------------------
// Aktionen (Status publizieren)
// -------------------------------
void openDoor() {
  motorState = "geöffnet";
  Serial.println("Öffne Klappe (Vollgas vorwärts)");
  motorForward();
  client.publish(status_topic, "geöffnet");
}

void closeDoor() {
  motorState = "geschlossen";
  Serial.println("Schließe Klappe (Vollgas rückwärts)");
  motorReverse();
  client.publish(status_topic, "geschlossen");
}

void stopMotor(bool dueToError) {
  motorState = "stopped";
  motorStop();
  Serial.println("Motor gestoppt.");
  client.publish(status_topic, dueToError ? "FEHLER: MOTORSTOPP" : "gestoppt");
}

// -------------------------------
// Initialisierung
// -------------------------------
void setup() {
  Serial.begin(115200);
  setup_wifi();

  // Motor-Pins konfigurieren
  pinMode(RPWM, OUTPUT);
  pinMode(LPWM, OUTPUT);
  motorStop(); // sicher starten

  // Enables aktivieren (oder auf dem Modul per Jumper auf HIGH)
  pinMode(R_EN_PIN, OUTPUT); digitalWrite(R_EN_PIN, HIGH);
  pinMode(L_EN_PIN, OUTPUT); digitalWrite(L_EN_PIN, HIGH);

  // MQTT-Client starten
  espClient.setInsecure();  // Demo: TLS ohne Zertifikatsprüfung
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(mqtt_callback);

  // Empfohlen: etwas großzügigeres KeepAlive + kurzer Socket-Timeout
  client.setKeepAlive(15);     // s  -> LWT nach ~22 s
  client.setSocketTimeout(5);  // s

  reconnect();
}

// -------------------------------
// Hauptloop: MQTT aktiv halten
// -------------------------------
void loop() {
  if (!client.connected()) reconnect();
  client.loop();
}
