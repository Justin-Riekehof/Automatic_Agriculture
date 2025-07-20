#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>

// -------------------------------
// WLAN Zugangsdaten
// -------------------------------
// const char* ssid = "FRITZ!Powerline 1260";                  // WLAN-Name
// const char* password = "70323227422421195243"; // WLAN-Passwort
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
const char* cmd_topic = "huehnerklappe/klappe1/cmd";       
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

// BTS7960 Überstrom-Pins
const int R_IS_PIN = 32;  // Vorwärts Warnung
const int L_IS_PIN = 33;  // Rückwärts Warnung

// -------------------------------
// Status-Variablen
// -------------------------------
String motorState = "stopped";  // Aktueller Motorstatus
int currentPWM = 200;           // Letzter PWM Wert

// -------------------------------
// Optionen
// -------------------------------
const bool ENABLE_OVERCURRENT_CHECK = false;  // true = Überstromüberwachung aktiv

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
  for (unsigned int i = 0; i < length; i++) {
    msg += (char)payload[i];
  }

  Serial.println("Empfangen: " + msg);

  int sep = msg.indexOf(':');
  String cmd = msg;
  int pwmValue = 200; // Standard PWM, falls nicht angegeben

  if (sep >= 0) {
    cmd = msg.substring(0, sep);
    pwmValue = msg.substring(sep + 1).toInt();
    pwmValue = constrain(pwmValue, 0, 255);
  }

  if (cmd == "open") {
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
// --> Umgekehrte Drehrichtung
// -------------------------------
void openDoor(int pwmValue) {
  currentPWM = pwmValue;
  motorState = "geöffnet";

  Serial.printf("Öffne Klappe mit PWM %d...\n", pwmValue);

  ledcWrite(0, 0);        // RPWM AUS
  ledcWrite(1, pwmValue); // LPWM AN

  client.publish(status_topic, "geöffnet");
}

// -------------------------------
// Klappe schließen (dauerhaft)
// --> Umgekehrte Drehrichtung
// -------------------------------
void closeDoor(int pwmValue) {
  currentPWM = pwmValue;
  motorState = "geschlossen";

  Serial.printf("Schließe Klappe mit PWM %d...\n", pwmValue);

  ledcWrite(0, pwmValue); // RPWM AN
  ledcWrite(1, 0);        // LPWM AUS

  client.publish(status_topic, "geschlossen");
}

// -------------------------------
// Motor sofort stoppen
// -------------------------------
void stopMotor(bool dueToError) {
  motorState = "stopped";

  Serial.println("Motor gestoppt.");

  ledcWrite(0, 0);
  ledcWrite(1, 0);

  if (dueToError) {
    client.publish(status_topic, "FEHLER: ZU HOHER MOTORSTROM");
  } else {
    client.publish(status_topic, "gestoppt");
  }
}

// -------------------------------
// Überstrom prüfen
// -------------------------------
void checkOvercurrent() {
  if (!ENABLE_OVERCURRENT_CHECK) return; // Prüfung deaktiviert

  bool r_is = digitalRead(R_IS_PIN);
  bool l_is = digitalRead(L_IS_PIN);

  if (r_is == HIGH || l_is == HIGH) {
    Serial.println("FEHLER: ZU HOHER MOTORSTROM erkannt!");
    stopMotor(true); // Stoppe mit Fehlerstatus
  }
}

// -------------------------------
// Initialisierung
// -------------------------------
void setup() {
  Serial.begin(115200);
  setup_wifi();

  // Configure PWM channels
  ledcSetup(0, pwmFreq, pwmResolution); // Channel 0 for RPWM
  ledcSetup(1, pwmFreq, pwmResolution); // Channel 1 for LPWM

  // Attach pins to channels
  ledcAttachPin(RPWM, 0);
  ledcAttachPin(LPWM, 1);

  // Überstrom Pins als Input konfigurieren
  pinMode(R_IS_PIN, INPUT);
  pinMode(L_IS_PIN, INPUT);

  // MQTT-Client starten
  espClient.setInsecure();  // TLS ohne Zertifikatsprüfung
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(mqtt_callback);
}

// -------------------------------
// Hauptloop: MQTT aktiv halten + Überstrom überwachen
// -------------------------------
void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  checkOvercurrent();
}
