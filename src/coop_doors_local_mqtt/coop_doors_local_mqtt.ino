#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>

// ===============================
// ESP32 talks to LOCAL Mosquitto
// ===============================
const bool USE_TLS        = false;  // ESP stays local; PC bridge does the cloud
const bool LOCAL_USE_AUTH = false;  // set true if your local Mosquitto requires user/pass

// -------------------------------
// Wi-Fi
// -------------------------------
const char* ssid     = "Pichler";
const char* password = "10031584918620092960";

// -------------------------------
// MQTT (LOCAL broker)
// -------------------------------
const char* mqtt_server_local = "192.168.178.83";   // <-- your PC’s IP here
const int   mqtt_port_plain   = 1883;
const char* mqtt_user_local   = "user";             // used only if LOCAL_USE_AUTH=true
const char* mqtt_pass_local   = "pass";

// (Optional: direct HiveMQ creds — NOT used when USE_TLS=false)
const char* mqtt_server_hivemq = "9f842ff8cdfa4626bbff7520495845c8.s1.eu.hivemq.cloud";
const int   mqtt_port_tls      = 8883;
const char* mqtt_user_hivemq   = "Pichler";
const char* mqtt_pass_hivemq   = "DPadgGLWnbdJ2025e!";

// IDs & Topics
const char* clientID     = "klappe2";
const char* cmd_topic    = "huehnerklappe/klappe2/cmd";
const char* status_topic = "huehnerklappe/klappe2/status";

// -------------------------------
// Clients
// -------------------------------
WiFiClientSecure tlsClient;
WiFiClient       tcpClient;
PubSubClient     client(USE_TLS ? (Client&)tlsClient : (Client&)tcpClient);

// -------------------------------
// BTS7960 wiring (digital HIGH/LOW only)
// -------------------------------
const int RPWM = 25;   // forward input
const int LPWM = 26;   // reverse input
const int R_EN_PIN = 27;  // enable (set HIGH)
const int L_EN_PIN = 14;  // enable (set HIGH)
const uint16_t DEADTIME_MS = 50; // dead-time when changing direction

// -------------------------------
// State
// -------------------------------
String motorState = "stopped";

// -------------------------------
// Helper: motor control (full speed)
// -------------------------------
void motorStop() {
  digitalWrite(RPWM, LOW);
  digitalWrite(LPWM, LOW);
}

void motorForward() {
  // enforce dead-time to avoid shoot-through
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

// -------------------------------
// Wi-Fi
// -------------------------------
void setup_wifi() {
  Serial.println("Verbinde mit WLAN...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(400);
    Serial.print(".");
  }
  Serial.printf("\nWLAN verbunden. IP: %s  RSSI: %d dBm\n",
                WiFi.localIP().toString().c_str(), WiFi.RSSI());
}

// -------------------------------
// MQTT callback
// -------------------------------
void openDoor();   // forward decls using digital control
void closeDoor();
void stopMotor(bool dueToError = false);

void mqtt_callback(char* topic, byte* payload, unsigned int length) {
  String msg; msg.reserve(length);
  for (unsigned int i = 0; i < length; i++) msg += (char)payload[i];
  Serial.printf("Empfangen auf %s: %s\n", topic, msg.c_str());

  // Accept optional "cmd:NNN" but we ignore the NNN (full speed always)
  int sep = msg.indexOf(':');
  String cmd = (sep >= 0) ? msg.substring(0, sep) : msg;

  if      (cmd == "open")  openDoor();
  else if (cmd == "close") closeDoor();
  else if (cmd == "stop")  stopMotor();
}

// -------------------------------
// MQTT connect (with LWT)
// -------------------------------
bool connectMqtt() {
  Serial.print("Verbinde zum MQTT-Broker... ");
  const char* willPayload = "offline";
  const bool  willRetain  = true;
  const int   willQoS     = 1;

  bool ok = false;
  if (USE_TLS) {
    ok = client.connect(clientID,
                        mqtt_user_hivemq, mqtt_pass_hivemq,
                        status_topic, willQoS, willRetain, willPayload);
  } else {
    if (LOCAL_USE_AUTH) {
      ok = client.connect(clientID,
                          mqtt_user_local, mqtt_pass_local,
                          status_topic, willQoS, willRetain, willPayload);
    } else {
      ok = client.connect(clientID,
                          status_topic, willQoS, willRetain, willPayload);
    }
  }

  if (ok) {
    Serial.println("verbunden.");
    client.subscribe(cmd_topic, 1);
    client.publish(status_topic, "online", true); // retained presence
  } else {
    Serial.printf("fehlgeschlagen, rc=%d\n", client.state());
  }
  return ok;
}

void reconnect() {
  while (!client.connected()) {
    if (connectMqtt()) break;
    delay(600);
  }
}

// -------------------------------
// Motor actions (publish status)
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
// Setup
// -------------------------------
void setup() {
  Serial.begin(115200);
  setup_wifi();

  // Configure motor pins
  pinMode(RPWM, OUTPUT);
  pinMode(LPWM, OUTPUT);
  motorStop(); // start safe

  // Enables HIGH (you can alternatively hard-jumper these to 5 V on the module; never feed 5 V into ESP32)
  if (R_EN_PIN >= 0) { pinMode(R_EN_PIN, OUTPUT); digitalWrite(R_EN_PIN, HIGH); }
  if (L_EN_PIN >= 0) { pinMode(L_EN_PIN, OUTPUT); digitalWrite(L_EN_PIN, HIGH); }

  // MQTT target
  if (USE_TLS) {
    tlsClient.setInsecure(); // demo only; normally verify CA!
    client.setServer(mqtt_server_hivemq, mqtt_port_tls);
    Serial.println("MQTT over TLS (direkt HiveMQ) – nur falls bewusst aktiviert.");
  } else {
    client.setServer(mqtt_server_local, mqtt_port_plain);
    Serial.println("MQTT lokal ohne TLS (Bridge spiegelt zur Cloud).");
  }

  client.setCallback(mqtt_callback);
  client.setKeepAlive(60);
  client.setSocketTimeout(5);

  reconnect();
}

// -------------------------------
// Loop
// -------------------------------
void loop() {
  if (!client.connected()) reconnect();
  client.loop();
}
