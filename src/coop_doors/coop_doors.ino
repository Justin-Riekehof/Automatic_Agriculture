#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <time.h>

// -------------------------------
// Wi-Fi
// -------------------------------
const char* ssid     = "Pichler";
const char* password = "10031584918620092960";

// -------------------------------
// HiveMQ MQTT Cloud (TLS)
// -------------------------------
const char* mqtt_server = "9f842ff8cdfa4626bbff7520495845c8.s1.eu.hivemq.cloud";
const int   mqtt_port   = 8883;
const char* mqtt_user   = "Pichler";
const char* mqtt_pass   = "DPadgGLWnbdJ2025e!";
const char* clientID    = "klappe2";  // wichtig: einzigartig

// Topics für Tür 1
const char* cmd_topic         = "huehnerklappe/klappe2/cmd";
const char* status_topic      = "huehnerklappe/klappe2/status";
const char* openTime_topic    = "huehnerklappe/klappe2/openTime";
const char* closeTime_topic   = "huehnerklappe/klappe2/closeTime";
const char* schedule_topic    = "huehnerklappe/klappe2/zeitstatus";

// -------------------------------
// NTP (CET/CEST Berlin)
// -------------------------------
const char* ntpServer          = "pool.ntp.org";
const long  gmtOffset_sec      = 3600;
const int   daylightOffset_sec = 3600;

// -------------------------------
// Zeitschaltung
// -------------------------------
int openHour   = 7;
int openMinute = 0;

int closeHour   = 20;
int closeMinute = 0;

// -1 bedeutet deaktiviert
bool openDisabled  = false;
bool closeDisabled = false;

bool openDoneToday  = false;
bool closeDoneToday = false;
int  lastDay        = -1;

// -------------------------------
bool timeSynced = false;
unsigned long lastTimeRetryMs = 0;
const unsigned long TIME_RETRY_INTERVAL_MS = 60000;

// -------------------------------
// Clients
// -------------------------------
WiFiClientSecure espClient;
PubSubClient client(espClient);

// -------------------------------
// BTS7960
// -------------------------------
const int RPWM = 25;
const int LPWM = 26;
const int R_EN_PIN = 27;
const int L_EN_PIN = 14;
const uint16_t DEADTIME_MS = 50;

// -------------------------------
String doorStatus = "Status unbekannt";
unsigned long lastHeartbeatMs = 0;
const unsigned long HEARTBEAT_INTERVAL_MS = 30000;

// -----------------------------------------------------------
// MOTORSTEUERUNG – DAUERBETRIEB BIS NEUER BEFEHL
// -----------------------------------------------------------
void motorStop() {
  digitalWrite(RPWM, LOW);
  digitalWrite(LPWM, LOW);
  Serial.println("[MOTOR] STOP");
}

void motorForward() {
  motorStop();
  delay(DEADTIME_MS);
  digitalWrite(RPWM, HIGH);
  digitalWrite(LPWM, LOW);
  Serial.println("[MOTOR] Öffnen (dauerhaft)...");
}

void motorReverse() {
  motorStop();
  delay(DEADTIME_MS);
  digitalWrite(RPWM, LOW);
  digitalWrite(LPWM, HIGH);
  Serial.println("[MOTOR] Schließen (dauerhaft)...");
}

void publishDoorStatusRetained() {
  client.publish(status_topic, doorStatus.c_str(), true);
}

// -----------------------------------------------------------
// ZEITSTATUS AUSGEBEN
// -----------------------------------------------------------
void publishScheduleInfo() {
  if (!client.connected()) return;

  if (!timeSynced) {
    client.publish(schedule_topic, "Kein Internet -> Keine Uhrzeit", true);
    return;
  }

  char oBuf[8];
  char cBuf[8];

  if (openDisabled)  strcpy(oBuf, "-");
  else snprintf(oBuf, sizeof(oBuf), "%02d:%02d", openHour, openMinute);

  if (closeDisabled) strcpy(cBuf, "-");
  else snprintf(cBuf, sizeof(cBuf), "%02d:%02d", closeHour, closeMinute);

  char msg[64];
  snprintf(msg, sizeof(msg), "Öffnen: %s; Schließen: %s", oBuf, cBuf);

  client.publish(schedule_topic, msg, true);
  Serial.printf("[MQTT] schedule -> %s\n", msg);
}

// -----------------------------------------------------------
// MOTOR KOMMANDOS
// -----------------------------------------------------------
void openDoor() {
  motorForward();
  doorStatus = "geöffnet";  // logisch
  publishDoorStatusRetained();
}

void closeDoor() {
  motorReverse();
  doorStatus = "geschlossen";
  publishDoorStatusRetained();
}

void stopMotorCommand() {
  motorStop();
  publishDoorStatusRetained();
}

// -----------------------------------------------------------
// NTP-ZEIT
// -----------------------------------------------------------
void setup_time() {
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

  struct tm timeinfo;
  int retry = 0;

  Serial.print("[TIME] Warte auf NTP");

  while (!getLocalTime(&timeinfo) && retry < 10) {
    Serial.print(".");
    delay(500);
    retry++;
  }

  if (!getLocalTime(&timeinfo)) {
    Serial.println("\n[TIME] Keine Zeit erhalten.");
    timeSynced = false;
  } else {
    Serial.println("\n[TIME] Zeit OK.");
    timeSynced = true;
    lastDay = timeinfo.tm_mday;
  }

  lastTimeRetryMs = millis();
}

void retryTimeIfNeeded() {
  if (timeSynced) return;
  if (millis() - lastTimeRetryMs < TIME_RETRY_INTERVAL_MS) return;

  lastTimeRetryMs = millis();

  struct tm timeinfo;
  if (getLocalTime(&timeinfo)) {
    Serial.println("[TIME] Zeit jetzt OK.");
    timeSynced = true;
    lastDay = timeinfo.tm_mday;
    publishScheduleInfo();
  }
}

// -----------------------------------------------------------
// ZEITSTRING PARSER
// -----------------------------------------------------------
bool setTimeFromString(const String& payload, int &hourVar, int &minVar, bool &disableFlag) {
  
  if (payload == "-1") {
    disableFlag = true;
    Serial.println("[TIME] Zeit deaktiviert (-1).");
    return true;
  }

  int h = 0, m = 0;
  if (sscanf(payload.c_str(), "%d:%d", &h, &m) == 2) {
    if (h >= 0 && h < 24 && m >= 0 && m < 60) {
      hourVar = h;
      minVar  = m;
      disableFlag = false;
      Serial.printf("[TIME] Neue Zeit %02d:%02d gesetzt\n", hourVar, minVar);
      return true;
    }
  }

  Serial.printf("[TIME] Ungültige Eingabe: '%s'\n", payload.c_str());
  return false;
}

// -----------------------------------------------------------
// AUTOMATIK FÜR ÖFFNEN / SCHLIEßEN
// -----------------------------------------------------------
void checkDoorSchedule() {
  if (!timeSynced) return;

  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) return;

  int h = timeinfo.tm_hour;
  int m = timeinfo.tm_min;
  int d = timeinfo.tm_mday;

  if (d != lastDay) {
    lastDay = d;
    openDoneToday = false;
    closeDoneToday = false;
  }

  if (!openDisabled &&
      !openDoneToday &&
      h == openHour &&
      m == openMinute) {

    Serial.println("[SCHED] Automatisches Öffnen");
    openDoor();
    openDoneToday = true;
  }

  if (!closeDisabled &&
      !closeDoneToday &&
      h == closeHour &&
      m == closeMinute) {

    Serial.println("[SCHED] Automatisches Schließen");
    closeDoor();
    closeDoneToday = true;
  }
}

// -----------------------------------------------------------
// WIFI
// -----------------------------------------------------------
void setup_wifi() {
  Serial.println("WiFi connect…");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(400);
    Serial.print(".");
  }
  Serial.printf("\nWiFi OK  IP=%s\n", WiFi.localIP().toString().c_str());
}

// -----------------------------------------------------------
// MQTT CALLBACK
// -----------------------------------------------------------
void mqtt_callback(char* topic, byte* payload, unsigned int length) {
  
  String msg;
  for (unsigned int i = 0; i < length; i++) msg += (char)payload[i];
  msg.trim();
  msg.toLowerCase();

  Serial.printf("[MQTT] RX %s: %s\n", topic, msg.c_str());

  if (strcmp(topic, cmd_topic) == 0) {
    if      (msg == "open")  openDoor();
    else if (msg == "close") closeDoor();
    else if (msg == "stop")  stopMotorCommand();
    return;
  }

  if (strcmp(topic, openTime_topic) == 0) {
    if (setTimeFromString(msg, openHour, openMinute, openDisabled)) {
      openDoneToday = false;
      publishScheduleInfo();
    }
    return;
  }

  if (strcmp(topic, closeTime_topic) == 0) {
    if (setTimeFromString(msg, closeHour, closeMinute, closeDisabled)) {
      closeDoneToday = false;
      publishScheduleInfo();
    }
    return;
  }
}

// -----------------------------------------------------------
// MQTT CONNECT
// -----------------------------------------------------------
void reconnect() {
  while (!client.connected()) {
    Serial.print("MQTT connect… ");

    bool ok = client.connect(
      clientID,
      mqtt_user, mqtt_pass,
      status_topic, 1, true, "offline"
    );

    if (ok) {
      Serial.println("OK");
      client.subscribe(cmd_topic, 1);
      client.subscribe(openTime_topic, 1);
      client.subscribe(closeTime_topic, 1);

      publishDoorStatusRetained();
      publishScheduleInfo();

    } else {
      Serial.printf("fail rc=%d retry in 2s\n", client.state());
      delay(2000);
    }
  }
}

// -----------------------------------------------------------
// SETUP / LOOP
// -----------------------------------------------------------
void setup() {
  Serial.begin(115200);
  setup_wifi();

  pinMode(RPWM, OUTPUT);
  pinMode(LPWM, OUTPUT);
  pinMode(R_EN_PIN, OUTPUT); digitalWrite(R_EN_PIN, HIGH);
  pinMode(L_EN_PIN, OUTPUT); digitalWrite(L_EN_PIN, HIGH);
  motorStop();

  espClient.setInsecure();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(mqtt_callback);

  setup_time();
  reconnect();

  publishDoorStatusRetained();
  publishScheduleInfo();

  lastHeartbeatMs = millis();
}

void loop() {
  if (!client.connected()) reconnect();
  client.loop();

  retryTimeIfNeeded();
  checkDoorSchedule();

  if (millis() - lastHeartbeatMs >= HEARTBEAT_INTERVAL_MS) {
    lastHeartbeatMs = millis();
    publishDoorStatusRetained();
  }
}
