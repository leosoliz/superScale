#include <Arduino.h>
#include <ArduinoOTA.h>
#include <DNSServer.h>
#include <HX711.h>
#include <Preferences.h>
#include <PubSubClient.h>
#include <WebServerSecure.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <WiFiManager.h>

#include <cmath>

namespace Pins {
constexpr uint8_t HX711_DOUT = 19;
constexpr uint8_t HX711_SCK = 18;
constexpr uint8_t THERMISTOR_ADC = 34;
}  // namespace Pins

namespace Defaults {
constexpr float SCALE_FACTOR = -21500.0f;
constexpr float TARE_KG = 32.0f;
constexpr float FULL_GLP_KG = 13.0f;
constexpr float GLP_DENSITY_KG_L = 0.54f;
constexpr float TEMP_SERIES_RESISTOR = 10000.0f;
constexpr float THERMISTOR_NOMINAL = 10000.0f;
constexpr float TEMPERATURE_NOMINAL = 25.0f;
constexpr float BETA_COEFFICIENT = 3950.0f;
constexpr uint16_t ADC_MAX = 4095;
constexpr uint32_t READ_INTERVAL_MS = 2000;
constexpr uint32_t MQTT_INTERVAL_MS = 10000;
constexpr uint16_t HTTPS_PORT = 443;
constexpr char HOSTNAME[] = "esp32-glp-scale";
constexpr char AP_NAME[] = "GLP-Scale-Setup";
constexpr char AP_PASSWORD[] = "glp12345";
constexpr char MQTT_TOPIC_PREFIX[] = "glp/scale";
}  // namespace Defaults

struct DeviceConfig {
  char mqttHost[64] = "";
  uint16_t mqttPort = 8883;
  char mqttUser[32] = "";
  char mqttPassword[32] = "";
  char mqttTopic[96] = Defaults::MQTT_TOPIC_PREFIX;
  char deviceId[32] = "balanca-glp-01";
  float scaleFactor = Defaults::SCALE_FACTOR;
  long hxOffset = 0;
  float tareKg = Defaults::TARE_KG;
  float fullGlpKg = Defaults::FULL_GLP_KG;
  float glpDensityKgL = Defaults::GLP_DENSITY_KG_L;
  float seriesResistor = Defaults::TEMP_SERIES_RESISTOR;
  float thermistorNominal = Defaults::THERMISTOR_NOMINAL;
  float temperatureNominal = Defaults::TEMPERATURE_NOMINAL;
  float betaCoefficient = Defaults::BETA_COEFFICIENT;
  bool mqttTls = true;
};

struct Measurements {
  float grossKg = 0.0f;
  float tareKg = Defaults::TARE_KG;
  float netGlpKg = 0.0f;
  float levelPct = 0.0f;
  float estimatedLiters = 0.0f;
  float temperatureC = 0.0f;
  bool valid = false;
};

Preferences preferences;
HX711 scale;
WiFiManager wifiManager;
WiFiClient espClient;
WiFiClientSecure espClientSecure;
PubSubClient mqttClient;
WebServerSecure httpsServer(Defaults::HTTPS_PORT);
DeviceConfig config;
Measurements current;
uint32_t lastReadMs = 0;
uint32_t lastMqttMs = 0;

static const char HTTPS_CERT[] PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
MIIDFTCCAf2gAwIBAgIUVf9BkRuR+wDicJAbBrgaDGfyBT4wDQYJKoZIhvcNAQEL
BQAwGjEYMBYGA1UEAwwPZXNwMzItZ2xwLmxvY2FsMB4XDTI2MDMyMjE0Mjc0M1oX
DTM2MDMxOTE0Mjc0M1owGjEYMBYGA1UEAwwPZXNwMzItZ2xwLmxvY2FsMIIBIjAN
BgkqhkiG9w0BAQEFAAOCAQ8AMIIBCgKCAQEAoXv888DZqONMDha1JNqjghfQdNMW
LgY3mPneubRHLNlZ1keZZbj6yQNmdiKTzUhktKdlfUkkHsxVH4AO0c27/f61qwyM
c+mlFnhROTA5lU8XaN9Tm/rKNEDJvrlKZOykwnc6NzPbTvaV+EHj6I//FqaIBffJ
Wd7VcFqapD7RCfssDu/VxZdPtf0BYIPszMTQpfh5id+xtjZk4ujcSZXcmfGMwIKh
k15wMCBfC7ZGu6Kmjq0JUHpNQWVD2uVPSXGYb5TuhHjnenIbMcp1vrvnUEMPWu22
0eadZuj8we4OJP3vXjzk93dYjYN2fy4Ak5YakhNGiKYzkTyV5QhIfg3AeQIDAQAB
o1MwUTAdBgNVHQ4EFgQUIqoipzVmZXOEuruZi4oRFKOWKKIwHwYDVR0jBBgwFoAU
IqoipzVmZXOEuruZi4oRFKOWKKIwDwYDVR0TAQH/BAUwAwEB/zANBgkqhkiG9w0B
AQsFAAOCAQEAkLqgzUsde+ITZnr3Xl0FQ8BoXUL9JpSZu+q0kPa/iyD+dC8mzqCm
Jtb+Ce+4/AF6q5orXU6cdQC+k/AivA+SiCqVEASzf0Pfs7IccAoals0cQJ1qjOgz
bs9crNrcOud5hMX+TAuLIwZAggX0IsbK1M1ILZv3mXXwcDxpF/24e5SDIDrzb1K+
thb7rczFWmOGf1+S4FsZii6+RdZQDoIEa6KSW2HbKUcMc1/1mBza7eWeHBAjJQLP
nG0h3hAJ2YoT4zBYKRDPmJDNZqIwHVr58Jhju54gzc0Tasgend9FgoHB87SfMnsV
IIxkyY5JABll3c8+aeoevp9huhT5FDtUiA==
-----END CERTIFICATE-----
)EOF";

static const char HTTPS_KEY[] PROGMEM = R"EOF(
-----BEGIN PRIVATE KEY-----
MIIEvAIBADANBgkqhkiG9w0BAQEFAASCBKYwggSiAgEAAoIBAQChe/zzwNmo40wO
FrUk2qOCF9B00xYuBjeY+d65tEcs2VnWR5lluPrJA2Z2IpPNSGS0p2V9SSQezFUf
gA7Rzbv9/rWrDIxz6aUWeFE5MDmVTxdo31Ob+so0QMm+uUpk7KTCdzo3M9tO9pX4
QePoj/8WpogF98lZ3tVwWpqkPtEJ+ywO79XFl0+1/QFgg+zMxNCl+HmJ37G2NmTi
6NxJldyZ8YzAgqGTXnAwIF8Ltka7oqaOrQlQek1BZUPa5U9JcZhvlO6EeOd6chsx
ynW+u+dQQw9a7bbR5p1m6PzB7g4k/e9ePOT3d1iNg3Z/LgCTlhqSE0aIpjORPJXl
CEh+DcB5AgMBAAECggEAEeatvtO1DTzNNI6d3A0ErSo4qIqx3b5AsYuZfhqW/UjG
5bMgNR+RZXEkDZp9qfcJeuFHpeDSyTs7gHdwrR9SZLC0sNZ+R2cyLtB9qIpJB90x
qiV/nj3p5mr8MlFWBuQYy5nt8SqleBZrv46GVkSIMZmaYJb8UiMapA7XL7fq8mEh
ZUZGT97SYfJkIrEjbDisFALYqj1MhtWa//Nb2IeMPTEI5mXnkDMrUv+KaoY1ZIus
ELEs0vPn4QQyses5LUq1F5EBWH2M6tSX167bzTn7IcwNdId6SuSZ2A0R/nP0RFw+
HmbHv87XMEYDlYSHm3EaYd6Lw93NeK5Qs1ON15KmAQKBgQDbvCObmkCL9DmuCI2I
w4jO4Cz61QimO/76+vf8Zm7YhgTRHuM6eHfbIVvFu3mTlNPlcAXKKRFJdIqOM5uY
i/3pZF3tyr6SPL04+0Pz/j+AMD0fcPwjPeC4bAFsDccfVOWDkmhVGzDCYfk67Gd/
dzxHu0tkz739qtO0GI3e0L0VOQKBgQC8Ir7aAE8uAYP3OuHF16/tc6i6HRvfLeMz
ufqJayexj5k3i9FM4n64E+raJeUaOtH5haTVFbH/XPAAz5E1DFE+RjvwbadlZs+I
4FhdLCTcQxz5OzQcbWgTQlV4H/H7GYMjbllU0Se3/nbSZ/U5i2Tt+Qi7Q4LbE3xj
321MbOtFQQKBgAM3IpJBVJZ3sWxhhaitq8/TKfVVrrW4BQgpKf3Qhei3NVTWDd8q
Kh7TwyMGlkXZlKwz0nHd3fnkMa7ZBHrb+ZAzvsgfAmDjcKVnz9u/KbC3g/10ysu9
OQ+ZzP+GyBYmSOF+//XW2wTNKDd7hBwyY3htPjIwdAhFOqnU/iZ3iXzJAoGAclnE
Gwk5F0OAjJLTi8cPfYphMc5jlIF1qYkPCNuCouAfRq3LJ6o7T3N7ueByokDCQ6HB
kGrBZ+97SWLgZZf5AZr676YWqwGUfyOtUeR0+xQn1izv/Z9aNOqKvJreH7tgA/cc
gPsn4yPd4QGjAbkCPcqyTfA45yOu5Z1194/0aIECgYAmtQ9GTh3+rpZXAW14FxN6
CMQypo9LPoBS43IpfnE0TUqYKVrl6C7HwP6/VIyG5Lu8Tqi+c88eaSp6D5MFQODI
Oyoq2NOP9HdqVsMdo0U/I/GVjOshvGMXEffiVGFisFyTc4W2ifFF3hI3qsTyoLEg
+X8P1qP3LMI1NjK3xPIRtg==
-----END PRIVATE KEY-----
)EOF";

String htmlEscape(const String& input) {
  String out;
  out.reserve(input.length());
  for (char c : input) {
    switch (c) {
      case '&': out += F("&amp;"); break;
      case '<': out += F("&lt;"); break;
      case '>': out += F("&gt;"); break;
      case '"': out += F("&quot;"); break;
      case '\'': out += F("&#39;"); break;
      default: out += c; break;
    }
  }
  return out;
}

void saveConfig() {
  preferences.begin("glp-scale", false);
  preferences.putBytes("config", &config, sizeof(config));
  preferences.end();
}

void loadConfig() {
  preferences.begin("glp-scale", true);
  if (preferences.getBytesLength("config") == sizeof(config)) {
    preferences.getBytes("config", &config, sizeof(config));
  }
  preferences.end();
}

float readTemperatureC() {
  int adc = analogRead(Pins::THERMISTOR_ADC);
  if (adc <= 0 || adc >= Defaults::ADC_MAX) return NAN;

  float resistance = config.seriesResistor / ((static_cast<float>(Defaults::ADC_MAX) / adc) - 1.0f);
  float steinhart = resistance / config.thermistorNominal;
  steinhart = log(steinhart);
  steinhart /= config.betaCoefficient;
  steinhart += 1.0f / (config.temperatureNominal + 273.15f);
  steinhart = 1.0f / steinhart;
  return steinhart - 273.15f;
}

void updateMeasurements() {
  if (!scale.is_ready()) {
    current.valid = false;
    return;
  }

  current.grossKg = scale.get_units(10);
  current.tareKg = config.tareKg;
  current.netGlpKg = max(0.0f, current.grossKg - config.tareKg);
  current.levelPct = constrain((current.netGlpKg / max(config.fullGlpKg, 0.1f)) * 100.0f, 0.0f, 100.0f);
  current.estimatedLiters = current.netGlpKg / max(config.glpDensityKgL, 0.1f);
  current.temperatureC = readTemperatureC();
  current.valid = true;
}

String buildJson() {
  String json = "{";
  json += "\"deviceId\":\"" + String(config.deviceId) + "\",";
  json += "\"wifi\":\"" + WiFi.localIP().toString() + "\",";
  json += "\"rssi\":" + String(WiFi.RSSI()) + ",";
  json += "\"grossKg\":" + String(current.grossKg, 3) + ",";
  json += "\"tareKg\":" + String(current.tareKg, 3) + ",";
  json += "\"netGlpKg\":" + String(current.netGlpKg, 3) + ",";
  json += "\"levelPct\":" + String(current.levelPct, 2) + ",";
  json += "\"estimatedLiters\":" + String(current.estimatedLiters, 2) + ",";
  json += "\"temperatureC\":" + String(current.temperatureC, 2) + ",";
  json += "\"mqttConnected\":" + String(mqttClient.connected() ? "true" : "false") + ",";
  json += "\"freeHeap\":" + String(ESP.getFreeHeap());
  json += "}";
  return json;
}

String buildPage() {
  String page;
  page.reserve(5000);
  page += F("<!DOCTYPE html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>");
  page += F("<title>Balança GLP</title><style>body{font-family:Arial,sans-serif;margin:24px;background:#f4f7fb;color:#1e293b}"
            ".card{background:#fff;border-radius:16px;padding:20px;margin-bottom:18px;box-shadow:0 10px 25px rgba(15,23,42,.08)}"
            "label{display:block;margin-top:12px;font-weight:600}input{width:100%;padding:10px;border:1px solid #cbd5e1;border-radius:10px}"
            "button{margin-top:14px;padding:12px 18px;border:none;border-radius:10px;background:#2563eb;color:#fff;font-weight:700}"
            ".grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(220px,1fr));gap:12px}.metric{font-size:1.6rem;font-weight:700}</style></head><body>");
  page += F("<h1>Balança GLP ESP32</h1>");
  page += F("<div class='card'><h2>Status</h2><div class='grid'>");
  page += "<div><div class='metric'>" + String(current.grossKg, 2) + F(" kg</div><div>Massa total</div></div>");
  page += "<div><div class='metric'>" + String(current.netGlpKg, 2) + F(" kg</div><div>GLP líquido</div></div>");
  page += "<div><div class='metric'>" + String(current.levelPct, 1) + F(" %</div><div>Nível estimado</div></div>");
  page += "<div><div class='metric'>" + String(current.temperatureC, 1) + F(" °C</div><div>Temperatura</div></div></div></div>");
  page += F("<div class='card'><h2>Ações</h2><form method='POST' action='/tare'><button type='submit'>Gravar tara atual do recipiente</button></form>");
  page += F("<form method='POST' action='/zero'><button type='submit'>Zerar plataforma vazia</button></form>");
  page += F("<form method='POST' action='/restart'><button type='submit'>Reiniciar ESP32</button></form></div>");
  page += F("<div class='card'><h2>Parâmetros</h2><form method='POST' action='/config'>");
  page += "<label>Device ID</label><input name='deviceId' value='" + htmlEscape(String(config.deviceId)) + "'>";
  page += "<label>MQTT host</label><input name='mqttHost' value='" + htmlEscape(String(config.mqttHost)) + "'>";
  page += "<label>MQTT port</label><input name='mqttPort' value='" + String(config.mqttPort) + "'>";
  page += "<label>MQTT usuário</label><input name='mqttUser' value='" + htmlEscape(String(config.mqttUser)) + "'>";
  page += "<label>MQTT senha</label><input name='mqttPassword' type='password' value='" + htmlEscape(String(config.mqttPassword)) + "'>";
  page += "<label>Tópico MQTT</label><input name='mqttTopic' value='" + htmlEscape(String(config.mqttTopic)) + "'>";
  page += "<label>Fator de calibração HX711</label><input name='scaleFactor' value='" + String(config.scaleFactor, 3) + "'>";
  page += "<label>Tara do recipiente (kg)</label><input name='tareKg' value='" + String(config.tareKg, 3) + "'>";
  page += "<label>Capacidade útil GLP (kg)</label><input name='fullGlpKg' value='" + String(config.fullGlpKg, 3) + "'>";
  page += "<label>Densidade GLP (kg/L)</label><input name='glpDensityKgL' value='" + String(config.glpDensityKgL, 3) + "'>";
  page += "<label>Resistor série termistor (ohms)</label><input name='seriesResistor' value='" + String(config.seriesResistor, 1) + "'>";
  page += "<label>Beta termistor</label><input name='betaCoefficient' value='" + String(config.betaCoefficient, 1) + "'>";
  page += F("<button type='submit'>Salvar parâmetros</button></form></div>");
  page += F("<div class='card'><h2>API</h2><p>JSON em <code>/api/status</code> e OTA pelo Arduino IDE/PlatformIO com o hostname configurado.</p>");
  page += F("<p><strong>Atenção:</strong> o HTTPS usa certificado autoassinado embutido; troque o certificado em produção.</p></div>");
  page += F("</body></html>");
  return page;
}

void handleRoot() { httpsServer.send(200, "text/html", buildPage()); }
void handleStatusApi() { httpsServer.send(200, "application/json", buildJson()); }

void handleConfigPost() {
  auto copyArg = [](const char* name, char* dest, size_t size) {
    String value = httpsServer.arg(name);
    value.trim();
    strlcpy(dest, value.c_str(), size);
  };

  copyArg("deviceId", config.deviceId, sizeof(config.deviceId));
  copyArg("mqttHost", config.mqttHost, sizeof(config.mqttHost));
  copyArg("mqttUser", config.mqttUser, sizeof(config.mqttUser));
  copyArg("mqttPassword", config.mqttPassword, sizeof(config.mqttPassword));
  copyArg("mqttTopic", config.mqttTopic, sizeof(config.mqttTopic));

  config.mqttPort = httpsServer.arg("mqttPort").toInt();
  config.scaleFactor = httpsServer.arg("scaleFactor").toFloat();
  config.tareKg = httpsServer.arg("tareKg").toFloat();
  config.fullGlpKg = httpsServer.arg("fullGlpKg").toFloat();
  config.glpDensityKgL = httpsServer.arg("glpDensityKgL").toFloat();
  config.seriesResistor = httpsServer.arg("seriesResistor").toFloat();
  config.betaCoefficient = httpsServer.arg("betaCoefficient").toFloat();

  saveConfig();
  scale.set_scale(config.scaleFactor);
  httpsServer.sendHeader("Location", "/");
  httpsServer.send(303, "text/plain", "Salvo");
}

void handleTare() {
  updateMeasurements();
  config.tareKg = current.grossKg;
  saveConfig();
  httpsServer.sendHeader("Location", "/");
  httpsServer.send(303, "text/plain", "Tara atualizada");
}

void handleZero() {
  if (scale.wait_ready_timeout(3000)) {
    config.hxOffset = scale.read_average(20);
    scale.set_offset(config.hxOffset);
    saveConfig();
  }
  httpsServer.sendHeader("Location", "/");
  httpsServer.send(303, "text/plain", "Offset atualizado");
}

void handleRestart() {
  httpsServer.send(200, "text/plain", "Reiniciando...");
  delay(500);
  ESP.restart();
}

void configureHttpsServer() {
  httpsServer.getServer().setRSACert(new BearSSL::X509List(HTTPS_CERT), new BearSSL::PrivateKey(HTTPS_KEY));
  httpsServer.on("/", HTTP_GET, handleRoot);
  httpsServer.on("/api/status", HTTP_GET, handleStatusApi);
  httpsServer.on("/config", HTTP_POST, handleConfigPost);
  httpsServer.on("/tare", HTTP_POST, handleTare);
  httpsServer.on("/zero", HTTP_POST, handleZero);
  httpsServer.on("/restart", HTTP_POST, handleRestart);
  httpsServer.begin();
}

void setupWifi() {
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(Defaults::HOSTNAME);

  WiFiManagerParameter mqttHost("mqtt_host", "MQTT host", config.mqttHost, sizeof(config.mqttHost));
  char mqttPortBuffer[8];
  snprintf(mqttPortBuffer, sizeof(mqttPortBuffer), "%u", config.mqttPort);
  WiFiManagerParameter mqttPort("mqtt_port", "MQTT port", mqttPortBuffer, sizeof(mqttPortBuffer));
  WiFiManagerParameter mqttUser("mqtt_user", "MQTT user", config.mqttUser, sizeof(config.mqttUser));
  WiFiManagerParameter mqttPass("mqtt_pass", "MQTT password", config.mqttPassword, sizeof(config.mqttPassword));
  WiFiManagerParameter mqttTopic("mqtt_topic", "MQTT topic", config.mqttTopic, sizeof(config.mqttTopic));

  wifiManager.setConfigPortalTimeout(180);
  wifiManager.addParameter(&mqttHost);
  wifiManager.addParameter(&mqttPort);
  wifiManager.addParameter(&mqttUser);
  wifiManager.addParameter(&mqttPass);
  wifiManager.addParameter(&mqttTopic);

  if (!wifiManager.autoConnect(Defaults::AP_NAME, Defaults::AP_PASSWORD)) {
    ESP.restart();
  }

  strlcpy(config.mqttHost, mqttHost.getValue(), sizeof(config.mqttHost));
  strlcpy(config.mqttUser, mqttUser.getValue(), sizeof(config.mqttUser));
  strlcpy(config.mqttPassword, mqttPass.getValue(), sizeof(config.mqttPassword));
  strlcpy(config.mqttTopic, mqttTopic.getValue(), sizeof(config.mqttTopic));
  config.mqttPort = atoi(mqttPort.getValue());
  saveConfig();
}

void setupMqtt() {
  mqttClient.setBufferSize(1024);
  if (config.mqttTls) {
    espClientSecure.setInsecure();
    mqttClient.setClient(espClientSecure);
  } else {
    mqttClient.setClient(espClient);
  }
  mqttClient.setServer(config.mqttHost, config.mqttPort);
}

void reconnectMqtt() {
  if (mqttClient.connected() || strlen(config.mqttHost) == 0) return;
  String clientId = String(config.deviceId) + "-" + String((uint32_t)ESP.getEfuseMac(), HEX);
  mqttClient.connect(clientId.c_str(), config.mqttUser, config.mqttPassword);
}

void publishMqtt() {
  if (!mqttClient.connected()) return;
  String topic = String(config.mqttTopic) + "/" + String(config.deviceId) + "/state";
  String payload = buildJson();
  mqttClient.publish(topic.c_str(), payload.c_str(), true);
}

void setupOta() {
  ArduinoOTA.setHostname(config.deviceId);
  ArduinoOTA.onStart([]() { Serial.println("OTA iniciada"); });
  ArduinoOTA.onEnd([]() { Serial.println("OTA concluída"); });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Erro OTA %u\n", error);
  });
  ArduinoOTA.begin();
}

void setupScale() {
  scale.begin(Pins::HX711_DOUT, Pins::HX711_SCK);
  scale.set_scale(config.scaleFactor);
  if (config.hxOffset == 0 && scale.wait_ready_timeout(2000)) {
    config.hxOffset = scale.read_average(20);
    saveConfig();
  }
  scale.set_offset(config.hxOffset);
}

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
  loadConfig();
  setupWifi();
  setupOta();
  setupScale();
  setupMqtt();
  configureHttpsServer();
  updateMeasurements();
}

void loop() {
  ArduinoOTA.handle();
  httpsServer.handleClient();
  reconnectMqtt();
  mqttClient.loop();

  uint32_t now = millis();
  if (now - lastReadMs >= Defaults::READ_INTERVAL_MS) {
    lastReadMs = now;
    updateMeasurements();
  }
  if (now - lastMqttMs >= Defaults::MQTT_INTERVAL_MS) {
    lastMqttMs = now;
    publishMqtt();
  }
}
