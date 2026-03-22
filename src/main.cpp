#include <Arduino.h>
#include <ArduinoOTA.h>
#include <AsyncTCP.h>
#include <ESPAsyncDNSServer.h>
#include <ESPAsyncWebServer.h>
#include <ESPAsync_WiFiManager.h>
#include <HX711.h>
#include <Preferences.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>

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
constexpr uint16_t HTTP_PORT = 80;
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
  float temperatureC = NAN;
  bool valid = false;
};

Preferences preferences;
HX711 scale;
AsyncWebServer server(Defaults::HTTP_PORT);
AsyncDNSServer dnsServer;
ESPAsync_WiFiManager wifiManager(&server, &dnsServer, Defaults::HOSTNAME);
WiFiClient wifiClient;
WiFiClientSecure wifiClientSecure;
PubSubClient mqttClient;
DeviceConfig config;
Measurements current;
uint32_t lastReadMs = 0;
uint32_t lastMqttMs = 0;

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

String requestValue(AsyncWebServerRequest* request, const char* name) {
  if (request->hasParam(name, true)) {
    return request->getParam(name, true)->value();
  }
  if (request->hasParam(name)) {
    return request->getParam(name)->value();
  }
  return String();
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
  if (adc <= 0 || adc >= Defaults::ADC_MAX) {
    return NAN;
  }

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
  current.levelPct = constrain(
      (current.netGlpKg / max(config.fullGlpKg, 0.1f)) * 100.0f,
      0.0f,
      100.0f);
  current.estimatedLiters = current.netGlpKg / max(config.glpDensityKgL, 0.1f);
  current.temperatureC = readTemperatureC();
  current.valid = true;
}

String buildJson() {
  String json = "{";
  json += "\"deviceId\":\"" + String(config.deviceId) + "\",";
  json += "\"ip\":\"" + WiFi.localIP().toString() + "\",";
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
  page.reserve(5500);
  page += F("<!DOCTYPE html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>");
  page += F("<title>Balança GLP</title><style>body{font-family:Arial,sans-serif;margin:24px;background:#f4f7fb;color:#1e293b}"
            ".card{background:#fff;border-radius:16px;padding:20px;margin-bottom:18px;box-shadow:0 10px 25px rgba(15,23,42,.08)}"
            "label{display:block;margin-top:12px;font-weight:600}input,select{width:100%;padding:10px;border:1px solid #cbd5e1;border-radius:10px}"
            "button{margin-top:14px;padding:12px 18px;border:none;border-radius:10px;background:#2563eb;color:#fff;font-weight:700;cursor:pointer}"
            ".grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(220px,1fr));gap:12px}.metric{font-size:1.6rem;font-weight:700}"
            ".note{font-size:.95rem;color:#475569}</style></head><body>");
  page += F("<h1>Balança GLP ESP32</h1>");
  page += F("<div class='card'><h2>Status</h2><div class='grid'>");
  page += "<div><div class='metric'>" + String(current.grossKg, 2) + F(" kg</div><div>Massa total</div></div>");
  page += "<div><div class='metric'>" + String(current.netGlpKg, 2) + F(" kg</div><div>GLP líquido</div></div>");
  page += "<div><div class='metric'>" + String(current.levelPct, 1) + F(" %</div><div>Nível estimado</div></div>");
  page += "<div><div class='metric'>" + String(current.estimatedLiters, 1) + F(" L</div><div>Volume estimado</div></div>");
  page += "<div><div class='metric'>" + String(current.temperatureC, 1) + F(" °C</div><div>Temperatura</div></div>");
  page += "<div><div class='metric'>" + WiFi.localIP().toString() + F("</div><div>IP local</div></div></div></div>");

  page += F("<div class='card'><h2>Ações</h2>");
  page += F("<form method='POST' action='/tare'><button type='submit'>Gravar tara atual do recipiente</button></form>");
  page += F("<form method='POST' action='/zero'><button type='submit'>Zerar plataforma vazia</button></form>");
  page += F("<form method='POST' action='/restart'><button type='submit'>Reiniciar ESP32</button></form>");
  page += F("<p class='note'>OTA permanece disponível via ArduinoOTA após a conexão na rede local.</p></div>");

  page += F("<div class='card'><h2>Parâmetros</h2><form method='POST' action='/config'>");
  page += "<label>Device ID</label><input name='deviceId' value='" + htmlEscape(String(config.deviceId)) + "'>";
  page += "<label>MQTT host</label><input name='mqttHost' value='" + htmlEscape(String(config.mqttHost)) + "'>";
  page += "<label>MQTT port</label><input name='mqttPort' value='" + String(config.mqttPort) + "'>";
  page += "<label>MQTT usuário</label><input name='mqttUser' value='" + htmlEscape(String(config.mqttUser)) + "'>";
  page += "<label>MQTT senha</label><input name='mqttPassword' type='password' value='" + htmlEscape(String(config.mqttPassword)) + "'>";
  page += "<label>Tópico MQTT</label><input name='mqttTopic' value='" + htmlEscape(String(config.mqttTopic)) + "'>";
  page += "<label>MQTT com TLS</label><select name='mqttTls'>";
  page += String(config.mqttTls ? "<option value='1' selected>Sim</option><option value='0'>Não</option>"
                                : "<option value='1'>Sim</option><option value='0' selected>Não</option>");
  page += "</select>";
  page += "<label>Fator de calibração HX711</label><input name='scaleFactor' value='" + String(config.scaleFactor, 3) + "'>";
  page += "<label>Tara do recipiente (kg)</label><input name='tareKg' value='" + String(config.tareKg, 3) + "'>";
  page += "<label>Capacidade útil GLP (kg)</label><input name='fullGlpKg' value='" + String(config.fullGlpKg, 3) + "'>";
  page += "<label>Densidade GLP (kg/L)</label><input name='glpDensityKgL' value='" + String(config.glpDensityKgL, 3) + "'>";
  page += "<label>Resistor série termistor (ohms)</label><input name='seriesResistor' value='" + String(config.seriesResistor, 1) + "'>";
  page += "<label>Resistência nominal termistor (ohms)</label><input name='thermistorNominal' value='" + String(config.thermistorNominal, 1) + "'>";
  page += "<label>Temperatura nominal do termistor (°C)</label><input name='temperatureNominal' value='" + String(config.temperatureNominal, 1) + "'>";
  page += "<label>Beta termistor</label><input name='betaCoefficient' value='" + String(config.betaCoefficient, 1) + "'>";
  page += F("<button type='submit'>Salvar parâmetros</button></form></div>");

  page += F("<div class='card'><h2>Integração</h2>");
  page += F("<p>JSON em <code>/api/status</code> e publicação MQTT em <code>&lt;topico-base&gt;/&lt;deviceId&gt;/state</code>.</p>");
  page += F("<p class='note'>Este firmware foi migrado para <code>ESPAsyncWebServer</code>. Como essa pilha é focada em HTTP assíncrono, o acesso local está em HTTP. Se HTTPS for obrigatório, recomenda-se um proxy reverso/TLS gateway na rede.</p></div>");
  page += F("</body></html>");
  return page;
}

void redirectToRoot(AsyncWebServerRequest* request) {
  request->redirect("/");
}

void applyPostedConfig(AsyncWebServerRequest* request) {
  auto copyField = [request](const char* name, char* target, size_t size) {
    String value = requestValue(request, name);
    value.trim();
    if (value.length() > 0) {
      strlcpy(target, value.c_str(), size);
    }
  };

  copyField("deviceId", config.deviceId, sizeof(config.deviceId));
  copyField("mqttHost", config.mqttHost, sizeof(config.mqttHost));
  copyField("mqttUser", config.mqttUser, sizeof(config.mqttUser));
  copyField("mqttPassword", config.mqttPassword, sizeof(config.mqttPassword));
  copyField("mqttTopic", config.mqttTopic, sizeof(config.mqttTopic));

  config.mqttPort = max(1, requestValue(request, "mqttPort").toInt());
  config.mqttTls = requestValue(request, "mqttTls") != "0";
  config.scaleFactor = requestValue(request, "scaleFactor").toFloat();
  config.tareKg = requestValue(request, "tareKg").toFloat();
  config.fullGlpKg = max(0.1f, requestValue(request, "fullGlpKg").toFloat());
  config.glpDensityKgL = max(0.1f, requestValue(request, "glpDensityKgL").toFloat());
  config.seriesResistor = max(1.0f, requestValue(request, "seriesResistor").toFloat());
  config.thermistorNominal = max(1.0f, requestValue(request, "thermistorNominal").toFloat());
  config.temperatureNominal = requestValue(request, "temperatureNominal").toFloat();
  config.betaCoefficient = max(1.0f, requestValue(request, "betaCoefficient").toFloat());

  saveConfig();
  scale.set_scale(config.scaleFactor);
  scale.set_offset(config.hxOffset);
  mqttClient.setServer(config.mqttHost, config.mqttPort);
}

void configureServer() {
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
    updateMeasurements();
    request->send(200, "text/html; charset=utf-8", buildPage());
  });

  server.on("/api/status", HTTP_GET, [](AsyncWebServerRequest* request) {
    updateMeasurements();
    request->send(200, "application/json", buildJson());
  });

  server.on("/config", HTTP_POST, [](AsyncWebServerRequest* request) {
    applyPostedConfig(request);
    redirectToRoot(request);
  });

  server.on("/tare", HTTP_POST, [](AsyncWebServerRequest* request) {
    updateMeasurements();
    if (current.valid) {
      config.tareKg = current.grossKg;
      saveConfig();
    }
    redirectToRoot(request);
  });

  server.on("/zero", HTTP_POST, [](AsyncWebServerRequest* request) {
    if (scale.wait_ready_timeout(3000)) {
      config.hxOffset = scale.read_average(20);
      scale.set_offset(config.hxOffset);
      saveConfig();
    }
    redirectToRoot(request);
  });

  server.on("/restart", HTTP_POST, [](AsyncWebServerRequest* request) {
    request->send(200, "text/plain", "Reiniciando...");
    delay(250);
    ESP.restart();
  });

  server.onNotFound([](AsyncWebServerRequest* request) {
    request->send(404, "application/json", "{\"error\":\"not_found\"}");
  });

  DefaultHeaders::Instance().addHeader("Cache-Control", "no-store");
  server.begin();
}

void setupWifi() {
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(Defaults::HOSTNAME);
  wifiManager.setConfigPortalTimeout(180);
  if (!wifiManager.autoConnect(Defaults::AP_NAME, Defaults::AP_PASSWORD)) {
    ESP.restart();
  }
}

void setupMqtt() {
  mqttClient.setBufferSize(1024);
  if (config.mqttTls) {
    wifiClientSecure.setInsecure();
    mqttClient.setClient(wifiClientSecure);
  } else {
    mqttClient.setClient(wifiClient);
  }
  mqttClient.setServer(config.mqttHost, config.mqttPort);
}

void reconnectMqtt() {
  if (mqttClient.connected() || strlen(config.mqttHost) == 0) {
    return;
  }

  if (config.mqttTls) {
    wifiClientSecure.setInsecure();
  }

  String clientId = String(config.deviceId) + "-" + String(static_cast<uint32_t>(ESP.getEfuseMac()), HEX);
  mqttClient.connect(clientId.c_str(), config.mqttUser, config.mqttPassword);
}

void publishMqtt() {
  if (!mqttClient.connected() || !current.valid) {
    return;
  }

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
  setupScale();
  setupWifi();
  configureServer();
  setupMqtt();
  setupOta();
  updateMeasurements();
}

void loop() {
  ArduinoOTA.handle();
  reconnectMqtt();
  mqttClient.loop();

  const uint32_t now = millis();
  if (now - lastReadMs >= Defaults::READ_INTERVAL_MS) {
    lastReadMs = now;
    updateMeasurements();
  }
  if (now - lastMqttMs >= Defaults::MQTT_INTERVAL_MS) {
    lastMqttMs = now;
    publishMqtt();
  }
}
