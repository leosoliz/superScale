#include <Arduino.h>
#include <esp_system.h>

const char* resetReasonName(esp_reset_reason_t reason) {
  switch (reason) {
    case ESP_RST_UNKNOWN: return "unknown";
    case ESP_RST_POWERON: return "power_on";
    case ESP_RST_EXT: return "external_pin";
    case ESP_RST_SW: return "software";
    case ESP_RST_PANIC: return "panic";
    case ESP_RST_INT_WDT: return "interrupt_wdt";
    case ESP_RST_TASK_WDT: return "task_wdt";
    case ESP_RST_WDT: return "other_wdt";
    case ESP_RST_DEEPSLEEP: return "deep_sleep";
    case ESP_RST_BROWNOUT: return "brownout";
    case ESP_RST_SDIO: return "sdio";
    default: return "unmapped";
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println();
  Serial.println("diag firmware boot");
  const esp_reset_reason_t reason = esp_reset_reason();
  Serial.printf("reset reason: %s (%d)\n", resetReasonName(reason), static_cast<int>(reason));
  Serial.printf("free heap: %u\n", ESP.getFreeHeap());
}

void loop() {
  static uint32_t lastHeartbeatMs = 0;
  const uint32_t now = millis();
  if (now - lastHeartbeatMs >= 1000) {
    lastHeartbeatMs = now;
    Serial.printf("diag heartbeat uptime=%lu heap=%u\n", now, ESP.getFreeHeap());
  }
  delay(10);
}
