#include <Arduino.h>
#include <arduino_freertos.h>

namespace {

void serialTask(void *) {
  uint32_t count = 0;

  while (true) {
    Serial.printf("{\"msg\":\"rtos_smoke\",\"count\":%lu,\"ms\":%lu}\r\n",
                  static_cast<unsigned long>(count++),
                  static_cast<unsigned long>(millis()));
    Serial.flush();
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

void blinkTask(void *) {
  arduino::pinMode(arduino::LED_BUILTIN, arduino::OUTPUT);
  bool on = false;

  while (true) {
    on = !on;
    arduino::digitalWriteFast(arduino::LED_BUILTIN, on ? arduino::HIGH : arduino::LOW);
    vTaskDelay(pdMS_TO_TICKS(250));
  }
}

}  // namespace

void setup() {
  Serial.begin(0);

  while (!Serial && millis() < 3000) {
  }

  Serial.printf("\r\n{\"msg\":\"rtos_boot\",\"kernel\":\"%s\"}\r\n",
                tskKERNEL_VERSION_NUMBER);
  Serial.flush();

  xTaskCreate(serialTask, "serial", 1024, nullptr, 2, nullptr);
  xTaskCreate(blinkTask, "blink", 128, nullptr, 1, nullptr);

  Serial.println("{\"msg\":\"scheduler_start\"}");
  Serial.flush();
  vTaskStartScheduler();

  Serial.println("{\"msg\":\"fatal\",\"code\":\"scheduler_failed\"}");
}

void loop() {
}
