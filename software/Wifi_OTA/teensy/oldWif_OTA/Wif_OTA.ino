// Generated on: 2026-06-17 14:30:00 UTC
// Wifi_OTA.ino - UART OTA Test via Pi5 Serial1

#include "FXUtil.h"
extern "C"
{
#include "FlashTxx.h"
}

//#define USING_HC05

//needed for compile-time switching
char c;
uint32_t buffer_addr, buffer_size;

#define BLINK_TIME_MSEC 200

void setup()
{
  Serial.begin(115200);      // USB debug port
  delay(2000);
  Serial.println("\nTo USB: === Wifi_OTA - Teensy Ready ===");
  Serial.println("To USB: Send 'U' from TeraTerm on Serial2 or Pi5 on Serial1 to trigger OTA update");

#ifdef USING_HC05
  Serial2.begin(115200);     // UART to HC05
  delay(2000);
  Serial2.println("\nTo TeraTerm: === Wifi_OTA - Teensy Ready ===");
  Serial2.println("To TeraTerm: Send 'U' from TeraTerm on Serial2 to trigger OTA update");
#else
  Serial1.begin(115200);     // UART to Pi5
  Serial1.println("\nTo Pi5: === Wifi_OTA - Teensy Ready ===");
  Serial1.println("To Pi5: Send 'U' from Pi5 on Serial1 to trigger OTA update");
  Serial1.flush();

#endif

  //07/02/26 revised for more definitive successful update indication
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(BLINK_TIME_MSEC);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  delay(BLINK_TIME_MSEC);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(BLINK_TIME_MSEC);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  delay(BLINK_TIME_MSEC);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(BLINK_TIME_MSEC);
}

void loop()
{
#ifdef USING_HC05

  if (Serial2.available())
  {
    c = Serial2.read();
    Serial2.printf("Received %c on Serial2: '\n", c);

    if (c == 'U' || c == 'u')
    {
      Serial2.println("\nStart Program Update - Send new HEX file!");
      digitalWrite(LED_BUILTIN, LOW);   // visual feedback
      delay(500);

      //uint32_t buffer_addr, buffer_size;
      if (firmware_buffer_init(&buffer_addr, &buffer_size) == 0)
      {
        Serial2.println("Failed to init buffer");
        digitalWrite(LED_BUILTIN, HIGH);
        return;
      }

      Serial2.println("Calling update_firmware() on Serial2...");
      while (Serial2.available()) Serial2.read(); // clear buffer
      update_firmware(&Serial2, &Serial2, buffer_addr, buffer_size);
    }
#else
  if (Serial1.available())
  {
    c = Serial1.read();
    Serial1.printf("Received %c on Serial1: '\n", c);
    Serial1.print(c);
  }
  if (c == 'U' || c == 'u')
  {
    Serial.println("\nStart Program Update - Send new HEX file!");
    digitalWrite(LED_BUILTIN, LOW);   // visual feedback
    delay(500);

    //uint32_t buffer_addr, buffer_size;
    if (firmware_buffer_init(&buffer_addr, &buffer_size) == 0)
    {
      Serial.println("Failed to init buffer");
      digitalWrite(LED_BUILTIN, HIGH);
      return;
    }

    while (Serial1.available()) Serial1.read(); // clear buffer

    Serial.println("Calling update_firmware() on Serial1...");
    update_firmware(&Serial1, &Serial1, buffer_addr, buffer_size);
#endif

    //firmware_buffer_free(buffer_addr, buffer_size);
#ifdef USING_HC05
    Serial2.println("after update_firmware");
    firmware_buffer_free(buffer_addr, buffer_size);
    Serial2.println("Firmware update call completed. About to REBOOT...");
#else
    Serial.println("update_firmware() returned - freeing buffer");
    firmware_buffer_free(buffer_addr, buffer_size);
    Serial.println("Firmware update call completed. About to REBOOT...");
#endif
    delay(2000);  // Give time for the message to be sent

    REBOOT;
  }
  }
