/*
 Name:		SerialPassthroughDemo.ino
 Created:	6/4/2026 8:32:40 PM
 Author:	Frank
*/

// Generated on: 2026-06-05 16:25:00 UTC

// Simple bidirectional Serial Pass-Through
// Pi5 UART <-> Teensy 4.1 USB Serial
// With upload confirmation and directional character transfer activity

#define BAUD_RATE                 115200
#define LED_PIN                   13
#define TEENSY_TO_PI_BLINK_DELAY  25
#define PI_TO_TEENSY_BLINK_DELAY  100

// Buffers for building complete lines before labeling
char fromPiBuffer[128];
int fromPiIdx = 0;

char fromUsbBuffer[128];
int fromUsbIdx = 0;

void setup()
{
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH); // LED on during setup
  Serial.begin(BAUD_RATE); // USB to PC / Serial Monitor
  Serial1.begin(BAUD_RATE); // Hardware UART to Pi5 (pins 0/1)
  // Visible firmware upload / restart indication
  Serial.println("\n=== SerialPassthroughDemo Ready ===");
  Serial.println("Firmware uploaded successfully");
  Serial.println("USB Serial (PC) <-> Serial1 (Pi5)");
  Serial.println("Direction indicators via different blink timings");
  Serial.println("==========================================\n");
  delay(500);
  digitalWrite(LED_PIN, HIGH); // Setup complete
}

void loop()
{
  // Forward from Pi5 to USB Monitor
  if (Serial1.available())
  {
    char c = Serial1.read();
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(PI_TO_TEENSY_BLINK_DELAY);

    if (c == '\n' || fromPiIdx >= (int)sizeof(fromPiBuffer) - 1)
    {
      fromPiBuffer[fromPiIdx] = '\0';
      Serial.print("Pi5 -> ");
      Serial.print(fromPiBuffer);
      Serial.println();           // New line after label
      fromPiIdx = 0;
    }
    else
    {
      fromPiBuffer[fromPiIdx++] = c;
    }
  }

  // Forward from USB Monitor to Pi5
  if (Serial.available())
  {
    char c = Serial.read();
    Serial1.write(c);
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(TEENSY_TO_PI_BLINK_DELAY);

    if (c == '\n' || fromUsbIdx >= (int)sizeof(fromUsbBuffer) - 1)
    {
      fromUsbBuffer[fromUsbIdx] = '\0';
      Serial.print("Sent -> ");
      Serial.print(fromUsbBuffer);
      Serial.println();           // New line after label
      fromUsbIdx = 0;
    }
    else
    {
      fromUsbBuffer[fromUsbIdx++] = c;
    }
  }
}