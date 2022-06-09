#include <Adafruit_NeoPixel.h>
#include <Encoder.h>
#include <RTClib.h>
#include <Wire.h>
#include <Limits.h>

#define PIN_RotaryButton 4
#define PIN_RotaryDecrease 5
#define PIN_RotaryIncrease 6
#define PIN_PixelOut 7
#define PIXEL_Count 24
#define PIXEL_Index12 0
#define PIXEL_Index3 6
#define PIXEL_Index6 12
#define PIXEL_Index9 18
#define PIXEL_IndexOffset 12
#define EEPROM_DeviceAddress 0x57
#define EEPROM_BrightnessTimeAddress 0
#define EEPROM_BrightnessCardinalAddress 1
#define EEPROM_HueOffsetHourAddress 2
#define EEPROM_HueOffsetMinuteAddress 4
#define EEPROM_HueOffsetSecondAddress 6
#define BRIGHTNESS_Increment 1
#define BRIGHTNESS_Min 3
#define BRIGHTNESS_Max 50
#define HUE_OffsetIncrement 128
#define TIME_ButtonLongPressDuration 10000
#define TIME_IdleTimeoutDuration 30000

Adafruit_NeoPixel pixels(PIXEL_Count, PIN_PixelOut, NEO_GRB + NEO_KHZ800);

RTC_DS3231 rtc;
DateTime currentTime;
int timeHour, timeMinute, timeSecond;

Encoder encoder(PIN_RotaryDecrease, PIN_RotaryIncrease);
bool encoderButton, encoderButtonDebounce;
int brightnessTime, brightnessCardinal;
int32_t encoderPosition;
uint16_t hueOffsetHour, hueOffsetMinute, hueOffsetSecond;
unsigned long buttonPressTimeMillis;
unsigned long rotaryNonIdleTimeMillis;
enum rotaryMode { Idle, BrightnessTime, BrightnessCardinal, HueOffsetHour, HueOffsetMinute, HueOffsetSecond };
rotaryMode rotaryMode = Idle;

void setup()
{
  Serial.begin(115200);
  pixels.begin();
  Wire.begin();
  if (!rtc.begin())
  {
    Serial.println("Error: Failed to initialize DS3231");
    abort();
  }
  pinMode(PIN_RotaryButton, INPUT_PULLUP);

  brightnessTime = (int)(ReadByteFromEEPROM(EEPROM_DeviceAddress, EEPROM_BrightnessTimeAddress));
  brightnessCardinal = (int) (ReadByteFromEEPROM(EEPROM_DeviceAddress, EEPROM_BrightnessCardinalAddress));
  hueOffsetHour = ReadUIntFromEEPROM(EEPROM_DeviceAddress, EEPROM_HueOffsetHourAddress);
  hueOffsetMinute = ReadUIntFromEEPROM(EEPROM_DeviceAddress, EEPROM_HueOffsetMinuteAddress);
  hueOffsetSecond = ReadUIntFromEEPROM(EEPROM_DeviceAddress, EEPROM_HueOffsetSecondAddress);
}

void loop()
{
  UpdateTime();
  ProcessEncoder();
  UpdateNeopixel();
}

void UpdateTime()
{
  currentTime = rtc.now();
  timeHour = currentTime.twelveHour();
  timeMinute = currentTime.minute();
  timeSecond = currentTime.second();

  //char buf[] = "Time: hh:mm:ss:ms";
  //char* timeStr = currentTime.toString(buf);
  //Serial.println(timeStr);
}

void ProcessEncoder()
{
  ProcessEncoderButton();
  ProcessEncoderDial();
  UpdateIdleTime();
}

void UpdateNeopixel()
{
  uint32_t cardinalColour = pixels.Color((uint8_t)brightnessCardinal, (uint8_t)brightnessCardinal, (uint8_t)brightnessCardinal);
  uint32_t hourHue = pixels.ColorHSV(hueOffsetHour + (UINT16_MAX / 3U * 2U), 255, brightnessTime);
  uint32_t minuteHue = pixels.ColorHSV(hueOffsetMinute + (UINT16_MAX / 3U), 255, brightnessTime);
  uint32_t secondHue = pixels.ColorHSV(hueOffsetSecond, 255, brightnessTime);

  uint32_t currentColour = 0;

  pixels.clear();

  // Cardinal Points
  if (rotaryMode == Idle || rotaryMode == BrightnessCardinal)
  {
    pixels.setPixelColor(PIXEL_Index12, cardinalColour);
    pixels.setPixelColor(PIXEL_Index3, cardinalColour);
    pixels.setPixelColor(PIXEL_Index6, cardinalColour);
    pixels.setPixelColor(PIXEL_Index9, cardinalColour);
  }

  // Hour
  if (rotaryMode == Idle || rotaryMode == BrightnessTime || rotaryMode == HueOffsetHour)
  {
    int hourIndex = (int)(timeHour / 12.0f * PIXEL_Count);
    if (hourIndex == PIXEL_Count) hourIndex = 0;
    if (timeMinute >= 30) hourIndex++;
    hourIndex = (hourIndex + PIXEL_IndexOffset) % PIXEL_Count;
    currentColour = pixels.getPixelColor(hourIndex);
    pixels.setPixelColor(hourIndex, hourHue + currentColour);
  }

  // Minute
  if (rotaryMode == Idle || rotaryMode == BrightnessTime || rotaryMode == HueOffsetMinute)
  {
    int minuteIndex = (int)(timeMinute / 60.0f * PIXEL_Count);
    if (minuteIndex == PIXEL_Count) minuteIndex--;
    minuteIndex = (minuteIndex + PIXEL_IndexOffset) % PIXEL_Count;
    currentColour = pixels.getPixelColor(minuteIndex);
    pixels.setPixelColor(minuteIndex, minuteHue + currentColour);
  }

  // Second
  if (rotaryMode == Idle || rotaryMode == BrightnessTime || rotaryMode == HueOffsetSecond)
  {
    int secondIndex = (int)(timeSecond / 60.0f * PIXEL_Count) % PIXEL_Count;
    secondIndex = (secondIndex + PIXEL_IndexOffset) % PIXEL_Count;

    currentColour = pixels.getPixelColor(secondIndex);
    pixels.setPixelColor(secondIndex, secondHue + currentColour);
  }

  pixels.show();
}

void ProcessEncoderButton()
{
  encoderButton = digitalRead(PIN_RotaryButton);
  if (!encoderButton) // Button is pressed
  {
    if (!encoderButtonDebounce)
    {
      buttonPressTimeMillis = millis();
      encoderButtonDebounce = true;
    }
    else
    {
      unsigned long timeNow = millis();
      unsigned long buttonPressDuration = abs(timeNow - buttonPressTimeMillis);
      if (buttonPressDuration >= TIME_ButtonLongPressDuration)
      {
        ResetToDefaults();
      }
    }
  }
  else // Button is released
  {
    if (encoderButtonDebounce)
    {
      encoderButtonDebounce = false;

      unsigned long timeNow = millis();
      unsigned long buttonPressDuration = abs(timeNow - buttonPressTimeMillis);
      if (buttonPressDuration < TIME_ButtonLongPressDuration) // Don't increment if this was a long button press
      {
        IncrementRotaryMode();
      }
    }
  }
}

void ProcessEncoderDial()
{
  if (rotaryMode == Idle) return;

  long previousEncoderPosition = encoderPosition;
  encoderPosition = encoder.read();
  if (encoderPosition != previousEncoderPosition)
  {
    float difference = encoderPosition - previousEncoderPosition;

    switch (rotaryMode)
    {
      case BrightnessTime:
        brightnessTime += (difference > 0 ? BRIGHTNESS_Increment : -BRIGHTNESS_Increment);
        if (brightnessTime > BRIGHTNESS_Max) brightnessTime = BRIGHTNESS_Max;
        else if (brightnessTime < BRIGHTNESS_Min) brightnessTime = BRIGHTNESS_Min;
        Serial.print("BrightnessTime: ");
        Serial.println(brightnessTime);
        break;

      case BrightnessCardinal:
        brightnessCardinal += (difference > 0 ? BRIGHTNESS_Increment : -BRIGHTNESS_Increment);
        if (brightnessCardinal > BRIGHTNESS_Max) brightnessCardinal = BRIGHTNESS_Max;
        else if (brightnessCardinal < 0) brightnessCardinal = 0;
        Serial.print("BrightnessCardinal: ");
        Serial.println(brightnessCardinal);
        break;

      case HueOffsetHour:
        hueOffsetHour += (difference > 0 ? HUE_OffsetIncrement : -HUE_OffsetIncrement);
        Serial.print("HueOffsetHour: ");
        Serial.println(hueOffsetHour);
        break;

      case HueOffsetMinute:
        hueOffsetMinute += (difference > 0 ? HUE_OffsetIncrement : -HUE_OffsetIncrement);
        Serial.print("HueOffsetMinute: ");
        Serial.println(hueOffsetMinute);
        break;

      case HueOffsetSecond:
        hueOffsetSecond += (difference > 0 ? HUE_OffsetIncrement : -HUE_OffsetIncrement);
        Serial.print("HueOffsetSecond: ");
        Serial.println(hueOffsetSecond);
        break;
    }
  }
}

void UpdateIdleTime()
{
  if (rotaryMode != Idle)
  {
    unsigned long currentTimeMillis = millis();
    unsigned long nonIdleDuration = abs(currentTimeMillis - rotaryNonIdleTimeMillis);
    if (nonIdleDuration > TIME_IdleTimeoutDuration)
    {
      rotaryMode = Idle;
    }
  }
}

void IncrementRotaryMode()
{
  switch (rotaryMode)
  {
    case Idle:
      rotaryMode = BrightnessTime;
      Serial.println("Rotary Mode: BrightnessTime");
      break;

    case BrightnessTime:
      WriteByteToEEPROM(EEPROM_DeviceAddress, EEPROM_BrightnessTimeAddress, brightnessTime);
      rotaryMode = BrightnessCardinal;
      Serial.println("Rotary Mode: BrightnessCardinal");
      break;

    case BrightnessCardinal:
      WriteByteToEEPROM(EEPROM_DeviceAddress, EEPROM_BrightnessCardinalAddress, brightnessCardinal);
      rotaryMode = HueOffsetHour;
      Serial.println("Rotary Mode: HueOffsetHour");
      break;

    case HueOffsetHour:
      WriteUIntToEEPROM(EEPROM_DeviceAddress, EEPROM_HueOffsetHourAddress, hueOffsetHour);
      rotaryMode = HueOffsetMinute;
      Serial.println("Rotary Mode: HueOffsetMinute");
      break;

    case HueOffsetMinute:
      WriteUIntToEEPROM(EEPROM_DeviceAddress, EEPROM_HueOffsetMinuteAddress, hueOffsetMinute);
      rotaryMode = HueOffsetSecond;
      Serial.println("Rotary Mode: HueOffsetSecond");
      break;

    case HueOffsetSecond:
      WriteUIntToEEPROM(EEPROM_DeviceAddress, EEPROM_HueOffsetSecondAddress, hueOffsetSecond);
      rotaryMode = Idle;
      Serial.println("Rotary Mode: Idle");
      break;
  }

  if (rotaryMode != Idle)
  {
    rotaryNonIdleTimeMillis = millis();
  }
}

void WriteByteToEEPROM(int deviceAddress, unsigned int eepromAddress, byte data)
{
  byte addressByte1 = eepromAddress >> 8;
  byte addressByte2 = eepromAddress & 0xFF;
  Wire.beginTransmission(deviceAddress);
  Wire.write(addressByte1);
  Wire.write(addressByte2);
  Wire.write(data);
  Wire.endTransmission();
}

byte ReadByteFromEEPROM(int deviceAddress, unsigned int eepromAddress)
{
  byte value;
  byte addressByte1 = eepromAddress >> 8;
  byte addressByte2 = eepromAddress & 0xFF;
  Wire.beginTransmission(deviceAddress);
  Wire.write(addressByte1);
  Wire.write(addressByte2);
  Wire.endTransmission();

  Wire.requestFrom(deviceAddress, 1);
  if (Wire.available()) value = Wire.read();
  return value;
}

void WriteUIntToEEPROM(int deviceAddress, unsigned int eepromAddress, unsigned int data)
{
  byte addressByte1 = eepromAddress >> 8; // Most significant byte
  byte addressByte2 = eepromAddress & 0xFF; // Least significant byte
  byte dataByte1 = data >> 8;
  byte dataByte2 = data & 0xFF;

  Wire.beginTransmission(deviceAddress);
  Wire.write(addressByte1);
  Wire.write(addressByte2);
  Wire.write(dataByte1);
  Wire.write(dataByte2);
  Wire.endTransmission();
}

unsigned int ReadUIntFromEEPROM(int deviceAddress, unsigned int eepromAddress)
{
  unsigned int value;
  byte valueByte1, valueByte2;
  byte addressByte1 = eepromAddress >> 8;
  byte addressByte2 = eepromAddress & 0xFF;

  Wire.beginTransmission(deviceAddress);
  Wire.write(addressByte1);
  Wire.write(addressByte2);
  Wire.endTransmission();
  Wire.requestFrom(deviceAddress, 2);
  if (Wire.available()) valueByte1 = Wire.read();
  if (Wire.available()) valueByte2 = Wire.read();
  value = (valueByte1 << 8) | valueByte2;
  return value;
}

void ResetToDefaults()
{
  const byte defaultBrightnessTime = BRIGHTNESS_Min;
  const byte defaultBrightnessCardinal = 1;
  const uint16_t defaultHueOffset = 0;

  brightnessTime = defaultBrightnessTime;
  brightnessCardinal = defaultBrightnessCardinal;
  hueOffsetHour = defaultHueOffset;
  hueOffsetMinute = defaultHueOffset;
  hueOffsetSecond = defaultHueOffset;
  rotaryMode = Idle;

  WriteByteToEEPROM(EEPROM_DeviceAddress, EEPROM_BrightnessTimeAddress, brightnessTime);
  WriteByteToEEPROM(EEPROM_DeviceAddress, EEPROM_BrightnessCardinalAddress, brightnessCardinal);
  WriteUIntToEEPROM(EEPROM_DeviceAddress, EEPROM_HueOffsetHourAddress, hueOffsetHour);
  WriteUIntToEEPROM(EEPROM_DeviceAddress, EEPROM_HueOffsetMinuteAddress, hueOffsetMinute);
  WriteUIntToEEPROM(EEPROM_DeviceAddress, EEPROM_HueOffsetSecondAddress, hueOffsetSecond);
}
