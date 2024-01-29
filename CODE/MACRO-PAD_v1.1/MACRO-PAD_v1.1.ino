#include "Adafruit_TinyUSB.h"
#include <Adafruit_NeoPixel.h>
#include <RotaryEncoder.h>
//LED輝度
#define BRIGHTNESS 255
// Create the rotary encoder
#define PIN_ROTA 8
#define PIN_ROTB 7
RotaryEncoder encoder(PIN_ROTA, PIN_ROTB, RotaryEncoder::LatchMode::FOUR3);
void checkPosition()
{
  encoder.tick(); // just call tick() to check the state.
}
// our encoder position state
int encoder_pos = 0;
const byte ROWS = 2; // rows
const byte COLS = 4; // columns
//define the symbols on the buttons of the keypads
byte colPins[COLS] = {12, 13, 14, 15}; //OUTPUT
byte rowPins[ROWS] = {10, 11};         //INPUT
//initialize an instance of class NewKeypad
bool currentStates[ROWS][COLS];
bool beforeStates[ROWS][COLS];
uint8_t keycode[6] = {0};
uint8_t sw1 = 0;
uint8_t sw2 = 0;
uint8_t sw3 = 0;
uint8_t sw4 = 0;
uint8_t sw5 = 0;
uint8_t sw6 = 0;
uint8_t sw7 = 0;
uint8_t sw8 = 0;
uint8_t sw_state = 0;
uint8_t keynum = 0;
#define PIN 9
#define NUMPIXELS 7
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

#if defined ARDUINO_SAMD_CIRCUITPLAYGROUND_EXPRESS
bool activeState = true;
#elif defined ARDUINO_NRF52840_FEATHER
bool activeState = false;
#else
bool activeState = false;
#endif
// Report ID
enum
{
  RID_KEYBOARD = 1,
  RID_MOUSE,
  RID_CONSUMER_CONTROL, // Media, volume etc ..
};
// HID report descriptor using TinyUSB's template
uint8_t const desc_hid_report[] =
{
  TUD_HID_REPORT_DESC_KEYBOARD(HID_REPORT_ID(RID_KEYBOARD)),
  TUD_HID_REPORT_DESC_MOUSE(HID_REPORT_ID(RID_MOUSE)),
  TUD_HID_REPORT_DESC_CONSUMER(HID_REPORT_ID(RID_CONSUMER_CONTROL))
};
// USB HID object
Adafruit_USBD_HID usb_hid;
// the setup function runs once when you press reset or power the board
void setup()
{
  for (int i = 0; i < COLS; i++)
  {
    pinMode(colPins[i], INPUT_PULLUP);
  }
  for (int i = 0; i < ROWS; i++)
  {
    pinMode(rowPins[i], OUTPUT);
  }
  for (int row = 0; row < ROWS; row++)
  {
    for (int c = 0; c < COLS; c++)
    {
      currentStates[row][c] = HIGH;
      beforeStates[row][c] = HIGH;
    }
    digitalWrite(rowPins[row], HIGH);
  }
  usb_hid.setPollInterval(2);
  usb_hid.setReportDescriptor(desc_hid_report, sizeof(desc_hid_report));
  //usb_hid.setStringDescriptor("TinyUSB HID Composite");
  usb_hid.begin();
  Serial.begin(115200);
  // wait until device mounted
  while (!USBDevice.mounted())
    delay(1);
  //LED処理
  pixels.begin();
  rainbow(2);
  gaming(BRIGHTNESS);

  // set rotary encoder inputs and interrupts
  pinMode(PIN_ROTA, INPUT_PULLUP);
  pinMode(PIN_ROTB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ROTA), checkPosition, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ROTB), checkPosition, CHANGE);
}

void loop()
{
  encoder.tick(); // check the encoder
  int newPos = encoder.getPosition();
  int DIRECTION = (int)encoder.getDirection();
  if (DIRECTION == 1)
  {
    usb_hid.sendReport16(RID_CONSUMER_CONTROL, HID_USAGE_CONSUMER_VOLUME_INCREMENT);
  }
  if (DIRECTION == -1)
  {
    usb_hid.sendReport16(RID_CONSUMER_CONTROL, HID_USAGE_CONSUMER_VOLUME_DECREMENT);
  }

  // poll gpio once each 10 ms
  delay(10);

  // Remote wakeup
  if (USBDevice.suspended())
  {
    // Wake up host if we are in suspend mode
    // and REMOTE_WAKEUP feature is enabled by host
    USBDevice.remoteWakeup();
  }

  /*------------- Keyboard -------------*/
  if (usb_hid.ready())
  {
    for (int row = 0; row < ROWS; row++)
    {
      digitalWrite(rowPins[row], LOW);
      for (int col = 0; col < COLS; col++)
      {
        currentStates[row][col] = digitalRead(colPins[col]);

        if (currentStates[row][col] != beforeStates[row][col])
        {
          if (row == 0)
          {
            sw1 = !digitalRead(colPins[0]);
            sw2 = !digitalRead(colPins[1]);
            sw3 = !digitalRead(colPins[2]);
            sw4 = !digitalRead(colPins[3]);
          }
          if (row == 1)
          {
            sw5 = !digitalRead(colPins[0]);
            sw6 = !digitalRead(colPins[1]);
            sw7 = !digitalRead(colPins[2]);
            sw8 = !digitalRead(colPins[3]);
          }
          sw_state = ((sw8 << 7) + (sw1 << 6) + (sw2 << 5) + (sw3 << 4) + (sw4 << 3) + (sw5 << 2) + (sw6 << 1) + (sw7 << 0));
          if (currentStates[row][col] == LOW)
          {
            //配列初期化
            for (int i = 0; i < 7; i++)
            {
              keycode[i] = 0;
            }
            keynum = 0;
            Serial.println(sw_state);

            if (sw_state & 0b10000000)
            {
              //ミュート
              usb_hid.sendReport16(RID_CONSUMER_CONTROL, HID_USAGE_CONSUMER_MUTE);
            }
            if (sw_state & 0b01000000)
            {
              //Serial.println("sw1");
              keynum = 0;
              keycode[2] = HID_KEY_1;
              normal_led(BRIGHTNESS);
            }
            if (sw_state & 0b00100000)
            {
              //Serial.println("sw2");
              keynum = 1;
              keycode[3] = HID_KEY_2;
              normal_led(BRIGHTNESS);
            }
            if (sw_state & 0b00010000)
            {
              //Serial.println("sw3");
              keynum = 2;
            
              normal_led(BRIGHTNESS);
            }
            if (sw_state & 0b00001000)
            {
              //Serial.println("sw4");
              keynum = 3;
              keycode[0] = HID_KEY_CONTROL_LEFT;
              keycode[1] = HID_KEY_C;
              normal_led(BRIGHTNESS);
            }
            if (sw_state & 0b00000100)
            {
              //Serial.println("sw5");
              keynum = 4;
              keycode[0] = HID_KEY_CONTROL_LEFT;
              keycode[1] = HID_KEY_V;
              normal_led(BRIGHTNESS);
            }
            if (sw_state & 0b00000010)
            {
              //Serial.println("sw6");
              keynum = 5;
              keycode[0] = HID_KEY_CONTROL_LEFT;
              keycode[1] = HID_KEY_Z;
              normal_led(BRIGHTNESS);
            }
            if (sw_state & 0b00000001)
            {
              //Serial.println("sw7");
              keynum = 6;
              keycode[0] = HID_KEY_CONTROL_LEFT;
              keycode[1] = HID_KEY_F;
              normal_led(BRIGHTNESS);
            }

            Serial.println(" Push");
            Serial.println(sw_state, BIN);
            for (int i = 0; i < 7; i++)
            {
              Serial.println(keycode[i]);
            }
            //キーコード送信
            usb_hid.keyboardReport(RID_KEYBOARD, 0, keycode);
          }
          else
          {
            Serial.println(" Release");
            usb_hid.keyboardRelease(RID_KEYBOARD);
            usb_hid.sendReport16(RID_CONSUMER_CONTROL, 0);
            gaming(BRIGHTNESS);
          }

          beforeStates[row][col] = currentStates[row][col];
        }
      }
      digitalWrite(rowPins[row], HIGH);
    }
  }
}

void normal_led(uint8_t brightness)
{
  for (int i = 0; i < NUMPIXELS; i++)
  {
    pixels.setPixelColor(i, pixels.ColorHSV(65535 / NUMPIXELS * keynum, 255, brightness));
  }
  pixels.show();
}

void gaming(uint8_t brightness)
{
  for (int i = 0; i < NUMPIXELS; i++)
  {
    pixels.setPixelColor(i, pixels.ColorHSV(65535 / NUMPIXELS * i, 255, brightness));
  }
  pixels.show();
}

void rainbow(uint8_t wait)
{
  uint16_t i, j;

  for (j = 0; j < 256; j++)
  {
    for (i = 0; i < pixels.numPixels(); i++)
    {
      pixels.setPixelColor(i, Wheel((i + j) & 255));
    }
    pixels.show();
    delay(wait);
  }
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos)
{
  WheelPos = 255 - WheelPos;
  if (WheelPos < 85)
  {
    return pixels.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  if (WheelPos < 170)
  {
    WheelPos -= 85;
    return pixels.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return pixels.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}
