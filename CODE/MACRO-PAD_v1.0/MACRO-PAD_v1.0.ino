#include "Adafruit_TinyUSB.h"
#include <Adafruit_NeoPixel.h>
#include "Adafruit_Keypad.h"
#include <RotaryEncoder.h>

// Create the rotary encoder
#define PIN_ROTA 8
#define PIN_ROTB 7
RotaryEncoder encoder(PIN_ROTA, PIN_ROTB, RotaryEncoder::LatchMode::FOUR3);
void checkPosition() {  encoder.tick(); } // just call tick() to check the state.
// our encoder position state
int encoder_pos = 0;

const byte ROWS = 4; // rows
const byte COLS = 2; // columns
//define the symbols on the buttons of the keypads
char keys[ROWS][COLS] = {
    {'1', '5'},
    {'2', '6'},
    {'3', '7'},
    {'4', '8'},
};
byte rowPins[ROWS] = {12, 13, 14, 15}; //connect to the row pinouts of the keypad
byte colPins[COLS] = {10, 11};         //connect to the column pinouts of the keypad

//initialize an instance of class NewKeypad
Adafruit_Keypad customKeypad = Adafruit_Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);


#define PIN            9
#define NUMPIXELS      7

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
  TUD_HID_REPORT_DESC_KEYBOARD( HID_REPORT_ID(RID_KEYBOARD) ),
  TUD_HID_REPORT_DESC_MOUSE   ( HID_REPORT_ID(RID_MOUSE) ),
  TUD_HID_REPORT_DESC_CONSUMER( HID_REPORT_ID(RID_CONSUMER_CONTROL) )
};

// USB HID object
Adafruit_USBD_HID usb_hid;

// the setup function runs once when you press reset or power the board
void setup()
{
  customKeypad.begin();
  usb_hid.setPollInterval(2);
  usb_hid.setReportDescriptor(desc_hid_report, sizeof(desc_hid_report));
  //usb_hid.setStringDescriptor("TinyUSB HID Composite");

  usb_hid.begin();

  Serial.begin(115200);
  Serial.println("Adafruit TinyUSB HID Composite example");

  // wait until device mounted
  while( !USBDevice.mounted() ) delay(1);

  //LED処理
  pixels.begin(); 
  for(int i=0;i<NUMPIXELS;i++){
      pixels.setPixelColor(i, pixels.Color(255,0,255));
      pixels.show();
  }

   // set rotary encoder inputs and interrupts
  pinMode(PIN_ROTA, INPUT_PULLUP);
  pinMode(PIN_ROTB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ROTA), checkPosition, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ROTB), checkPosition, CHANGE);  
}

void loop()
{
  encoder.tick();          // check the encoder
  int newPos = encoder.getPosition();
  int DIRECTION = (int)encoder.getDirection();
  if(DIRECTION == 1)
  {
    usb_hid.sendReport16(RID_CONSUMER_CONTROL, HID_USAGE_CONSUMER_VOLUME_INCREMENT);
  }
  if(DIRECTION == -1)
  {
    usb_hid.sendReport16(RID_CONSUMER_CONTROL, HID_USAGE_CONSUMER_VOLUME_DECREMENT);
  }
  
  // poll gpio once each 10 ms
  delay(10);

  // Remote wakeup
  if ( USBDevice.suspended())
  {
    // Wake up host if we are in suspend mode
    // and REMOTE_WAKEUP feature is enabled by host
    USBDevice.remoteWakeup();
  }

  /*------------- Keyboard -------------*/
  if ( usb_hid.ready() )
  {
    // use to send key release report
    static bool has_key = false;
    /*
    if ( btn_pressed )
    {
      uint8_t keycode[6] = { 0 };
      keycode[0] = HID_KEY_A;

      usb_hid.keyboardReport(RID_KEYBOARD, 0, keycode);

      has_key = true;
    }else
    {
      // send empty key report if previously has key pressed
      if (has_key) usb_hid.keyboardRelease(RID_KEYBOARD);
      has_key = false;
    }
    */

    // delay a bit before attempt to send consumer report
    delay(10);
  }

  /*------------- Consumer Control -------------*/
  if ( usb_hid.ready() )
  {
    // Consumer Control is used to control Media playback, Volume, Brightness etc ...
    // Consumer report is 2-byte containing the control code of the key
    // For list of control check out https://github.com/hathach/tinyusb/blob/master/src/class/hid/hid.h#L544

    // use to send consumer release report
    static bool has_consumer_key = false;
    /*
    if ( btn_pressed )
    {
      // send volume down (0x00EA)
      usb_hid.sendReport16(RID_CONSUMER_CONTROL, HID_USAGE_CONSUMER_VOLUME_DECREMENT);
      has_consumer_key = true;
    }else
    {
      // release the consume key by sending zero (0x0000)
      if (has_consumer_key) usb_hid.sendReport16(RID_CONSUMER_CONTROL, 0);
      has_consumer_key = false;
    }
    */
  }
  //マトリックス
  customKeypad.tick();
  while (customKeypad.available())
  {
    keypadEvent e = customKeypad.read();
    //Serial.print((char)e.bit.KEY);
    if (e.bit.EVENT == KEY_JUST_PRESSED)
    {
      Serial.println(" pressed");
      uint8_t row = e.bit.ROW;
      uint8_t col = e.bit.COL;
      Serial.print("Row: "); Serial.print(row);
      Serial.print(" col: "); Serial.print(col);
      Serial.print(" -> ");
      uint16_t keynum;
      if (row % 2 == 0) { // even row
        keynum = row * COLS + col;
      } else { // odd row the neopixels go BACKWARDS!
        keynum = row * COLS + (5 - col);
      }
      Serial.println(keynum);
      if(keynum == 0){
      
      }
       if(keynum == 7){
      
      }
       if(keynum == 4){
        usb_hid.sendReport16(RID_CONSUMER_CONTROL, HID_USAGE_CONSUMER_MUTE);
      }
      if(keynum == 11){
    
      }
      if(keynum == 1){
     
      }
      if(keynum == 6){
        
      }
      if(keynum == 5){
        
      }
      
    }
    else if (e.bit.EVENT == KEY_JUST_RELEASED)
    {
      Serial.println(" released");
      usb_hid.sendReport16(RID_CONSUMER_CONTROL, 0);
    }
  }
}
