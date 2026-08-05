/////////////////////////////////////////////////
//         ATCK for Yaesu FTDX101MP/D          //
//        For instructions and updates         //
//           check the github page             //
// https://github.com/vlachosjm/ATCK-for-Yaesu //
/////////////////////////////////////////////////

//The external automatic tuner needs to be connected to the ANT1 output of the tranceiver
//The transceiver's '232 Rate' should be set to '38400bps'
//The transceiver's 'TUNER SELECT' should be set to 'INT'

#include <Preferences.h>      //The library needed to store and retrieve data from the on-board non-volatile memory
#include <ESP32Time.h>        //To use the internal RTC
#include <Adafruit_seesaw.h>  //The library needed for the Adafruit I2C rotary encoders
#include <mLink.h>            //The library needed for the I2C relays from Hobby Components Ltd
#include <TFT_eSPI.h>         //To handle the TFT screen

#define BAUD_RATE 38400  //Sets the speed of the communication with the transceiver
#define RXPIN 4          //RX pin for the serial communication with the transceiver
#define TXPIN 5          //TX pin for the serial communication with the transceiver
#define Button 1         //Tuner button.
#define TXGND 7          //Connected to the TX-GND jack of the tranceiver. It's low when the transceiver transmits

#define CommandDelay 8  //Number of milliseconds to wait after we send a coomand to the transceiver. Best option found is 12 ms

#define SS_SWITCH 24         //The pin that controls the rotary encoder on the I2C board
#define SEESAW_ADDR1 0x36    //The I2C address of the first encoder - Default address
#define SEESAW_ADDR2 0x37    //The I2C address of the second encoder - A0 jumber pads on the rotary encoder should be bridged
#define SEESAW_ADDR3 0x38    //The I2C address of the third encoder - A1 jumber pads on the rotary encoder should be bridged
#define SEESAW_ADDR4 0x39    //The I2C address of the forth encoder - Both A0 and A1 jumbers pads on the rotary encoder should be bridged
#define RotatorDirection -1  //Use 1 for Adafruit 4991, or -1 for Adafruit 5880 I2C rotary encoders

#define I2C_REL_ADD 0x52  // Default I2C address for the relay module

#define RIG_Model 'FTDX101MP'  // Can be FTDX101D or FTDX101MP

//Here we define the TFT pins. It is highly recommented not to change these pins, otherwise you risk the SPI interface (and thus the TFT display) to be very slow
#define TFT_CS 10
#define TFT_RST 13
#define TFT_DC 14
#define TFT_MOSI 11  // Data out
#define TFT_SCLK 12  // Clock out

#define TFT_TEAL 0x0410   // Define the Teal color for the TFT display
#define TFT_EBONY 0x52EA  // Define the Ebony color for the TFT display

TaskHandle_t SecondCoreTask;  //A task handle to the function that run on core 0

Preferences preferences;  //Define the oncject to hadndle the read/write to the on-board non-volatile memory

Adafruit_seesaw RE1;  //Define the object of the 1st rotary encoder
Adafruit_seesaw RE2;  //Define the object of the 2nd rotary encoder
Adafruit_seesaw RE3;  //Define the object of the 3rd rotary encoder
Adafruit_seesaw RE4;  //Define the object of the 4th rotary encoder

mLink Relay;  //Define the object for the relay

ESP32Time rtc;  // internal RTC wrapper

TFT_eSPI tft = TFT_eSPI();              //Defive the object to handle the TFT display
TFT_eSprite Upper = TFT_eSprite(&tft);  //Define a sprite for the upper part of the display
TFT_eSprite Lower = TFT_eSprite(&tft);  //Define a sprite for the lower part of the display
TFT_eSprite ULC = TFT_eSprite(&tft);    //Define a sprite for the upper left part of the display
TFT_eSprite URC = TFT_eSprite(&tft);    //Define a sprite for the upper right part of the display
TFT_eSprite LLC = TFT_eSprite(&tft);    //Define a sprite for the lower left part of the display
TFT_eSprite LRC = TFT_eSprite(&tft);    //Define a sprite for the lower right part of the display

String Build = "260805";

bool Tuned = false;  // True if in a specific range around the last tuned frequency, false if otherwise

unsigned long InfoDelay;  // The application should wait until that time to show another info on the upper screen (in milliseconds from the start of the initilization of the ATCK device)
int Message = 0;          // Each info message has a number. We keep here the number of the message that shows on the display

long CurrentFrequencyTX = 0;   //The Current frequency of the VFO that has the TX
long CurrentFrequencyRX = 0;   //The Current frequency of the VFO that has the RX
long LastTunedFrequency = 0;   //The Last Tuned Frequency for the external tuner.
long LastFailedFrequency = 0;  //The Last Failed Frequency for the external tuner.

int SWR;              //The last SWR reading
int Steps;            //The steping for changing the frequency using the rotaring the encoder
int MAINSUBTX = 0;    //Main (0) or sub (1) for transmit
int MAINSUBRX = 0;    //Main (0) or sub (1) for receive
int SelectedFilter;   //The selected filter. Default = 0 (None)
int TransmitAntenna;  //Number of the antenna used for transmition
int VOXStatus;        //The status of VOX

int PreviousPower = -1;       //The RF power level
int PreviousCompressor = -1;  //The status of the compressor (1=on, 0=off)
int PreviousEQ = -1;          //The EQ status
int PreviousKeyer = -1;       //The keyer status (1=on, 0=off)

int32_t encoder_position1;  //The value of the first encoder
int32_t encoder_position2;  //The value of the second encoder
int32_t encoder_position3;  //The value of the third encoder
int32_t encoder_position4;  //The value of the fourth encoder


uint16_t TFT_BACKGROUND = TFT_EBONY;   //This variable holds the backgournd color of the disaplay
uint16_t TFT_FOREGROUND = TFT_YELLOW;  //This variable holds the foreground color of the disaplay

unsigned long ONAirStartTime;  //Marks the time that on air activity started
int ONAirTime;                 //Actual time on air in seconds;
int MaxAirTime;                //Maximum air time in second. When we reach 80% of the end of time the timer on the display turns red

int SecondRelayActivationTime = 0;  //The second relay activation in seconds relative to the tuner activation. Negative values shows that the second relay activates before the tuner activation. Positive value means after the tuner activation. A value of zero means activates with the tuner activation.
int SecondRelayDelay = 0;           //The time in seconds that the second relay stays activated after it gets activated. Only possitive values f course.
unsigned long SecondRelayActivationMillis = 0;
unsigned long SecondRelayDeactivationMillis = 0;
bool RLY1 = false;  //To follow the status of RLY1

int PPO;  //Peak Power Out

volatile unsigned long start;  //This value is used when we are in a menu, to mark the the start of inactivity time before we auto-exit the menu

// Shows the parameter that can be modified with the specific encoder: 1 - Squelch, 2 - Memory channel, 3 - Notch filter width, 4 - Contour Width, 5 - Contour Level, 6- Power Level, 7- Frequency, 8- Frequency Main, 9- Frequency Sub
int ExtendedParameter1;
int ExtendedParameter2;
int ExtendedParameter3;
int ExtendedParameter4;

//Conflicting Frequencies
//The second parameter is the modulation type: 1=LSB, 2=USB, 8=Data-L, C=Data-U, etc
long ConflictFr[33][2] = {
  { 7195000, 1 },
  { 7197000, 1 },
  { 14280000, 2 },
  { 1840000, 2 },
  { 3573000, 2 },
  { 5357000, 2 },
  { 7074000, 2 },
  { 10136000, 2 },
  { 14074000, 2 },
  { 18100000, 2 },
  { 21074000, 2 },
  { 24915000, 2 },
  { 28074000, 2 },
  { 50313000, 2 },
  { 14230000, 2 },
  { 3690000, 1 },
  { 7090000, 1 },
  { 14285000, 2 },
  { 18130000, 2 },
  { 21285000, 2 },
  { 24950000, 2 },
  { 28360000, 2 },
  { 1995000, 2 },
  { 3595000, 2 },
  { 5355000, 2 },
  { 7105000, 2 },
  { 10133000, 2 },
  { 14105000, 2 },
  { 18107000, 2 },
  { 21105000, 2 },
  { 24927000, 2 },
  { 28105000, 2 },
  { 50330000, 2 }

};

String ConflictText[33] = { "Greek Net", "Greek Net", "Greek Net", "FT8", "FT8", "FT8", "FT8", "FT8", "FT8", "FT8", "FT8", "FT8", "FT8", "FT8", "SSTV", "SSB QRP", "SSB QRP", "SSB QRP", "SSB QRP", "SSB QRP", "SSB QRP", "SSB QRP", "VarAC", "VarAC", "VarAC", "VarAC", "VarAC", "VarAC", "VarAC", "VarAC", "VarAC", "VarAC", "VarAC" };

bool ButtonShortPress = false;  // Checks in the tune button has been pressed for a long time (>1 sec?)
bool ButtonLongPress = false;
unsigned long ButtonPressTime;  //The point in time where the button is pressed


void IRAM_ATTR Interupt1() {  // The IRAM_ATTR parameter is used to store the function in the RAM and not int the flash memory, for faster execution. Also due to a compiler limitation, the functions needs to be declared before setup() function
  if (digitalRead(TXGND) == 0) {
    start = 0;  // Exit from possible menus by setting to 0 all user input wait time. Give priority to handle the transmition.
  }
}

void setup() {
  //seting the pin mode of the pins that we use
  pinMode(RXPIN, INPUT);
  pinMode(TXPIN, OUTPUT);
  pinMode(Button, INPUT_PULLUP);
  pinMode(TXGND, INPUT_PULLUP);

  //Read the stored preferences
  preferences.begin("ATCK", false);
  ExtendedParameter1 = preferences.getInt("Parameter1", 0);
  ExtendedParameter2 = preferences.getInt("Parameter2", 0);
  ExtendedParameter3 = preferences.getInt("Parameter3", 0);
  ExtendedParameter4 = preferences.getInt("Parameter4", 0);
  Steps = preferences.getInt("Steps", 0);
  SelectedFilter = preferences.getInt("Filter", 0);
  MaxAirTime = preferences.getInt("MaxAirTime", 0);
  SecondRelayActivationTime = preferences.getInt("SRAT", 0);
  SecondRelayDelay = preferences.getInt("SRD", 0);
  preferences.end();

  Serial.begin(115200);                                //Initiate the serial monitor port
  Serial2.begin(BAUD_RATE, SERIAL_8N1, RXPIN, TXPIN);  //Initiate Serial port
  Wire.begin(8, 9);                                    //Initiate the I2C
  Relay.init();                                        //Initalize the relay module
  RE1.begin(SEESAW_ADDR1);                             //Initiate the 1st rotary encoder
  RE2.begin(SEESAW_ADDR2);                             //Initiate the 2nd rotary encoder
  RE3.begin(SEESAW_ADDR3);                             //Initiate the 3rd rotary encoder
  RE4.begin(SEESAW_ADDR4);                             //Initiate the 4th rotary encoder

  tft.init();                    //Initiate the tft display
  Upper.createSprite(320, 112);  //Create the sprite for the upper part of the display
  Lower.createSprite(280, 29);   //Create the sprite for the lower part of the display
  ULC.createSprite(150, 30);     //Create the sprite for the upper left corner of the display
  LLC.createSprite(150, 30);     //Create the sprite for the lower left corner of the display
  URC.createSprite(150, 30);     //Create the sprite for the upper right corner of the display
  LRC.createSprite(150, 30);     //Create the sprite for the lower right corner of the display

  tft.setRotation(1);
  tft.setTextWrap(false);
  tft.fillScreen(TFT_BLACK);

  //Splash screen
  tft.fillRect(0, 16, 320, 194, TFT_YELLOW);
  tft.setFreeFont(&FreeSansBold18pt7b);
  TFT_FOREGROUND = TFT_BLACK;
  TFT_BACKGROUND = TFT_YELLOW;
  PrintTextCentered(0, 320, 58, "ATCK");
  PrintTextCentered(0, 320, 98, "for");
  if (RIG_Model == 'FTDX101MP') {
    PrintTextCentered(0, 320, 130, "Yaesu FTDX101MP");
  } else if (RIG_Model == 'FTDX101D') {
    PrintTextCentered(0, 320, 138, "Yaesu FTDX101D");
  } else {
    PrintTextCentered(0, 320, 138, "unknown model");
  }
  tft.setFreeFont(&FreeSansBold12pt7b);
  PrintTextCentered(0, 320, 178, "---by SV1RQJ/F4VTR---");
  PrintTextCentered(0, 320, 205, "Build " + Build);
  delay(1000);
  tft.fillRect(0, 0, 320, 240, TFT_BLACK);
  tft.fillRect(0, 8, 320, 172, TFT_EBONY);

  tft.setTextColor(TFT_YELLOW);
  TFT_BACKGROUND = TFT_EBONY;

  //Set the local RTC from the radio
  String Result, Result2;
  Result = ReadTime();
  Result2 = ReadDate();
  rtc.setTime(30, Result.substring(2, 4).toInt(), Result.substring(0, 2).toInt(), Result2.substring(6, 8).toInt(), Result2.substring(4, 6).toInt(), Result2.substring(0, 4).toInt());

  attachInterrupt(digitalPinToInterrupt(TXGND), Interupt1, CHANGE);  //Setup the interrupt routine to call and the condition to call it

  xTaskCreatePinnedToCore(  //Task for second core
    SecondCoreTaskCode,     /* Task function. */
    "SecondCoreTask",       /* name of task. */
    10000,                  /* Stack size of task */
    NULL,                   /* parameter of the task */
    1,                      /* priority of the task */
    &SecondCoreTask,        /* Task handle to keep track of created task */
    0);                     /* pin task to core 0 */
}

void loop() {
  char a;                  //Generic character string
  String Result, Result2;  //Generic results storage
  String LockStatus;       //Lock status
  String TimeText;         //To hold the on air time string
  int MPO;                 //Max Power Out

  //If an encoder has moved...
  if (encoder_position1 != RotatorDirection * RE1.getEncoderPosition() || encoder_position2 != RotatorDirection * RE2.getEncoderPosition() || encoder_position3 != RotatorDirection * RE3.getEncoderPosition() || encoder_position4 != RotatorDirection * RE4.getEncoderPosition()) {
    ReadEncoder();
  }


  //Check if an encoder's button is pressed and released
  if (!RE1.digitalRead(SS_SWITCH)) {
    while (!RE1.digitalRead(SS_SWITCH)) {}
    MenuHandle_1stEncoder();
  }

  if (!RE2.digitalRead(SS_SWITCH)) {
    while (!RE2.digitalRead(SS_SWITCH)) {}
    MenuHandle_2ndEncoder();
  }

  if (!RE3.digitalRead(SS_SWITCH)) {
    while (!RE3.digitalRead(SS_SWITCH)) {}
    MenuHandle_3rdEncoder();
  }

  if (!RE4.digitalRead(SS_SWITCH)) {
    while (!RE4.digitalRead(SS_SWITCH)) {}
    MenuHandle_4thEncoder();
  }

  if (Message != 4) {  //In case we have a Message 4 (No Communication) don't display the 4 corners
    DisplayURC();
    DisplayLRC();
    DisplayULC();
    DisplayLLC();
  }

  // Here we check if the transmit is on Main or Sub
  MAINSUBTX = ReadTX();

  //Check Which antenna is used for transmit
  TransmitAntenna = ReadAntenna(MAINSUBTX);

  // The rest of the code runs if we use the antenna having the external tuner
  if (TransmitAntenna == 1) {
    //Read current Frequency and check if it is still in tuned range
    CurrentFrequencyTX = ReadFrequency(MAINSUBTX);     //Check the current frequency of the VFO that has the transmision
    Tuned = IsInTunedFrequencies(CurrentFrequencyTX);  //Check if we are still in tuned range

    if (ButtonLongPress) {
      SystemMenu();
      ButtonShortPress = false;
      ButtonLongPress = false;
    }

    //If the transmniter transmits in a non-tuned frequency (or the tune button is pressed) while the transmit is in the allowed bands...
    if (((digitalRead(TXGND) == 0 && !Tuned) || ButtonShortPress)
        && ((CurrentFrequencyTX >= 1800000 && CurrentFrequencyTX < 2000000)
            || (CurrentFrequencyTX >= 3500000 && CurrentFrequencyTX < 3800000)
            || (CurrentFrequencyTX >= 5351500 && CurrentFrequencyTX < 5366500)
            || (CurrentFrequencyTX >= 7000000 && CurrentFrequencyTX < 7200000)
            || (CurrentFrequencyTX >= 10100000 && CurrentFrequencyTX < 10150000)
            || (CurrentFrequencyTX >= 14000000 && CurrentFrequencyTX < 14350000)
            || (CurrentFrequencyTX >= 18068000 && CurrentFrequencyTX < 18168000)
            || (CurrentFrequencyTX >= 21000000 && CurrentFrequencyTX < 21450000)
            || (CurrentFrequencyTX >= 24890000 && CurrentFrequencyTX < 24990000)
            || (CurrentFrequencyTX >= 28000000 && CurrentFrequencyTX < 29700000)
            || (CurrentFrequencyTX >= 50000000 && CurrentFrequencyTX < 52000000))) {
      Serial2.print("TX0;");  //Stop transmition due to CAT
      delay(CommandDelay);
      Serial2.print("MX0;");  //Stop transmition due to MOX
      delay(CommandDelay);
      Relay.SET_RLY0(I2C_REL_ADD, HIGH);  //Disconnect the PTT button (Stop transmition due to Mic)
      VOXStatus = ReadVOXStatus();        //Check and stop VOX
      Serial2.print("VX0;");
      delay(CommandDelay);

      //I lock the dial in order to avoid accidental change of frequency while tuning
      FlushSerialInput();
      Serial2.print("LK;");
      delay(CommandDelay);
      Result = "";
      while (Serial2.available() > 0) {
        a = Serial2.read();
        Result = Result + a;
        if (a == ';') {
          LockStatus = Result.substring(2, Result.length() - 1);
        }
      }
      Serial2.print("LK7;");
      delay(CommandDelay);

      if (SecondRelayActivationTime >= 0) {
        SecondRelayActivationMillis = millis() + SecondRelayActivationTime * 1000;
        SecondRelayDeactivationMillis = millis() + SecondRelayActivationTime * 1000 + SecondRelayDelay * 1000;
        ActivateExternalTuner();  //Start tune process for external tuner
      } else {
        SecondRelayActivationMillis = millis();
        SecondRelayDeactivationMillis = millis() + SecondRelayDelay * 1000;
        unsigned long DelayBeforeActivation = millis() - SecondRelayActivationTime * 1000;
        while (millis() < DelayBeforeActivation) {
        }
        ActivateExternalTuner();  //Start tune process for external tuner
      }

      //Restore dial lock status
      Serial2.print("LK" + LockStatus + ";");
      delay(CommandDelay);

      //Restore VOX if it was active
      if (VOXStatus == 1) {
        Serial2.print("VX1;");
        delay(CommandDelay);
      }

      Relay.SET_RLY0(I2C_REL_ADD, LOW);  //Re-connect the PTT button
      ButtonShortPress = false;          //Reset the button flag

    } else if (digitalRead(TXGND) == 0 && Tuned) {  //If we transmit and is tuned

      if (millis() / 1000 > ONAirStartTime / 1000 + ONAirTime + 2) {  //The 2 represends 2 seconds that we allow to depress PTT but still keep counting
        ONAirStartTime = millis();                                    //Reseting on air timer
        ONAirTime = 0;                                                //Reseting time past
        PPO = 0;                                                      //Reseting the PPO
      }

      TimeText = "";

      TFT_BACKGROUND = TFT_EBONY;
      TFT_FOREGROUND = TFT_YELLOW;
      UpperClearDisplay();
      Upper.setFreeFont(&FreeSansBold24pt7b);
      UpperPrintText(8, 70, "ON AIR");

      do {
        ONAirTime = (millis() - ONAirStartTime) / 1000;

        if (ONAirTime / 60 < 10) {
          TimeText = "0" + String(ONAirTime / 60) + ":";
        } else {
          TimeText = String(ONAirTime / 60) + ":";
        }
        if (ONAirTime % 60 < 10) {
          TimeText += "0" + String(ONAirTime % 60) + " ";
        } else {
          TimeText += String(ONAirTime % 60) + " ";
        }
        Upper.setFreeFont(&FreeSansBold24pt7b);
        if (ONAirTime > MaxAirTime * .8) {
          TFT_FOREGROUND = TFT_RED;
        } else {
          TFT_FOREGROUND = TFT_YELLOW;
        }
        UpperPrintText(195, 70, TimeText);

        MPO = ReadPowerOut();
        if (MPO > PPO) PPO = MPO;

        Upper.setFreeFont(&FreeSansBold12pt7b);
        TFT_FOREGROUND = TFT_YELLOW;
        UpperPrintText(180, 100, String(NormalizePO(PPO)) + " W ");
        UpperPrintText(110, 100, "PEP :");
        Upper.pushSprite(0, 38);
      } while (digitalRead(TXGND) == 0);

      Upper.setFreeFont(&FreeSansBold24pt7b);
      UpperPrintText(8, 70, "Air time");
      Upper.pushSprite(0, 38);
      InfoDelay = millis() + 5000;
      Message = 0;
    }
    InfoScreen();
  } else if (TransmitAntenna > 1) {
    if (Message != 5) {
      TFT_BACKGROUND = TFT_EBONY;
      TFT_FOREGROUND = TFT_YELLOW;
      UpperClearDisplay();
      Upper.setFreeFont(&FreeSansBold24pt7b);
      UpperPrintTextCentered(0, 320, 60, "No tuner on");
      UpperPrintTextCentered(0, 320, 105, "this antenna!");
      Upper.pushSprite(0, 38);
      Message = 5;
    }
  } else {
    TFT_BACKGROUND = TFT_EBONY;
    TFT_FOREGROUND = TFT_YELLOW;
    tft.setFreeFont(&FreeSansBold18pt7b);
    tft.drawRect(0, 8, 320, 172, TFT_RED);
    tft.drawRect(1, 9, 318, 170, TFT_RED);
    do {
      tft.fillRect(2, 10, 317, 168, TFT_EBONY);
      PrintTextCentered(2, 318, 80, "Communication");
      PrintTextCentered(2, 318, 115, "error !");
      delay(2000);
      if (ReadPower() != 0) break;
      tft.fillRect(2, 10, 317, 168, TFT_EBONY);
      PrintTextCentered(2, 317, 60, "Pls check your");
      PrintTextCentered(2, 317, 96, "serial cable and");
      PrintTextCentered(2, 317, 132, "set the 232C rate");
      PrintTextCentered(2, 317, 168, "to 38400bps");
      delay(2000);
    } while (ReadPower() == 0);

    while (ReadTime() == "000000") {  //Wait until can read the tranceiver's time
      delay(CommandDelay);
    }
    tft.fillRect(0, 8, 320, 172, TFT_EBONY);

    //Refresh the local RTC from the radio
    Result = ReadTime();
    Result2 = ReadDate();
    rtc.setTime(30, Result.substring(2, 4).toInt(), Result.substring(0, 2).toInt(), Result2.substring(6, 8).toInt(), Result2.substring(4, 6).toInt(), Result2.substring(0, 4).toInt());

    InfoScreen();
  }
}

bool IsInTunedFrequencies(long x) {  //Check if we are in the range of the last tuned frquency

  //int i;  //Generic counter

  // XXXXXXXXXXXXXXXXXXXXXXXXX   I need to see if the internal tuner is engaged or not. I need to add a routine


  if (TransmitAntenna == 1) {  //I also need to see if the internal tuner is disabled XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
    if (x >= LastTunedFrequency * 0.99 && x <= LastTunedFrequency * 1.01) {
      return true;
    }
    return false;
  }

  //if (TransmitAntenna == 2) {  //I also need to see if the internal tuner is enabledd XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
  //  for (i = 0; i < 10; i++) {
  //    if (x >= TunedFrequencies[i] - 10000 && x <= TunedFrequencies[i] + 10000) {
  //      return true;
  //    }
  //  }
  //  return false;
  //}

  return false;
}

void ActivateExternalTuner() {  //This function activates external tuner

  long PreviousFrequency;  // Changing mode (for the tuning) also changes the current frequency. We keep track of the original frequency in order to restore it each time we change mode.

  int PreviousMic;      //The mic level before we start tuning
  String PreviousMode;  //The mode before we start tuning

  Serial2.print("AC000;");  //Disable internal tuner (just in case)
  PreviousFrequency = CurrentFrequencyTX;
  PreviousMode = ReadMode(MAINSUBTX);
  PreviousPower = ReadPower();
  PreviousCompressor = ReadCompressor();
  SetCompressor(0);                            //Remove compresssor before change to AM
  SetMode(MAINSUBTX, "5");                     //Set AM Mode
  SetFrequency(MAINSUBTX, PreviousFrequency);  //Correct the frequency
  Serial2.print("PC010;");                     //Set power at 10 watts
  PreviousMic = ReadMic();                     //Read Mic gain after changing to AM
  SetMic(0);                                   //Mute Mic

  TFT_FOREGROUND = TFT_YELLOW;
  TFT_BACKGROUND = TFT_EBONY;
  ClearDisplay();
  tft.setFreeFont(&FreeSansBold24pt7b);
  PrintTextCentered(0, 320, 113, "Tuning...");

  Serial2.print("TX1;");  //Transmit

  delay(500);
  CheckTuning();
  delay(500);
  Serial2.print("TX0;");                       //Stop Transmit
  delay(100);                                  //Needed when communicating in higher baud rates
  SetMic(PreviousMic);                         //Restore Mic Gain
  SetMode(MAINSUBTX, PreviousMode);            //Restore previous mode
  SetFrequency(MAINSUBTX, PreviousFrequency);  //Correct the frequency
  SetPower(PreviousPower);                     //Restore previous power
  SetCompressor(PreviousCompressor);           //Restore compressor status

  SetFilter(MAINSUBRX, SelectedFilter);  //Usefull for the digital modes

  //Check to see if the tuner tuned

  if (!Tuned) {
    ClearDisplay();
    tft.setFreeFont(&FreeSansBold24pt7b);
    TFT_FOREGROUND = TFT_RED;
    PrintTextCentered(0, 320, 90, "Tune");
    PrintTextCentered(0, 320, 135, "FAILED");
    //tft.setFreeFont(&FreeSansBold18pt7b);
    //TFT_FOREGROUND = TFT_YELLOW;
    Message = 1;
    LastTunedFrequency = 0;
    LastFailedFrequency = CurrentFrequencyTX;
  } else {
    ClearDisplay();

    tft.setFreeFont(&FreeSansBold12pt7b);
    if (SWR <= 13) {
      TFT_FOREGROUND = TFT_GREEN;
      PrintTextCentered(0, 320, 145, "SWR ~1.1");
    } else if (SWR <= 26) {
      TFT_FOREGROUND = TFT_GREEN;
      PrintTextCentered(0, 320, 145, "SWR ~1.2");
    } else if (SWR <= 39) {
      TFT_FOREGROUND = TFT_GREEN;
      PrintTextCentered(0, 320, 145, "SWR ~1.5");
    } else if (SWR <= 80) {
      TFT_FOREGROUND = TFT_YELLOW;
      PrintTextCentered(0, 320, 145, "SWR < 2");
    }

    tft.setFreeFont(&FreeSansBold24pt7b);
    PrintTextCentered(0, 320, 118, "TUNED");

    LastTunedFrequency = CurrentFrequencyTX;  //LastTunedFrequency is used by external tuner.
    LastFailedFrequency = 0;
    Message = 0;
  }
  InfoDelay = millis() + 2000;
}

int ReadActiveVFO() {
  char a;
  String Result;

  FlushSerialInput();
  Serial2.print("VS;");
  delay(CommandDelay);

  Result = "";
  while (Serial2.available() > 0) {
    a = Serial2.read();
    Result = Result + a;
    if (a == ';') {
      Result = Result.substring(2, Result.length() - 1);
    }
  }
  return Result.toInt();
}

long ReadFrequency(int MainSub) {
  char a;
  String Result;

  FlushSerialInput();
  if (MainSub == 0) {
    Serial2.print("FA;");
  } else if (MainSub == 1) {
    Serial2.print("FB;");
  }
  delay(CommandDelay);

  Result = "";
  while (Serial2.available() > 0) {
    a = Serial2.read();
    Result = Result + a;
    if (a == ';') {
      Result = Result.substring(2, Result.length() - 1);
    }
  }
  return Result.toInt();
}

int ReadPower() {

  char a;
  String Result;

  FlushSerialInput();
  Serial2.print("PC;");
  delay(CommandDelay);

  Result = "";
  while (Serial2.available() > 0) {
    a = Serial2.read();
    Result = Result + a;
    if (a == ';') {
      Result = Result.substring(2, Result.length() - 1);
    }
  }
  return Result.toInt();
}

int ReadPowerOut() {

  char a;
  String Result;

  FlushSerialInput();
  Serial2.print("RM5;");
  delay(CommandDelay);

  Result = "";
  while (Serial2.available() > 0) {
    a = Serial2.read();
    Result = Result + a;
    if (a == ';') {
      Result = Result.substring(3, Result.length() - 4);
    }
  }
  return Result.toInt();
}

void SetPower(int Power) {

  unsigned int i;
  String PowerText;

  PowerText = String(Power);
  for (i = 3; i > String(Power).length(); i--) {
    PowerText = "0" + PowerText;
  }
  Serial2.print("PC" + PowerText + ";");
}

int ReadMic() {

  char a;
  String Result;

  FlushSerialInput();
  Serial2.print("MG;");
  delay(CommandDelay);

  Result = "";
  while (Serial2.available() > 0) {
    a = Serial2.read();
    Result = Result + a;
    if (a == ';') {
      Result = Result.substring(2, Result.length() - 1);
    }
  }
  return Result.toInt();
}

void SetMic(int Mic) {
  unsigned int i;
  String MicText;

  MicText = String(Mic);
  for (i = 3; i > String(Mic).length(); i--) {
    MicText = "0" + MicText;
  }
  Serial2.print("MG" + MicText + ";");
}

String ReadMode(int MainSub) {

  char a;
  String Result;

  FlushSerialInput();
  if (MainSub == 0) {
    Serial2.print("MD0;");
  } else if (MainSub == 1) {
    Serial2.print("MD1;");
  }
  delay(CommandDelay);

  Result = "";
  while (Serial2.available() > 0) {
    a = Serial2.read();
    Result = Result + a;
    if (a == ';') {
      Result = Result.substring(3, Result.length() - 1);
    }
  }
  return Result;
}

void SetMode(int MainSub, String Mode) {
  if (MainSub == 0) {
    Serial2.print("MD0" + Mode + ";");
  } else if (MainSub == 1) {
    Serial2.print("MD1" + Mode + ";");
  }
  delay(CommandDelay);
}

void SetFrequency(int MainSub, long Frequency) {
  unsigned int i;
  String FrequencyText;

  FrequencyText = String(Frequency);
  for (i = 9; i > String(Frequency).length(); i--) {
    FrequencyText = "0" + FrequencyText;
  }
  if (MainSub == 0) {
    Serial2.print("FA" + FrequencyText + ";");
  } else if (MainSub == 1) {
    Serial2.print("FB" + FrequencyText + ";");
  }
}

void SetFilter(int MainSub, int filter) {  //Set the roofing filter
  if (filter < 1 || filter > 5) return;
  if (MainSub == 0) {
    Serial2.print("RF0" + String(filter) + ";");  //RF 0/1 - main/sub filter 1/2/3/4/5 - 12K/3K/1.2K/600/300
  } else if (MainSub == 1) {
    Serial2.print("RF1" + String(filter) + ";");  //RF 0/1 - main/sub filter 1/2/3/4/5 - 12K/3K/1.2K/600/300
  }
  delay(CommandDelay);
}

int ReadSWR() {
  char a;
  String Result;

  FlushSerialInput();
  Serial2.print("RM6;");
  delay(CommandDelay);

  Result = "";
  while (Serial2.available() > 0) {
    a = Serial2.read();
    Result = Result + a;
    if (a == ';') {
      Result = Result.substring(3, Result.length() - 4);
      SWR = Result.toInt();
    }
  }
  return Result.toInt();
}

int ReadSQL() {  //Read SQL level on active VFO
  char a;
  String Result;

  FlushSerialInput();
  switch (ReadActiveVFO()) {
    case 0:
      Serial2.print("SQ0;");
      break;
    case 1:
      Serial2.print("SQ1;");
      break;
    default:
      break;
  }
  delay(CommandDelay);

  Result = "";
  while (Serial2.available() > 0) {
    a = Serial2.read();
    Result = Result + a;
    if (a == ';') {
      Result = Result.substring(3, Result.length() - 1);
    }
  }
  return Result.toInt();
}

void SetSQL(int SQL) {  //Set SQL level on active VFO
  unsigned int i;
  String SQLText;

  SQLText = String(SQL);
  for (i = 3; i > String(SQL).length(); i--) {
    SQLText = "0" + SQLText;
  }
  switch (ReadActiveVFO()) {
    case 0:
      Serial2.print("SQ0" + SQLText + ";");
      break;
    case 1:
      Serial2.print("SQ1" + SQLText + ";");
      break;
    default:
      break;
  }
  delay(CommandDelay);
}

int ReadNotchWidth() {
  char a;
  String Result;

  FlushSerialInput();
  Serial2.print("EX030205;");
  delay(CommandDelay);

  Result = "";
  while (Serial2.available() > 0) {
    a = Serial2.read();
    Result = Result + a;
    if (a == ';') {
      Result = Result.substring(8, Result.length() - 1);
    }
  }
  return Result.toInt();
}

void SetNotchWidth(int NotchWidth) {  //Accepts only 0 (Narrow) and 1 (Wide).Anything else is not processed
  if (NotchWidth == 0) {
    Serial2.print("EX0302050;");
  }
  if (NotchWidth == 1) {
    Serial2.print("EX0302051;");
  }
}

int ReadMemory() {
  char a;
  String Result;

  FlushSerialInput();
  Serial2.print("MC;");
  delay(CommandDelay);

  Result = "";
  while (Serial2.available() > 0) {
    a = Serial2.read();
    Result = Result + a;
    if (a == ';') {
      Result = Result.substring(2, Result.length() - 1);
    }
  }
  return Result.toInt();
}

void SetMemory(int Memory) {
  unsigned int i;
  String MemoryText;

  MemoryText = String(Memory);
  for (i = 3; i > String(Memory).length(); i--) {
    MemoryText = "0" + MemoryText;
  }
  Serial2.print("MC" + MemoryText + ";");
}

String ReadTime() {  //Reading the time from the tranceiver
  char a;
  String Result;

  FlushSerialInput();
  Serial2.print("DT1;");
  delay(CommandDelay);

  Result = "";
  while (Serial2.available() > 0) {
    a = Serial2.read();
    Result = Result + a;
    if (a == ';') {
      Result = Result.substring(3, Result.length() - 1);
    }
  }
  return Result;
}

String ReadDate() {  //Reading the date from the tranceiver
  char a;
  String Result;

  FlushSerialInput();
  Serial2.print("DT0;");
  delay(CommandDelay);

  Result = "";
  while (Serial2.available() > 0) {
    a = Serial2.read();
    Result = Result + a;
    if (a == ';') {
      Result = Result.substring(3, Result.length() - 1);
    }
  }
  return Result;
}

void CheckTuning() {

  int i, SWRReadings[10] = { 255, 255, 255, 255, 255, 255, 255, 255, 255, 255 };
  long SWRAverage = 0;
  int SWRMeasurments = 10;
  unsigned long TunerTimer;

  Tuned = false;
  TunerTimer = millis();

  do {
    SWR = ReadSWR();
    for (i = 9; i > 0; i--) {
      SWRReadings[i] = SWRReadings[i - 1];
    }
    SWRReadings[0] = SWR;

    SWRAverage = 0;
    for (i = 0; i < 10; i++) {
      SWRAverage = SWRAverage + SWRReadings[i];
    }
    if (SWRAverage / SWRMeasurments <= 80) {  //Under this limit, it's considered tuned
      Tuned = true;
      break;
    }
    delay(50);
  } while (TunerTimer + 5000 > millis());  // Repeat for 5 secs (actually it is longer)
}

void InfoScreen() {                  // Displays messages on the display
  if (InfoDelay > millis()) return;  //Check if we have to delay

  TFT_FOREGROUND = TFT_YELLOW;
  TFT_BACKGROUND = TFT_EBONY;

  if (!Tuned && Message != 1) {
    UpperClearDisplay();
    Upper.setFreeFont(&FreeSansBold24pt7b);
    //UpperPrintTextCentered(0, 320, 60, "NOT");
    //UpperPrintTextCentered(0, 320, 105, "TUNED");
    UpperPrintTextCentered(0, 320, 80, "NOT TUNED");
    Upper.pushSprite(0, 38);
    Message = 1;
  }

  if (Tuned && Message != 2) {  //If the tuner is tuned
    UpperClearDisplay();
    Upper.setFreeFont(&FreeSansBold24pt7b);
    if (SWR <= 39) {
      TFT_FOREGROUND = TFT_GREEN;
    } else {
      TFT_FOREGROUND = TFT_YELLOW;
    }
    UpperPrintTextCentered(0, 320, 80, "TUNED");
    Upper.pushSprite(0, 38);
    Message = 2;
  }

  TFT_FOREGROUND = TFT_YELLOW;
  Upper.setFreeFont(&FreeSansBold12pt7b);
  UpperPrintTextCentered(0, 320, 25, rtc.getTime("%d/%B %H:%Mz"));
  Upper.pushSprite(0, 38);

  FrCheck();

  PrintStatus();
}

void PrintStatus(void) {
  int a, b, c, d, i = 0;
  unsigned long Timer = millis();
  bool BigChars = true;

  a = ReadPower();
  b = ReadEQ();
  c = ReadCompressor();
  d = ReadKeyer();

  if (a == 0) BigChars = false;

  if (a == PreviousPower && b == PreviousEQ && c == PreviousCompressor && d == PreviousKeyer) {
    return;
  }

  if (b != PreviousEQ) {
    if (ReadEQ() == 1) {
      tft.fillRoundRect(0, 210, 54, 30, 5, TFT_GREEN);
      tft.setFreeFont(&FreeSansBold12pt7b);
      TFT_FOREGROUND = TFT_BLACK;
      TFT_BACKGROUND = TFT_GREEN;
    } else {
      TFT_FOREGROUND = TFT_BLACK;
      tft.fillRoundRect(0, 210, 54, 30, 5, TFT_RED);
      tft.fillRoundRect(2, 212, 50, 26, 5, TFT_BLACK);
      tft.setFreeFont(&FreeSansBold12pt7b);
      TFT_FOREGROUND = TFT_RED;
      TFT_BACKGROUND = TFT_BLACK;
    }
    PrintText(9, 238, "EQ");
    TFT_BACKGROUND = TFT_EBONY;
    PreviousEQ = b;
  }


  if (c != PreviousCompressor) {
    if (ReadCompressor() == 1) {
      tft.fillRoundRect(60, 210, 88, 30, 5, TFT_GREEN);
      tft.setFreeFont(&FreeSansBold12pt7b);
      TFT_FOREGROUND = TFT_BLACK;
      TFT_BACKGROUND = TFT_GREEN;
    } else {
      TFT_FOREGROUND = TFT_BLACK;
      tft.fillRoundRect(60, 210, 88, 30, 5, TFT_RED);
      tft.fillRoundRect(62, 212, 84, 26, 5, TFT_BLACK);
      tft.setFreeFont(&FreeSansBold12pt7b);
      TFT_FOREGROUND = TFT_RED;
      TFT_BACKGROUND = TFT_BLACK;
    }
    PrintText(66, 238, "COMP");
    TFT_BACKGROUND = TFT_EBONY;
    PreviousCompressor = c;
  }

  if (d != PreviousKeyer) {
    if (ReadKeyer() == 1) {
      tft.fillRoundRect(156, 210, 93, 30, 5, TFT_GREEN);
      tft.setFreeFont(&FreeSansBold12pt7b);
      TFT_FOREGROUND = TFT_BLACK;
      TFT_BACKGROUND = TFT_GREEN;
    } else {
      TFT_FOREGROUND = TFT_BLACK;
      tft.fillRoundRect(156, 210, 93, 30, 5, TFT_RED);
      tft.fillRoundRect(158, 212, 89, 26, 5, TFT_BLACK);
      tft.setFreeFont(&FreeSansBold12pt7b);
      TFT_FOREGROUND = TFT_RED;
      TFT_BACKGROUND = TFT_BLACK;
    }
    PrintText(162, 238, "KEYER");
    TFT_BACKGROUND = TFT_EBONY;
    PreviousKeyer = d;
  }

  if (a != PreviousPower) {
    do {
      a = ReadPower();
      if (a != PreviousPower) {
        TFT_FOREGROUND = TFT_YELLOW;
        if (BigChars) {
          TFT_BACKGROUND = TFT_EBONY;
          UpperClearDisplay();
          Upper.setFreeFont(&FreeSansBold24pt7b);
          UpperPrintTextCentered(0, 320, 75, "PWR: " + String(a) + "W");  //Print in the upper screen
          Upper.pushSprite(0, 38);
          Timer = millis();
        }
        Lower.fillRect(0, 0, Lower.width(), Lower.height(), TFT_BLACK);
        Lower.setFreeFont(&FreeSansBold12pt7b);
        TFT_BACKGROUND = TFT_BLACK;
        LowerPrintText(1, 28, "PWR: " + String(a) + "W");  //Print to the lower screen
        Lower.pushSprite(0, 181);
        if (ExtendedParameter1 == 6) { DisplayULC(); }  //Print to the right corner too
        if (ExtendedParameter2 == 6) { DisplayLLC(); }
        if (ExtendedParameter3 == 6) { DisplayLRC(); }
        if (ExtendedParameter4 == 6) { DisplayURC(); }
        PreviousPower = a;
      }
      if (a == 0) Timer = 0;  //If the power = 0 then no need to wait
    } while (Timer + 800 > millis());
    if (BigChars == true) Message = 0;  //To allow to print another message over big characters
  }
}

void SystemMenu() {
  Upper.deleteSprite();
  Upper.createSprite(320, 172);
  int ChosenMenu = DisplayMenu(7, 1, 1, "System Menu", "Frequency steps", "Roofing filter", "RF Power", "Transmit timer", "2nd relay", "Save parameters");

  if (ChosenMenu > 0) {
    SubMenu(String(ChosenMenu));
  }
  Upper.deleteSprite();
  Upper.createSprite(320, 112);
  ClearDisplay();
  Message = 0;
}

void SubMenu(String smenu) {
  start = millis();
  bool Redraw = true;
  int i, j;
  //String GenericText = "";  //To be removed
  int timeout = 3000;
  int TempMaxAirTime = MaxAirTime;
  int TempSecondRelayActivationTime = SecondRelayActivationTime;
  int TempSecondRelayDelay = SecondRelayDelay;
  bool FirstParameterSet = false;  //Specifically defined for 2nd relay function
  int ChosenMenu = 0;

  switch (smenu.toInt()) {
    case 1:
      ChosenMenu = DisplayMenu(5, 1, 1, "Frequency steps", "100Hz", "250Hz", "500Hz", "1KHz");
      break;
    case 2:
      ChosenMenu = DisplayMenu(6, 1, 1, "Roofing filter", "  None ", " 12 KHz", "  3 KHz", " 600 Hz", " 300 Hz");
      break;
    case 3:
      ChosenMenu = DisplayMenu(7, 1, 1, "RF Power", "5W", "25W", "50W", "100W", "150W", "200W");
      break;
    case 6:
      ChosenMenu = DisplayMenu(3, 1, 1, "Save parameters", "No", "Yes");
      break;
    default:
      break;
  }

  if (ChosenMenu == 0) {  //If no standard option was chosen (smenu 4 and 5)

    TFT_FOREGROUND = TFT_YELLOW;
    TFT_BACKGROUND = TFT_EBONY;
    UpperClearDisplay();

    UpperPrintTextCentered(0, 320, 43, "--------------------");
    if (smenu == "4") {
      UpperPrintTextCentered(0, 320, 28, "Transmit timer");
    } else if (smenu == "5") {
      UpperPrintTextCentered(0, 320, 28, "2nd Relay");
    }

    do {
      if (Redraw == true) {
        TFT_FOREGROUND = TFT_YELLOW;
        TFT_BACKGROUND = TFT_EBONY;


        if (smenu == "4") {
          TFT_BACKGROUND = TFT_RED;
          UpperPrintTextCentered(0, 320, 1 * 26 + 65, String(TempMaxAirTime) + " Sec");
        } else if (smenu == "5") {
          TFT_BACKGROUND = TFT_EBONY;
          UpperPrintTextCentered(0, 160, 1 * 26 + 65, "Time");
          UpperPrintTextCentered(161, 320, 1 * 26 + 65, "Delay");
          if (FirstParameterSet == false) {
            UpperPrintTextCentered(161, 320, 2 * 26 + 65, String(TempSecondRelayDelay));
            TFT_BACKGROUND = TFT_RED;
            UpperPrintTextCentered(0, 160, 2 * 26 + 65, String(TempSecondRelayActivationTime));
          } else {
            UpperPrintTextCentered(0, 160, 2 * 26 + 65, String(TempSecondRelayActivationTime));
            TFT_BACKGROUND = TFT_RED;
            UpperPrintTextCentered(161, 320, 2 * 26 + 65, String(TempSecondRelayDelay));
          }
        }
        TFT_BACKGROUND = TFT_EBONY;

        Upper.pushSprite(0, 8);
        Redraw = false;
      }

      int32_t new_position1 = RotatorDirection * RE1.getEncoderPosition();  // Reads the current position of the rotary encoder
      if (new_position1 != encoder_position1) {                             //If the encoder has been moved
        if (smenu == "4") {                                                 //if we are in the 4th menu...
          if (new_position1 < encoder_position1) {
            TempMaxAirTime++;
            if (TempMaxAirTime > 1000) TempMaxAirTime = 999;
          } else {
            TempMaxAirTime--;
            if (TempMaxAirTime < 1) TempMaxAirTime = 1;
          }
        } else if (smenu == "5") {  //if we are in the 5th menu...
          if (FirstParameterSet == false) {
            if (new_position1 < encoder_position1) {
              TempSecondRelayActivationTime++;
              if (TempSecondRelayActivationTime > 10) TempSecondRelayActivationTime = 10;
            } else {
              TempSecondRelayActivationTime--;
              if (TempSecondRelayActivationTime < -10) TempSecondRelayActivationTime = -10;
            }
          } else {
            if (new_position1 < encoder_position1) {
              TempSecondRelayDelay++;
              if (TempSecondRelayDelay > 20) TempSecondRelayDelay = 20;
            } else {
              TempSecondRelayDelay--;
              if (TempSecondRelayDelay < 0) TempSecondRelayDelay = 0;
            }
          }
        }
        Redraw = true;
        start = millis();
      }
      encoder_position1 = new_position1;  // Updates the current position of the rotary encoder


      //Now we will see if the button was pressed (for smenu 4 and 5) and will take action
      if (!RE1.digitalRead(SS_SWITCH)) {  // Here we see if the button was pressed
        do {
          delay(CommandDelay);
        } while (!RE1.digitalRead(SS_SWITCH));  //Loop while it is pressed

        //Now we take action
        if (smenu == "4") {
          MaxAirTime = TempMaxAirTime;
        } else if (smenu == "5") {
          if (FirstParameterSet == false) {
            FirstParameterSet = true;
            Redraw = true;
          } else {
            SecondRelayActivationTime = TempSecondRelayActivationTime;
            SecondRelayDelay = TempSecondRelayDelay;
            FirstParameterSet = false;
          }
        }
        if (FirstParameterSet == true) {
          start = millis();
        } else {
          start = 0;
          timeout = 0;
        }
      }

      PrintStatus();
    } while (millis() < start + timeout);
  } else {  //Now we will see if a menu option was chosen and will take action

    //Now we take action
    if (smenu == "1") {
      switch (ChosenMenu) {
        case 1:
          Steps = 100;
          break;
        case 2:
          Steps = 250;
          break;
        case 3:
          Steps = 500;
          break;
        case 4:
          Steps = 1000;
          break;
        default:
          break;
      }
    } else if (smenu == "2") {
      switch (ChosenMenu) {
        case 1:
          SelectedFilter = 0;
          break;
        case 2:
          SelectedFilter = 1;
          break;
        case 3:
          SelectedFilter = 2;
          break;
        case 4:
          SelectedFilter = 4;
          break;
        case 5:
          SelectedFilter = 5;
          break;
        default:
          break;
      }
    } else if (smenu == "3") {
      //Set power
      switch (ChosenMenu) {
        case 1:
          SetPower(5);
          break;
        case 2:
          SetPower(25);
          break;
        case 3:
          SetPower(50);
          break;
        case 4:
          SetPower(100);
          break;
        case 5:
          SetPower(150);
          break;
        case 6:
          SetPower(200);
          break;
        default:
          break;
      }
    } else if (smenu == "6") {
      if (ChosenMenu == 2) {
        preferences.begin("ATCK", false);
        preferences.putInt("Parameter1", ExtendedParameter1);
        preferences.putInt("Parameter2", ExtendedParameter2);
        preferences.putInt("Parameter3", ExtendedParameter3);
        preferences.putInt("Parameter4", ExtendedParameter4);
        preferences.putInt("Steps", Steps);
        preferences.putInt("Filter", SelectedFilter);
        preferences.putInt("MaxAirTime", MaxAirTime);
        preferences.putInt("SRAT", SecondRelayActivationTime);
        preferences.putInt("SRD", SecondRelayDelay);
        preferences.end();
      }
    }
  }
}

int ReadAntenna(int MainSub) {
  char a;
  String Result;

  FlushSerialInput();
  if (MainSub == 0) {
    Serial2.print("AN0;");
  } else if (MainSub == 1) {
    Serial2.print("AN1;");
  }
  delay(CommandDelay);


  Result = "";
  while (Serial2.available() > 0) {
    a = Serial2.read();
    Result = Result + a;
    if (a == ';') {
      Result = Result.substring(3, Result.length() - 2);
    }
  }
  return Result.toInt();
}

int ReadVOXStatus(void) {  //Return 1 if active, 0 if not
  char a;
  String Result;

  FlushSerialInput();
  Serial2.print("VX;");
  delay(CommandDelay);

  Result = "";
  while (Serial2.available() > 0) {
    a = Serial2.read();
    Result = Result + a;
    if (a == ';') {
      Result = Result.substring(3, Result.length() - 2);
    }
  }
  return Result.toInt();
}

int ReadTX() {  //Read where is the TX (main or sub)
  char a;
  String Result;

  FlushSerialInput();
  Serial2.print("FT;");
  delay(CommandDelay);

  Result = "";
  while (Serial2.available() > 0) {
    a = Serial2.read();
    Result = Result + a;
    if (a == ';') {
      Result = Result.substring(2, Result.length() - 1);
    }
  }
  return Result.toInt();
}

int VFORead() {  //Read which VFO is active
  char a;
  String Result;

  FlushSerialInput();
  Serial2.print("VS;");
  delay(CommandDelay);

  Result = "";
  while (Serial2.available() > 0) {
    a = Serial2.read();
    Result = Result + a;
    if (a == ';') {
      Result = Result.substring(2, Result.length() - 1);
    }
  }
  return Result.toInt();
}

int ReadCompressor() {
  char a;
  String Result;

  FlushSerialInput();
  Serial2.print("PR0;");
  delay(CommandDelay);

  Result = "";
  while (Serial2.available() > 0) {
    a = Serial2.read();
    Result = Result + a;
    if (a == ';') {
      Result = Result.substring(3, Result.length() - 1);
    }
  }
  return Result.toInt();
}

void SetCompressor(int x) {
  if (x == 1) {
    Serial2.print("PR01;");
  } else if (x == 0) {
    Serial2.print("PR00;");
  }
  delay(CommandDelay);
}

int ReadKeyer() {
  char a;
  String Result;

  FlushSerialInput();
  Serial2.print("KR;");
  delay(CommandDelay);

  Result = "";
  while (Serial2.available() > 0) {
    a = Serial2.read();
    Result = Result + a;
    if (a == ';') {
      Result = Result.substring(2, Result.length() - 1);
    }
  }
  return Result.toInt();
}

int ReadEQ() {
  char a;
  String Result;

  FlushSerialInput();
  Serial2.print("PR1;");
  delay(CommandDelay);

  Result = "";
  while (Serial2.available() > 0) {
    a = Serial2.read();
    Result = Result + a;
    if (a == ';') {
      Result = Result.substring(3, Result.length() - 1);
    }
  }
  return Result.toInt();
}

void ClearDisplay() {
  tft.fillRect(0, 8, 320, 172, TFT_BACKGROUND);
}

void UpperClearDisplay() {
  Upper.fillRect(0, 0, Upper.width(), Upper.height(), TFT_BACKGROUND);
}

void FlushSerialInput() {
  char a;
  while (Serial2.available() > 0) {
    a = Serial2.read();
  }
}

void MenuHandle_1stEncoder() {
  int HighlightedMenu = ExtendedParameter1;  //The menu that is highlighted is the one already chosen for this encoder

  Upper.deleteSprite();
  Upper.createSprite(320, 172);

  HighlightedMenu = DisplayMenu(10, ExtendedParameter1, 1, "Select parameter", "Squelch", "Memory Channel", "Notch", "Contour width", "Contour level", "Power level", "Frequency Active", "Frequency Main", "Frequency Sub");

  if (HighlightedMenu != 0) {
    ExtendedParameter1 = HighlightedMenu;
  }

  Upper.deleteSprite();
  Upper.createSprite(320, 112);
  ClearDisplay();

  Message = 0;
  PrintStatus();
}

void MenuHandle_2ndEncoder() {
  int HighlightedMenu = ExtendedParameter2;  //The menu that is highlighted is the one already chosen for this encoder

  Upper.deleteSprite();
  Upper.createSprite(320, 172);

  HighlightedMenu = DisplayMenu(10, ExtendedParameter2, 2, "Select parameter", "Squelch", "Memory Channel", "Notch", "Contour width", "Contour level", "Power level", "Frequency Active", "Frequency Main", "Frequency Sub");

  if (HighlightedMenu != 0) {
    ExtendedParameter2 = HighlightedMenu;
  }

  Upper.deleteSprite();
  Upper.createSprite(320, 112);
  ClearDisplay();

  Message = 0;
  PrintStatus();
}

void MenuHandle_3rdEncoder() {
  int HighlightedMenu = ExtendedParameter3;  //The menu that is highlighted is the one already chosen for this encoder

  Upper.deleteSprite();
  Upper.createSprite(320, 172);

  HighlightedMenu = DisplayMenu(10, ExtendedParameter3, 3, "Select parameter", "Squelch", "Memory Channel", "Notch", "Contour width", "Contour level", "Power level", "Frequency Active", "Frequency Main", "Frequency Sub");

  if (HighlightedMenu != 0) {
    ExtendedParameter3 = HighlightedMenu;
  }

  Upper.deleteSprite();
  Upper.createSprite(320, 112);
  ClearDisplay();

  Message = 0;
  PrintStatus();
}

void MenuHandle_4thEncoder() {
  int HighlightedMenu = ExtendedParameter4;  //The menu that is highlighted is the one already chosen for this encoder

  Upper.deleteSprite();
  Upper.createSprite(320, 172);

  HighlightedMenu = DisplayMenu(10, ExtendedParameter4, 4, "Select parameter", "Squelch", "Memory Channel", "Notch", "Contour width", "Contour level", "Power level", "Frequency Active", "Frequency Main", "Frequency Sub");

  if (HighlightedMenu != 0) {
    ExtendedParameter4 = HighlightedMenu;
  }

  Upper.deleteSprite();
  Upper.createSprite(320, 112);
  ClearDisplay();

  Message = 0;
  PrintStatus();
}

void ReadEncoder() {  //Unified ReadEncoder function
  int SQL;
  int Memory;
  int NotchWidth;
  int ContourWidth;
  int ContourLevel;
  int PowerLevel;
  int32_t new_position;
  int32_t encoder_position;
  //long CurrentFrequencyRX;
  int RotatorEncoder;  //Which is the rotaror that we work with.
  int ExtendedParameter;
  int i = 0;
  long Frequency = 0;
  start = millis();

  SQL = ReadSQL();
  Memory = ReadMemory();
  NotchWidth = ReadNotchWidth();
  ContourWidth = ReadContourWidth();
  ContourLevel = ReadContourLevel();
  PowerLevel = ReadPower();
  CurrentFrequencyRX = ReadFrequency(VFORead());

  //Find which rotator is moving
  if (encoder_position1 != RotatorDirection * RE1.getEncoderPosition()) {
    RotatorEncoder = 1;
  } else if (encoder_position2 != RotatorDirection * RE2.getEncoderPosition()) {
    RotatorEncoder = 2;
  } else if (encoder_position3 != RotatorDirection * RE3.getEncoderPosition()) {
    RotatorEncoder = 3;
  } else if (encoder_position4 != RotatorDirection * RE4.getEncoderPosition()) {
    RotatorEncoder = 4;
  }

  //Find out which parameter we are working with
  switch (RotatorEncoder) {
    case 1:
      ExtendedParameter = ExtendedParameter1;
      break;
    case 2:
      ExtendedParameter = ExtendedParameter2;
      break;
    case 3:
      ExtendedParameter = ExtendedParameter3;
      break;
    case 4:
      ExtendedParameter = ExtendedParameter4;
      break;
    default:
      break;
  }

  do {
    // If the previous and the current state of the outputA are different, that means a Pulse has occured
    switch (RotatorEncoder) {
      case 1:
        new_position = RotatorDirection * RE1.getEncoderPosition();
        break;
      case 2:
        new_position = RotatorDirection * RE2.getEncoderPosition();
        break;
      case 3:
        new_position = RotatorDirection * RE3.getEncoderPosition();
        break;
      case 4:
        new_position = RotatorDirection * RE4.getEncoderPosition();
        break;
      default:
        break;
    }

    switch (RotatorEncoder) {
      case 1:
        encoder_position = encoder_position1;
        break;
      case 2:
        encoder_position = encoder_position2;
        break;
      case 3:
        encoder_position = encoder_position3;
        break;
      case 4:
        encoder_position = encoder_position4;
        break;
      default:
        break;
    }



    if (encoder_position != new_position) {
      if (Message != 0) {
        TFT_FOREGROUND = TFT_YELLOW;
        TFT_BACKGROUND = TFT_EBONY;
        UpperClearDisplay();
        Message = 0;
      }


      if (new_position < encoder_position) {  // If the rotator moved clockwise

        switch (ExtendedParameter) {
          case 1:
            SQL += 10 * (encoder_position - new_position);
            if (SQL > 100) { SQL = 100; }
            SetSQL(SQL);
            break;
          case 2:
            Memory += (encoder_position - new_position);
            if (Memory > 100) { Memory = 100; }
            SetMemory(Memory);
            break;
          case 3:
            if (NotchWidth == 0) {
              NotchWidth = 1;
              SetNotchWidth(NotchWidth);
            }
            break;
          case 4:
            ContourWidth += (encoder_position - new_position);
            if (ContourWidth > 11) { ContourWidth = 11; }
            SetContourWidth(ContourWidth);
            break;
          case 5:
            ContourLevel += (encoder_position - new_position);
            if (ContourLevel > 20) { ContourLevel = 20; }
            SetContourLevel(ContourLevel);
            break;
          case 6:
            PowerLevel += (encoder_position - new_position);
            if (RIG_Model == 'FTDX101MP' && PowerLevel > 200) {
              PowerLevel = 200;
            } else if (RIG_Model == 'FTDX101D' && PowerLevel > 100) {
              PowerLevel = 100;
            }
            SetPower(PowerLevel);
            break;
          case 7:
            CurrentFrequencyRX = CurrentFrequencyRX / Steps * Steps;
            CurrentFrequencyRX = CurrentFrequencyRX + Steps * (encoder_position - new_position);
            SetFrequency(VFORead(), CurrentFrequencyRX);
            break;
          case 8:
            Frequency = ReadFrequency(0);
            Frequency = Frequency / Steps * Steps;
            Frequency = Frequency + Steps * (encoder_position - new_position);
            SetFrequency(0, Frequency);
            break;
          case 9:
            Frequency = ReadFrequency(1);
            Frequency = Frequency / Steps * Steps;
            Frequency = Frequency + Steps * (encoder_position - new_position);
            SetFrequency(1, Frequency);
            break;
          default:
            break;
        }
      } else {  //The rotator turned counterclockwise

        switch (ExtendedParameter) {
          case 1:
            SQL -= 10 * (new_position - encoder_position);
            if (SQL < 0) { SQL = 0; }
            SetSQL(SQL);
            break;
          case 2:
            Memory -= (new_position - encoder_position);
            if (Memory < 1) { Memory = 1; }
            SetMemory(Memory);
            break;
          case 3:
            if (NotchWidth == 1) {
              NotchWidth = 0;
              SetNotchWidth(NotchWidth);
            }
            break;
          case 4:
            ContourWidth -= (new_position - encoder_position);
            if (ContourWidth < 1) { ContourWidth = 1; }
            SetContourWidth(ContourWidth);
            break;
          case 5:
            ContourLevel -= (new_position - encoder_position);
            if (ContourLevel < -40) { ContourLevel = -40; }
            SetContourLevel(ContourLevel);
            break;
          case 6:
            PowerLevel -= (new_position - encoder_position);
            if (PowerLevel < 5) { PowerLevel = 5; }
            SetPower(PowerLevel);
            break;
          case 7:
            CurrentFrequencyRX = (CurrentFrequencyRX + (Steps - 1)) / Steps * Steps;
            CurrentFrequencyRX = CurrentFrequencyRX - Steps * (new_position - encoder_position);
            SetFrequency(VFORead(), CurrentFrequencyRX);
            break;
          case 8:
            Frequency = ReadFrequency(0);
            Frequency = (Frequency + (Steps - 1)) / Steps * Steps;
            Frequency = Frequency - Steps * (new_position - encoder_position);
            SetFrequency(0, Frequency);
            break;
          case 9:
            Frequency = ReadFrequency(1);
            Frequency = (Frequency + (Steps - 1)) / Steps * Steps;
            Frequency = Frequency - Steps * (new_position - encoder_position);
            SetFrequency(1, Frequency);
            break;
          default:
            break;
        }
      }

      switch (RotatorEncoder) {
        case 1:
          encoder_position1 = new_position;
          break;
        case 2:
          encoder_position2 = new_position;
          break;
        case 3:
          encoder_position3 = new_position;
          break;
        case 4:
          encoder_position4 = new_position;
          break;
        default:
          break;
      }

      TFT_FOREGROUND = TFT_YELLOW;
      TFT_BACKGROUND = TFT_EBONY;
      switch (ExtendedParameter) {
        case 1:
          Upper.setFreeFont(&FreeSansBold24pt7b);
          UpperPrintTextCentered(0, 320, 75, "SQL: " + String(ReadSQL()));
          break;
        case 2:
          Upper.setFreeFont(&FreeSansBold24pt7b);
          if (Memory != ReadMemory()) {
            UpperPrintTextCentered(0, 320, 75, "MEM: " + String(Memory) + " E");
          } else {
            UpperPrintTextCentered(0, 320, 75, "MEM: " + String(Memory));
          }
          break;
        case 3:
          Upper.setFreeFont(&FreeSansBold24pt7b);
          if (NotchWidth == 0) {
            UpperPrintTextCentered(0, 320, 75, "Narrow notch");
          } else if (NotchWidth == 1) {
            UpperPrintTextCentered(0, 320, 75, "Wide notch");
          }
          break;
        case 4:
          Upper.setFreeFont(&FreeSansBold24pt7b);
          UpperPrintTextCentered(0, 320, 75, "Cont W: " + String(ContourWidth));
          break;
        case 5:
          Upper.setFreeFont(&FreeSansBold24pt7b);
          UpperPrintTextCentered(0, 320, 75, "Cont L: " + String(ContourLevel));
          break;
        case 6:
          Upper.setFreeFont(&FreeSansBold24pt7b);
          UpperPrintTextCentered(0, 320, 75, "PWR: " + String(PowerLevel) + "W");
          Lower.fillRect(0, 0, Lower.width(), Lower.height(), TFT_BLACK);
          Lower.setFreeFont(&FreeSansBold12pt7b);
          TFT_BACKGROUND = TFT_BLACK;
          LowerPrintText(1, 28, "PWR: " + String(PowerLevel) + "W");
          Lower.pushSprite(0, 181);
          PreviousPower = PowerLevel;  //In order to avoid to force status update because of the power change
          break;
        case 7:
          //The following block of code is done to avoid some flickering while changing the frequentcy on the tft screen
          Upper.setFreeFont(&FreeSansBold24pt7b);
          if (CurrentFrequencyRX > 9999999) {
            UpperPrintTextCentered(0, 320, 75, String(CurrentFrequencyRX).substring(0, 2) + "." + String(CurrentFrequencyRX).substring(2, 5) + "." + String(CurrentFrequencyRX).substring(5, 8));
          } else {
            UpperPrintTextCentered(0, 320, 75, String(CurrentFrequencyRX).substring(0, 1) + "." + String(CurrentFrequencyRX).substring(1, 4) + "." + String(CurrentFrequencyRX).substring(4, 7));
          }
          FrCheck();
          break;
        case 8:
          //The following block of code is done to avoid some flickering while changing the frequentcy on the tft screen
          Upper.setFreeFont(&FreeSansBold24pt7b);
          if (Frequency > 9999999) {
            UpperPrintTextCentered(0, 320, 75, String(Frequency).substring(0, 2) + "." + String(Frequency).substring(2, 5) + "." + String(Frequency).substring(5, 8));
          } else {
            UpperPrintTextCentered(0, 320, 75, String(Frequency).substring(0, 1) + "." + String(Frequency).substring(1, 4) + "." + String(Frequency).substring(4, 7));
          }
          FrCheck();
          break;
        case 9:
          //The following block of code is done to avoid some flickering while changing the frequentcy on the tft screen
          Upper.setFreeFont(&FreeSansBold24pt7b);
          if (Frequency > 9999999) {
            UpperPrintTextCentered(0, 320, 75, String(Frequency).substring(0, 2) + "." + String(Frequency).substring(2, 5) + "." + String(Frequency).substring(5, 8));
          } else {
            UpperPrintTextCentered(0, 320, 75, String(Frequency).substring(0, 1) + "." + String(Frequency).substring(1, 4) + "." + String(Frequency).substring(4, 7));
          }
          FrCheck();
          break;
        default:
          break;
      }
      Upper.pushSprite(0, 38);

      switch (RotatorEncoder) {  //Update the display on the right corner
        case 1:
          DisplayULC();
          break;
        case 2:
          DisplayLLC();
          break;
        case 3:
          DisplayLRC();
          break;
        case 4:
          DisplayURC();
          break;
        default:
          break;
      }

      start = millis();
    }

    switch (RotatorEncoder) {
      case 1:
        if (!RE1.digitalRead(SS_SWITCH)) {  // Reads if the rotator button is pressed...
          do {
          } while (!RE1.digitalRead(SS_SWITCH));  //...end wait until released
          encoder_position1 = 0;
          RE1.setEncoderPosition(0);
          return;
        }
        break;
      case 2:
        if (!RE2.digitalRead(SS_SWITCH)) {  // Reads if the rotator button is pressed...
          do {
          } while (!RE2.digitalRead(SS_SWITCH));  //...end wait until released
          encoder_position2 = 0;
          RE2.setEncoderPosition(0);
          return;
        }
        break;
      case 3:
        if (!RE3.digitalRead(SS_SWITCH)) {  // Reads if the rotator button is pressed...
          do {
          } while (!RE3.digitalRead(SS_SWITCH));  //...end wait until released
          encoder_position3 = 0;
          RE3.setEncoderPosition(0);
          return;
        }
        break;
      case 4:
        if (!RE4.digitalRead(SS_SWITCH)) {  // Reads if the rotator button is pressed...
          do {
          } while (!RE4.digitalRead(SS_SWITCH));  //...end wait until released
          encoder_position4 = 0;
          RE4.setEncoderPosition(0);
          return;
        }
        break;
      default:
        break;
    }

    PrintStatus();
  } while (millis() < start + 1000);


  switch (RotatorEncoder) {
    case 1:
      encoder_position1 = 0;
      RE1.setEncoderPosition(0);
      break;
    case 2:
      encoder_position2 = 0;
      RE2.setEncoderPosition(0);
      break;
    case 3:
      encoder_position3 = 0;
      RE3.setEncoderPosition(0);
      break;
    case 4:
      encoder_position4 = 0;
      RE4.setEncoderPosition(0);
      break;
    default:
      break;
  }
}

int ReadContourWidth() {
  char a;
  String Result;

  FlushSerialInput();
  Serial2.print("EX030203;");
  delay(CommandDelay);

  Result = "";
  while (Serial2.available() > 0) {
    a = Serial2.read();
    Result = Result + a;
    if (a == ';') {
      Result = Result.substring(8, Result.length() - 1);
    }
  }
  return Result.toInt();
}

void SetContourWidth(int ContourWidth) {
  unsigned int i;
  String ContourText;

  if (ContourWidth < 10) {
    ContourText = "0" + String(ContourWidth);
  } else {
    ContourText = String(ContourWidth);
  }
  Serial2.print("EX030203" + ContourText + ";");
}

int ReadContourLevel() {
  char a;
  String Result;

  FlushSerialInput();
  Serial2.print("EX030202;");
  delay(CommandDelay);

  Result = "";
  while (Serial2.available() > 0) {
    a = Serial2.read();
    Result = Result + a;
    if (a == ';') {
      Result = Result.substring(8, Result.length() - 1);
    }
  }
  return Result.toInt();
}

void SetContourLevel(int ContourLevel) {
  unsigned int i;
  String ContourText;

  if (ContourLevel < -9) {
    ContourText = String(ContourLevel);
  } else if (ContourLevel < 0) {
    ContourText = "-0" + String(abs(ContourLevel));
  } else if (ContourLevel < 10) {
    ContourText = "+0" + String(ContourLevel);
  } else {
    ContourText = "+" + String(ContourLevel);
  }
  Serial2.print("EX030202" + ContourText + ";");
}

void PrintText(int x, int y, String text) {
  int16_t x1, y1;
  uint16_t w, h;

  /*tft.getTextBounds(text, x, y, &x1, &y1, &w, &h);
  tft.fillRect(x1, y1, w + 10, h, TFT_BACKGROUND);
  tft.setCursor(x, y);
  tft.print(text);*/
  tft.setTextColor(TFT_FOREGROUND, TFT_BACKGROUND);
  tft.setTextDatum(BL_DATUM);
  tft.drawString(text, x, y, 1);
}

void UpperPrintText(int x, int y, String text) {  //xl indicates leftmost pont, xr indicated rightmost point, y and text as normal
  Upper.setTextColor(TFT_FOREGROUND, TFT_BACKGROUND);
  Upper.setTextDatum(BL_DATUM);
  Upper.drawString(text, x, y, 1);
}

void LowerPrintText(int x, int y, String text) {  //xl indicates leftmost pont, xr indicated rightmost point, y and text as normal
  Lower.setTextColor(TFT_FOREGROUND, TFT_BACKGROUND);
  Lower.setTextDatum(BL_DATUM);
  Lower.drawString(text, x, y, 1);
}

void PrintTextCentered(int xl, int xr, int y, String text) {  //xl indicates leftmost pont, xr indicated rightmost point, y and text as normal
  tft.fillRect(xl, y - tft.fontHeight() + 2, xr - xl, tft.fontHeight(), TFT_BACKGROUND);
  tft.setTextColor(TFT_FOREGROUND, TFT_BACKGROUND);
  tft.setTextDatum(BC_DATUM);
  tft.drawString(text, xl + (xr - xl) / 2, y, 1);
}

void UpperPrintTextCentered(int xl, int xr, int y, String text) {  //xl indicates leftmost pont, xr indicated rightmost point, y and text as normal
  Upper.fillRect(xl, y - Upper.fontHeight() + 2, xr - xl, Upper.fontHeight(), TFT_BACKGROUND);
  Upper.setTextColor(TFT_FOREGROUND, TFT_BACKGROUND);
  Upper.setTextDatum(BC_DATUM);
  Upper.drawString(text, xl + (xr - xl) / 2, y, 1);
}

void LowerPrintTextCentered(int xl, int xr, int y, String text) {  //xl indicates leftmost pont, xr indicated rightmost point, y and text as normal
  Lower.fillRect(xl, y - Lower.fontHeight() + 2, xr - xl, Lower.fontHeight(), TFT_BACKGROUND);
  Lower.setTextColor(TFT_FOREGROUND, TFT_BACKGROUND);
  Lower.setTextDatum(BC_DATUM);
  Lower.drawString(text, xl + (xr - xl) / 2, y, 1);
}

int NormalizePO(int x) {
  //int PowerArray[23] = { 0, 29, 54, 69, 82, 97, 111, 120, 130, 139, 149, 157, 163, 171, 176, 184, 190, 197, 203, 210, 216, 223, 255 };  //My scale
  //int PowerArray[23] = { 0, 30, 43, 63, 79, 94, 105, 115, 125, 135, 145, 153, 159, 164, 169, 174, 179, 186, 193, 200, 206, 212, 255 };  //Optimistic scale
  //int PowerArray[23] = { 0, 23, 44, 67, 87, 103, 115, 130, 139, 147, 155, 164, 171, 176, 183, 191, 196, 203, 208, 213, 220, 227, 255 }; //What read from the m eter using the RF gain method
  //int PowerArray[23] = { 0, 30, 55, 69, 82, 97, 111, 120, 131, 141, 150, 160, 167, 174, 179, 187, 193, 198, 206, 212, 217, 225, 255 };  //What read from the m eter using the RTTY method
  int Array_end;
  int i, Result;

#if (RIG_Model == 'FTDX101MP')
  int PowerArray[23] = { 0, 30, 55, 69, 82, 97, 111, 120, 131, 141, 150, 160, 167, 174, 179, 187, 193, 198, 206, 212, 217, 225, 255 };
  Array_end = 22;
#else if (RIG_Model == 'FTDX101D')
  int PowerArray[13] = { 0, 32, 54, 82, 104, 128, 147, 159, 169, 183, 193, 200, 255 };
  Array_end = 12;
#endif

  for (i = Array_end; i >= 0; i--) {
    if (x >= PowerArray[i]) {
      Result = (x - PowerArray[i]) * 10 / (double(PowerArray[i + 1]) - double(PowerArray[i])) + (i - 1) * 10;
      break;
    }
  }
  if (i == 1) { Result = 5; }
  if (i == 0) { Result = 0; }
  return Result;
}

void SecondCoreTaskCode(void* pvParameters) {
  // In this core I execute all the tasks that are time sensitive or independent
  // The secondd releay is an independent task
  // The short/long press of the main button is time sensitive (to give good responsiveness)

  do {
    if (millis() > SecondRelayActivationMillis && millis() < SecondRelayDeactivationMillis && RLY1 == false) {
      Relay.SET_RLY1(I2C_REL_ADD, HIGH);
      RLY1 = true;
    }
    if (millis() > SecondRelayDeactivationMillis && RLY1 == true) {
      Relay.SET_RLY1(I2C_REL_ADD, LOW);
      RLY1 = false;
    }
    delay(CommandDelay);

    //Here we see if the Tune button is shortpressed and need to tune, or longpressed and open the sytem menu.
    if (digitalRead(Button) == 0 && ButtonShortPress == false && ButtonLongPress == false) {  //if the tune button is pressed and not short or long flags set (meaning the main code doesn't run a button related task)
      ButtonPressTime = millis();                                                             //Mark the time that the button was pressed...

      while (ButtonPressTime + 1000 > millis()) {  //...and we wait for 1 sec...
        if (digitalRead(Button) != 0) {
          delay(CommandDelay);  // Buffer to avoid switch bouncing
          break;
        }
      }
      if (digitalRead(Button) == 0) {  // ...and if the button is still pressed we set the ButtonLongPress to true in order to open the system menu
        ButtonLongPress = true;
      } else {  // ...otherwise,  we set the ButtonShortPress to true in order to activate tuning.
        ButtonShortPress = true;
      }
    }

  } while (true);
}

// Function to handle the display of a menu
int DisplayMenu(int count, int HighlightedMenu, int Control, ...) {  //Count = the number of items to display including the title. HighlightedMenu = The menu that should be highlighted. Control = the rottay controler that controls the menu
  va_list args;
  va_start(args, count);
  String Lines[10];  //Manimum number of menu lines
  int32_t new_position;
  int32_t encoder_position = encoder_position1;
  bool RotaryEncoderButtonPressed = false;
  int MenuTopPosition;  // Shows which menu option is on top (we have only 5 options to dsisplay).

  for (int i = 0; i < count; i++) {
    Lines[i] = va_arg(args, const char*);
  }

  bool Redraw = true;
  uint16_t SwapColors;

  if (HighlightedMenu <= 5) {  //If the initial highlighted option is in the first 5...
    MenuTopPosition = 1;
  } else {  //if not, adjust which is the first option to display
    MenuTopPosition = HighlightedMenu - 4;
    HighlightedMenu = 5;
  }

  TFT_FOREGROUND = TFT_YELLOW;
  TFT_BACKGROUND = TFT_EBONY;

  UpperClearDisplay();

  start = millis();

  do {
    if (Redraw == true) {
      TFT_FOREGROUND = TFT_YELLOW;
      TFT_BACKGROUND = TFT_EBONY;
      Upper.setFreeFont(&FreeSansBold12pt7b);
      UpperPrintTextCentered(0, 320, 43, "--------------------");
      UpperPrintTextCentered(0, 320, 28, Lines[0]);
      if (HighlightedMenu == 1) TFT_BACKGROUND = TFT_RED;
      UpperPrintTextCentered(0, 320, 65, Lines[MenuTopPosition]);
      TFT_BACKGROUND = TFT_EBONY;
      if (HighlightedMenu == 2) TFT_BACKGROUND = TFT_RED;
      UpperPrintTextCentered(0, 320, 91, Lines[MenuTopPosition + 1]);
      TFT_BACKGROUND = TFT_EBONY;
      if (HighlightedMenu == 3) TFT_BACKGROUND = TFT_RED;
      UpperPrintTextCentered(0, 320, 117, Lines[MenuTopPosition + 2]);
      TFT_BACKGROUND = TFT_EBONY;
      if (HighlightedMenu == 4) TFT_BACKGROUND = TFT_RED;
      UpperPrintTextCentered(0, 320, 143, Lines[MenuTopPosition + 3]);
      TFT_BACKGROUND = TFT_EBONY;
      if (HighlightedMenu == 5) TFT_BACKGROUND = TFT_RED;
      UpperPrintTextCentered(0, 320, 169, Lines[MenuTopPosition + 4]);
      TFT_BACKGROUND = TFT_EBONY;
      //Upper.drawRect(100, 117, 320 - 2 * 100, 1, TFT_YELLOW);
      Upper.pushSprite(0, 8);
      Redraw = false;
    }

    while (digitalRead(Button) == 0) {  //Loop while the button is pressed. This it to assure that we wait the button to be released after the first menu draw and beforwe we proceed further. The button is not used after this.
      delay(CommandDelay);
      start = millis();
    }

    switch (Control) {  // Reads the "current" position of the rotary encoder who controls the menu
      case 1:
        new_position = RotatorDirection * RE1.getEncoderPosition();
        break;
      case 2:
        new_position = RotatorDirection * RE2.getEncoderPosition();
        break;
      case 3:
        new_position = RotatorDirection * RE3.getEncoderPosition();
        break;
      case 4:
        new_position = RotatorDirection * RE4.getEncoderPosition();
        break;
      default:
        break;
    }
    if (new_position != encoder_position) {  //If the encoder has been moved
      if (new_position < encoder_position) {
        HighlightedMenu++;
        if (HighlightedMenu >= count) {  //In case we have fewer that 5 menu items
          HighlightedMenu = count - 1;
        }
        if (HighlightedMenu > 5) {
          HighlightedMenu = 5;
          MenuTopPosition++;
          if (MenuTopPosition > count - 5) MenuTopPosition = count - 5;  //-5 because we can display 5 lines of menu on the screen (expluding the menu title)
        }
      } else {
        HighlightedMenu--;
        if (HighlightedMenu < 1) {
          HighlightedMenu = 1;
          MenuTopPosition--;
          if (MenuTopPosition < 1) MenuTopPosition = 1;
        }
      }
      Redraw = true;
      start = millis();
    }
    encoder_position = new_position;  // Updates the previous encoder position with the current one

    switch (Control) {  // Set the correct encoder_positionX
      case 1:
        encoder_position1 = encoder_position;
        break;
      case 2:
        encoder_position2 = encoder_position;
        break;
      case 3:
        encoder_position3 = encoder_position;
        break;
      case 4:
        encoder_position4 = encoder_position;
        break;
      default:
        break;
    }


    //Here starts the code that checks rotary button press
    switch (Control) {
      case 1:
        if (!RE1.digitalRead(SS_SWITCH)) {  // If the rotary encoder's button is pressed
          do {
            delay(CommandDelay);
          } while (!RE1.digitalRead(SS_SWITCH));  //Loop while the rotary encoder's button is pressed
          RotaryEncoderButtonPressed = true;
        }
        break;
      case 2:
        if (!RE2.digitalRead(SS_SWITCH)) {  // If the rotary encoder's button is pressed
          do {
            delay(CommandDelay);
          } while (!RE2.digitalRead(SS_SWITCH));  //Loop while the rotary encoder's button is pressed
          RotaryEncoderButtonPressed = true;
        }
        break;
      case 3:
        if (!RE3.digitalRead(SS_SWITCH)) {  // If the rotary encoder's button is pressed
          do {
            delay(CommandDelay);
          } while (!RE3.digitalRead(SS_SWITCH));  //Loop while the rotary encoder's button is pressed
          RotaryEncoderButtonPressed = true;
        }
        break;
      case 4:
        if (!RE4.digitalRead(SS_SWITCH)) {  // If the rotary encoder's button is pressed
          do {
            delay(CommandDelay);
          } while (!RE4.digitalRead(SS_SWITCH));  //Loop while the rotary encoder's button is pressed
          RotaryEncoderButtonPressed = true;
        }
        break;
      default:
        break;
    }

    if (RotaryEncoderButtonPressed) {
      // Now forget any encoder rotation that happened while we were in the Menu
      RE1.setEncoderPosition(0);
      RE2.setEncoderPosition(0);
      RE3.setEncoderPosition(0);
      RE4.setEncoderPosition(0);
      encoder_position1 = 0;
      encoder_position2 = 0;
      encoder_position3 = 0;
      encoder_position4 = 0;

      va_end(args);
      return MenuTopPosition + HighlightedMenu - 1;
    }
    PrintStatus();
  } while (millis() < start + 5000);

  // Now forget any encoder rotation that happened while we were in the Menu
  RE1.setEncoderPosition(0);
  RE2.setEncoderPosition(0);
  RE3.setEncoderPosition(0);
  RE4.setEncoderPosition(0);
  encoder_position1 = 0;
  encoder_position2 = 0;
  encoder_position3 = 0;
  encoder_position4 = 0;

  Message = 0;
  va_end(args);
  return 0;
}

void DisplayURC(void) {
  String Result;
  long Frequency;

  URC.fillRect(0, 0, URC.width(), URC.height(), TFT_EBONY);
  URC.setFreeFont(&FreeSansBold12pt7b);
  URC.setTextColor(TFT_WHITE, TFT_EBONY);
  URC.setTextDatum(BR_DATUM);

  switch (ExtendedParameter4) {
    case 1:
      Result = "SQL " + String(ReadSQL());
      break;
    case 2:
      Result = "MEM " + String(ReadMemory());
      break;
    case 3:
      if (ReadNotchWidth() == 0) {
        Result = "Notch N";
      } else {
        Result = "Notch W";
      }
      break;
    case 4:  //Contour width
      Result = "Cont. W " + String(ReadContourWidth());
      if (ReadContourWidth() == 10) {
        URC.setTextColor(TFT_YELLOW, TFT_EBONY);
      } else {
        URC.setTextColor(TFT_WHITE, TFT_EBONY);
      }
      break;
    case 5:  //Contour level
      if (ReadContourLevel() == -15) {
        URC.setTextColor(TFT_YELLOW, TFT_EBONY);
      } else {
        URC.setTextColor(TFT_WHITE, TFT_EBONY);
      }
      Result = "Cont. L " + String(ReadContourLevel());
      break;
    case 6:  //Power level
      Result = "Power " + String(ReadPower()) + "W";
      break;
    case 7:  //Frequency
      CurrentFrequencyRX = ReadFrequency(VFORead());
      if (CurrentFrequencyRX > 9999999) {
        Result = "F:" + String(CurrentFrequencyRX).substring(0, 2) + "." + String(CurrentFrequencyRX).substring(2, 5) + "." + String(CurrentFrequencyRX).substring(5, 8);
      } else {
        Result = "F:" + String(CurrentFrequencyRX).substring(0, 1) + "." + String(CurrentFrequencyRX).substring(1, 4) + "." + String(CurrentFrequencyRX).substring(4, 7);
      }
      break;
    case 8:  //Frequency Main
      Frequency = ReadFrequency(0);
      if (Frequency > 9999999) {
        Result = "M:" + String(Frequency).substring(0, 2) + "." + String(Frequency).substring(2, 5) + "." + String(Frequency).substring(5, 8);
      } else {
        Result = "M:" + String(Frequency).substring(0, 1) + "." + String(Frequency).substring(1, 4) + "." + String(Frequency).substring(4, 7);
      }
      break;
    case 9:  //Frequency Sub
      Frequency = ReadFrequency(1);
      if (Frequency > 9999999) {
        Result = "S:" + String(Frequency).substring(0, 2) + "." + String(Frequency).substring(2, 5) + "." + String(Frequency).substring(5, 8);
      } else {
        Result = "S:" + String(Frequency).substring(0, 1) + "." + String(Frequency).substring(1, 4) + "." + String(Frequency).substring(4, 7);
      }
      break;
    default:
      break;
  }
  URC.drawString(Result, 150, 30, 1);
  URC.pushSprite(165, 8);
}

void DisplayLRC(void) {
  String Result;
  long Frequency;

  LRC.fillRect(0, 0, LRC.width(), LRC.height(), TFT_EBONY);
  LRC.setFreeFont(&FreeSansBold12pt7b);
  LRC.setTextColor(TFT_WHITE, TFT_EBONY);
  LRC.setTextDatum(BR_DATUM);

  switch (ExtendedParameter3) {
    case 1:
      Result = "SQL " + String(ReadSQL());
      break;
    case 2:
      Result = "MEM " + String(ReadMemory());
      break;
    case 3:
      if (ReadNotchWidth() == 0) {
        Result = "Notch N";
      } else {
        Result = "Notch W";
      }
      break;
    case 4:  //Contour width
      Result = "Cont. W " + String(ReadContourWidth());
      if (ReadContourWidth() == 10) {
        LRC.setTextColor(TFT_YELLOW, TFT_EBONY);
      } else {
        LRC.setTextColor(TFT_WHITE, TFT_EBONY);
      }
      break;
    case 5:  //Contour level
      if (ReadContourLevel() == -15) {
        LRC.setTextColor(TFT_YELLOW, TFT_EBONY);
      } else {
        LRC.setTextColor(TFT_WHITE, TFT_EBONY);
      }
      Result = "Cont. L " + String(ReadContourLevel());
      break;
    case 6:  //Power level
      Result = "Power " + String(ReadPower()) + "W";
      break;
    case 7:  //Frequency
      CurrentFrequencyRX = ReadFrequency(VFORead());
      if (CurrentFrequencyRX > 9999999) {
        Result = "F:" + String(CurrentFrequencyRX).substring(0, 2) + "." + String(CurrentFrequencyRX).substring(2, 5) + "." + String(CurrentFrequencyRX).substring(5, 8);
      } else {
        Result = "F:" + String(CurrentFrequencyRX).substring(0, 1) + "." + String(CurrentFrequencyRX).substring(1, 4) + "." + String(CurrentFrequencyRX).substring(4, 7);
      }
      break;
    case 8:  //Frequency Main
      Frequency = ReadFrequency(0);
      if (Frequency > 9999999) {
        Result = "M:" + String(Frequency).substring(0, 2) + "." + String(Frequency).substring(2, 5) + "." + String(Frequency).substring(5, 8);
      } else {
        Result = "M:" + String(Frequency).substring(0, 1) + "." + String(Frequency).substring(1, 4) + "." + String(Frequency).substring(4, 7);
      }
      break;
    case 9:  //Frequency Sub
      Frequency = ReadFrequency(1);
      if (Frequency > 9999999) {
        Result = "S:" + String(Frequency).substring(0, 2) + "." + String(Frequency).substring(2, 5) + "." + String(Frequency).substring(5, 8);
      } else {
        Result = "S:" + String(Frequency).substring(0, 1) + "." + String(Frequency).substring(1, 4) + "." + String(Frequency).substring(4, 7);
      }
      break;
    default:
      break;
  }
  LRC.drawString(Result, 150, 30, 1);
  LRC.pushSprite(165, 150);
}

void DisplayULC(void) {
  String Result;
  long Frequency;

  ULC.fillRect(0, 0, ULC.width(), ULC.height(), TFT_EBONY);
  ULC.setFreeFont(&FreeSansBold12pt7b);
  ULC.setTextColor(TFT_WHITE, TFT_EBONY);
  ULC.setTextDatum(BL_DATUM);

  switch (ExtendedParameter1) {
    case 1:
      Result = "SQL " + String(ReadSQL());
      break;
    case 2:
      Result = "MEM " + String(ReadMemory());
      break;
    case 3:
      if (ReadNotchWidth() == 0) {
        Result = "Notch N";
      } else {
        Result = "Notch W";
      }
      break;
    case 4:  //Contour width
      Result = "Cont. W " + String(ReadContourWidth());
      if (ReadContourWidth() == 10) {
        ULC.setTextColor(TFT_YELLOW, TFT_EBONY);
      } else {
        ULC.setTextColor(TFT_WHITE, TFT_EBONY);
      }
      break;
    case 5:  //Contour level
      if (ReadContourLevel() == -15) {
        ULC.setTextColor(TFT_YELLOW, TFT_EBONY);
      } else {
        ULC.setTextColor(TFT_WHITE, TFT_EBONY);
      }
      Result = "Cont. L " + String(ReadContourLevel());
      break;
    case 6:  //Power level
      Result = "Power " + String(ReadPower()) + "W";
      break;
    case 7:  //Frequency
      CurrentFrequencyRX = ReadFrequency(VFORead());
      if (CurrentFrequencyRX > 9999999) {
        Result = "F:" + String(CurrentFrequencyRX).substring(0, 2) + "." + String(CurrentFrequencyRX).substring(2, 5) + "." + String(CurrentFrequencyRX).substring(5, 8);
      } else {
        Result = "F:" + String(CurrentFrequencyRX).substring(0, 1) + "." + String(CurrentFrequencyRX).substring(1, 4) + "." + String(CurrentFrequencyRX).substring(4, 7);
      }
      break;
    case 8:  //Frequency Main
      Frequency = ReadFrequency(0);
      if (Frequency > 9999999) {
        Result = "M:" + String(Frequency).substring(0, 2) + "." + String(Frequency).substring(2, 5) + "." + String(Frequency).substring(5, 8);
      } else {
        Result = "M:" + String(Frequency).substring(0, 1) + "." + String(Frequency).substring(1, 4) + "." + String(Frequency).substring(4, 7);
      }
      break;
    case 9:  //Frequency Sub
      Frequency = ReadFrequency(1);
      if (Frequency > 9999999) {
        Result = "S:" + String(Frequency).substring(0, 2) + "." + String(Frequency).substring(2, 5) + "." + String(Frequency).substring(5, 8);
      } else {
        Result = "S:" + String(Frequency).substring(0, 1) + "." + String(Frequency).substring(1, 4) + "." + String(Frequency).substring(4, 7);
      }
      break;
    default:
      break;
  }
  ULC.drawString(Result, 0, 30, 1);
  ULC.pushSprite(5, 8);
}

void DisplayLLC(void) {
  String Result;
  long Frequency;

  LLC.fillRect(0, 0, LLC.width(), LLC.height(), TFT_EBONY);
  LLC.setFreeFont(&FreeSansBold12pt7b);
  LLC.setTextColor(TFT_WHITE, TFT_EBONY);
  LLC.setTextDatum(BL_DATUM);

  switch (ExtendedParameter2) {
    case 1:
      Result = "SQL " + String(ReadSQL());
      break;
    case 2:
      Result = "MEM " + String(ReadMemory());
      break;
    case 3:
      if (ReadNotchWidth() == 0) {
        Result = "Notch N";
      } else {
        Result = "Notch W";
      }
      break;
    case 4:  //Contour width
      Result = "Cont. W " + String(ReadContourWidth());
      if (ReadContourWidth() == 10) {
        LLC.setTextColor(TFT_YELLOW, TFT_EBONY);
      } else {
        LLC.setTextColor(TFT_WHITE, TFT_EBONY);
      }
      break;
    case 5:  //Contour level
      if (ReadContourLevel() == -15) {
        LLC.setTextColor(TFT_YELLOW, TFT_EBONY);
      } else {
        LLC.setTextColor(TFT_WHITE, TFT_EBONY);
      }
      Result = "Cont. L " + String(ReadContourLevel());
      break;
    case 6:  //Power level
      Result = "Power " + String(ReadPower()) + "W";
      break;
    case 7:  //Frequency
      CurrentFrequencyRX = ReadFrequency(VFORead());
      if (CurrentFrequencyRX > 9999999) {
        Result = "F:" + String(CurrentFrequencyRX).substring(0, 2) + "." + String(CurrentFrequencyRX).substring(2, 5) + "." + String(CurrentFrequencyRX).substring(5, 8);
      } else {
        Result = "F:" + String(CurrentFrequencyRX).substring(0, 1) + "." + String(CurrentFrequencyRX).substring(1, 4) + "." + String(CurrentFrequencyRX).substring(4, 7);
      }
      break;
    case 8:  //Frequency Main
      Frequency = ReadFrequency(0);
      if (Frequency > 9999999) {
        Result = "M:" + String(Frequency).substring(0, 2) + "." + String(Frequency).substring(2, 5) + "." + String(Frequency).substring(5, 8);
      } else {
        Result = "M:" + String(Frequency).substring(0, 1) + "." + String(Frequency).substring(1, 4) + "." + String(Frequency).substring(4, 7);
      }
      break;
    case 9:  //Frequency Sub
      Frequency = ReadFrequency(1);
      if (Frequency > 9999999) {
        Result = "S:" + String(Frequency).substring(0, 2) + "." + String(Frequency).substring(2, 5) + "." + String(Frequency).substring(5, 8);
      } else {
        Result = "S:" + String(Frequency).substring(0, 1) + "." + String(Frequency).substring(1, 4) + "." + String(Frequency).substring(4, 7);
      }
      break;
    default:
      break;
  }
  LLC.drawString(Result, 0, 30, 1);
  LLC.pushSprite(5, 150);
}

void FrCheck(void) {  //Check if the current TX Frequency conflicts with the band plan
  String Conf = "Test";
  long CurrentFrequencyTXStart;
  long CurrentFrequencyTXEnd;
  long ConflictFrStart;
  long ConflictFrEnd;
  int i;
  bool Found;

  TFT_FOREGROUND = TFT_YELLOW;

  String Mode = ReadMode(MAINSUBTX);

  MAINSUBTX = ReadTX();
  CurrentFrequencyTX = ReadFrequency(MAINSUBTX);

  if (Mode == "1" || Mode == "8") {
    CurrentFrequencyTXStart = CurrentFrequencyTX - 3000;
    CurrentFrequencyTXEnd = CurrentFrequencyTX;
  } else if (Mode == "2" || Mode == "C") {
    CurrentFrequencyTXStart = CurrentFrequencyTX;
    CurrentFrequencyTXEnd = CurrentFrequencyTX + 3000;
  }


  Found = false;
  for (i = 0; i < 33; i++) {

    if (ConflictFr[i][1] == 1) {
      ConflictFrStart = ConflictFr[i][0] - 2999;
      ConflictFrEnd = ConflictFr[i][0];
    } else if (ConflictFr[i][1] == 2) {
      ConflictFrStart = ConflictFr[i][0];
      ConflictFrEnd = ConflictFr[i][0] + 2999;
    }

    if ((CurrentFrequencyTXStart >= ConflictFrStart && CurrentFrequencyTXStart <= ConflictFrEnd) || (CurrentFrequencyTXEnd <= ConflictFrEnd && CurrentFrequencyTXEnd >= ConflictFrStart)) {
      Upper.setFreeFont(&FreeSansBold12pt7b);
      UpperPrintTextCentered(0, 320, 110, "Used by : " + ConflictText[i]);
      i = 33;
      Found = true;
    };
  }
  if (Found == false) {
    Upper.setFreeFont(&FreeSansBold12pt7b);
    UpperPrintTextCentered(0, 320, 110, "                   ");
  }
  Upper.pushSprite(0, 38);
}