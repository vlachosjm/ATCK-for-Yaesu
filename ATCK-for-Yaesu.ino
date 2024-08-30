/////////////////////////////////////////////////
//         ATCK for Yaesu FTDX101MP/D          //
//        For instructions and updates         //
//           check the github page             //
// https://github.com/vlachosjm/ATCK-for-Yaesu //
/////////////////////////////////////////////////

//The external automatic tuner needs to be connected to the ANT1 output of the tranceiver
//The transceiver's '232 Rate' should be set to '38400bps'
//The transceiver's 'TUNER SELECT' should be set to 'INT'


#include <Adafruit_seesaw.h>  //The library needed for the Adafruit I2C rotary encoders
#include <TFT_eSPI.h>         //To handle the TFT screen
#include <mLink.h>            //The library needed for the I2C relays from Hobby Components Ltd
#include <Preferences.h>      //The library needed to store and retrieve data from the on-board non-volatile memory

#define BAUD_RATE 38400  //Sets the speed of the communication with the transceiver
#define RXPIN 4          //RX pin for the serial communication with the transceiver
#define TXPIN 5          //TX pin for the serial communication with the transceiver
#define Button 1         //Tuner button.
#define TXGND 7          //Connected to the TX-GND jack of the tranceiver. It's low when the transceiver transmits

#define CommandDelay 12  //Number of milliseconds to wait after we send a coomand to the transceiver

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

Preferences preferences;  //Define the oncject to hadndle the read/write to the on-board non-volatile memory

Adafruit_seesaw RE1;  //Define the object of the 1st rotary encoder
Adafruit_seesaw RE2;  //Define the object of the 2nd rotary encoder
Adafruit_seesaw RE3;  //Define the object of the 3rd rotary encoder
Adafruit_seesaw RE4;  //Define the object of the 4th rotary encoder

mLink Relay;  //Define the object for the relay

TFT_eSPI tft = TFT_eSPI();              //Defive the object to handle the TFT display
TFT_eSprite Upper = TFT_eSprite(&tft);  //Define a sprite for the upper part of the display
TFT_eSprite Lower = TFT_eSprite(&tft);  //Define a sprite for the lower part of the display

boolean Tuned = false;  // True if in a specific range around the last tuned frequency, false if otherwise

unsigned long InfoDelay;  // The application should wait until that time to show another info on the upper screen (in milliseconds from the start of the initilization of the ATCK device)
int Message = 0;          // Each info message has a number. We keep here the number of the message that shows on the display

long CurrentFrequencyTX = 0;   //The Current frequency of the VFO that has the TX
long LastTunedFrequency = 0;   //The Last Tuned Frequency for the external tuner.
long LastFailedFrequency = 0;  //The Last Failed Frequency for the external tuner.

int SWR;              //The last SWR reading
int Steps;            //The steping for changing the frequency using the rotaring the encoder
int MAINSUBTX = 0;    //Main (0) or sub (1) for transmit
int MAINSUBRX = 0;    //Main (0) or sub (1) for receive
int SelectedFilter;   //The selected filter. Default = 0 (None)
int TransmitAntenna;  //Number of the antenna used for transmition

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

//Loading the menu data on a constant array
const char* SubMenus[18][3] = { { "1", "100Hz", "100" }, { "1", "250Hz", "250" }, { "1", "500Hz", "500" }, { "1", "1KHz", "1000" }, { "2", "  None ", "0" }, { "2", " 12 KHz", "1" }, { "2", "  3 KHz", "2" }, { "2", " 600 Hz", "4" }, { "2", " 300 Hz", "5" }, { "3", "5W", "5" }, { "3", "20W", "20" }, { "3", "40W", "40" }, { "3", "100W", "100" }, { "3", "200W", "200" }, { "4", "", "" }, { "4", "", "" }, { "5", "No", "0" }, { "5", "Yes", "1" } };

unsigned long ONAirStartTime;  //Marks the time that on air activity started
long ONAirTime;                //Actual time on air in minutes

int PPO;  //Peak Power Out

volatile unsigned long start;  //This value is used when we are in a menu, to mark the the start of inactivity time before we auto-exit the menu

int ExtendedParameter1;  // Shows the parameter that can be modified with the specific encoder: 1 - Squelch, 2 - Memory channel, 3 - Notch filter width, 4 - Contour Width, 5 - Contour Level, 6- Power Level, 7- Frequency
int ExtendedParameter2;  // Shows the parameter that can be modified with the specific encoder: 1 - Squelch, 2 - Memory channel, 3 - Notch filter width, 4 - Contour Width, 5 - Contour Level, 6- Power Level, 7- Frequency
int ExtendedParameter3;  // Shows the parameter that can be modified with the specific encoder: 1 - Squelch, 2 - Memory channel, 3 - Notch filter width, 4 - Contour Width, 5 - Contour Level, 6- Power Level, 7- Frequency
int ExtendedParameter4;  // Shows the parameter that can be modified with the specific encoder: 1 - Squelch, 2 - Memory channel, 3 - Notch filter width, 4 - Contour Width, 5 - Contour Level, 6- Power Level, 7- Frequency

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
  preferences.end();

  //Serial.begin(115200);                                //Initiate the serial monitor port
  Serial2.begin(BAUD_RATE, SERIAL_8N1, RXPIN, TXPIN);  //Initiate Serial port
  Wire.begin(8, 9);                                    //Initiate the I2C
  Relay.init();                                        //Initalize the relay module
  RE1.begin(SEESAW_ADDR1);                             //Initiate the 1st rotary encoder
  RE2.begin(SEESAW_ADDR2);                             //Initiate the 2nd rotary encoder
  RE3.begin(SEESAW_ADDR3);                             //Initiate the 3rd rotary encoder
  RE4.begin(SEESAW_ADDR4);                             //Initiate the 4th rotary encoder
  tft.init();                                          //Initiate the tft display

  Upper.createSprite(320, 172);  //Create the sprite for the upper part of the display
  Lower.createSprite(280, 29);   //Create the sprite for the lower part of the display

  tft.setRotation(1);
  tft.setTextWrap(false);
  tft.fillScreen(TFT_BLACK);

  Upper.fillRect(0, 8, 320, 172, TFT_YELLOW);
  Upper.setFreeFont(&FreeSansBold18pt7b);
  TFT_FOREGROUND = TFT_BLACK;
  TFT_BACKGROUND = TFT_YELLOW;
  UpperPrintTextCentered(0, 320, 50, "ATCK");
  UpperPrintTextCentered(0, 320, 90, "for");
  if (RIG_Model == 'FTDX101MP') {
    UpperPrintTextCentered(0, 320, 130, "Yaesu FTDX101MP");
  } else if (RIG_Model == 'FTDX101D') {
    UpperPrintTextCentered(0, 320, 130, "Yaesu FTDX101D");
  } else {
    UpperPrintTextCentered(0, 320, 130, "unknown model");
  }
  Upper.setFreeFont(&FreeSansBold12pt7b);
  UpperPrintTextCentered(0, 320, 170, "---by SV1RQJ/F4VTR---");
  Upper.pushSprite(0, 8);
  delay(2000);

  tft.setTextColor(TFT_YELLOW);
  TFT_BACKGROUND = TFT_EBONY;

  attachInterrupt(digitalPinToInterrupt(TXGND), Interupt1, CHANGE);  //Setup the interrupt routine to call and the condition to call it
}

void loop() {
  char a;                            //Generic character string
  String Result;                     //Generic results storage
  String LockStatus;                 //Lock status
  boolean ButtonShortPress = false;  // Checks in the tune button has been pressed for a long time (>1 sec?)
  unsigned long ButtonPressTime;     //The point in time where the button is pressed
  String TimeText;                   //To hold the on air time string
  int MPO;                           //Max Power Out

  int32_t new_position1 = RotatorDirection * RE1.getEncoderPosition();  // If 1st encoder moved
  if (encoder_position1 != new_position1) {
    Read1stEncoder();  //To manage the turn of the encoder when we are not in a menu
  }

  int32_t new_position2 = RotatorDirection * RE2.getEncoderPosition();  // If 2nd encoder moved
  if (encoder_position2 != new_position2) {
    Read2ndEncoder();  //To manage the turn of the 2nd encoder
  }

  int32_t new_position3 = RotatorDirection * RE3.getEncoderPosition();  // If 3rd encoder moved
  if (encoder_position3 != new_position3) {
    Read3rdEncoder();  //To manage the turn of the 3rd encoder
  }

  int32_t new_position4 = RotatorDirection * RE4.getEncoderPosition();  // If 4th encoder moved
  if (encoder_position4 != new_position4) {
    Read4thEncoder();  //To manage the turn of the 4th encoder
  }

  //Check if an encoder's button is pressed
  if (!RE1.digitalRead(SS_SWITCH)) {
    MenuHandle_1stEncoder();
  }

  if (!RE2.digitalRead(SS_SWITCH)) {
    MenuHandle_2ndEncoder();
  }

  if (!RE3.digitalRead(SS_SWITCH)) {
    MenuHandle_3rdEncoder();
  }

  if (!RE4.digitalRead(SS_SWITCH)) {
    MenuHandle_4thEncoder();
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

    //Here we see if the Tune button is longpressed and need to tune, or shortpressed. I don't have an action assigned to short press
    if (digitalRead(Button) == 0) {  //if the tune button is pressed...
      ButtonPressTime = millis();    //Mark the time that the button was pressed...

      while (ButtonPressTime + 1000 > millis()) {  //...and we wait for 1000ms...
        if (digitalRead(Button) != 0) {
          delay(CommandDelay);  // Buffer to settle the physical button move
          break;
        }
      }
      if (digitalRead(Button) == 0) {  // ...and if the button is still pressed we set the ButtonShortPress to false and we open the system menu
        ButtonShortPress = false;
        SystemMenu();
      } else {  // ...otherwise we set the ButtonShortPress to false and proceed to tuning.
        ButtonShortPress = true;
      }
    }

    //If the transmniter transmits in a non-tuned frequency (or the tune button is pressed) while the transmit is in the allowed bands...
    if (((digitalRead(TXGND) == 0 && !Tuned) || ButtonShortPress) && ((CurrentFrequencyTX >= 1800000 && CurrentFrequencyTX < 2000000) || (CurrentFrequencyTX >= 3500000 && CurrentFrequencyTX < 3800000) || (CurrentFrequencyTX >= 5351500 && CurrentFrequencyTX < 5366500) || (CurrentFrequencyTX >= 7000000 && CurrentFrequencyTX < 7200000) || (CurrentFrequencyTX >= 10100000 && CurrentFrequencyTX < 10150000) || (CurrentFrequencyTX >= 14000000 && CurrentFrequencyTX < 14350000) || (CurrentFrequencyTX >= 18068000 && CurrentFrequencyTX < 18168000) || (CurrentFrequencyTX >= 21000000 && CurrentFrequencyTX < 21450000) || (CurrentFrequencyTX >= 24890000 && CurrentFrequencyTX < 24990000) || (CurrentFrequencyTX >= 28000000 && CurrentFrequencyTX < 29700000) || (CurrentFrequencyTX >= 50000000 && CurrentFrequencyTX < 52000000))) {
      Serial2.print("TX0;");  //Stop transmition due to CAT
      delay(CommandDelay);
      Serial2.print("MX0;");  //Stop transmition due to MOX
      delay(CommandDelay);
      Relay.SET_RLY0(I2C_REL_ADD, HIGH);  //Disconnect the PTT button (Stop transmition due to Mic)

      //I lock the dial in order to avoid accidental change of frequency while tuning
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

      ActivateExternalTuner();  //Start tune process for external tuner

      //Restore dial lock status
      Serial2.print("LK" + LockStatus + ";");
      delay(CommandDelay);

      Relay.SET_RLY0(I2C_REL_ADD, LOW);             //Re-connect the PTT button
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
      UpperPrintText(8, 100, "ON AIR");

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
        UpperPrintText(195, 100, TimeText);

        MPO = ReadPowerOut();
        if (MPO > PPO) PPO = MPO;

        Upper.setFreeFont(&FreeSansBold12pt7b);
        UpperPrintText(180, 140, String(NormalizePO(PPO)) + " W ");
        UpperPrintText(110, 140, "PEP :");
        Upper.pushSprite(0, 8);
      } while (digitalRead(TXGND) == 0);

      Upper.setFreeFont(&FreeSansBold24pt7b);
      UpperPrintText(8, 100, "Air time");
      Upper.pushSprite(0, 8);
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
      UpperPrintTextCentered(0, 320, 60, "No tuner");
      UpperPrintTextCentered(0, 320, 110, "on this");
      UpperPrintTextCentered(0, 320, 160, "antenna!");
      Upper.pushSprite(0, 8);
      Message = 5;
    }
  } else {
    if (Message != 4) {
      TFT_BACKGROUND = TFT_EBONY;
      TFT_FOREGROUND = TFT_YELLOW;
      UpperClearDisplay();
      Upper.setFreeFont(&FreeSansBold18pt7b);
      UpperPrintTextCentered(0, 320, 36, "Communication");
      UpperPrintTextCentered(0, 320, 69, "error. Pls check");
      UpperPrintTextCentered(0, 320, 102, "your serial cable");
      UpperPrintTextCentered(0, 320, 135, "and set the 232C");
      UpperPrintTextCentered(0, 320, 168, "rate to 38400bps");
      Upper.drawRect(0, 0, 320, 172, TFT_RED);
      Upper.drawRect(1, 1, 318, 170, TFT_RED);
      Upper.pushSprite(0, 8);
      Message = 4;
    }
    PrintStatus();
  }
}

boolean IsInTunedFrequencies(long x) {  //Check if we are in the range of the last tuned frquency

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

void ActivateExternalTuner() {  //This function activates internal tuner, check that thuning is done and shifts/stores the tuned frequencies

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
  PrintTextCentered(0, 320, 100, "Tuning...");

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
      PrintTextCentered(0, 320, 130, "SWR ~1.1");
    } else if (SWR <= 26) {
      TFT_FOREGROUND = TFT_GREEN;
      PrintTextCentered(0, 320, 130, "SWR ~1.2");
    } else if (SWR <= 39) {
      TFT_FOREGROUND = TFT_GREEN;
      PrintTextCentered(0, 320, 130, "SWR ~1.5");
    } else if (SWR <= 80) {
      TFT_FOREGROUND = TFT_YELLOW;
      PrintTextCentered(0, 320, 130, "SWR < 2");
    }

    tft.setFreeFont(&FreeSansBold24pt7b);
    PrintTextCentered(0, 320, 100, "TUNED");

    LastTunedFrequency = CurrentFrequencyTX;  //LastTunedFrequency is used by external tuner.
    LastFailedFrequency = 0;
    Message = 0;
    InfoDelay = millis() + 2000;
  }
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
    UpperPrintTextCentered(0, 320, 90, "NOT");
    UpperPrintTextCentered(0, 320, 135, "TUNED");
    Upper.pushSprite(0, 8);
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
    UpperPrintTextCentered(0, 320, 100, "TUNED");
    Upper.pushSprite(0, 8);
    Message = 2;
  }

  PrintStatus();
}

void PrintStatus(void) {
  int a, b, c, d, i = 0;
  unsigned long Timer = millis();
  boolean BigChars = true;

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
          UpperPrintTextCentered(0, 320, 100, "PWR: " + String(a) + "W");
          Upper.pushSprite(0, 8);
          Timer = millis();
        }
        Lower.fillRect(0, 0, Lower.width(), Lower.height(), TFT_BLACK);
        Lower.setFreeFont(&FreeSansBold12pt7b);
        TFT_BACKGROUND = TFT_BLACK;
        LowerPrintText(1, 28, "PWR: " + String(a) + "W");
        Lower.pushSprite(0, 181);
        PreviousPower = a;
      }
      if (a == 0) Timer = 0;  //If the power = 0 then no need to wait
    } while (Timer + 800 > millis());
    if (BigChars == true) Message = 0;  //To allow to print another message over big characters
  }
}

void SystemMenu() {
  start = millis();
  int HighlightedMenu = 1;  //The menu that is highlighted
  bool Redraw = true;
  uint16_t SwapColors;

  do {
    delay(CommandDelay);
  } while (!RE1.digitalRead(SS_SWITCH));  //Loop while the rotary encoder's button is pressed

  TFT_FOREGROUND = TFT_YELLOW;
  TFT_BACKGROUND = TFT_EBONY;
  UpperClearDisplay();

  do {
    if (Redraw == true) {
      Upper.setFreeFont(&FreeSansBold12pt7b);
      UpperPrintTextCentered(0, 320, 43, "--------------------");
      UpperPrintTextCentered(0, 320, 28, "System Menu");
      if (HighlightedMenu == 1) TFT_BACKGROUND = TFT_RED;
      UpperPrintTextCentered(0, 320, 65, "Frequency steps");
      TFT_BACKGROUND = TFT_EBONY;
      if (HighlightedMenu == 2) TFT_BACKGROUND = TFT_RED;
      UpperPrintTextCentered(0, 320, 91, "Roffing filter");
      TFT_BACKGROUND = TFT_EBONY;
      if (HighlightedMenu == 3) TFT_BACKGROUND = TFT_RED;
      UpperPrintTextCentered(0, 320, 117, "RF Power");
      TFT_BACKGROUND = TFT_EBONY;
      if (HighlightedMenu == 4) TFT_BACKGROUND = TFT_RED;
      UpperPrintTextCentered(0, 320, 143, " ");
      TFT_BACKGROUND = TFT_EBONY;
      if (HighlightedMenu == 5) TFT_BACKGROUND = TFT_RED;
      UpperPrintTextCentered(0, 320, 169, "Save parameters");
      TFT_BACKGROUND = TFT_EBONY;
      Upper.drawRect(100, 117, 320 - 2 * 100, 1, TFT_YELLOW);
      Upper.pushSprite(0, 8);
      Redraw = false;
    }

    int32_t new_position1 = RotatorDirection * RE1.getEncoderPosition();  // Reads the "current" position of the rotary encoder
    if (new_position1 != encoder_position1) {                             //If the encoder has been moved
      if (new_position1 < encoder_position1) {
        HighlightedMenu++;
        if (HighlightedMenu > 5) HighlightedMenu = 5;
      } else {
        HighlightedMenu--;
        if (HighlightedMenu < 1) HighlightedMenu = 1;
      }
      Redraw = true;
      start = millis();
    }
    encoder_position1 = new_position1;  // Updates the previous encoder position with the current one

    if (!RE1.digitalRead(SS_SWITCH)) {  // If the rotary encoder's button is pressed
      do {
        delay(CommandDelay);
      } while (!RE1.digitalRead(SS_SWITCH));  //Loop while the rotary encoder's button is pressed
      switch (HighlightedMenu) {
        case 1:
          SubMenu("1");
          break;
        case 2:
          SubMenu("2");
          break;
        case 3:
          SubMenu("3");
          break;
        case 4:
          //SubMenu("4");
          break;
        case 5:
          SubMenu("5");
          break;

        default:
          break;
      }
    }
  } while (millis() < start + 5000);
  Message = 0;
}

void SubMenu(String smenu) {
  start = millis();
  int HighlightedMenu = 1;
  bool Redraw = true;
  int i, j;
  int NumberOfItems = 0;
  String GenericText = "";
  int timeout = 3000;

  UpperClearDisplay();
  do {
    if (Redraw == true) {
      NumberOfItems = 0;
      UpperPrintTextCentered(0, 320, 45, "----------------");
      if (smenu == "1") {
        UpperPrintTextCentered(0, 320, 28, "Frequency steps");
      } else if (smenu == "2") {
        UpperPrintTextCentered(0, 320, 28, "Roffing filter");
      } else if (smenu == "3") {
        UpperPrintTextCentered(0, 320, 28, "RF Power");
      } else if (smenu == "4") {
        UpperPrintTextCentered(0, 320, 28, " ");
      } else if (smenu == "5") {
        UpperPrintTextCentered(0, 320, 28, "Save parameters");
      }

      for (i = 0; i < 18; i++) {
        if (String(SubMenus[i][0]) == smenu) {
          if (NumberOfItems + 1 == HighlightedMenu) TFT_BACKGROUND = TFT_RED;
          UpperPrintTextCentered(0, 320, NumberOfItems * 26 + 65, SubMenus[i][1]);
          TFT_BACKGROUND = TFT_EBONY;
          NumberOfItems++;
        }
      }
      Upper.pushSprite(0, 8);
      Redraw = false;
    }

    int32_t new_position1 = RotatorDirection * RE1.getEncoderPosition();  // Reads the current position of the rotary encoder
    if (new_position1 != encoder_position1) {                             //If the encoder has been moved
      if (new_position1 < encoder_position1) {
        HighlightedMenu++;
        if (HighlightedMenu > NumberOfItems) HighlightedMenu = NumberOfItems;
      } else {
        HighlightedMenu--;
        if (HighlightedMenu < 1) HighlightedMenu = 1;
      }
      Redraw = true;
      start = millis();
    }
    encoder_position1 = new_position1;  // Updates the current position of the rotary encoder


    //Now we will see if the button was pressed and will take action
    if (!RE1.digitalRead(SS_SWITCH)) {  // Here we see if the button was pressed
      do {
        delay(CommandDelay);
      } while (!RE1.digitalRead(SS_SWITCH));  //Loop while it is pressed

      //Now we search which option we have actually chosen
      j = 0;
      for (i = 0; i < 19; i++) {
        if (String(SubMenus[i][0]) == smenu) {
          j++;
          if (j == HighlightedMenu) {
            break;
          }
        }
      }

      //Now we take action
      if (smenu == "1") {
        GenericText = SubMenus[i][2];
        Steps = GenericText.toInt();
      } else if (smenu == "2") {
        GenericText = SubMenus[i][2];
        SelectedFilter = GenericText.toInt();
      } else if (smenu == "3") {
        //Set power
        GenericText = SubMenus[i][2];
        SetPower(GenericText.toInt());
      } else if (smenu == "4") {
        //GenericText = SubMenus[i][2];
        //AutoTune = GenericText.toInt();
      } else if (smenu == "5") {
        GenericText = SubMenus[i][2];
        if (GenericText == "1") {
          preferences.begin("ATCK", false);
          preferences.putInt("Parameter1", ExtendedParameter1);
          preferences.putInt("Parameter2", ExtendedParameter2);
          preferences.putInt("Parameter3", ExtendedParameter3);
          preferences.putInt("Parameter4", ExtendedParameter4);
          preferences.putInt("Steps", Steps);
          preferences.putInt("Filter", SelectedFilter);
          preferences.end();
        } else {
        }
      }
      start = 0;
      timeout = 0;
    }
  } while (millis() < start + timeout);
  PreviousPower = 0;  //To force status update
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
  Upper.fillRect(0, 0, 320, 172, TFT_BACKGROUND);
}

void FlushSerialInput() {
  char a;
  while (Serial2.available() > 0) {
    a = Serial2.read();
  }
}

void MenuHandle_1stEncoder() {

  start = millis();
  int HighlightedMenu = ExtendedParameter1;  //The menu that is highlighted is the one already chosen for this encoder
  bool Redraw = true;

  do {
    delay(CommandDelay);
  } while (!RE1.digitalRead(SS_SWITCH));  //Loop while the button is pressed
  RE1.setEncoderPosition(0);

  TFT_FOREGROUND = TFT_YELLOW;
  TFT_BACKGROUND = TFT_EBONY;
  UpperClearDisplay();

  do {
    if (Redraw == true) {
      Upper.setFreeFont(&FreeSansBold12pt7b);
      UpperPrintTextCentered(0, 320, 43, "----------------");
      UpperPrintTextCentered(0, 320, 28, "Select parameter");
      if (HighlightedMenu == 1) TFT_BACKGROUND = TFT_RED;
      UpperPrintTextCentered(0, 160, 65, "Squelch");
      TFT_BACKGROUND = TFT_EBONY;
      if (HighlightedMenu == 2) TFT_BACKGROUND = TFT_RED;
      UpperPrintTextCentered(0, 160, 91, "Memory Ch.");
      TFT_BACKGROUND = TFT_EBONY;
      if (HighlightedMenu == 3) TFT_BACKGROUND = TFT_RED;
      UpperPrintTextCentered(0, 160, 117, "Notch");
      TFT_BACKGROUND = TFT_EBONY;
      if (HighlightedMenu == 4) TFT_BACKGROUND = TFT_RED;
      UpperPrintTextCentered(0, 160, 143, "Contour width");
      TFT_BACKGROUND = TFT_EBONY;
      if (HighlightedMenu == 5) TFT_BACKGROUND = TFT_RED;
      UpperPrintTextCentered(0, 160, 169, "Contour level");
      TFT_BACKGROUND = TFT_EBONY;
      if (HighlightedMenu == 6) TFT_BACKGROUND = TFT_RED;
      UpperPrintTextCentered(160, 320, 65, "Power level");
      TFT_BACKGROUND = TFT_EBONY;
      if (HighlightedMenu == 7) TFT_BACKGROUND = TFT_RED;
      UpperPrintTextCentered(160, 320, 91, "Frequency");
      TFT_BACKGROUND = TFT_EBONY;
      Upper.pushSprite(0, 8);

      Redraw = false;
    }

    int32_t new_position1 = RotatorDirection * RE1.getEncoderPosition();
    if (new_position1 != 0) {  // If the position has changed ...
      if (new_position1 < 0) {
        HighlightedMenu += abs(new_position1);
        if (HighlightedMenu > 7) HighlightedMenu = 7;
      } else {
        HighlightedMenu -= abs(new_position1);
        if (HighlightedMenu < 1) HighlightedMenu = 1;
      }
      RE1.setEncoderPosition(0);
      Redraw = true;
      start = millis();
    }


    if (!RE1.digitalRead(SS_SWITCH)) {  // Reads if the button on the encoder is pressed
      do {
        delay(CommandDelay);
      } while (!RE1.digitalRead(SS_SWITCH));  //Loop while it is pressed

      switch (HighlightedMenu) {
        case 1:
          ExtendedParameter1 = 1;
          break;
        case 2:
          ExtendedParameter1 = 2;
          break;
        case 3:
          ExtendedParameter1 = 3;
          break;
        case 4:
          ExtendedParameter1 = 4;
          break;
        case 5:
          ExtendedParameter1 = 5;
          break;
        case 6:
          ExtendedParameter1 = 6;
          break;
        case 7:
          ExtendedParameter1 = 7;
          break;
        default:
          break;
      }
      break;
    }
  } while (millis() < start + 5000);
  Message = 0;
}

void MenuHandle_2ndEncoder() {

  start = millis();
  int HighlightedMenu = ExtendedParameter2;  //The menu that is highlighted is the one already chosen for this encoder
  bool Redraw = true;

  do {
    delay(CommandDelay);
  } while (!RE2.digitalRead(SS_SWITCH));  //Loop while the button is pressed
  RE2.setEncoderPosition(0);

  TFT_FOREGROUND = TFT_YELLOW;
  TFT_BACKGROUND = TFT_EBONY;
  UpperClearDisplay();

  do {
    if (Redraw == true) {
      Upper.setFreeFont(&FreeSansBold12pt7b);
      UpperPrintTextCentered(0, 320, 43, "----------------");
      UpperPrintTextCentered(0, 320, 28, "Select parameter");
      if (HighlightedMenu == 1) TFT_BACKGROUND = TFT_RED;
      UpperPrintTextCentered(0, 160, 65, "Squelch");
      TFT_BACKGROUND = TFT_EBONY;
      if (HighlightedMenu == 2) TFT_BACKGROUND = TFT_RED;
      UpperPrintTextCentered(0, 160, 91, "Memory Ch.");
      TFT_BACKGROUND = TFT_EBONY;
      if (HighlightedMenu == 3) TFT_BACKGROUND = TFT_RED;
      UpperPrintTextCentered(0, 160, 117, "Notch");
      TFT_BACKGROUND = TFT_EBONY;
      if (HighlightedMenu == 4) TFT_BACKGROUND = TFT_RED;
      UpperPrintTextCentered(0, 160, 143, "Contour width");
      TFT_BACKGROUND = TFT_EBONY;
      if (HighlightedMenu == 5) TFT_BACKGROUND = TFT_RED;
      UpperPrintTextCentered(0, 160, 169, "Contour level");
      TFT_BACKGROUND = TFT_EBONY;
      if (HighlightedMenu == 6) TFT_BACKGROUND = TFT_RED;
      UpperPrintTextCentered(160, 320, 65, "Power level");
      TFT_BACKGROUND = TFT_EBONY;
      if (HighlightedMenu == 7) TFT_BACKGROUND = TFT_RED;
      UpperPrintTextCentered(160, 320, 91, "Frequency");
      TFT_BACKGROUND = TFT_EBONY;
      Upper.pushSprite(0, 8);

      Redraw = false;
    }

    int32_t new_position2 = RotatorDirection * RE2.getEncoderPosition();
    if (new_position2 != 0) {  // If the position has changed ...
      if (new_position2 < 0) {
        HighlightedMenu += abs(new_position2);
        if (HighlightedMenu > 7) HighlightedMenu = 7;
      } else {
        HighlightedMenu -= abs(new_position2);
        if (HighlightedMenu < 1) HighlightedMenu = 1;
      }
      RE2.setEncoderPosition(0);
      Redraw = true;
      start = millis();
    }


    if (!RE2.digitalRead(SS_SWITCH)) {  // Reads if the button on the encoder is pressed
      do {
        delay(CommandDelay);
      } while (!RE2.digitalRead(SS_SWITCH));  //Loop while it is pressed

      switch (HighlightedMenu) {
        case 1:
          ExtendedParameter2 = 1;
          break;
        case 2:
          ExtendedParameter2 = 2;
          break;
        case 3:
          ExtendedParameter2 = 3;
          break;
        case 4:
          ExtendedParameter2 = 4;
          break;
        case 5:
          ExtendedParameter2 = 5;
          break;
        case 6:
          ExtendedParameter2 = 6;
          break;
        case 7:
          ExtendedParameter2 = 7;
          break;
        default:
          break;
      }
      break;
    }
  } while (millis() < start + 5000);
  Message = 0;
}

void MenuHandle_3rdEncoder() {

  start = millis();
  int HighlightedMenu = ExtendedParameter3;  //The menu that is highlighted is the one already chosen for this encoder
  bool Redraw = true;

  do {
    delay(CommandDelay);
  } while (!RE3.digitalRead(SS_SWITCH));  //Loop while the button is pressed
  RE3.setEncoderPosition(0);

  TFT_FOREGROUND = TFT_YELLOW;
  TFT_BACKGROUND = TFT_EBONY;
  UpperClearDisplay();

  do {
    if (Redraw == true) {
      Upper.setFreeFont(&FreeSansBold12pt7b);
      UpperPrintTextCentered(0, 320, 43, "----------------");
      UpperPrintTextCentered(0, 320, 28, "Select parameter");
      if (HighlightedMenu == 1) TFT_BACKGROUND = TFT_RED;
      UpperPrintTextCentered(0, 160, 65, "Squelch");
      TFT_BACKGROUND = TFT_EBONY;
      if (HighlightedMenu == 2) TFT_BACKGROUND = TFT_RED;
      UpperPrintTextCentered(0, 160, 91, "Memory Ch.");
      TFT_BACKGROUND = TFT_EBONY;
      if (HighlightedMenu == 3) TFT_BACKGROUND = TFT_RED;
      UpperPrintTextCentered(0, 160, 117, "Notch");
      TFT_BACKGROUND = TFT_EBONY;
      if (HighlightedMenu == 4) TFT_BACKGROUND = TFT_RED;
      UpperPrintTextCentered(0, 160, 143, "Contour width");
      TFT_BACKGROUND = TFT_EBONY;
      if (HighlightedMenu == 5) TFT_BACKGROUND = TFT_RED;
      UpperPrintTextCentered(0, 160, 169, "Contour level");
      TFT_BACKGROUND = TFT_EBONY;
      if (HighlightedMenu == 6) TFT_BACKGROUND = TFT_RED;
      UpperPrintTextCentered(160, 320, 65, "Power level");
      TFT_BACKGROUND = TFT_EBONY;
      if (HighlightedMenu == 7) TFT_BACKGROUND = TFT_RED;
      UpperPrintTextCentered(160, 320, 91, "Frequency");
      TFT_BACKGROUND = TFT_EBONY;
      Upper.pushSprite(0, 8);

      Redraw = false;
    }

    int32_t new_position3 = RotatorDirection * RE3.getEncoderPosition();
    if (new_position3 != 0) {  // If the position has changed ...
      if (new_position3 < 0) {
        HighlightedMenu += abs(new_position3);
        if (HighlightedMenu > 7) HighlightedMenu = 7;
      } else {
        HighlightedMenu -= abs(new_position3);
        if (HighlightedMenu < 1) HighlightedMenu = 1;
      }
      RE3.setEncoderPosition(0);
      Redraw = true;
      start = millis();
    }


    if (!RE3.digitalRead(SS_SWITCH)) {  // Reads if the button on the encoder is pressed
      do {
        delay(CommandDelay);
      } while (!RE3.digitalRead(SS_SWITCH));  //Loop while it is pressed

      switch (HighlightedMenu) {
        case 1:
          ExtendedParameter3 = 1;
          break;
        case 2:
          ExtendedParameter3 = 2;
          break;
        case 3:
          ExtendedParameter3 = 3;
          break;
        case 4:
          ExtendedParameter3 = 4;
          break;
        case 5:
          ExtendedParameter3 = 5;
          break;
        case 6:
          ExtendedParameter3 = 6;
          break;
        case 7:
          ExtendedParameter3 = 7;
          break;
        default:
          break;
      }
      break;
    }
  } while (millis() < start + 5000);
  Message = 0;
}

void MenuHandle_4thEncoder() {

  start = millis();
  int HighlightedMenu = ExtendedParameter4;  //The menu that is highlighted is the one already chosen for this encoder
  bool Redraw = true;

  do {
    delay(CommandDelay);
  } while (!RE4.digitalRead(SS_SWITCH));  //Loop while the button is pressed
  RE4.setEncoderPosition(0);

  TFT_FOREGROUND = TFT_YELLOW;
  TFT_BACKGROUND = TFT_EBONY;
  UpperClearDisplay();

  do {
    if (Redraw == true) {
      Upper.setFreeFont(&FreeSansBold12pt7b);
      UpperPrintTextCentered(0, 320, 43, "----------------");
      UpperPrintTextCentered(0, 320, 28, "Select parameter");
      if (HighlightedMenu == 1) TFT_BACKGROUND = TFT_RED;
      UpperPrintTextCentered(0, 160, 65, "Squelch");
      TFT_BACKGROUND = TFT_EBONY;
      if (HighlightedMenu == 2) TFT_BACKGROUND = TFT_RED;
      UpperPrintTextCentered(0, 160, 91, "Memory Ch.");
      TFT_BACKGROUND = TFT_EBONY;
      if (HighlightedMenu == 3) TFT_BACKGROUND = TFT_RED;
      UpperPrintTextCentered(0, 160, 117, "Notch");
      TFT_BACKGROUND = TFT_EBONY;
      if (HighlightedMenu == 4) TFT_BACKGROUND = TFT_RED;
      UpperPrintTextCentered(0, 160, 143, "Contour width");
      TFT_BACKGROUND = TFT_EBONY;
      if (HighlightedMenu == 5) TFT_BACKGROUND = TFT_RED;
      UpperPrintTextCentered(0, 160, 169, "Contour level");
      TFT_BACKGROUND = TFT_EBONY;
      if (HighlightedMenu == 6) TFT_BACKGROUND = TFT_RED;
      UpperPrintTextCentered(160, 320, 65, "Power level");
      TFT_BACKGROUND = TFT_EBONY;
      if (HighlightedMenu == 7) TFT_BACKGROUND = TFT_RED;
      UpperPrintTextCentered(160, 320, 91, "Frequency");
      TFT_BACKGROUND = TFT_EBONY;
      Upper.pushSprite(0, 8);

      Redraw = false;
    }

    int32_t new_position4 = RotatorDirection * RE4.getEncoderPosition();
    if (new_position4 != 0) {  // If the position has changed ...
      if (new_position4 < 0) {
        HighlightedMenu += abs(new_position4);
        if (HighlightedMenu > 7) HighlightedMenu = 7;
      } else {
        HighlightedMenu -= abs(new_position4);
        if (HighlightedMenu < 1) HighlightedMenu = 1;
      }
      RE4.setEncoderPosition(0);
      Redraw = true;
      start = millis();
    }


    if (!RE4.digitalRead(SS_SWITCH)) {  // Reads if the button on the encoder is pressed
      do {
        delay(CommandDelay);
      } while (!RE4.digitalRead(SS_SWITCH));  //Loop while it is pressed

      switch (HighlightedMenu) {
        case 1:
          ExtendedParameter4 = 1;
          break;
        case 2:
          ExtendedParameter4 = 2;
          break;
        case 3:
          ExtendedParameter4 = 3;
          break;
        case 4:
          ExtendedParameter4 = 4;
          break;
        case 5:
          ExtendedParameter4 = 5;
          break;
        case 6:
          ExtendedParameter4 = 6;
          break;
        case 7:
          ExtendedParameter4 = 7;
          break;
        default:
          break;
      }
      break;
    }
  } while (millis() < start + 5000);
  Message = 0;
}

void Read1stEncoder() {
  int SQL;
  int Memory;
  int NotchWidth;
  int ContourWidth;
  int ContourLevel;
  int PowerLevel;
  long CurrentFrequencyRX;
  int i = 0;
  start = millis();

  SQL = ReadSQL();
  Memory = ReadMemory();
  NotchWidth = ReadNotchWidth();
  ContourWidth = ReadContourWidth();
  ContourLevel = ReadContourLevel();
  PowerLevel = ReadPower();
  CurrentFrequencyRX = ReadFrequency(VFORead());


  do {
    // If the previous and the current state of the outputA are different, that means a Pulse has occured
    int32_t new_position1 = RotatorDirection * RE1.getEncoderPosition();
    if (encoder_position1 != new_position1) {
      if (Message != 0) {
        TFT_FOREGROUND = TFT_YELLOW;
        TFT_BACKGROUND = TFT_EBONY;
        UpperClearDisplay();
        Message = 0;
      }
      if (new_position1 < encoder_position1) {  // If the rotator moved clockwise
        switch (ExtendedParameter1) {
          case 1:
            SQL += 10 * (encoder_position1 - new_position1);
            if (SQL > 100) { SQL = 100; }
            SetSQL(SQL);
            break;
          case 2:
            Memory += (encoder_position1 - new_position1);
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
            ContourWidth += (encoder_position1 - new_position1);
            if (ContourWidth > 11) { ContourWidth = 11; }
            SetContourWidth(ContourWidth);
            break;
          case 5:
            ContourLevel += (encoder_position1 - new_position1);
            if (ContourLevel > 20) { ContourLevel = 20; }
            SetContourLevel(ContourLevel);
            break;
          case 6:
            PowerLevel += (encoder_position1 - new_position1);
            if (RIG_Model == 'FTDX101MP' && PowerLevel > 200) {
              PowerLevel = 200;
            } else if (RIG_Model == 'FTDX101D' && PowerLevel > 100) {
              PowerLevel = 100;
            }
            SetPower(PowerLevel);
            break;
          case 7:
            CurrentFrequencyRX = CurrentFrequencyRX / Steps * Steps;
            CurrentFrequencyRX = CurrentFrequencyRX + Steps * (encoder_position1 - new_position1);
            SetFrequency(VFORead(), CurrentFrequencyRX);
            break;
          default: break;
        }
      } else {  //The rotator turned counterclockwise
        switch (ExtendedParameter1) {
          case 1:
            SQL -= 10 * (new_position1 - encoder_position1);
            if (SQL < 0) { SQL = 0; }
            SetSQL(SQL);
            break;
          case 2:
            Memory -= (new_position1 - encoder_position1);
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
            ContourWidth -= (new_position1 - encoder_position1);
            if (ContourWidth < 1) { ContourWidth = 1; }
            SetContourWidth(ContourWidth);
            break;
          case 5:
            ContourLevel -= (new_position1 - encoder_position1);
            if (ContourLevel < -40) { ContourLevel = -40; }
            SetContourLevel(ContourLevel);
            break;
          case 6:
            PowerLevel -= (new_position1 - encoder_position1);
            if (PowerLevel < 5) { PowerLevel = 5; }
            SetPower(PowerLevel);
            break;
          case 7:
            CurrentFrequencyRX = (CurrentFrequencyRX + (Steps - 1)) / Steps * Steps;
            CurrentFrequencyRX = CurrentFrequencyRX - Steps * (new_position1 - encoder_position1);
            SetFrequency(VFORead(), CurrentFrequencyRX);
            break;
          default:
            break;
        }
      }
      encoder_position1 = new_position1;

      switch (ExtendedParameter1) {
        case 1:
          Upper.setFreeFont(&FreeSansBold24pt7b);
          UpperPrintTextCentered(0, 320, 100, "SQL: " + String(ReadSQL()));
          break;
        case 2:
          Upper.setFreeFont(&FreeSansBold24pt7b);
          if (Memory != ReadMemory()) {
            UpperPrintTextCentered(0, 320, 100, "MEM: " + String(Memory) + " E");
          } else {
            UpperPrintTextCentered(0, 320, 100, "MEM: " + String(Memory));
          }
          break;
        case 3:
          Upper.setFreeFont(&FreeSansBold24pt7b);
          if (NotchWidth == 0) {
            UpperPrintTextCentered(0, 320, 100, "Narrow notch");
          } else if (NotchWidth == 1) {
            UpperPrintTextCentered(0, 320, 100, "Wide notch");
          }
          break;
        case 4:
          Upper.setFreeFont(&FreeSansBold24pt7b);
          UpperPrintTextCentered(0, 320, 100, "Cont W: " + String(ContourWidth));
          break;
        case 5:
          Upper.setFreeFont(&FreeSansBold24pt7b);
          UpperPrintTextCentered(0, 320, 100, "Cont L: " + String(ContourLevel));
          break;
        case 6:
          Upper.setFreeFont(&FreeSansBold24pt7b);
          UpperPrintTextCentered(0, 320, 100, "PWR: " + String(PowerLevel) + "W");
          Lower.fillRect(0, 0, Lower.width(), Lower.height(), TFT_BLACK);
          Lower.setFreeFont(&FreeSansBold12pt7b);
          TFT_BACKGROUND = TFT_BLACK;
          LowerPrintText(1, 28, "PWR: " + String(PowerLevel) + "W");
          Lower.pushSprite(0, 181);
          TFT_BACKGROUND = TFT_EBONY;
          PreviousPower = PowerLevel;  //In order to avoid to force status update because of the power change
          break;
        case 7:
          //The following block of code is done to avoid some flickering while changing the frequentcy on the tft screen
          Upper.setFreeFont(&FreeSansBold24pt7b);
          TFT_FOREGROUND = TFT_YELLOW;
          if (CurrentFrequencyRX > 9999999) {
            UpperPrintTextCentered(0, 320, 100, String(CurrentFrequencyRX).substring(0, 2) + "." + String(CurrentFrequencyRX).substring(2, 5) + "." + String(CurrentFrequencyRX).substring(5, 8));
          } else {
            UpperPrintTextCentered(0, 320, 100, String(CurrentFrequencyRX).substring(0, 1) + "." + String(CurrentFrequencyRX).substring(1, 4) + "." + String(CurrentFrequencyRX).substring(4, 7));
          }
          break;
        default:
          break;
      }
      Upper.pushSprite(0, 8);
      start = millis();
    }

    if (!RE1.digitalRead(SS_SWITCH)) {  // Reads if the rotator button is pressed...
      do {
      } while (!RE1.digitalRead(SS_SWITCH));  //...end wait until released
      encoder_position1 = 0;
      RE1.setEncoderPosition(0);
      return;
    }
  } while (millis() < start + 1000);
  encoder_position1 = 0;
  RE1.setEncoderPosition(0);
}

void Read2ndEncoder() {
  int SQL;
  int Memory;
  int NotchWidth;
  int ContourWidth;
  int ContourLevel;
  int PowerLevel;
  long CurrentFrequencyRX;
  int i = 0;
  start = millis();

  SQL = ReadSQL();
  Memory = ReadMemory();
  NotchWidth = ReadNotchWidth();
  ContourWidth = ReadContourWidth();
  ContourLevel = ReadContourLevel();
  PowerLevel = ReadPower();
  CurrentFrequencyRX = ReadFrequency(VFORead());


  do {
    // If the previous and the current state of the outputA are different, that means a Pulse has occured
    int32_t new_position2 = RotatorDirection * RE2.getEncoderPosition();
    if (encoder_position2 != new_position2) {
      if (Message != 0) {
        TFT_FOREGROUND = TFT_YELLOW;
        TFT_BACKGROUND = TFT_EBONY;
        UpperClearDisplay();
        Message = 0;
      }
      if (new_position2 < encoder_position2) {  // If the rotator moved clockwise
        switch (ExtendedParameter2) {
          case 1:
            SQL += 10 * (encoder_position2 - new_position2);
            if (SQL > 100) { SQL = 100; }
            SetSQL(SQL);
            break;
          case 2:
            Memory += (encoder_position2 - new_position2);
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
            ContourWidth += (encoder_position2 - new_position2);
            if (ContourWidth > 11) { ContourWidth = 11; }
            SetContourWidth(ContourWidth);
            break;
          case 5:
            ContourLevel += (encoder_position2 - new_position2);
            if (ContourLevel > 20) { ContourLevel = 20; }
            SetContourLevel(ContourLevel);
            break;
          case 6:
            PowerLevel += (encoder_position2 - new_position2);
            if (RIG_Model == 'FTDX101MP' && PowerLevel > 200) {
              PowerLevel = 200;
            } else if (RIG_Model == 'FTDX101D' && PowerLevel > 100) {
              PowerLevel = 100;
            }
            SetPower(PowerLevel);
            break;
          case 7:
            CurrentFrequencyRX = CurrentFrequencyRX / Steps * Steps;
            CurrentFrequencyRX = CurrentFrequencyRX + Steps * (encoder_position2 - new_position2);
            SetFrequency(VFORead(), CurrentFrequencyRX);
            break;
          default: break;
        }
      } else {  //The rotator turned counterclockwise
        switch (ExtendedParameter2) {
          case 1:
            SQL -= 10 * (new_position2 - encoder_position2);
            if (SQL < 0) { SQL = 0; }
            SetSQL(SQL);
            break;
          case 2:
            Memory -= (new_position2 - encoder_position2);
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
            ContourWidth -= (new_position2 - encoder_position2);
            if (ContourWidth < 1) { ContourWidth = 1; }
            SetContourWidth(ContourWidth);
            break;
          case 5:
            ContourLevel -= (new_position2 - encoder_position2);
            if (ContourLevel < -40) { ContourLevel = -40; }
            SetContourLevel(ContourLevel);
            break;
          case 6:
            PowerLevel -= (new_position2 - encoder_position2);
            if (PowerLevel < 5) { PowerLevel = 5; }
            SetPower(PowerLevel);
            break;
          case 7:
            CurrentFrequencyRX = (CurrentFrequencyRX + (Steps - 1)) / Steps * Steps;
            CurrentFrequencyRX = CurrentFrequencyRX - Steps * (new_position2 - encoder_position2);
            SetFrequency(VFORead(), CurrentFrequencyRX);
            break;
          default:
            break;
        }
      }
      encoder_position2 = new_position2;

      switch (ExtendedParameter2) {
        case 1:
          Upper.setFreeFont(&FreeSansBold24pt7b);
          UpperPrintTextCentered(0, 320, 100, "SQL: " + String(ReadSQL()));
          break;
        case 2:
          Upper.setFreeFont(&FreeSansBold24pt7b);
          if (Memory != ReadMemory()) {
            UpperPrintTextCentered(0, 320, 100, "MEM: " + String(Memory) + " E");
          } else {
            UpperPrintTextCentered(0, 320, 100, "MEM: " + String(Memory));
          }
          break;
        case 3:
          Upper.setFreeFont(&FreeSansBold24pt7b);
          if (NotchWidth == 0) {
            UpperPrintTextCentered(0, 320, 100, "Narrow notch");
          } else if (NotchWidth == 1) {
            UpperPrintTextCentered(0, 320, 100, "Wide notch");
          }
          break;
        case 4:
          Upper.setFreeFont(&FreeSansBold24pt7b);
          UpperPrintTextCentered(0, 320, 100, "Cont W: " + String(ContourWidth));
          break;
        case 5:
          Upper.setFreeFont(&FreeSansBold24pt7b);
          UpperPrintTextCentered(0, 320, 100, "Cont L: " + String(ContourLevel));
          break;
        case 6:
          Upper.setFreeFont(&FreeSansBold24pt7b);
          UpperPrintTextCentered(0, 320, 100, "PWR: " + String(PowerLevel) + "W");
          Lower.fillRect(0, 0, Lower.width(), Lower.height(), TFT_BLACK);
          Lower.setFreeFont(&FreeSansBold12pt7b);
          TFT_BACKGROUND = TFT_BLACK;
          LowerPrintText(1, 28, "PWR: " + String(PowerLevel) + "W");
          Lower.pushSprite(0, 181);
          TFT_BACKGROUND = TFT_EBONY;
          PreviousPower = PowerLevel;  //In order to avoid to force status update because of the power change
          break;
        case 7:
          //The following block of code is done to avoid some flickering while changing the frequentcy on the tft screen
          Upper.setFreeFont(&FreeSansBold24pt7b);
          TFT_FOREGROUND = TFT_YELLOW;
          if (CurrentFrequencyRX > 9999999) {
            UpperPrintTextCentered(0, 320, 100, String(CurrentFrequencyRX).substring(0, 2) + "." + String(CurrentFrequencyRX).substring(2, 5) + "." + String(CurrentFrequencyRX).substring(5, 8));
          } else {
            UpperPrintTextCentered(0, 320, 100, String(CurrentFrequencyRX).substring(0, 1) + "." + String(CurrentFrequencyRX).substring(1, 4) + "." + String(CurrentFrequencyRX).substring(4, 7));
          }
          break;
        default:
          break;
      }
      Upper.pushSprite(0, 8);
      start = millis();
    }

    if (!RE2.digitalRead(SS_SWITCH)) {  // Reads if the rotator button is pressed...
      do {
      } while (!RE2.digitalRead(SS_SWITCH));  //...end wait until released
      encoder_position2 = 0;
      RE2.setEncoderPosition(0);
      return;
    }
  } while (millis() < start + 1000);
  encoder_position2 = 0;
  RE2.setEncoderPosition(0);
}

void Read3rdEncoder() {
  int SQL;
  int Memory;
  int NotchWidth;
  int ContourWidth;
  int ContourLevel;
  int PowerLevel;
  long CurrentFrequencyRX;
  int i = 0;
  start = millis();

  SQL = ReadSQL();
  Memory = ReadMemory();
  NotchWidth = ReadNotchWidth();
  ContourWidth = ReadContourWidth();
  ContourLevel = ReadContourLevel();
  PowerLevel = ReadPower();
  CurrentFrequencyRX = ReadFrequency(VFORead());


  do {
    // If the previous and the current state of the outputA are different, that means a Pulse has occured
    int32_t new_position3 = RotatorDirection * RE3.getEncoderPosition();
    if (encoder_position3 != new_position3) {
      if (Message != 0) {
        TFT_FOREGROUND = TFT_YELLOW;
        TFT_BACKGROUND = TFT_EBONY;
        UpperClearDisplay();
        Message = 0;
      }
      if (new_position3 < encoder_position3) {  // If the rotator moved clockwise
        switch (ExtendedParameter3) {
          case 1:
            SQL += 10 * (encoder_position3 - new_position3);
            if (SQL > 100) { SQL = 100; }
            SetSQL(SQL);
            break;
          case 2:
            Memory += (encoder_position3 - new_position3);
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
            ContourWidth += (encoder_position3 - new_position3);
            if (ContourWidth > 11) { ContourWidth = 11; }
            SetContourWidth(ContourWidth);
            break;
          case 5:
            ContourLevel += (encoder_position3 - new_position3);
            if (ContourLevel > 20) { ContourLevel = 20; }
            SetContourLevel(ContourLevel);
            break;
          case 6:
            PowerLevel += (encoder_position3 - new_position3);
            if (RIG_Model == 'FTDX101MP' && PowerLevel > 200) {
              PowerLevel = 200;
            } else if (RIG_Model == 'FTDX101D' && PowerLevel > 100) {
              PowerLevel = 100;
            }
            SetPower(PowerLevel);
            break;
          case 7:
            CurrentFrequencyRX = CurrentFrequencyRX / Steps * Steps;
            CurrentFrequencyRX = CurrentFrequencyRX + Steps * (encoder_position3 - new_position3);
            SetFrequency(VFORead(), CurrentFrequencyRX);
            break;
          default: break;
        }
      } else {  //The rotator turned counterclockwise
        switch (ExtendedParameter3) {
          case 1:
            SQL -= 10 * (new_position3 - encoder_position3);
            if (SQL < 0) { SQL = 0; }
            SetSQL(SQL);
            break;
          case 2:
            Memory -= (new_position3 - encoder_position3);
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
            ContourWidth -= (new_position3 - encoder_position3);
            if (ContourWidth < 1) { ContourWidth = 1; }
            SetContourWidth(ContourWidth);
            break;
          case 5:
            ContourLevel -= (new_position3 - encoder_position3);
            if (ContourLevel < -40) { ContourLevel = -40; }
            SetContourLevel(ContourLevel);
            break;
          case 6:
            PowerLevel -= (new_position3 - encoder_position3);
            if (PowerLevel < 5) { PowerLevel = 5; }
            SetPower(PowerLevel);
            break;
          case 7:
            CurrentFrequencyRX = (CurrentFrequencyRX + (Steps - 1)) / Steps * Steps;
            CurrentFrequencyRX = CurrentFrequencyRX - Steps * (new_position3 - encoder_position3);
            SetFrequency(VFORead(), CurrentFrequencyRX);
            break;
          default:
            break;
        }
      }
      encoder_position3 = new_position3;

      switch (ExtendedParameter3) {
        case 1:
          Upper.setFreeFont(&FreeSansBold24pt7b);
          UpperPrintTextCentered(0, 320, 100, "SQL: " + String(ReadSQL()));
          break;
        case 2:
          Upper.setFreeFont(&FreeSansBold24pt7b);
          if (Memory != ReadMemory()) {
            UpperPrintTextCentered(0, 320, 100, "MEM: " + String(Memory) + " E");
          } else {
            UpperPrintTextCentered(0, 320, 100, "MEM: " + String(Memory));
          }
          break;
        case 3:
          Upper.setFreeFont(&FreeSansBold24pt7b);
          if (NotchWidth == 0) {
            UpperPrintTextCentered(0, 320, 100, "Narrow notch");
          } else if (NotchWidth == 1) {
            UpperPrintTextCentered(0, 320, 100, "Wide notch");
          }
          break;
        case 4:
          Upper.setFreeFont(&FreeSansBold24pt7b);
          UpperPrintTextCentered(0, 320, 100, "Cont W: " + String(ContourWidth));
          break;
        case 5:
          Upper.setFreeFont(&FreeSansBold24pt7b);
          UpperPrintTextCentered(0, 320, 100, "Cont L: " + String(ContourLevel));
          break;
        case 6:
          Upper.setFreeFont(&FreeSansBold24pt7b);
          UpperPrintTextCentered(0, 320, 100, "PWR: " + String(PowerLevel) + "W");
          Lower.fillRect(0, 0, Lower.width(), Lower.height(), TFT_BLACK);
          Lower.setFreeFont(&FreeSansBold12pt7b);
          TFT_BACKGROUND = TFT_BLACK;
          LowerPrintText(1, 28, "PWR: " + String(PowerLevel) + "W");
          Lower.pushSprite(0, 181);
          TFT_BACKGROUND = TFT_EBONY;
          PreviousPower = PowerLevel;  //In order to avoid to force status update because of the power change
          break;
        case 7:
          //The following block of code is done to avoid some flickering while changing the frequentcy on the tft screen
          Upper.setFreeFont(&FreeSansBold24pt7b);
          TFT_FOREGROUND = TFT_YELLOW;
          if (CurrentFrequencyRX > 9999999) {
            UpperPrintTextCentered(0, 320, 100, String(CurrentFrequencyRX).substring(0, 2) + "." + String(CurrentFrequencyRX).substring(2, 5) + "." + String(CurrentFrequencyRX).substring(5, 8));
          } else {
            UpperPrintTextCentered(0, 320, 100, String(CurrentFrequencyRX).substring(0, 1) + "." + String(CurrentFrequencyRX).substring(1, 4) + "." + String(CurrentFrequencyRX).substring(4, 7));
          }
          break;
        default:
          break;
      }
      Upper.pushSprite(0, 8);
      start = millis();
    }

    if (!RE3.digitalRead(SS_SWITCH)) {  // Reads if the rotator button is pressed...
      do {
      } while (!RE3.digitalRead(SS_SWITCH));  //...end wait until released
      encoder_position3 = 0;
      RE3.setEncoderPosition(0);
      return;
    }
  } while (millis() < start + 1000);
  encoder_position3 = 0;
  RE3.setEncoderPosition(0);
}

void Read4thEncoder() {
  int SQL;
  int Memory;
  int NotchWidth;
  int ContourWidth;
  int ContourLevel;
  int PowerLevel;
  long CurrentFrequencyRX;
  int i = 0;
  start = millis();

  SQL = ReadSQL();
  Memory = ReadMemory();
  NotchWidth = ReadNotchWidth();
  ContourWidth = ReadContourWidth();
  ContourLevel = ReadContourLevel();
  PowerLevel = ReadPower();
  CurrentFrequencyRX = ReadFrequency(VFORead());


  do {
    // If the previous and the current state of the outputA are different, that means a Pulse has occured
    int32_t new_position4 = RotatorDirection * RE4.getEncoderPosition();
    if (encoder_position4 != new_position4) {
      if (Message != 0) {
        TFT_FOREGROUND = TFT_YELLOW;
        TFT_BACKGROUND = TFT_EBONY;
        UpperClearDisplay();
        Message = 0;
      }
      if (new_position4 < encoder_position4) {  // If the rotator moved clockwise
        switch (ExtendedParameter4) {
          case 1:
            SQL += 10 * (encoder_position4 - new_position4);
            if (SQL > 100) { SQL = 100; }
            SetSQL(SQL);
            break;
          case 2:
            Memory += (encoder_position4 - new_position4);
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
            ContourWidth += (encoder_position4 - new_position4);
            if (ContourWidth > 11) { ContourWidth = 11; }
            SetContourWidth(ContourWidth);
            break;
          case 5:
            ContourLevel += (encoder_position4 - new_position4);
            if (ContourLevel > 20) { ContourLevel = 20; }
            SetContourLevel(ContourLevel);
            break;
          case 6:
            PowerLevel += (encoder_position4 - new_position4);
            if (RIG_Model == 'FTDX101MP' && PowerLevel > 200) {
              PowerLevel = 200;
            } else if (RIG_Model == 'FTDX101D' && PowerLevel > 100) {
              PowerLevel = 100;
            }
            SetPower(PowerLevel);
            break;
          case 7:
            CurrentFrequencyRX = CurrentFrequencyRX / Steps * Steps;
            CurrentFrequencyRX = CurrentFrequencyRX + Steps * (encoder_position4 - new_position4);
            SetFrequency(VFORead(), CurrentFrequencyRX);
            break;
          default: break;
        }
      } else {  //The rotator turned counterclockwise
        switch (ExtendedParameter4) {
          case 1:
            SQL -= 10 * (new_position4 - encoder_position4);
            if (SQL < 0) { SQL = 0; }
            SetSQL(SQL);
            break;
          case 2:
            Memory -= (new_position4 - encoder_position4);
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
            ContourWidth -= (new_position4 - encoder_position4);
            if (ContourWidth < 1) { ContourWidth = 1; }
            SetContourWidth(ContourWidth);
            break;
          case 5:
            ContourLevel -= (new_position4 - encoder_position4);
            if (ContourLevel < -40) { ContourLevel = -40; }
            SetContourLevel(ContourLevel);
            break;
          case 6:
            PowerLevel -= (new_position4 - encoder_position4);
            if (PowerLevel < 5) { PowerLevel = 5; }
            SetPower(PowerLevel);
            break;
          case 7:
            CurrentFrequencyRX = (CurrentFrequencyRX + (Steps - 1)) / Steps * Steps;
            CurrentFrequencyRX = CurrentFrequencyRX - Steps * (new_position4 - encoder_position4);
            SetFrequency(VFORead(), CurrentFrequencyRX);
            break;
          default:
            break;
        }
      }
      encoder_position4 = new_position4;

      switch (ExtendedParameter4) {
        case 1:
          Upper.setFreeFont(&FreeSansBold24pt7b);
          UpperPrintTextCentered(0, 320, 100, "SQL: " + String(ReadSQL()));
          break;
        case 2:
          Upper.setFreeFont(&FreeSansBold24pt7b);
          if (Memory != ReadMemory()) {
            UpperPrintTextCentered(0, 320, 100, "MEM: " + String(Memory) + " E");
          } else {
            UpperPrintTextCentered(0, 320, 100, "MEM: " + String(Memory));
          }
          break;
        case 3:
          Upper.setFreeFont(&FreeSansBold24pt7b);
          if (NotchWidth == 0) {
            UpperPrintTextCentered(0, 320, 100, "Narrow notch");
          } else if (NotchWidth == 1) {
            UpperPrintTextCentered(0, 320, 100, "Wide notch");
          }
          break;
        case 4:
          Upper.setFreeFont(&FreeSansBold24pt7b);
          UpperPrintTextCentered(0, 320, 100, "Cont W: " + String(ContourWidth));
          break;
        case 5:
          Upper.setFreeFont(&FreeSansBold24pt7b);
          UpperPrintTextCentered(0, 320, 100, "Cont L: " + String(ContourLevel));
          break;
        case 6:
          Upper.setFreeFont(&FreeSansBold24pt7b);
          UpperPrintTextCentered(0, 320, 100, "PWR: " + String(PowerLevel) + "W");
          Lower.fillRect(0, 0, Lower.width(), Lower.height(), TFT_BLACK);
          Lower.setFreeFont(&FreeSansBold12pt7b);
          TFT_BACKGROUND = TFT_BLACK;
          LowerPrintText(1, 28, "PWR: " + String(PowerLevel) + "W");
          Lower.pushSprite(0, 181);
          TFT_BACKGROUND = TFT_EBONY;
          PreviousPower = PowerLevel;  //In order to avoid to force status update because of the power change
          break;
        case 7:
          //The following block of code is done to avoid some flickering while changing the frequentcy on the tft screen
          Upper.setFreeFont(&FreeSansBold24pt7b);
          TFT_FOREGROUND = TFT_YELLOW;
          if (CurrentFrequencyRX > 9999999) {
            UpperPrintTextCentered(0, 320, 100, String(CurrentFrequencyRX).substring(0, 2) + "." + String(CurrentFrequencyRX).substring(2, 5) + "." + String(CurrentFrequencyRX).substring(5, 8));
          } else {
            UpperPrintTextCentered(0, 320, 100, String(CurrentFrequencyRX).substring(0, 1) + "." + String(CurrentFrequencyRX).substring(1, 4) + "." + String(CurrentFrequencyRX).substring(4, 7));
          }
          break;
        default:
          break;
      }
      Upper.pushSprite(0, 8);
      start = millis();
    }

    if (!RE4.digitalRead(SS_SWITCH)) {  // Reads if the rotator button is pressed...
      do {
      } while (!RE4.digitalRead(SS_SWITCH));  //...end wait until released
      encoder_position4 = 0;
      RE4.setEncoderPosition(0);
      return;
    }
  } while (millis() < start + 1000);
  encoder_position4 = 0;
  RE4.setEncoderPosition(0);
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
  int Array_end;
  int i, Result;

#if (RIG_Model == 'FTDX101MP')
  int PowerArray[23] = { 0, 29, 54, 69, 82, 97, 111, 120, 130, 139, 149, 157, 163, 171, 176, 184, 190, 197, 203, 210, 216, 223, 255 };
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

void Interupt1() {
  if (digitalRead(TXGND) == 0) {
    start = 0;  // Exit from possible menus by setting to 0 all user input wait time. Give priority to handle the transmition.
  }
}