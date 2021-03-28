#include <Arduino.h>

#include <LiquidCrystal_I2C.h>
#include <SPI.h>
#include <Adafruit_Fingerprint.h>
#include <Keypad.h>
#include "algorithm_by_RF.h"
#include "max30102.h"
#include "MAX30105.h" // MAX3010x library
#include "heartRate.h"
#include <Wire.h>
// #define DEBUG                                           // Uncomment for debug output to the Serial stream

LiquidCrystal_I2C lcd(0x27, 20, 4);
// Interrupt pin
const byte oxiInt = 13; // pin connected to MAX30102 INT
byte readLED = 2;       //Blinks with each data read
MAX30105 particleSensor;
uint32_t aun_ir_buffer[BUFFER_SIZE];  //infrared LED sensor data
uint32_t aun_red_buffer[BUFFER_SIZE]; //red LED sensor data
float old_n_spo2;                     // Previous SPO2 value
uint8_t uch_dummy, k;
bool isFingerPlaced = false;
const byte RATE_SIZE = 4; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE];    //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred
float beatsPerMinute;
int beatAvg;
long irValue;
uint8_t revID;
uint8_t partID;
float n_spo2, ratio, correl; //SPO2 value
int8_t ch_spo2_valid;        //indicator to show if the SPO2 calculation is valid
int32_t n_heart_rate;        //heart rate value
int8_t ch_hr_valid;          //indicator to show if the heart rate calculation is valid
int32_t i;
int first = 1;

#define COLUMS 16
#define ROWS 2
#define PAGE ((COLUMS) * (ROWS))
#define fpSerial Serial2

const int ROW_NUM = 4;    //four rows
const int COLUMN_NUM = 4; //four columns
char keys[ROW_NUM][COLUMN_NUM] = {
    {'1', '2', '3', 'A'},
    {'4', '5', '6', 'B'},
    {'7', '8', '9', 'C'},
    {'*', '0', '#', 'D'}};
uint8_t id;
byte pin_rows[ROW_NUM] = {14, 27, 26, 25};      //connect to the row pinouts of the keypad
byte pin_column[COLUMN_NUM] = {33, 32, 35, 34}; //connect to the column pinouts of the keypad
char key;

Adafruit_Fingerprint finger = Adafruit_Fingerprint(&fpSerial);
Keypad keypad = Keypad(makeKeymap(keys), pin_rows, pin_column, ROW_NUM, COLUMN_NUM);

void processHRandSPO2();
void bpm();
void take_bpm();
uint8_t getFingerprintEnroll()
{
  int p = -1;
  lcd.clear();
  lcd.print("place finter");
  Serial.println(id);
  while (p != FINGERPRINT_OK)
  {
    p = finger.getImage();
    switch (p)
    {
    case FINGERPRINT_OK:
      Serial.println("Image taken");
      break;
    case FINGERPRINT_NOFINGER:
      Serial.println(".");
      break;
    case FINGERPRINT_PACKETRECIEVEERR:
      Serial.println("Communication error");
      break;
    case FINGERPRINT_IMAGEFAIL:
      Serial.println("Imaging error");
      break;
    default:
      Serial.println("Unknown error");
      break;
    }
  }

  // OK success!

  p = finger.image2Tz(1);
  switch (p)
  {
  case FINGERPRINT_OK:
    Serial.println("Image converted");
    break;
  case FINGERPRINT_IMAGEMESS:
    Serial.println("Image too messy");
    return p;
  case FINGERPRINT_PACKETRECIEVEERR:
    Serial.println("Communication error");
    return p;
  case FINGERPRINT_FEATUREFAIL:
    Serial.println("Could not find fingerprint features");
    return p;
  case FINGERPRINT_INVALIDIMAGE:
    Serial.println("Could not find fingerprint features");
    return p;
  default:
    Serial.println("Unknown error");
    return p;
  }

  lcd.clear();
  lcd.print("Remove finger");
  delay(2000);
  p = 0;
  while (p != FINGERPRINT_NOFINGER)
  {
    p = finger.getImage();
  }
  Serial.print("ID ");
  Serial.println(id);
  p = -1;
  lcd.clear();
  lcd.print("Place same finger");
  while (p != FINGERPRINT_OK)
  {
    p = finger.getImage();
    switch (p)
    {
    case FINGERPRINT_OK:
      Serial.println("Image taken");
      break;
    case FINGERPRINT_NOFINGER:
      Serial.print(".");
      break;
    case FINGERPRINT_PACKETRECIEVEERR:
      Serial.println("Communication error");
      break;
    case FINGERPRINT_IMAGEFAIL:
      Serial.println("Imaging error");
      break;
    default:
      Serial.println("Unknown error");
      break;
    }
  }

  // OK success!

  p = finger.image2Tz(2);
  switch (p)
  {
  case FINGERPRINT_OK:
    Serial.println("Image converted");
    break;
  case FINGERPRINT_IMAGEMESS:
    Serial.println("Image too messy");
    return p;
  case FINGERPRINT_PACKETRECIEVEERR:
    Serial.println("Communication error");
    return p;
  case FINGERPRINT_FEATUREFAIL:
    Serial.println("Could not find fingerprint features");
    return p;
  case FINGERPRINT_INVALIDIMAGE:
    Serial.println("Could not find fingerprint features");
    return p;
  default:
    Serial.println("Unknown error");
    return p;
  }

  // OK converted!
  Serial.print("Creating model for #");
  Serial.println(id);

  p = finger.createModel();
  if (p == FINGERPRINT_OK)
  {
    lcd.clear();
    lcd.print("Prints matched!");
  }
  /*else if (p == FINGERPRINT_PACKETRECIEVEERR)
	{
		Serial.println("Communication error");
		return p;
	}
	else if (p == FINGERPRINT_ENROLLMISMATCH)
	{
		Serial.println("Fingerprints did not match");
		return p;
	}*/
  else
  {
    lcd.clear();
    lcd.print("Unknown error");
    return p;
  }

  Serial.print("ID ");
  Serial.println(id);
  p = finger.storeModel(id);
  if (p == FINGERPRINT_OK)
  {
    lcd.clear();
    lcd.print("Stored!");
    delay(1000);
  }
  /*else if (p == FINGERPRINT_PACKETRECIEVEERR)
	{
		Serial.println("Communication error");
		return p;
	}
	else if (p == FINGERPRINT_BADLOCATION)
	{
		Serial.println("Could not store in that location");
		return p;
	}
	else if (p == FINGERPRINT_FLASHERR)
	{
		Serial.println("Error writing to flash");
		return p;
	}*/
  else
  {
    lcd.clear();
    lcd.print("saving error");
    return p;
  }

  return true;
}
int getFingerprintIDez()
{
  uint8_t p = finger.getImage();
  if (p != FINGERPRINT_OK)
    return -1;

  p = finger.image2Tz();
  if (p != FINGERPRINT_OK)
    return -1;

  p = finger.fingerFastSearch();
  if (p != FINGERPRINT_OK)
    return -1;

  // found a match!
  Serial.print("Found ID #");
  Serial.print(finger.fingerID);
  Serial.print(" with confidence of ");
  Serial.println(finger.confidence);
  return finger.fingerID;
}
void enrole()
{
  String input_id;
  while (1)
  {
    lcd.clear();
    lcd.print("Enter ID # 1-127");
    while (1)
    {
      key = keypad.getKey();
      if (key)
      {
        if (key == '#')
        {

          Serial.println("Break");
          break;
        }
        else if (key == '1' || key == '2' || key == '3' || key == '4' || key == '5' || key == '6' || key == '7' || key == '8' || key == '9' || key == '0')
        {
          input_id += key;
          lcd.clear();
          lcd.print("Enter ID # 1-127");
          lcd.setCursor(0, 1);
          lcd.print(input_id);
        }
        else if (key == 'C')
        {

          break;
        }
      }
    }
    if (key == 'C')
    {

      lcd.clear();
      lcd.print("Cancel");
      break;
    }
    id = input_id.toInt();
    if (id == 0)
    {
      lcd.clear();
      lcd.print("0 not Allowed");
    }
    else
    {
      lcd.clear();
      lcd.print("Enrolling ID # ");
      lcd.print(id);
      while (!getFingerprintEnroll())
        ;
    }
  }
}
// int limit = 800;
// int data[5][2] = {{1, 1},
// 				  {1, 1},
// 				  {1, 1},
// 				  {1, 1},
// 				  {1, 1}};
// int plus_one[5][2] = {{1, 1},
// 					  {0, 1},
// 					  {0, 0},
// 					  {1, 0},
// 					  {1, 1}};
// int minus_one[5][2] = {{1, 1},
// 					   {1, 0},
// 					   {0, 0},
// 					   {0, 1},
// 					   {1, 1}};
int ii;
int j;
bool equal = true;
int person = 0;

void setup()
{
  lcd.init(); // initialize the lcd
  lcd.backlight();
  Serial.begin(9600);
  Wire.setClock(400000); // Set I2C speed to 400kHz

  pinMode(oxiInt, INPUT); //pin D10 connects to the interrupt output pin of the MAX30102
  pinMode(readLED, OUTPUT);
  Wire.begin();

  particleSensor.begin(Wire, I2C_SPEED_STANDARD); // Use default I2C port, 400kHz speed
  particleSensor.setup();
  particleSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running
  particleSensor.setPulseAmplitudeGreen(0);  //Turn off Green LED
  Serial.begin(115200);
  maxim_max30102_reset(); //resets the MAX30102
  delay(1000);
  maxim_max30102_read_reg(REG_INTR_STATUS_1, &uch_dummy); //Reads/clears the interrupt status register
  maxim_max30102_init();                                  //initialize the MAX30102
  old_n_spo2 = 0.0;

  maxim_max30102_read_reg(0xFE, &revID);
  maxim_max30102_read_reg(0xFF, &partID);

#ifdef DEBUG
  Serial.print("Rev ");
  Serial.print(revID, HEX);
  Serial.print(" Part ");
  Serial.println(partID, HEX);
  Serial.print("-------------------------------------");
  while (1)
  {
  }
#endif
}

//Continuously taking samples from MAX30102.  Heart rate and SpO2 are calculated every ST seconds
void loop()
{

  lcd.clear();
  lcd.print("OK");
  delay(100);
  take_bpm();
  // maxim_max30102_reset();
  // delay(1000);
  // maxim_max30102_read_reg(REG_INTR_STATUS_1, &uch_dummy); //Reads/clears the interrupt status register
  // maxim_max30102_init();                                  //initialize the MAX30102
  // old_n_spo2 = 0.0;

  // maxim_max30102_read_reg(0xFE, &revID);
  // maxim_max30102_read_reg(0xFF, &partID);
  // n_spo2 = -999;
  // while (n_spo2 == -999)
  // {
  //   processHRandSPO2();
  //   Serial.print(n_spo2);
  // }
  // Serial.println(n_spo2);
  // first = 1;
  // for (int ii = 0; ii < 200; ii++)
  // {
  //   bpm();
  // }
  // lcd.clear();
  // Serial.print("IR=");
  // Serial.print(irValue);
  // Serial.print(", BPM=");
  // lcd.print("BPM= ");
  // lcd.print(beatAvg);
  // Serial.print(beatsPerMinute);
  // Serial.print(", Avg BPM=");
  // Serial.print(beatAvg);

  // if (irValue < 50000)
  //   Serial.print(" No finger?");

  // Serial.println();

  // Serial.println("Break.........");
}
void take_bpm()
{
  maxim_max30102_reset();
  delay(1000);
  maxim_max30102_read_reg(REG_INTR_STATUS_1, &uch_dummy); //Reads/clears the interrupt status register
  maxim_max30102_init();                                  //initialize the MAX30102
  old_n_spo2 = 0.0;

  maxim_max30102_read_reg(0xFE, &revID);
  maxim_max30102_read_reg(0xFF, &partID);
  n_spo2 = -999;
  // while (n_spo2 == -999)
  // {
  processHRandSPO2();
  // }
  Serial.println(n_spo2);
  first = 1;
  irValue = particleSensor.getIR();
  if (irValue > 50000)
  {
    for (int ii = 0; ii < 200; ii++)
    {
      bpm();
    }
    lcd.clear();
    Serial.print("IR=");
    Serial.print(irValue);
    Serial.print(", BPM=");
    lcd.print("BPM= ");
    lcd.print(beatAvg);
    Serial.print(beatsPerMinute);
    Serial.print(", Avg BPM=");
    Serial.print(beatAvg);
  }
  else
  {
    Serial.print(" No finger?");
  }

  Serial.println();

  Serial.println("Break.........");
}
void bpm()
{
  irValue = particleSensor.getIR();
  if (checkForBeat(irValue) == true)
  {
    digitalWrite(readLED, !digitalRead(readLED));
    //We sensed a beat!
    long delta = millis() - lastBeat;
    lastBeat = millis();

    if (1)
    {
      first = 0;
      beatsPerMinute = 60 / (delta / 1000.0);

      if (beatsPerMinute < 255 && beatsPerMinute > 20)
      {
        rates[rateSpot++] = (byte)beatsPerMinute; //Store this reading in the array
        rateSpot %= RATE_SIZE;                    //Wrap variable

        //Take average of readings
        beatAvg = 0;
        for (byte x = 0; x < RATE_SIZE; x++)
          beatAvg += rates[x];
        beatAvg /= RATE_SIZE;
      }
    }
  }
}
void processHRandSPO2()
{
  irValue = particleSensor.getIR(); // Reading the IR value it will permit us to know if there's a finger on the sensor or not
  if (irValue > 50000)
  {
    if (isFingerPlaced == false)
    {
      isFingerPlaced = true;
      Serial.println("measuring vitals...");
    }

    //buffer length of BUFFER_SIZE stores ST seconds of samples running at FS sps
    //read BUFFER_SIZE samples, and determine the signal range
    for (int ii = 0; ii < BUFFER_SIZE; ii++)
    {

      while (digitalRead(oxiInt) == 1)
      { //wait until the interrupt pin asserts
        yield();
      }
      digitalWrite(readLED, !digitalRead(readLED));

      //IMPORTANT:
      //IR and LED are swapped here for MH-ET MAX30102. Check your vendor for MAX30102
      //and use this or the commented-out function call.
      maxim_max30102_read_fifo((aun_ir_buffer + i), (aun_red_buffer + ii));
      //maxim_max30102_read_fifo((aun_red_buffer+i), (aun_ir_buffer+i));
    }

    //calculate heart rate and SpO2 after BUFFER_SIZE samples (ST seconds of samples) using Robert's method
    rf_heart_rate_and_oxygen_saturation(aun_ir_buffer, BUFFER_SIZE, aun_red_buffer, &n_spo2, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid, &ratio, &correl);
    Serial.println(n_spo2);
#ifdef DEBUG
    Serial.println("--RF--");
    Serial.print("\tSpO2: ");
    Serial.print(n_spo2);
    Serial.print("\tHR: ");
    Serial.print(n_heart_rate, DEC);
    Serial.print("\t");
    Serial.println("------");
#endif
  }
  else
  {
    digitalWrite(readLED, HIGH);
    isFingerPlaced = false;
    Serial.println("No finger");
    Serial.println("Place your finger on sensor and wait..");
  }
}