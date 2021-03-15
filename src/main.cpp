#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "algorithm_by_RF.h"
#include "max30102.h"
#include "MAX30105.h" // MAX3010x library
#include "heartRate.h"
// #define DEBUG                                           // Uncomment for debug output to the Serial stream


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

void processHRandSPO2();
void bpm();


void setup()
{
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
  while(1){}
#endif

}

//Continuously taking samples from MAX30102.  Heart rate and SpO2 are calculated every ST seconds
void loop()
{
  maxim_max30102_reset();
  delay(1000);
  maxim_max30102_read_reg(REG_INTR_STATUS_1, &uch_dummy); //Reads/clears the interrupt status register
  maxim_max30102_init();                                  //initialize the MAX30102
  old_n_spo2 = 0.0;

  maxim_max30102_read_reg(0xFE, &revID);
  maxim_max30102_read_reg(0xFF, &partID);
  n_spo2 = -999;
  while (n_spo2 == -999)
  {
    processHRandSPO2();
  }
  Serial.println(n_spo2);
  first = 1;
  for (int i = 0; i < 200; i++)
  {
    bpm();
  }
  Serial.print("IR=");
  Serial.print(irValue);
  Serial.print(", BPM=");
  Serial.print(beatsPerMinute);
  Serial.print(", Avg BPM=");
  Serial.print(beatAvg);

  if (irValue < 50000)
    Serial.print(" No finger?");

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
    for (int i = 0; i < BUFFER_SIZE; i++)
    {
      while (digitalRead(oxiInt) == 1)
      { //wait until the interrupt pin asserts
        yield();
      }
      digitalWrite(readLED, !digitalRead(readLED));

      //IMPORTANT:
      //IR and LED are swapped here for MH-ET MAX30102. Check your vendor for MAX30102
      //and use this or the commented-out function call.
      maxim_max30102_read_fifo((aun_ir_buffer + i), (aun_red_buffer + i));
      //maxim_max30102_read_fifo((aun_red_buffer+i), (aun_ir_buffer+i));
    }

    //calculate heart rate and SpO2 after BUFFER_SIZE samples (ST seconds of samples) using Robert's method
    rf_heart_rate_and_oxygen_saturation(aun_ir_buffer, BUFFER_SIZE, aun_red_buffer, &n_spo2, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid, &ratio, &correl);

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