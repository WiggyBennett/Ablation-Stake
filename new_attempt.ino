#include <Adafruit_GPS.h>
#include <AltSoftSerial.h>
#include <EepromAnything.h>
#include <Time.h>
#include <DS1307RTC.h>
#include <Wire.h>    
#include <Arduino.h>
#include <stdlib.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <NewPing.h>
#include <EEPROM.h>

//SoftwareSerial mySerial(8, 9);
AltSoftSerial mySerial;

Adafruit_GPS GPS(&mySerial);

//Setup the hardware pins for the ultrasonic sensor
#define trigPin 7
#define echoPin 3
#define powerPin 4
#define MAX_DISTANCE 200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

#define LDRpowerPin 12
#define lightSensorPin A0
#define temperaturePin A3

#define GPSEnable 5
#define Alarm 2

boolean saveDone = false;

struct record{
  uint16_t Year;
  uint8_t Month;
  uint8_t Day;
  uint8_t Hour;
  uint8_t Minute;
  float Latitude;
  float Longitude;
  float Altitude;
  uint16_t Distance;
  uint16_t Temperature;
  uint16_t Light;
};
  record myRecord;
  record output;
  unsigned int address;
  boolean awake = false;
  char line[80] = {0};
  tmElements_t tm;
  boolean alarmFired;
  byte readInterval;


  
  byte CTRLWord = 0x05; //bits 0 & 2 set to enable alarm 1 and to set the interrupt control bit    
 
  
void setup()
{
  EEPROM.get(0, readInterval);//The number of hours between readings  
  if (readInterval < 1){readInterval = 1;}
  Wire.begin();
  // We will use the main serial port for programming, downloading data and debugging
  //The software serial will be used for talking to the GPS
  Serial.begin(9600);
  GPS.begin(9600);

  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
  delay(1000);

  //The first two bytes of the EEPROM are reserved for the address pointer. Whenever data is written to EEPROM, 
  //the pointer should be incremented such that the address pointer points to the next available address
  EEPROM_readAnything(0, address);//this reads the current address pointer from EEPROM
  
  //Setup the hardware pins
  
  pinMode(powerPin, OUTPUT);
  pinMode(LDRpowerPin, OUTPUT);
  pinMode(GPSEnable, OUTPUT);
  pinMode(Alarm, INPUT);

  //Set ultrasonic sensor to low power
  digitalWrite(powerPin, HIGH);
  
  //Setup the RTC
  RTC.setAlarm();
  RTC.setCTRLWord(CTRLWord);
  RTC.resetAlarmInterrupts();
  
  
  attachInterrupt(0,wakeUp, FALLING);
  digitalWrite(GPSEnable, HIGH);
  
  
}
     
     
     
uint32_t timer = millis();

void loop() // run over and over again

{
  
  if(alarmFired){
    
    Serial.println("CPU Awake");
    alarmFired = false;
    
    RTC.read(tm);
    if(tm.Hour % readInterval == 0){
      Serial.println("Waiting for GPS fix...");
      digitalWrite(GPSEnable, HIGH);   
    }else{
      Serial.println("No reading needed, going back to sleep.");
      digitalWrite(GPSEnable, LOW);
      sleepNow();
    } 
    
    GPS.fixquality = 0;
    saveDone = false; 
  }
  
  
  GPS.read();
    if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return; // we can fail to parse a sentence in which case we should just wait for another
  }

    
    if (GPS.fixquality>0 && GPS.fix>0 && GPS.satellites>4) {
      delay(10000);//Wait a bit to make sure we've read the GPS time before setting the clock
      //Once we have a fix, update the real time clock's time to the GPS time.
      tm.Hour = GPS.hour;
      tm.Minute = GPS.minute;
      tm.Second = GPS.seconds;
      tm.Day = GPS.day;
      tm.Month = GPS.month;
      tm.Year = CalendarYrToTm(2000 + GPS.year);
      RTC.write(tm);
        
      if(!saveDone){        
        //Update the data structure with the new GPS data
        myRecord.Year = 2000 + GPS.year;
        myRecord.Month = GPS.month;
        myRecord.Day = GPS.day;
        myRecord.Hour = GPS.hour;
        myRecord.Minute = GPS.minute;
        myRecord.Latitude = GPS.latitudeDegrees;
        myRecord.Longitude = GPS.longitudeDegrees;
        myRecord.Altitude = GPS.altitude;
        digitalWrite(GPSEnable, LOW);
        
        //Take the measurements
        myRecord.Distance = Measure(); //ultrasonic distance sensor(cm)
        myRecord.Light = readMyLightSensor(); //light intensity sensor(resistance?) 
        myRecord.Temperature = readTemperatureSensor();//temperature measured in tenths of a degree C       
    
        //Write the data to EEPROM
        EEPROM_writeAnything(address, myRecord);
        //Increment the address pointer
        address = address + sizeof(record);
        EEPROM_writeAnything(0, address);
        saveDone = true;
        sleepNow();
    }

  }

   
  if (Serial){ 
    if (Serial.available()>0){ 
      if (read_line(line, sizeof(line)) < 0) {
        Serial.println("Error: line too long");
        return; // skip command processing and try again on next iteration of loop
      }
    }
      if (strcmp(line, "dump") == 0) {
          Serial.println("Timestamp format(yyyy-mm-dd hh:mm), latitude, longitude, altitude(m), distance(cm), temperature(degC), light(lux)");
        for (int i=2;i<address;i+=sizeof(record)){
          EEPROM_readAnything(i,output);//Read the each record and print it to the serial port
          //output = myRecord;
          Serial.print(output.Year);
          Serial.print("-");
          printDigits(output.Month);
          Serial.print("-");
          printDigits(output.Day);
          Serial.print(" ");        
          printDigits(output.Hour);
          Serial.print(":");  
          printDigits(output.Minute);
          Serial.print(",");
          Serial.print(output.Latitude,4);
          Serial.print(",");
          Serial.print(output.Longitude,4);
          Serial.print(",");
          Serial.print(output.Altitude);
          Serial.print(",");
          Serial.print(output.Distance);
          Serial.print(",");
          Serial.print((float) output.Temperature/10);
          Serial.print(",");
          Serial.println(output.Light);
        }
        line[0] = 0;        
      }else if (strcmp(line, "records") == 0) {
        Serial.print((address-2)/sizeof(record));
        Serial.println(" records");
        line[0] = 0;
      }else if (strcmp(line, "address") == 0) {
        Serial.println(address);
        line[0] = 0;
      }else if (strcmp(line, "year") == 0) {
        Serial.println(GPS.year);
        line[0] = 0;
      }else if (strcmp(line, "light") == 0) {
        Serial.print(readMyLightSensor());
        Serial.println(" lux");
        line[0] = 0;    
      }else if (strcmp(line, "temp") == 0) {
        float temp = (float)readTemperatureSensor()/10;
        Serial.print(temp);
        Serial.println("°C");
        line[0] = 0;         
      }else if (strcmp(line, "status word") == 0) {
        Serial.println(RTC.readStatusWord());
        line[0] = 0;
      }else if (strcmp(line, "Erase All") == 0) {
        eraseMemory();
        Serial.println("Memory Erased");
        line[0] = 0;
      }else if (strcmp(line, "altitude") == 0) {
        Serial.print("Altitude: ");
        Serial.println(GPS.altitude);
        line[0] = 0;
      }else if (strcmp(line, "fix") == 0) {
        Serial.print("Fix Quality: ");
        Serial.println((int)GPS.fixquality);
        line[0] = 0;  
      }else if (strcmp(line, "satellites") == 0) {
        Serial.print("Satellites: ");
        Serial.println((int)GPS.satellites);
        line[0] = 0;          
      }else if (strcmp(line, "measure") == 0) {
        int distance = Measure();
        Serial.print(distance);
        Serial.println("cm");
        line[0] = 0;        
      }else if (strcmp(line, "RTC Time") == 0) {
            if(RTC.read(tm)){
              Serial.print("RTC Found, Time = ");
              printDigits(tm.Hour);
              Serial.write(':');
              printDigits(tm.Minute);
              Serial.write(':');
              printDigits(tm.Second);
              Serial.print(", Date (D/M/Y) = ");
              Serial.print(tm.Day);
              Serial.write('/');
              Serial.print(tm.Month);
              Serial.write('/');
              Serial.print(tmYearToCalendar(tm.Year));
              Serial.println();
            }else{
              Serial.println("No Real Time Clock found");
            }
            line[0] = 0;
      } else if (strncmp(line, "Interval", 8 ) == 0) {
        if(strlen(line)>9)//the user has supplied a new value
          {
            Serial.println(strlen(line));
            readInterval = atoi(&line[9]);
            EEPROM.put(0,readInterval);//Write the read interval to non-volatile memory(on chip, not external EEPROM)  
            Serial.println("New value for read interval saved.");
          }      
        Serial.print("Interval = ");
        Serial.println(readInterval);
        line[0] = 0;           
      } else if (strcmp(line, "read") == 0) {
        Serial.println(EEPROM.read(0));// Empty line: no command 
        line[0] = 0;       
      } else if (strcmp(line, "") == 0) {
        ;// Empty line: no command
      } else {
        Serial.print("Error: unknown command: \"");
        Serial.print(line);
        Serial.println("\" Available commands: ");
        Serial.print("\"");
        Serial.print("dump");
        Serial.println("\",");
        Serial.print("\"");
        Serial.print("records");
        Serial.println("\",");
        Serial.print("\"");
        Serial.print("address");
        Serial.println("\","); 
         Serial.print("\"");
        Serial.print("year");
        Serial.println("\",");
         Serial.print("\"");
        Serial.print("status word");
        Serial.println("\",");  
        Serial.print("\"");
        Serial.print("Erase All");
        Serial.println("\","); 
         Serial.print("\"");
        Serial.print("altitude");
        Serial.println("\","); 
         Serial.print("\"");
        Serial.print("temp(for temperature)");
        Serial.println("\","); 
         Serial.print("\"");
        Serial.print("light");
        Serial.println("\","); 
         Serial.print("\"");         
        Serial.print("fix");
        Serial.println("\","); 
         Serial.print("\"");         
        Serial.print("measure");        
        Serial.println("\",");  
        Serial.print("\"");         
        Serial.print("Interval to read, Interval:n to set a new value.");        
        Serial.println("\",");         
        Serial.print("\"");
        Serial.print("RTC Time");
        Serial.println("\"");        
        line[0] = 0;
      }
  }   

}

void wakeUp(){
  detachInterrupt(0);
  alarmFired = true;
}

// Function to trigger the ultrasonic sensor and calculate the distance based on the time of flight
unsigned int Measure(){
  NewPing sonar(trigPin, echoPin, 200); // NewPing setup of pins and maximum distance.

  digitalWrite(powerPin, LOW);
  delay(100);
  unsigned int uS = sonar.ping_median(10);
  digitalWrite(powerPin, HIGH);//High switches it off, conserving power
  unsigned int distance = uS / US_ROUNDTRIP_CM;
  if(distance >= 2){distance = distance - 2;} //Subtract 2cm as the sensor is recessed by 2cm inside the housing
  return distance;  
}

uint16_t readTemperatureSensor(){
  //Take a temperature measurement
  analogReference(INTERNAL);  
  for (int i=0; i<5; i++){
  analogRead(temperaturePin);
  delay(100);
  }

  int tempSensorValue = analogRead(temperaturePin);
  //analogReference(DEFAULT);
  //Serial.println(tempSensorValue);
  //Substitute the correct reference voltage. default reference is Vcc, internal is 1.1V
  float voltage = tempSensorValue * (1.009 / 1024.0);//set to 1.009 empiracally because it was reading high
  //store value as tenths of a degree. 10mV = 1 degC.
  return (uint16_t)(voltage*1000);
}

void printDigits(int digits) {
  // utility function for digital clock display: prints preceding colon and leading 0
  if(digits < 10)
    Serial.print('0');
  Serial.print(digits);
}

void sleepNow()
{
  RTC.resetAlarmInterrupts();
  Serial.println("CPU going to sleep"); 
    digitalWrite(13,LOW);
  /* Setup pin2 as an interrupt and attach handler. */
  attachInterrupt(0, wakeUp, FALLING);
  delay(100);
  
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  
  sleep_enable();
  
  sleep_mode();
  
  /* The program will continue from here. */
  
  /* First thing to do is disable sleep. */
  sleep_disable(); 
}

void eraseMemory()
{
    address = 2;//reset the address pointer. The first 2 bytes are for the address pointer itself.
    //There's no real point in deleting the data, resetting the pointer will cause it to be overwritten.
    EEPROM_writeAnything(0, address); 
}

uint16_t readMyLightSensor(){
      //Take a light intensity measurement
    digitalWrite(LDRpowerPin, HIGH);
    delay(100);
    analogReference(DEFAULT);
    int lightSensorValue = analogRead(lightSensorPin);
    //Serial.print("lightSensorValue: ");
    //Serial.println(lightSensorValue);
    float volts = (3.3/1024)*lightSensorValue;
    //Serial.print("Volts:");
    //Serial.println(volts);
    float resistance = (10000 * (3.3 - volts))/volts;    
    float lux = 500/(resistance/1000);
    //Serial.print("Lux: ");
    //Serial.println(lux);
    return (uint16_t)lux;
    digitalWrite(LDRpowerPin, LOW);
}

int read_line(char* buffer, int bufsize)
{
  for (int index = 0; index < bufsize; index++) {
    // Wait until characters are available
    while (Serial.available() == 0) {
    }

    char ch = Serial.read(); // read next character
    //Serial.print(ch); // echo it back: useful with the serial monitor (optional)

    if (ch == '\n') {
      buffer[index] = 0; // end of line reached: null terminate string
      return index; // success: return length of string (zero if string is empty)
    }

    buffer[index] = ch; // Append character to buffer
  }

  // Reached end of buffer, but have not seen the end-of-line yet.
  // Discard the rest of the line (safer than returning a partial line).

  char ch;
  do {
    // Wait until characters are available
    while (Serial.available() == 0) {
    }
    ch = Serial.read(); // read next character (and discard it)
    Serial.print("empty"); // echo it back
  } while (ch != '\n');

  buffer[0] = 0; // set buffer to empty string even though it should not be used
  return -1; // error: return negative one to indicate the input was too long
}
