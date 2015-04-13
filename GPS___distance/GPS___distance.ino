
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <EepromAnything.h>
#include <Time.h>
#include <DS1307RTC.h>
#include <Wire.h>    
#include <Arduino.h>
#include <stdlib.h>
#include <avr/sleep.h>
 


SoftwareSerial mySerial(6, 5);

Adafruit_GPS GPS(&mySerial);


// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences. 
#define GPSECHO  true

//Setup the hardware pins for the ultrasonic sensor
#define trigPin 3
#define echoPin 7
#define powerPin 4
#define GPSEnable 8
#define Alarm 2



// this keeps track of whether we're using the interrupt
// off by default!
boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy

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
  
  tmElements_t tm;

  
  char inData[20]; // Allocate some space for the string
  char inChar=-1; // Where to store the character read
  byte index = 0; // Index into array; where to store the character
  
  byte CTRLWord = 0x05; //bits 0 & 3 set to enable alarm 1 and to set the interrupt control bit

void setup()  
{
  Wire.begin();
  Serial.begin(9600);
  

  //The first two bytes of the EEPROM are reserved for the address pointer. Whenever data is written to EEPROM, 
  //the pointer should be incremented such that the address pointer points to the next available address
  EEPROM_readAnything(0, address);//this reads the current address pointer from EEPROM
  
  myRecord.Year = 2015;
  myRecord.Month = 4;
  myRecord.Day = 7;
  myRecord.Hour = 10;
  myRecord.Minute = 23;
  myRecord.Latitude = 52.1423;
  myRecord.Longitude = -0.4613;
  myRecord.Altitude = -19.10;
  myRecord.Distance = 2547;
  myRecord.Temperature = 258;//25.8 degrees
  myRecord.Light = 65534;//Not sure what this will be yet


  

  
    
  // We will use the main serial port for programming, downloading data and debugging
  //The software serial will be used for talking to the GPS

  
  //Setup the hardware pins
  
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(powerPin, OUTPUT);
  pinMode(GPSEnable, OUTPUT);
  pinMode(Alarm, INPUT);
  
  //Setup the RTC
  RTC.setCTRLWord(CTRLWord);
  RTC.setAlarm();
  
  attachInterrupt(0,wakeUp, RISING);

  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
  
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time
  
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz

  useInterrupt(true);

  delay(1000);
  // Ask for firmware version
  mySerial.println(PMTK_Q_RELEASE);
}


// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
#ifdef UDR0
  if (GPSECHO)
   // if (c) UDR0 = c; 
  ; 
    // writing direct to UDR0 is much much faster than Serial.print 
    // but only one character can be written at a time. 
#endif
}


 


void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}

uint32_t timer = millis();

// Function to trigger the ultrasonic sensor and calculate the distance based on the time of flight
int Measure(){
  int duration, distance;
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10); 
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = (duration/2) / 29.1;
  return distance;  
}

void wakeUp(){
  detachInterrupt(0);
  sleep_disable();
}

void printDigits(int digits) {
  // utility function for digital clock display: prints preceding colon and leading 0
  if(digits < 10)
    Serial.print('0');
  Serial.print(digits);
}

void loop()                     // run over and over again
{
    digitalWrite(GPSEnable, HIGH);
    
    // if a sentence is received, we can check the checksum, parse it...
    if (GPS.newNMEAreceived()) {
      if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
        return;  // we can fail to parse a sentence in which case we should just wait for another
    }
  
  
  
    RTC.resetAlarmInterrupts();
  
      if (GPS.fix) {
        
  
        //Once we have a fix, update the real time clock's time to the GPS time.
        tm.Hour = hour();
        tm.Minute = minute();
        tm.Second = second();
        tm.Day = day();
        tm.Month = month();
        tm.Year = CalendarYrToTm(year());
        RTC.write(tm);
        
        //Update the data structure with the new GPS data
        myRecord.Year = 2000 + GPS.year;
        myRecord.Month = GPS.month;
        myRecord.Day = GPS.day;
        myRecord.Hour = GPS.hour;
        myRecord.Minute = GPS.minute;
        myRecord.Latitude = GPS.latitudeDegrees;
        myRecord.Longitude = GPS.longitudeDegrees;
        myRecord.Altitude = GPS.altitude;
        //Take a distance measurement
        digitalWrite(powerPin, HIGH);
        delay(500);
        myRecord.Distance = Measure();
        digitalWrite(powerPin, LOW);
        
        digitalWrite(GPSEnable, LOW);
        
        //Take a temperature measurment
        myRecord.Temperature = 258;//measured in tenths of a degree C
        
        //Take a light intensity measurement
        myRecord.Light = 350;//Measured in Lux
        
        //Write the data to EEPROM
        EEPROM_writeAnything(address, myRecord);
        //Increment the address pointer
        address = address + sizeof(record);
        EEPROM_writeAnything(0, address);
        
        if(!Serial){sleepNow();}
      }
  
  

  if (Serial){
      char line[80]; 
    if (Serial.available()>0){ 
      if (read_line(line, sizeof(line)) < 0) {
        Serial.println("Error: line too long");
        return; // skip command processing and try again on next iteration of loop
      }
    }
    
      if (strcmp(line, "dump") == 0) {
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
          Serial.print(output.Latitude);
          Serial.print(",");
          Serial.print(output.Longitude);
          Serial.print(",");
          Serial.print(output.Altitude);
          Serial.print(",");
          Serial.print(output.Distance);
          Serial.print(",");
          Serial.print(output.Temperature);
          Serial.print(",");
          Serial.println(output.Light);
        }        
      }else if (strcmp(line, "records") == 0) {
        Serial.println((address-2)/sizeof(record));
      }else if (strcmp(line, "address") == 0) {
        Serial.println(address);      
      }else if (strcmp(line, "year") == 0) {
        Serial.println(GPS.year);      
      }else if (strcmp(line, "status word") == 0) {
        Serial.println(RTC.readStatusWord()); 
      }else if (strcmp(line, "Erase All") == 0) {
        eraseMemory();
        Serial.println("Memory Erased");       
      }else if (strcmp(line, "altitude") == 0) {
        Serial.println(GPS.altitude);         
      }else if (strcmp(line, "RTC Time") == 0) {
            RTC.read(tm);
            Serial.print("Ok, Time = ");
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
      } else if (strcmp(line, "") == 0) {
        ;// Empty line: no command
      } else {
        Serial.print("Error: unknown command: \"");
        Serial.print(line);
        Serial.println("\" (available commands: \"off\", \"on\")");
      }
      
  }  
  

    
}

void sleepNow()
{
    sleep_enable();
    attachInterrupt(0, wakeUp, RISING);
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    cli();
   // sleep_bod_disable();
    sei();
    sleep_cpu();
}

void eraseMemory()
{
    address = 2;//reset the address pointer. The first 2 bytes are for the address pointer itself.
    //There's no real point in deleting the data, resetting the pointer will cause it to be overwritten.
    EEPROM_writeAnything(0, address); 
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


