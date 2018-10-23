/*  my own take on the reverse geocache box.
 *  heavily used the following info...
 *  https://learn.adafruit.com/reverse-geocache-engagement-box
 *  https://learn.adafruit.com/adafruit-ultimate-gps
 *  https://learn.adafruit.com/character-lcds/
 *  http://arduiniana.org/libraries/pwmservo/
 *  
 *  to make it work you'll need the libraries from the gps,lcd, and the pwmservo from the websites above.
 *  wiring is mirrored from the reverse geocache box.
 *  
 *  Servo
 *  black goes to ground.
 *  red goes to +5v.
 *  yellow to pin 9.
 *  
 *  GPS
 *  vin to +5v.
 *  gnd to gnd.
 *  RX to 2.
 *  TX to 3.
 *  
 *  LCD display
 *  VSS to gnd.
 *  VDD to +5V.
 *  V0 to middle pin of pot.
 *  RS to pin 7.
 *  RW to gnd.
 *  E to pin 8.
 *  D4 to pin 6.
 *  D5 to pin 10.
 *  D6 to pin 11.
 *  D7 to pin 12.
 *  A to +5v.
 *  K to gnd
 *  
 *  
 *  LCD pot
 *  one side +5v.
 *  other side gnd.
 *  middle to VO on the lcd display.
 *  
 *  change the Lat2 and Long2 to the decimal latitude and longitude of the place to unlock the box.
 *  distance from the destination is measured in meters.
 *  changes rEarth to the radius of earth in whatever measurement you'd prefer.
 *  
 *  first finished 9/19/18.
 * 
 */


#include <LiquidCrystal.h>
//library for the lcd display.
#include <Adafruit_GPS.h>
//library for the gps breakout board.
#include <SoftwareSerial.h>
//library for serial communication to the gps.
#include <PWMServo.h>
//library for the servo.
#include <math.h>
//library for the fancy maths.


LiquidCrystal lcd(7, 8, 6, 10, 11, 12);
//set the pins on the arduino to the pins on the lcd display.
SoftwareSerial mySerial(3, 2);
//set the pins on the arduino to the gps serial.
Adafruit_GPS GPS(&mySerial);
//create the object for the gps.
#define GPSECHO  false
//gps raw data.  true for debugging, false for quiet.
boolean usingInterrupt = false;
//variable to keep track if we're using the interrupt for the gps.
//leave off by default.
PWMServo myservo;
//create the servo object.
int pos = 0;
//variable to store the servo position.

float Lat2 = 33.761685 * 0.0174533;
float Long2 = -82.278742 * 0.0174533;
//gps location of the destination to unlock the box.
//needs to be in decimal degrees and not in the standard way.
//then convert the degrees to radians.
const float rEarth = 6371000.0;
//radius of the earth.
//in meters
bool lock = 0;
//variable to track if it's been closer than 20 meters to the location.
float d = 0;

void setup() {
  lcd.begin(16, 2);
  //initialize the lcd display.
  Serial.begin(112500);
  //start the serial for debugging.
  //has to be high for all that gps data.
  GPS.begin(9600);
  //start the gps serial output.
  //9600 is the default.
  myservo.attach(SERVO_PIN_A);
  //set the servo to pin A which is 9 on the uno by default.


  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  //turns on RMS (recommended minimum) and GGA (fix data).
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  //set the update rate to 1hz, the recomended highest speed.
  GPS.sendCommand(PGCMD_ANTENNA);
  //send updates on the antenna fix.
  //comment out to quiet.
  useInterrupt(true);
  //use an interrupt to check the gps unit for updates.

  myservo.write(0);
  //move the servo to the closed position. 
  
  Serial.println("Start");
  //test of the serial output, also end of the setup part.
}

// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;  
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
//timer.

void loop() {
  if (GPS.newNMEAreceived()) {
    //if we get data from the gps, do stuff with it.
    if (!GPS.parse(GPS.lastNMEA()))
    // this also sets the newNMEAreceived() flag to false
    return;
    // we can fail to parse a sentence in which case we should just wait for another
  }
  
  if (timer > millis())  timer = millis();
  //reset the timer if it's time to reset.
  
  if (millis() - timer > 2000) { 
    //about every 2 seconds print the update.
    timer = millis(); // reset the timer

    if(GPS.fix){
      //if the gps is connected do things.
      float Lat1 = ((GPS.latitudeDegrees) * 0.0174533);
      float Long1 = ((GPS.longitudeDegrees) * 0.0174533);
      
      float h = sq((sin((Lat1 - Lat2) / 2.0))) + (cos(Lat1) * cos(Lat2) * sq((sin((Long1 - Long2) / 2.0))));
      d = 2.0 * rEarth * asin (sqrt(h)); 
      //maths to calculate the distance between two points using longitude and latitude.
      //shamelessly borrowed from,
      //https://learn.adafruit.com/reverse-geocache-engagement-box
      
      if(d < 20){
        //if you're within 20 meters of the point.
        lock = 1;
        //set the lock flag to 1.
      }
      showLock();
    }

    else{
      //if the gps isn't connected, say so.
      lcd.setCursor(0, 0);
      lcd.print("Acquiring");
      lcd.setCursor(0, 1);
      lcd.print("signal");
    }
  }
}

void showLock(){
  if(lock == 1){
    //if the lock flag has been set to 1 then unlock.
     lcd.setCursor(0, 0);
     lcd.print("The box is");
     lcd.setCursor(0, 1); 
     lcd.print("now unlocked.");
     myservo.write(180);
     //move the servo to open the box.
  }

  else{
    //further than 20 meters.
    lcd.setCursor(0, 0);
    lcd.print("Distance=");
    lcd.setCursor(0, 1);
    lcd.print(d);
    //tell the person how far to go.    
  }
}
