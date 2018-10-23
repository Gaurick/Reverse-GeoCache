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
 *  LCD pot
 *  one side +5v.
 *  other side gnd.
 *  middle to VO on the lcd display.
 *  
 *  change the Lat2 and Long2 to the decimal latitude and longitude of the place to unlock the box.
 *  distance from the destination is measured in meters.
 *  changes rEarth to the radius of earth in whatever measurement you'd prefer.
 *  
 *  v 1.0 first finished 9/19/18.
 *  
 *  v 1.1 finished 10/23/18.
 *  separated the distance calculations for longitude and latitude to it's own function.
 *  moved everything but the timer to a separate function and dumped into a switch case statement to step through.
 *  put in a timer (about 15 seconds) of unlocking when you first turn it on to stuff things in, test, or just fix.
 *  put in the ability to have waypoints on the way to the final location to unlock (currently it's 2 waypoints and the final location).
 *  put a servo wiggle when the box gets to a waypoint to let you know you've gotten close enough.
 *  removed the decimal part of the distance and added "m" to show it's in meters and not feet or whatever.
 *  set update for lcd and all steps to every second instead of every 2 seconds.
 *  more comments.
 *  probably more that i can't remember.
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

float Lat1 = 33.761685 * 0.0174533;
float Long1 = -82.278742 * 0.0174533;
//gps location of the first point needed to unlock the box.
float Lat2 = 33.761450 * 0.0174533;
float Long2 = -82.278070 * 0.0174533;
//gps location of the second point needed to unlock the box.
float Lat3 = 33.761685 * 0.0174533;
float Long3 = -82.278742 * 0.0174533;
//gps location of the first point needed to unlock the box.
//needs to be in decimal degrees and not in the standard way.
//then convert the degrees to radians.
const float rEarth = 6371000.0;
//radius of the earth.
//in meters
bool lock = 0;
//variable to track if it's been closer than 20 meters to the location.
int d = 0;
//variable for the distance to the destination.

int steps = 0;
//variable to track what's been done.
int counter = 0;
//counter variable to be used to count.

float h = 0;


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
  
  if (millis() - timer > 1000) { 
    //about every 2 seconds print the update.
    timer = millis(); 
    // reset the timer
    if(counter == 16){
      counter = 0;
    }

    else{
      counter ++;
    }
    
    stages();
  }
}


void stages(){
  switch (steps){
    case 0:
    //stage to allow person time to put things into the box before it locks.
    if(counter < 15){
      //if the count down is still counting, show it.
      lcd.clear();
      lcd.print("Locking in");
      lcd.setCursor(0, 1);
      lcd.print(15 - counter);
      myservo.write(180);
    }

    else{
      //if the count down is done counting lock the box and move to the next step.
      myservo.write(0);
      steps ++;
    }
    break;

    case 1:
    if(GPS.fix){
      //if the gps unit has a fix on it's current location, skip this step.
      steps ++;
    }

    else{
      //if the gps isn't connected, say so.
      lcd.clear();
      lcd.print("Acquiring");
      lcd.setCursor(0, 1);
      lcd.print("signal");
    }
    break;
    

    case 2:
    //run the first location into the distance calculation function to figure out how far you're from the first point.
    distance(Lat1, Long1);
    
    if(d < 20){
      //if you're within 20 meters, move on to the unlock step.
      steps ++;
      //wiggle the servo to make a noise that you've moved to the next step.
      myservo.write(10);
      delay(250);
      myservo.write(0);
    }

    else{ 
      //further than 20 meters.
      lcd.clear();
      lcd.print("Distance to 1=");
      lcd.setCursor(0, 1);
      lcd.print(d);
      lcd.print(" m");
      //tell the person how far to go.   
    }
    break;

    case 3:
    //feed the second location into the distance function to see how far you are from it.
    distance(Lat2, Long2);
      
    if(d < 20){
      //if you're within 20 meters, move on to the unlock step.
      steps ++;
      //wiggle the servo to make a noise that you've moved to the next step.
      myservo.write(10);
      delay(250);
      myservo.write(0);
    }

    else{ 
      //further than 20 meters.
      lcd.clear();
      lcd.print("Distance to 2=");
      lcd.setCursor(0, 1);
      lcd.print(d);
      lcd.print(" m");
      //tell the person how far to go.   
    }
    break;

    case 4:
    //feed the third location to the distance function to figure out how far it is from you.
    distance(Lat3, Long3);
      
    if(d < 20){
      //if you're within 20 meters, move on to the unlock step.
      steps ++;
    }

    else{ 
      //further than 20 meters.
      lcd.clear();
      lcd.print("Distance to 3=");
      lcd.setCursor(0, 1);
      lcd.print(d);
      lcd.print(" m");
      //tell the person how far to go.   
    }
    break;

    case 5:
    //if the final point is close enough to open, open the box.
    lcd.clear();
    lcd.print("The box is");
    lcd.setCursor(0, 1); 
    lcd.print("now unlocked.");
    myservo.write(180);
    //move the servo to open the box.
    break;   
  }
}

void distance(float lat2, float long2){
  //calculate the distance from your current location to the location you need to go to to unlock the step.
    float lat1 = ((GPS.latitudeDegrees) * 0.0174533);
    float long1 = ((GPS.longitudeDegrees) * 0.0174533);
      
    h = sq((sin((lat1 - lat2) / 2.0))) + (cos(lat1) * cos(lat2) * sq((sin((long1 - long2) / 2.0))));
    d = 2.0 * rEarth * asin (sqrt(h)); 
    //maths to calculate the distance between two points using longitude and latitude.
    //shamelessly borrowed from,
    //https://learn.adafruit.com/reverse-geocache-engagement-box
}
