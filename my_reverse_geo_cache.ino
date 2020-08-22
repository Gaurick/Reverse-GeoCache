//below are the variables for the location.
//coordinates MUST be in degrees.
float Lat1 = 33.761685;
float Long1 = -82.278742;
//first location.

float Lat2 = 33.762728;
float Long2 = -82.280019;
//second location.

float Lat3 = 33.761811;
float Long3 = -82.280744;
//third location.

float Lat4 = 33.759503;
float Long4 = -82.282178;
//fourth location.

float Lat5 = 33.759886;
float Long5 = -82.282886;
//fifth location.

float Lat6 = 33.760142;
float Long6 = -82.283031;
//sixth location.

float Lat7 = 33.759531;
float Long7 = -82.281453;
//seventh location.

float Lat8 = 33.760242;
float Long8 = -82.279994;
//eigth location.

float Lat9 = 33.761472;
float Long9 = -82.278050;
//ninth location.

//messages displayed on the way to the locations.
//must be only 16 characters, and needs quotes at beginning and end of word(s).
String locWords1 = "1st point";
//words displayed on the way to the first point.
String locWords2 = "2nd point";
//words displayed on the way to the first point.
String locWords3 = "3rd point";
//words displayed on the way to the first point.
String locWords4 = "4th point";
//words displayed on the way to the first point.
String locWords5 = "5th point";
//words displayed on the way to the first point.
String locWords6 = "6th point";
//words displayed on the way to the first point.
String locWords7 = "7th point";
//words displayed on the way to the first point.
String locWords8 = "8th point";
//words displayed on the way to the first point.
String locWords9 = "finish";
//words displayed at last location.
String locWords10 = "end.";
//second line of words at last location.

//distance units.
//delete the double slashes before the const, string, and int lines to use that unit.
//make sure there are the double slashes before all the other options.
const float rEarth = 20903251;
String mEarth = "ft";
int dThreshold = 30;
//display distance and units in feet.

//const float rEarth = 6371000000;
//String mEarth = "m";
//int dThreshold = 10;
//display distance and units in meters.

//const float rEarth = 6371;
//String mEarth = "km";
//int dThreshold = 1;
//display distance and units in kilometers

//const float rEarth = 3958.8;
//String mEarth = "mi";
//int dThreshold = .5;
//display distance and units in miles.

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
 *  v1.2 finished 3/26/19.
 *  changed meters to feet.
 *  added a bunch of locations.
 *  changed "distance to" words to things to pray for.
 *  hope it works out in the field.
 *  
 *  v1.3 finished 4/1/19.
 *  actually took it to the field and found some issues to fix.
 *  moved latitude and longitude global variables to very top of program.
 *  removed conversion of degrees to radians from the global variables and added it to the setup.
 *  put measurements in feet, meters, kilometers, and miles up at the top.
 *  commented out the other distance measurement options.
 *  put in variable for the unit measuring distance.
 *  added in global variable for threshold of distance to a point and put specific values for each of the distances.
 *  reduced timer before locking at the beginning to 5 seconds.
 *  put global variables for the words that show up when the locations are reached, for instructions/inspiration/whatever.
 *  put instructions on how to set it yourself up top with the variables.
 *  changed the stages function to just check the distance vs threshold then either show message and distance or advance to the next step.
 *  loop has the switch case for the locations now.
 *
 *  v1.31 finished 8/22/20.
 *  cleaned up the formatting of the locations above.
 *  cleaned up the formatting for the location words above.
 *  removed a few to do's.
 *  
 *  to do
 *  put in a beeper for feedback of reaching the location and it's time for the other one?
 *  build box within box.
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

bool lock = 0;
//variable to track if it's been closer than 20 meters to the location.
int d = 0;
//variable for the distance to the destination.

int steps = 0;
//variable to track what's been done.
int counter = 0;
//counter variable to be used to count.
float h = 0;
//variable to hold the distance equation result.

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

  Lat1 = Lat1 * 0.0174533;
  Long1 = Long1 * 0.0174533;
  Lat2 = Lat2 * 0.0174533;
  Long2 = Long2 * 0.0174533;
  Lat3 = Lat3 * 0.0174533;
  Long3 = Long3 * 0.0174533;
  Lat4 = Lat4 * 0.0174533;
  Long4 = Long4 * 0.0174533;
  Lat5 = Lat5 * 0.0174533;
  Long5 = Long5 * 0.0174533;
  Lat6 = Lat6 * 0.0174533;
  Long6 = Long6 * 0.0174533;
  Lat7 = Lat7 * 0.0174533;
  Long7 = Long7 * 0.0174533;
  Lat8 = Lat8 * 0.0174533;
  Long8 = Long8 * 0.0174533;
  Lat9 = Lat9 * 0.0174533;
  Long9 = Long9 * 0.0174533;
  //convert all lat and long from degrees to radians for the distance calculations.
  
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
    if(counter == 6){
      counter = 0;
    }

    else{
      counter ++;
    }

    switch (steps) {
      case 0:
      //stage to allow person time to put things into the box before it locks.
      if(counter < 5){
        //if the count down is still counting, show it.
        lcd.clear();
        lcd.print("Locking in");
        lcd.setCursor(0, 1);
        lcd.print(5 - counter);
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
      distance(Lat1, Long1);
      stages(locWords1);
      break;

      case 3:
      distance(Lat2, Long2);
      stages(locWords2);
      break;

      case 4:
      distance(Lat3, Long3);
      stages(locWords3);
      break;

      case 5:
      distance(Lat4, Long4);
      stages(locWords4);
      break;

      case 6:
      distance(Lat5, Long5);
      stages(locWords5);
      break;

      case 7:
      distance(Lat6, Long6);
      stages(locWords6);
      break;

      case 8:
      distance(Lat7, Long7);
      stages(locWords7);
      break;

      case 9:
      distance(Lat8, Long8);
      stages(locWords8);
      break;

      case 10:
      distance(Lat9, Long9);
      stages(locWords9);
      break;

      case 11:
      //if the final point is close enough to open, open the box.
      lcd.clear();
      lcd.print(locWords9);
      lcd.setCursor(0, 1); 
      lcd.print(locWords10);
      myservo.write(180);
      //move the servo to open the box.
      break;   
    }
  }
}


void stages(String locWords){
    if(d < dThreshold){
      //if you're within the distance threshold, move on to the unlock step.
      steps ++;
      //wiggle the servo to make a noise that you've moved to the next step.
      myservo.write(10);
      delay(250);
      myservo.write(0);
    }

    else{ 
      //further than distance threshold.
      lcd.clear();
      lcd.print(locWords);
      lcd.setCursor(0, 1);
      lcd.print(d);
      lcd.print(mEarth);
      //tell the person how far to go.   
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
