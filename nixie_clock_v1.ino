#include "RTClib.h" //gives functions that allow for easy communication with with ds3231 

//*********************************************
//------------ pin assigments -----------------
//*********************************************

//shift reg
#define latchPin 8   //Pin connected to ST_CP  
#define clearPin 9   //Pin connected to SRCLR
#define clockPin 12  //Pin connected to SH_CP
#define dataPin 11   //Pin connected to DS

//rotary encoder
#define buttonPin 3  //Pin connected to SW
#define rotClock 5   //Pin connected to CLK
#define rotData 4    //Pin connected to DT

//*********************************************
//---------- function prototypes --------------
//*********************************************
byte dec2BCD(byte n);
void fsm(void);
void outputTime(void);

//*********************************************
//--------------- global vars -----------------
//********************************************* 
volatile boolean fsmFlag = false;

RTC_DS3231 rtc; //rtc object

//*********************************************
//------------- initializations ---------------
//*********************************************
void setup () {

  
  Serial.begin(57600);

  #ifndef ESP8266   
    while (!Serial); // wait for serial port to connect. Needed for native USB
  #endif

  //make sure RTC is connected
  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    Serial.flush();
    abort();
  }

  //reset time if rtc has lost power
  if (rtc.lostPower()) {
    Serial.println("RTC lost power, let's set the time!");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  //initialize shift reg pins
  pinMode(latchPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(dataPin, OUTPUT);
  pinMode(clearPin, OUTPUT);

  //initialize rotary encoder pins
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(rotClock,INPUT);
  pinMode(rotData,INPUT);

  //not using clear rn 
  digitalWrite(clearPin, HIGH);

  //attach interrupt to button
  attachInterrupt(digitalPinToInterrupt(buttonPin), fsmInterrupt, FALLING);
  
}

//*********************************************
//---------------- main loop ------------------
//*********************************************
void loop () {
  outputTime();
  
  while(fsmFlag){
      fsm();
      delay(10);  //can use delay because keeping track of time rtc and using interrupt for button              
  }
}

//*********************************************
//---------- function declarations ------------
//*********************************************

// finite state machine for user input
void fsm(void){

    //enumerates states in fsm
    typedef enum{
    
      init,
      released,
      nextnext,

    }states;

    //fsm control variables
    static states state = init; //static variable to keep track of which state we are in
    static int loops = 0; //keep track of time inside the fsm

    static int currentStateCLK;
    static int lastStateCLK = digitalRead(rotClock);
    static int counter = 0;

    switch (state){
      case init:
        if(digitalRead(buttonPin) == HIGH){
          state = released;
        }
        break;
        
      case released:
        Serial.println("Hey whats up it worked!"); 
        fsmFlag = false;
        state = init;       
        break;
        
      default:
        break;
    }
    
}

// displays current time
void outputTime(void){
  
  static int prevMin = -1;  //-1 is an invalid minute so this ensures time is displayed on startup
  DateTime now = rtc.now(); //create RTC object

  //only update clock when minute changes
  if(prevMin != now.minute()){
    prevMin = now.minute(); //current min is now previous min

    //display hours
    digitalWrite(latchPin, LOW);// tell register to start listening
    if(now.hour()>12)shiftOut(dataPin, clockPin, MSBFIRST, dec2BCD(now.hour()-12)); //handle 24-12 hour time 
    else if(now.hour()==0)shiftOut(dataPin, clockPin, MSBFIRST, dec2BCD(12));
    else shiftOut(dataPin, clockPin, MSBFIRST, dec2BCD(now.hour()));
    digitalWrite(latchPin, HIGH); //stop listening
    
    //for debugging
    //delay(3000);

    //display mins
    digitalWrite(latchPin, LOW);
    shiftOut(dataPin, clockPin, MSBFIRST, dec2BCD(now.minute()));
    digitalWrite(latchPin, HIGH); 
  } 
}

// coneverts a decimal number to BCD
byte dec2BCD(byte val)
{
  return( (val/10*16) + (val%10) );
}

// interrupt service routine 
void fsmInterrupt(){
  fsmFlag = true;
}
