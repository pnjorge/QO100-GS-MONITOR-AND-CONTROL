

/*
 
  Arduino IoT Cloud Variables description

  The following variables are automatically generated and updated when changes are made to the Thing

  String infoGPS;
  float humidityBME280;
  float inputCurrent;
  float inputPower;
  float pressureBME280;
  float swr_Value_Full_IOT;
  float temperatureBME280;
  float vFWD_Value_Full_IOT;
  float volts_12V_Value_Full_IOT;
  float volts_28V_Value_Full_IOT;
  float volts_5V1_Value_Full_IOT;
  float volts_5V2_Value_Full_IOT;
  float vREV_Value_Full_IOT;
  int highSWRlimitIOT;
  CloudLocation coordinatesIOT;
  bool alarmLED;
  bool inputSW1;
  bool inputSW2;
  bool outputLED1;
  bool outputLED2;
  bool triggerGS;

  Variables which are marked as READ/WRITE in the Cloud Thing will also have functions
  which are called when their values are changed from the Dashboard.
  These functions are generated with the Thing and added at the end of this sketch.
*/



#include "thingProperties.h"


#include "ArduinoGraphics.h"
#include "Arduino_LED_Matrix.h"

#include "fonts.h"                    // header files for Fonts and Frames
#include "frames.h"               


#include "Wire.h"

#include "SparkFunBME280.h"

#include "Adafruit_GPS.h"


#include "maidenhead.h"

#include "Adafruit_INA219.h"




ArduinoLEDMatrix matrix;

BME280 mySensor;

Adafruit_INA219 ina219;



                                  // Define analog inputs:
                                  
//#define ANALOG_12V_IN_PIN A0    // Not required as we use INA219 (keep as spare input)
#define ANALOG_28V_IN_PIN A1      
#define ANALOG_5V1_IN_PIN A2      
#define ANALOG_5V2_IN_PIN A3
#define ANALOG_FWD_IN_PIN A4      
#define ANALOG_REV_IN_PIN A5


#define buzzerPin 3

#define alarmLEDpin 8

#define outputLED1pin 10           // Outputs 
#define outputLED2pin 9

#define switch1pin 7              // Inputs 
#define switch2pin 6


#define optionPin1 13             // for bootup Option (leave HIGH for IOT, make LOW for LOCAL)
#define optionPin2 12             // Future Options


#define scrollSelectPin 0         // For either scrolling Voltages or Ambient info   



uint8_t frame[8][12] = 
{
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
};



const int firstDigitPos = 2;



const float maxFWDvoltsReading = 3.0;                 //  Maximum FWD voltage measured from directional coupler
const float maxREVvoltsReading = 3.0;                 //  Maximum REV voltage measured from directional coupler 



// Floats for FWD ADC voltage & Input voltage
float adc_FWD_voltage = 0.0;
float in_FWD_voltage = 0.0;

// Floats for REV ADC voltage & Input voltage
float adc_REV_voltage = 0.0;
float in_REV_voltage = 0.0;




// Float for Reference Voltage

const float ref_voltage = 3.28;                   //    Measure the exact value of the stable '3.3V' reference and then specify here






// For ADC values. 
// Accurately measure each individual R1 and R2 resistor that will be used on PCB and update value below.

/*
                                          // Variables for the 12V DC input supply
                                          // Not required!! ...as we will use INA219's busVoltage value 
int   adc_12V_value   = 0;                
float adc_12V_voltage = 0;
float in_12V_voltage  = 0;
const float R1ohms12V = 18040.0;          // Measure and set R1 and R2 for potential divider
const float R2ohms12V =  5080.0;
*/
float in_12V_voltage  = 0;



unsigned int adc_28V_value   = 0;         // Variables for the 28V PA supply
float adc_28V_voltage = 0;
float in_28V_voltage  = 0;
const float R1ohms28V = 46900.0;          // Measure and then set here R1 and R2 for potential divider
const float R2ohms28V =  5570.0;



unsigned int adc_5V1_value   = 0;         // Variables for the 5V USB1 supply
float adc_5V1_voltage = 0;
float in_5V1_voltage  = 0;
const float R1ohms5V1 =  2680.0;          // measure and then set here R1 and R2 for potential divider
const float R2ohms5V1 =  3270.0;



unsigned int adc_5V2_value   = 0;         // Variables for the 5V USB1 supply
float adc_5V2_voltage = 0;
float in_5V2_voltage  = 0;
const float R1ohms5V2 =  2680.0;          // Measure and then set here R1 and R2 for potential divider
const float R2ohms5V2 =  3270.0;



unsigned int adc_FWD_value = 0;           // Variables for FWD and REV
unsigned int adc_REV_value = 0;
float SWR_Value_Full = 0.0;


const unsigned int fsdSWR = 10;



unsigned int digit1value = 0;
unsigned int digit2value = 0;


char digit1st = '0';
char digit2nd = '0';


unsigned int SWR_Value_IntegerPart = 0;
float        SWR_Value_DecimalPart = 0.0;




const float minMeasThreashold = 0.1;                    // Anything below this will not trigger a SWR measurement      



bool displaySWRenable     = false;
const unsigned int  displayTimeout = 6000;              // Period of time after no SWR measured that the display will return to scrolling


unsigned long timeLastMeas = 0;



bool highSWRenable = false;

unsigned int  highSWRlimit = 0;                         // High SWR alarm limit is 0 (disabled) at power ON, but will change from dashboard.


const unsigned int  refreshSWR = 300;                   // Slows update rate of SWR measurement to avoid decimals flickering too fast


unsigned int buzzerDelayCount = 0;
const unsigned int buzzerDelay = 20;                    // Delays buzzer sounding until set number of high SWR violations have occurred




const int highFreq = 1;     // High or Low buzzer tone
const int lowFreq  = 2;     

const int shortTone = 40;   // Short or Long buzzer duration
const int longTone  = 120;



char splashText[]   =   "    F5VMJ .... MONITOR & CONTROL    ";
char promptText[]   =   "    ....    ";



bool scrollSelect  = 0;
unsigned int scrollChoice  = 0;



bool optionIOT = true;        // true is IOT (default)        



// Connect to the GPS on the hardware I2C port
Adafruit_GPS GPS(&Wire1);  

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO false


unsigned long searchFixTime       = 0;

unsigned long millisSinceLastFix  = 0;              // For counting since last GPS fix

unsigned int minsSinceLastFix     = 0;

unsigned int counterGPS     = 0;

const unsigned int searchFixMinutes  = 5;           // Interval between reading GPS data (in mins). Set to more than a few minutes...
                                                    // ...for program flow and to avoid too many GPS updates.


bool  gpsFix  = false;                              // Flag to indicate GPS is locked if needed


float  Lat    = 0;                                  // Defaults to 0, 0 position near Gulf of Guinea
float  Long   = 0;      

char* locatorGrid   = "NO FIX";

char searchingLock[]    = "    SEARCHING    ";








//-------------------------------------------------------------------------------------------------------
// S E T U P



void setup() 
{
  
  
  // Initialize serial and wait for port to open:
  Serial.begin(115200);
  delay(15000);                               // 15secs wait for Web ditor serial monitor to connect (less time if local)
  Serial.println(F("Serial Port ...OK"));
  
  
  
  
  pinMode(optionPin1, INPUT_PULLUP);
  pinMode(optionPin2, INPUT_PULLUP);
  
  
  pinMode(switch1pin, INPUT_PULLUP);
  pinMode(switch2pin, INPUT_PULLUP);
  
  
  pinMode(alarmLEDpin, OUTPUT);
  
  pinMode(outputLED1pin, OUTPUT);
  pinMode(outputLED2pin, OUTPUT);
  
  pinMode(buzzerPin, OUTPUT) ;  
  
  pinMode(scrollSelectPin, INPUT_PULLUP);
  
  
  alarmLED  = false;
      
  inputSW1  = false;
  inputSW2  = false;
  
  
  optionIOT = digitalRead(optionPin1);                    // For either LOCal or IOT mode of operation
  
  scrollChoice = digitalRead(scrollSelectPin);            // For either scrolling Voltages/LOW or Ambient/HIGH info  
  scrollChoice = scrollChoice + 1;                        // ...for easy reading of 'switch/case' later, Voltages/1st or Ambient/2nd (and future choices)


  Serial.print(F("scrollChoice: ")); Serial.println( scrollChoice );
  
  
 
 
 
 
  if (optionIOT == true) 
  { 
 
      Serial.println(F("Boot option: IOT"));
      

      
  
      // Defined in thingProperties.h
      initProperties();

      // Connect to Arduino IoT Cloud
      ArduinoCloud.begin(ArduinoIoTPreferredConnection);

      /*
        The following function allows you to obtain more information
        related to the state of network and IoT Cloud connection and errors
        the higher number the more granular information you’ll get.
        The default is 0 (only errors).
        Maximum is 4
      */
      setDebugMessageLevel(4);
      ArduinoCloud.printDebugInfo();
      

  
      inputCurrent  = 0;
      inputPower    = 0;

      volts_12V_Value_Full_IOT = 0;     
      volts_28V_Value_Full_IOT = 0;
      volts_5V1_Value_Full_IOT = 0;  
      volts_5V2_Value_Full_IOT = 0;
      vFWD_Value_Full_IOT = 0;
      vREV_Value_Full_IOT = 0;
      swr_Value_Full_IOT  = 0;
  
  
  
      analogReference(AR_EXTERNAL);                     // Using the external voltage reference
  


      matrix.begin();


      Serial.println(F("GROUNDSTATION MONITOR & CONTROL"));
  
  
      Wire1.begin();
      

      mySensor.setI2CAddress(0x76);                     // Set the I2C address (beacuse default is 0x77 in SparkFunBME280 library)    

      if (mySensor.beginI2C(Wire1) == false)            // Begin communication over I2C
      {
        
        Serial.println(F("The BME280 sensor did not respond."));
        
        digitalWrite(alarmLEDpin, HIGH);    // Turn the alarm LED on 
        buzzNow(lowFreq, 500);              // Sound a long error tone
        
        while(1) { delay(10); }; //Freeze
        
      }
      else
      {
        
        Serial.println(F("BME280 sensor ...OK"));
        
      }

  
  
      if (! ina219.begin(&Wire1))         // pass &Wire1 to ina219.begin() to use other I2C STEMMA/QWIIC
      {
      Serial.println(F("The INA219 sensor did not respond."));
      
      digitalWrite(alarmLEDpin, HIGH);    // Turn the alarm LED on 
      buzzNow(lowFreq, 500);              // Sound a long error tone
      
      while (1) { delay(10); }    //Freeze
      
      }
      else
      { 
        
      Serial.println(F("INA219 sensor ...OK"));
        
      }
  
  
  


      Serial.println(F("PA1010D GPS   ...Start"));
  
      // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
      GPS.begin(0x10);  // The I2C address to use is 0x10
      // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
      GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
      // uncomment this line to turn on only the "minimum recommended" data
      //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
      // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
      // the parser doesn't care about other sentences at this time
      // Set the update rate
      GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
      // For the parsing code to work nicely and have time to sort thru the data, and
      // print it out we don't suggest using anything higher than 1 Hz

      // Request updates on antenna status, comment out to keep quiet
      GPS.sendCommand(PGCMD_ANTENNA);

      delay(1000);

      // Ask for firmware version
      GPS.println(PMTK_Q_RELEASE);
  
 
  
      triggerGS = false;
      delay(300);
      triggerGS = true;                 // toggle state to send email or notification when triggered
  
  
  
      buzzNow(highFreq, 30);            // A quick 'R' if all well at start
      delay (30);
      buzzNow(highFreq, 90);
      delay (30);
      buzzNow(highFreq, 30);
      
      scrollMessageText( splashText );
  

      buzzNow(highFreq, shortTone);
      scrollMessageText( promptText ); 
  
      readAnalogInput1();
      buzzNow(highFreq, shortTone);  
      delay (300);
      scrollVoltageText(in_12V_voltage);

      readAnalogInput2();
      buzzNow(highFreq, shortTone); 
      delay (300); 
      scrollVoltageText(in_28V_voltage);

      readAnalogInput3();
      buzzNow(highFreq, shortTone);
      delay (300);  
      scrollVoltageText(in_5V1_voltage);

      readAnalogInput4();
      buzzNow(highFreq, shortTone);
      delay (300);  
      scrollVoltageText(in_5V2_voltage);
  
  
  
      temperatureBME280 = mySensor.readTempC();
      Serial.print(F("Temp: "));
      Serial.print(temperatureBME280, 1);   Serial.println(F(" degC"));
      buzzNow(highFreq, shortTone);
      delay (300);  
      scrollSensorText(temperatureBME280, 1);  
  
      humidityBME280 = mySensor.readFloatHumidity(); 
      Serial.print(F("Humidity: "));
      Serial.print(humidityBME280, 1);      Serial.println(F(" %RH"));  
      buzzNow(highFreq, shortTone);
      delay (300);  
      scrollSensorText(humidityBME280, 2);  
  
      pressureBME280 = mySensor.readFloatPressure();  
      pressureBME280 = ( pressureBME280 / 100);                                      // convert to hPa
      Serial.print(F("Pressure: "));  
      Serial.print(pressureBME280, 1);      Serial.println(F(" hPa"));  
      buzzNow(highFreq, shortTone);
      delay (300);  
      scrollSensorText(pressureBME280, 3);  
      
      
      
      Serial.print(F("Grid: "));  
      Serial.println( locatorGrid );        
      buzzNow(highFreq, shortTone);
      delay (300);  
      scrollMessageText( searchingLock );  
      
      
      
      flushGPS();                           // Empty buffer at boot/reset just in case.
  
      searchFixTime   = millis();
  
      

  }
  
  else
  
  {
    

      Serial.println(F("Boot option: LOW/LOCAL"));
      
      // Any Setup parameters for Local/Test mode go here
    
    
  }

  

  
  
  
  
  
}


  





//-------------------------------------------------------------------------------------------------------
// L O O P  
  
  
  
  
  
void loop() 
{
  Serial.print(F("searchFixTime = ")); Serial.println(searchFixTime); 
  
  

  if (optionIOT == true) 
  {
    
    
    
      if ( displaySWRenable == false )                                // get GPS position every few minutes but not during SWR measurement.   
      {
      
              unsigned long searchFixMilliSecs = 0;
              
              searchFixMilliSecs = 60000 * searchFixMinutes;          // Convert minutes to milliseconds
              

              Serial.print(F("millis() = ")); Serial.print(millis()); Serial.print(F("   searchFixTime = ")); Serial.print(searchFixTime); 
              Serial.print(F("   searchFixTime + searchFixMilliSecs = ")); Serial.println( searchFixTime + searchFixMilliSecs  ); 
              
   
              
              if (  (millis() - searchFixTime) >= searchFixMilliSecs  )                           // every so often read GPS
              {
  
                Serial.println(F("searchFixTime STARTING")); 
                
                
                flushGPS();                                           // Empty GPS buffer
                
                for (int seeGPS = 1; seeGPS <= 2000 ; seeGPS++)       // gather the latest NMEA data being received
                {
 
                readGPS();                                    
                
                }
                
                
                showGPSstats();
                
                
                if (gpsFix == true) 
                {
                
                  getMaidenhead();                              // convert from Lat/Long to Maidenh
                
                }
                
                else
                
                {
                  
                  locatorGrid = "NO FIX";                       // GPS not locked so show/message 'No Fix' and Lat Long defaults to 0, 0 on map.
                  
                  Lat  = 0;
                  Long = 0;
                  
                }
                
                
                coordinatesIOT = {Lat, Long};                 // update Cloud variable
                
                
                
                infoGPS = "GPS update No." + String(counterGPS) + " since switch ON." + "\n\r"
                        + "Position is:    Lat " +  String(Lat, 4) + "  Long " + String(Long, 4) + "\n\r"     // update IOT Cloud message box
                        + "Maidenhead Grid is:  "+ locatorGrid + "\n\r";
                
                
                
                
                millisSinceLastFix = millis();                // reset millisSinceLastFix counting
             
                
                searchFixTime = millis();                     // reset time to wait for next read of GPS
              
              }
      
              else
            
              {

                loopIOT();        //  For boot option IOT
    
              }
  
 
  
      }
      
      else
      
      {
  
 
      loopIOT();        //  For boot option IOT
      
      searchFixTime = millis();                     // Resets, so will wait searchFixMinutes after power ON until next GPS read... 
                                                    // ...or after SWR measurements completed (when displaySWRenable goes to  false).
                                                    
      
      
      }




      minsSinceLastFix = ( millis() - millisSinceLastFix ) / 60000;
      Serial.print(F("minsSinceLastFix = ")); Serial.print(minsSinceLastFix); Serial.print(F("   (millis() - millisSinceLastFix) = ")); Serial.println( millis() - millisSinceLastFix );


  
  } 
  
  
  
  
  else 
  
  
  
  
  {
    
  Serial.println(F("Calling LoopLocal"));  
  
  loopLocal();        // For boot option LOCAL
  
  }  

  



}













//-------------------------------------------------------------------------------------------------------
// F U N C T I O N S





void readINA219()
{
  
  float busVoltage  = 0; 
  float current_mA  = 0;
  float power_mW    = 0;
  
  float shuntvoltage  = 0;
  float loadvoltage   = 0;

  
  

  shuntvoltage  = ina219.getShuntVoltage_mV();
  busVoltage    = ina219.getBusVoltage_V();
  current_mA    = 2 * (ina219.getCurrent_mA() );           // Double because of 2nd parallel 0.1 Ohm resistor (making 0.05 Ohms)
  power_mW      = 2 * ( ina219.getPower_mW() );            // Double because of 2nd parallel 0.1 Ohm resistor (making 0.05 Ohms)
 
  loadvoltage = busVoltage + (shuntvoltage / 1000);        // This is total across both loads ( load of shunt + load being powered )
  
  
  if (current_mA < 0)       // Avoid noise showing negative value when no current is drawn
  {
    current_mA = 0;   
  }
  
  
  Serial.print(F("Bus Voltage:   ")); Serial.print(busVoltage);    Serial.println(F(" V"));
  Serial.print(F("Shunt Voltage: ")); Serial.print(shuntvoltage);  Serial.println(F(" mV"));
  Serial.print(F("Load Voltage:  ")); Serial.print(loadvoltage);   Serial.println(F(" V"));
  Serial.print(F("Current:       ")); Serial.print(current_mA);    Serial.println(F(" mA"));
  Serial.print(F("Power:         ")); Serial.print(power_mW);      Serial.println(F(" mW"));
  
  
  
  in_12V_voltage = busVoltage;
  
  inputCurrent = ( current_mA / 1000.0 );
  inputCurrent = roundUpTo1dp(inputCurrent);   // Round up to 1 decimal place for dashboard gauge
  
  inputPower = ( power_mW / 1000.0 );
  inputPower = roundUpTo1dp(inputPower);      // Round up to 1 decimal place for dashboard gauge
  
  
}











void checkInputSwitches()
{
  
  
  bool switch1state = digitalRead(switch1pin);
  
  if (switch1state == LOW) 
  {
    inputSW1 = true;
    Serial.println(F("inputSW1 is CLOSED "));   
  }
  else
  {
    inputSW1 = false;
    Serial.println(F("inputSW1 is OPEN "));  
  }
  
 
 
  bool switch2state = digitalRead(switch2pin);
  
  if (switch2state == LOW) 
  {
    inputSW2 = true;
    Serial.println(F("inputSW2 is CLOSED "));   
  }
  else
  {
    inputSW2 = false;
    Serial.println(F("inputSW2 is OPEN "));  
  }
 
 
  
  
}







void getMaidenhead()
{
  
  
    locatorGrid = get_mh(Lat, Long, 6);
    Serial.print(F("The 6 character locatorGrid is: "));  Serial.println(locatorGrid);
  
  
}






void  flushGPS()
{

    Serial.println(F("Flushing GPS"));
  
    matrix.loadFrame(recycle2);               // Load and display GPS recycling icon
    

    for (int i = 1; i <= 8192; i++)           // does the PA1010D have an 8k buffer ??
    {
      char trash = GPS.read();
    }

     
}






float roundUpTo1dp(float valEntry)
{

  float valExit;
  

  valExit = valEntry * 10;

  valExit = round(valExit);

  valExit = valExit/10;


  return valExit;

  
}









void  readGPS()
{
  
  
  // read data from the GPS in the 'main loop'
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
  if (GPSECHO)
    if (c) Serial.print(c);
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trying to print out data
    Serial.println(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return; // we can fail to parse a sentence in which case we should just wait for another
  }




}








void  showGPSstats()
{
  

    Serial.print("\nTime: ");
    
    if (GPS.hour < 10) 
    { 
      Serial.print('0'); 
    }
    Serial.print(GPS.hour, DEC); Serial.print(':');
    
    if (GPS.minute < 10) 
    { 
      Serial.print('0'); 
    }
    Serial.print(GPS.minute, DEC); Serial.print(':');
    
    if (GPS.seconds < 10) 
    { 
      Serial.print('0'); 
    }
    Serial.print(GPS.seconds, DEC); Serial.print('.');
    
    if (GPS.milliseconds < 10) 
    {
      Serial.print("00");
    } 
    else if (GPS.milliseconds > 9 && GPS.milliseconds < 100) 
    {
      Serial.print("0");
    }
    
    Serial.println(GPS.milliseconds);
    Serial.print("Date: ");
    Serial.print(GPS.day, DEC); Serial.print('/');
    Serial.print(GPS.month, DEC); Serial.print("/20");
    Serial.println(GPS.year, DEC);
    Serial.print("Fix: "); Serial.print((int)GPS.fix);
    Serial.print(" quality: "); Serial.println((int)GPS.fixquality);
    
    
    if (GPS.fix) 
    {
      gpsFix = true;
      
      Serial.print("Location: ");
      Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
      Serial.print(", ");
      Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);


      convertLatLong();
    
      
      Serial.print("Speed (knots): "); Serial.println(GPS.speed);
      Serial.print("Angle: "); Serial.println(GPS.angle);
      Serial.print("Altitude: "); Serial.println(GPS.altitude);
      Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
    }
    else
    {
      gpsFix = false;
    }
    
    Serial.println("");
   
    
    counterGPS = counterGPS + 1;
    
  
}










void convertLatLong()
{
  
  Serial.println(F("Converting Lat/Long"));
  

  float latitudeFromGPS = 0;
  int   degLatitudeInt  = 0;
  float minsLatitude    = 0;
  float degLatitudeDec  = 0;
  int   northORsouth    = 0;


  float longitudeFromGPS = 0;
  int   degLongitudeInt  = 0;
  float minsLongitude    = 0;
  float degLongitudeDec  = 0;
  int   eastORwest       = 0;



      latitudeFromGPS = GPS.latitude;
      Serial.print(F("latitudeFromGPS:  ")); Serial.println(latitudeFromGPS, 4);
      
      degLatitudeInt = ( latitudeFromGPS / 100 );
      //Serial.print("degLatitudeInt = latitudeFromGPS / 100 :  "); Serial.println(degLatitudeInt);
      
      minsLatitude = latitudeFromGPS - (degLatitudeInt * 100);
      //Serial.print("minsLatitude = latitudeFromGPS - (degLatitudeInt * 100) :  "); Serial.println(minsLatitude, 4);
      
      degLatitudeDec = ( minsLatitude / 60 );
      //Serial.print("degLatitudeDec = minsLatitude / 60 :  "); Serial.println(degLatitudeDec, 6);
      
      Lat = degLatitudeInt + degLatitudeDec;
      //Serial.print("Lat = degLatitudeInt + degLatitudeDec :  "); Serial.println(Lat, 6);
      
      
      northORsouth = (GPS.lat);

      if ( northORsouth == 83 )             // ASCII code is S so make a negative latitude
      {
        Lat = Lat * -1;
      }

      Serial.print(F("Lat N/S:  ")); Serial.println(Lat, 6);





      
      longitudeFromGPS = GPS.longitude;
      Serial.print(F("longitudeFromGPS:  ")); Serial.println(longitudeFromGPS, 4);
      
      degLongitudeInt = ( longitudeFromGPS / 100 );
      //Serial.print("degLongitudeInt = longitudeFromGPS / 100 :  "); Serial.println(degLongitudeInt);
      
      minsLongitude = longitudeFromGPS - (degLongitudeInt * 100);
      //Serial.print("minsLongitude = longitudeFromGPS - (degLongitudeInt * 100) :  "); Serial.println(minsLongitude, 4);
      
      degLongitudeDec = ( minsLongitude / 60 );
      //Serial.print("degLongitudeDec = minsLongitude / 60 :  "); Serial.println(degLongitudeDec, 6);
      
      Long = degLongitudeInt + degLongitudeDec;
      //Serial.print("Long = degLongitudeInt + degLongitudeDec :  "); Serial.println(Long, 6);
      

      eastORwest = (GPS.lon);

      if ( eastORwest == 87 )             // ASCII code is W so make a negative longitude
      {
        Long = Long * -1;
      }

      Serial.print(F("Long E/W: ")); Serial.println(Long, 6);


  
}










void loopIOT()                    // Option IOT
{
  
    ArduinoCloud.update();


    Serial.println(F("Inside loopIOT"));

    
    readAnalogInput1();
    readAnalogInput2();
    readAnalogInput3();
    readAnalogInput4();
    
    readBME280();
    
    
    checkInputSwitches();
    


    readAnalogFwdRevInputs();

    SWR_Calc();

    highSWRalert();

    SWR_seperateIntDec();

    fsdMeter();

    convertDigits();    

    clear_frame();
    
    
    
    
    
    if ( displaySWRenable == false )              // scrolls voltage values if no SWR measurement active
    {
      
      scrollMessageText( promptText ); 
      
      for (int idx = 1; idx <= 4 ; idx++)
      {
        
        
        readAnalogInput1();
        readAnalogInput2();
        readAnalogInput3();
        readAnalogInput4();
        
        readBME280();
        
        
        checkInputSwitches();

        
        ArduinoCloud.update();
        
        
      
        readAnalogFwdRevInputs();                 // checks to see if there is a FWD voltage  
                                                  // if so will stop scrolling and do SWR measurement
                                                  
        if ( displaySWRenable == false )
        {
          
          
          switch (scrollChoice) 
          {
  
          case 1:
            scrollVoltages(idx);
          break;
    
          case 2:
            scrollAmbient(idx);
          break;  
          
          case 3:
            // For a 3rd future scrolling option
          break;  
  
          case 4:
            // For a 4th future scrolling option
          break;  
  
  
          default:
            // never here
          break;
    
          }
  

        
        }
        
        else
        
        {
          
          idx = 5;
          
        }
        
        
      }
      
      
      
    }

    else
    
    {
    
    
    add1stDigit_to_frame(digit1st, firstDigitPos);
    add2ndDigit_to_frame(digit2nd, firstDigitPos);
    frame[4][firstDigitPos + 4] = 1;                      //For decimal point



    plotFWD();              // plot on matrix
    
    vFWD_Value_Full_IOT = ( in_FWD_voltage / maxFWDvoltsReading ) * 100;    // convert to percentage and update cloud variable
    
    
    
    plotREV();              // plot on matrix
    
    vREV_Value_Full_IOT = ( in_REV_voltage / maxREVvoltsReading ) * 100;     // convert to percentage and update cloud variable


    }


    display_frame();


}









void loopLocal()                    // Option LOCAL for booting a second sketch (or testing hardware as below)    
{
  
  Serial.println(F("Inside Boot option: loopLocal"));  
  
  static bool localFirstRun = true;
  Serial.print(F("localFirstRun: "));  Serial.println(localFirstRun);  
  
  
  if (localFirstRun == true)            
  {

    digitalWrite(alarmLEDpin, HIGH);      // Turn the alarm LED on 
    buzzNow(lowFreq,  200);               // Check buzzer with two tones on first pass only
    delay(50);
    buzzNow(highFreq, 70);              
    
    localFirstRun = false;
  
  }
  
  else
  
  {
    
    checkInputSwitches();
  
  
    if ( inputSW1 == true )                   // All LEDs tested as on (push to extinguish and test switches)
    {
      digitalWrite(outputLED1pin, LOW);       // turn the LED off 
    }
    else
    {
      digitalWrite(outputLED1pin, HIGH);      // turn the LED on 
    }
    
    
    if ( inputSW2 == true )
    {
      digitalWrite(outputLED2pin, LOW);      // turn the LED off 
    }
    else
    {
      digitalWrite(outputLED2pin, HIGH);      // turn the LED on 
    }
    
    
  }
  
  
}










void readBME280()
{
  
    float temperatureBME280temp = mySensor.readTempC();
    temperatureBME280 = roundUpTo1dp(temperatureBME280temp);                                          // round and update dashboard
    //Serial.print("temperatureBME280 (ROUNDED) =  "); Serial.print(temperatureBME280); Serial.println(" degC");
    
    
    float humidityBME280temp    = mySensor.readFloatHumidity(); 
    humidityBME280 = roundUpTo1dp(humidityBME280temp);                                                  // round and update dashboard
    //Serial.print("humidityBME280 (ROUNDED)    =  "); Serial.print(humidityBME280); Serial.println(" %RH");    
    
    
    
    float pressureBME280temp    = (  (mySensor.readFloatPressure() / 100) );                            // read and convert to hPa  
    pressureBME280 = roundUpTo1dp(pressureBME280temp);                                                  // round and update dashboard
    //Serial.print("pressureBME280 (ROUNDED)    =  "); Serial.print(pressureBME280); Serial.println(" hPa");  
    
  
}







void scrollAmbient(int poolAmbient)
{
  
  char gridBuffer[]   = "              ";
                           
  char* extraSpaces   = "    ";
  
  
    
  
  switch (poolAmbient) 
  {
  
    case 1:
      temperatureBME280 = mySensor.readTempC();
      Serial.print(F("Scrolling Temp: "));
      Serial.print(temperatureBME280, 1);   Serial.println(F(" degC"));
      delay (300);  
      scrollSensorText(temperatureBME280, 1);
    break;
    
    
    case 2:
      humidityBME280 = mySensor.readFloatHumidity(); 
      Serial.print(F("Scrolling Humidity: "));
      Serial.print(humidityBME280, 1);      Serial.println(F(" %RH"));  
      delay (300);  
      scrollSensorText(humidityBME280, 2);
    break;
    
    
    case 3:
      pressureBME280 = mySensor.readFloatPressure();  
      pressureBME280 = ( pressureBME280 / 100);                                      // convert to hPa
      Serial.print(F("Scrolling Pressure: "));  
      Serial.print(pressureBME280, 1);      Serial.println(F(" hPa"));  
      delay (300);  
      scrollSensorText(pressureBME280, 3); 
    break;
    
    
    case 4:
    
    strcpy( gridBuffer, extraSpaces );
    strcpy( gridBuffer + strlen(extraSpaces), locatorGrid );
    strcpy( gridBuffer + ( strlen(extraSpaces) + strlen(locatorGrid) ), extraSpaces );
    
    Serial.print(F("Scrolling gridBuffer: ")); Serial.println( gridBuffer );  
    
    delay (300);  
    scrollMessageText( gridBuffer );
  
    break;    
    
    
    
    default:
            // never here
    break;
    
  }
  
  
  
  
}








void scrollVoltages(int poolDVM)
{


  switch (poolDVM) 
  {
  
    case 1:
            readAnalogInput1();
            delay (300);
            scrollVoltageText(in_12V_voltage);
    break;
    
    case 2:
            readAnalogInput2();
            delay (300);
            scrollVoltageText(in_28V_voltage);
    break;
    
    case 3:
            readAnalogInput3();
            delay (300);
            scrollVoltageText(in_5V1_voltage);
    break;
    
    case 4:
            readAnalogInput4();
            delay (300);
            scrollVoltageText(in_5V2_voltage);
    break;    
    
    
    default:
            // never here
    break;
    
  }



}







void readAnalogInput1()
{
  
  
  readINA219();
  
  volts_12V_Value_Full_IOT = roundUpTo1dp(in_12V_voltage);     // round and update dashboard
  
  
  /* Below is not required as we will use INA219's 'busVoltage value' for 'volts_12V_Value_Full_IOT'.

   // read 12V DC input voltage:
   
   adc_12V_value = analogRead(ANALOG_12V_IN_PIN);

   adc_12V_voltage  = (adc_12V_value * ref_voltage) / 1024.0; 

   float potDiv12Vcorrection =  ( R2ohms12V / (R1ohms12V + R2ohms12V ) );        // Calculate voltage at divider input
   
   in_12V_voltage = ( adc_12V_voltage / potDiv12Vcorrection  );

   Serial.print(F("in_12V_voltage =  ")); Serial.print(in_12V_voltage); Serial.println(F(" V"));
   
   
   volts_12V_Value_Full_IOT = roundUpTo1dp(in_12V_voltage);     // round and update dashboard

  */

}





void readAnalogInput2()
{

   // read 28V PA supply voltage:
   
   adc_28V_value = analogRead(ANALOG_28V_IN_PIN);

   adc_28V_voltage  = (adc_28V_value * ref_voltage) / 1024.0; 

   float potDiv28Vcorrection =  ( R2ohms28V / (R1ohms28V + R2ohms28V ) );        // Calculate voltage at divider input
   
   in_28V_voltage = ( adc_28V_voltage / potDiv28Vcorrection  );

   Serial.print(F("in_28V_voltage =  ")); Serial.print(in_28V_voltage); Serial.println(F(" V"));
   
   
   volts_28V_Value_Full_IOT = roundUpTo1dp(in_28V_voltage);     // round and update dashboard


}





void readAnalogInput3()
{

   // read 5V USB1 supply voltage:
   
   adc_5V1_value = analogRead(ANALOG_5V1_IN_PIN);

   adc_5V1_voltage  = (adc_5V1_value * ref_voltage) / 1024.0; 

   float potDiv5V1correction =  ( R2ohms5V1 / (R1ohms5V1 + R2ohms5V1 ) );        // Calculate voltage at divider input
   
   in_5V1_voltage = ( adc_5V1_voltage / potDiv5V1correction  );

   Serial.print(F("in_5V1_voltage =  ")); Serial.print(in_5V1_voltage); Serial.println(F(" V"));

   
   volts_5V1_Value_Full_IOT = roundUpTo1dp(in_5V1_voltage);     // round and update dashboard
   

}





void readAnalogInput4()
{

   // read 5V USB2 supply voltage:
   
   adc_5V2_value = analogRead(ANALOG_5V2_IN_PIN);

   adc_5V2_voltage  = (adc_5V2_value * ref_voltage) / 1024.0; 

   float potDiv5V2correction =  ( R2ohms5V2 / (R1ohms5V2 + R2ohms5V2 ) );        // Calculate voltage at divider input
   
   in_5V2_voltage = ( adc_5V2_voltage / potDiv5V2correction  );

   Serial.print(F("in_5V2_voltage =  ")); Serial.print(in_5V2_voltage); Serial.println(F(" V"));
   
   volts_5V2_Value_Full_IOT = roundUpTo1dp(in_5V2_voltage);     // round and update dashboard


}







void scrollMessageText(char messageToScroll[])
{


  // Make it scroll!
  matrix.beginDraw();

  matrix.stroke(0xFFFFFFFF);
  matrix.textScrollSpeed(50);

  
  matrix.textFont(Font_5x7);
  matrix.beginText(0, 1, 0xFFFFFF);
  matrix.println(messageToScroll);
  matrix.endText(SCROLL_LEFT);

  matrix.endDraw();
  

}







void scrollVoltageText(float voltageValueToScroll)
{
    
  char textToScroll[] = "            ";

                                                          // minimum of 12 characters, left indentation.
                                                          // https://www.programmingelectronics.com/dtostrf/
  dtostrf( voltageValueToScroll, -12, 1, textToScroll );  
 

  Serial.print(F("textToScroll = ")); Serial.println(textToScroll);
  


  if ( voltageValueToScroll >= 10 )     // add 'v' after to indicate 'voltage' unit
  {

    textToScroll[4] = 'v';         

  }

  else

  {

    textToScroll[3] = 'v';         
    
  }




  // Make it scroll!
  matrix.beginDraw();

  matrix.stroke(0xFFFFFFFF);
  matrix.textScrollSpeed(50);
  
  
  matrix.textFont(Font_5x7);
  matrix.beginText(11, 1, 0xFFFFFF);
  matrix.println(textToScroll);
  matrix.endText(SCROLL_LEFT);

  matrix.endDraw();

  
}







void scrollSensorText(float sensorValueToScroll, int unitOfSensor)
{
    
  char textToScroll[] = "            ";
  
  int unitPosition = 0;

                                                          // minimum of 12 characters, left indentation.
                                                          // https://www.programmingelectronics.com/dtostrf/
  dtostrf( sensorValueToScroll, -12, 1, textToScroll );  
 


  if ( sensorValueToScroll >= 1000 )
  {
    unitPosition = 6;
  }
  else if ( sensorValueToScroll >= 100 )
  {
    unitPosition = 5;
  }
  else if( sensorValueToScroll >= 10 )
  {
    unitPosition = 4;
  }
  else
  {
    unitPosition = 3;
  }  





  switch (unitOfSensor) 

  {
  
  
    case 1:
            textToScroll[unitPosition]      = 'd'; 
            textToScroll[unitPosition + 1]  = 'e'; 
            textToScroll[unitPosition + 2]  = 'g'; 
            textToScroll[unitPosition + 3]  = 'C'; 
    break;
    
    
    case 2:
            textToScroll[unitPosition]      = '%'; 
            textToScroll[unitPosition + 1]  = 'R'; 
            textToScroll[unitPosition + 2]  = 'H'; 
    break;
    
    
    case 3:
            textToScroll[unitPosition]      = 'h'; 
            textToScroll[unitPosition + 1]  = 'P'; 
            textToScroll[unitPosition + 2]  = 'a'; 
    break;
    
    

    default:
            // never here
    break;
    
    
  }




  // Make it scroll!
  matrix.beginDraw();

  matrix.stroke(0xFFFFFFFF);
  matrix.textScrollSpeed(50);
  
  
  matrix.textFont(Font_5x7);
  matrix.beginText(11, 1, 0xFFFFFF);
  matrix.println(textToScroll);
  matrix.endText(SCROLL_LEFT);

  matrix.endDraw();

  
  
}














void buzzNow(int freq, int buzzTime)
{

          for (int i = 0; i < buzzTime; i++)          // Sound buzzer 
          {
            digitalWrite (buzzerPin, HIGH) ;
            delay (freq) ;
            digitalWrite (buzzerPin, LOW) ;
            delay (freq) ;
          }

  
}








void readAnalogFwdRevInputs()
{


  Serial.println(F("Inside readAnalogFwdRevInputs()"));
  
  
  delay(refreshSWR);
  

  
   // Read the Analog FWD Input
   adc_FWD_value = analogRead(ANALOG_FWD_IN_PIN);
   
   // Determine voltage at ADC FWD input
   adc_FWD_voltage  = (adc_FWD_value * ref_voltage) / 1024.0; 
   
   in_FWD_voltage = adc_FWD_voltage;

   in_FWD_voltage = constrain(in_FWD_voltage, 0, maxFWDvoltsReading);

    if ( in_FWD_voltage >= minMeasThreashold )
    {
      displaySWRenable = true;
      timeLastMeas = millis();
    }
    else
    {
      float timeNow1 = millis();

      if ( timeNow1 >= (timeLastMeas + displayTimeout) )
      {
          displaySWRenable = false;                     // Will stop SWR measuring after some time after last measurement (displayTimeout)
      }
    
    
    }
   


   // Read the Analog REV Input
   adc_REV_value = analogRead(ANALOG_REV_IN_PIN);
   
   // Determine voltage at ADC REV input
   adc_REV_voltage  = (adc_REV_value * ref_voltage) / 1024.0; 
   
   in_REV_voltage = adc_REV_voltage;

   in_REV_voltage = constrain(in_REV_voltage, 0, maxREVvoltsReading);

 
  
  if (in_REV_voltage >= in_FWD_voltage)             // Keeps REV voltage artificially below FWD voltage read
  {

    in_REV_voltage = in_FWD_voltage;
    in_FWD_voltage = ( in_FWD_voltage + 0.01 );
     
  }


   // Print results to Serial Monitor to 2 decimal places
  Serial.print(F("FWD Input Voltage = "));
  Serial.print(in_FWD_voltage, 2);
  Serial.print(F("   REV Input Voltage = "));
  Serial.println(in_REV_voltage, 2);
  
  
}






void plotFWD()
{
  

  float resolutionFWDplot = ( maxFWDvoltsReading / 12 );

  
  float frameBitsFWDtoPlot = ( in_FWD_voltage / resolutionFWDplot );


  if ( in_FWD_voltage >= 0.1 )
  {
    
    for (int column = 0; column < frameBitsFWDtoPlot; column++)
    {
    frame[6][column] = 1;
    }
    

  }
  
  else
  
  {                                                             // 0 to 0.1 led is off, for noise.

    for (int column = 0; column < frameBitsFWDtoPlot; column++)     
    {
      
    frame[6][column] = 0;
   
    }
    
  }


  
}






void plotREV()
{
  

  float resolutionREVplot = ( maxREVvoltsReading / 12 );

  
  float frameBitsREVtoPlot = ( in_REV_voltage / resolutionREVplot );
  

  if ( in_REV_voltage >= 0.1 )
  {
    
    for (int column = 0; column < frameBitsREVtoPlot; column++)    
    {
    frame[7][column] = 1;
    }

  }
  
  else
  
  {                                                             // 0 to 0.1 led is off, for noise

    for (int column = 0; column < frameBitsREVtoPlot; column++)
    {
    frame[7][column] = 0;
    }
    
  }


  
}







void  highSWRalert()
{

  if (    ( (SWR_Value_Full >= highSWRlimit) && (highSWRenable == true) )   &&   ( displaySWRenable == true)    )
  {

      buzzerDelayCount = buzzerDelayCount + 1;

      if ( buzzerDelayCount >= buzzerDelay )
      {
        
        Serial.println(F("The highSWRlimit exceeded!"));

        digitalWrite(alarmLEDpin, HIGH);      // turn the LED on 
   
        buzzNow(highFreq, shortTone);
                 
        digitalWrite(alarmLEDpin, LOW);       // turn the LED off 
        
        alarmLED = true;

      }
  
  }

  else

  {

    buzzerDelayCount = 0;
    
    alarmLED = false;
    
  }

  
}








void SWR_Calc()
{

  float voltsInAdded = 0;
  float voltsInSubtracted = 0;

  
  voltsInAdded      = ( in_FWD_voltage + in_REV_voltage );

  voltsInSubtracted = ( in_FWD_voltage - in_REV_voltage );
  
  
  SWR_Value_Full = ( voltsInAdded / voltsInSubtracted );
  

  Serial.print(F("SWR = "));  Serial.println(SWR_Value_Full, 2);

  
  
}








void SWR_seperateIntDec()
{
  

  SWR_Value_IntegerPart = (int)SWR_Value_Full;                     // Extract integer part



  SWR_Value_DecimalPart = SWR_Value_Full - SWR_Value_IntegerPart;   // Extract decimal part

  SWR_Value_DecimalPart = SWR_Value_DecimalPart + 0.05;             // Round up to nearest digit

  int decimalPartAsInt = 10 * SWR_Value_DecimalPart;

  
  

  if (decimalPartAsInt >= 10)      
  {
    SWR_Value_IntegerPart = SWR_Value_IntegerPart + 1;
    decimalPartAsInt = 0;
  }


  digit1value = SWR_Value_IntegerPart;
   
  digit2value = decimalPartAsInt;
   
   
   
   

   
  swr_Value_Full_IOT =      digit1value + ( float(digit2value) / 10 );            // SWR value to 1 decimal place also updated to cloud




  
}









void fsdMeter()
{

  if (digit1value >= fsdSWR)      
  {
  
  digit1value = fsdSWR - 1;
  digit2value = 9;
  
  }


}







void convertDigits()
{

  digit1st = digit1value + 48;   // Adds offset of ASCII table value
  digit2nd = digit2value + 48;

}






void clear_frame() 
{
  for (int row = 0; row < 8; row++) {
    for (int col = 0; col < 12; col++) {
      frame[row][col] = 0;
    }
  }
}






void display_frame() 
{
  matrix.renderBitmap(frame, 8, 12);
}






void add1stDigit_to_frame(char c, int pos) 
{

  int extractedBit = 0;
  int index = -1;
  
  if (c >= '0' && c <= '9')
  {
    index = c - '0';
  }
  else 
  {
    Serial.println(F("Character not supported!"));
    return;
  }


  for (int row = 0; row < 5; row++) 
  {
  int frameBit = (firstDigitPos + 2);  
  int temp = fonts[index][row] ;

    for (int xtract = 0; xtract < 3; xtract++) 
    {
    extractedBit = bitRead(temp, xtract);

    
    frame[row][frameBit] = extractedBit;
    frameBit = frameBit - 1;

    
    }
  }


}






void add2ndDigit_to_frame(char c, int pos) 
{

  int extractedBit = 0;
  int index = -1;
  
  if (c >= '0' && c <= '9')
  {
    index = c - '0';
  }
  else 
  {
    Serial.println(F("Character not supported!"));
    return;
  }


  for (int row = 0; row < 5; row++) 
  {
  int frameBit = (firstDigitPos + 8);  
  int temp = fonts[index][row] ;

    for (int xtract = 0; xtract < 3; xtract++) 
    {
    extractedBit = bitRead(temp, xtract);

    
    frame[row][frameBit] = extractedBit;
    frameBit = frameBit - 1;

    
    }
  }



}














//-------------------------------------------------------------------------------------------------------
// U P D A T E   F R O M   D A S H B O A R D



/*
  Since OutputLED1 is READ_WRITE variable, onOutputLED1Change() is
  executed every time a new value is received from IoT Cloud.
*/
void onOutputLED1Change()  
{
  // Add your code here to act upon OutputLED1 change
  
  if (outputLED1 == true) 
  {
    digitalWrite(outputLED1pin, HIGH);      // turn the LED on 
    Serial.println(F("outputLED1pin is ON "));   
  }
  else
  {
    digitalWrite(outputLED1pin, LOW);      // turn the LED off
    Serial.println(F("outputLED1pin is OFF "));  
  }
  
}




/*
  Since OutputLED2 is READ_WRITE variable, onOutputLED2Change() is
  executed every time a new value is received from IoT Cloud.
*/
void onOutputLED2Change()  
{
  // Add your code here to act upon OutputLED2 change
  
  if (outputLED2 == true) 
  {
    digitalWrite(outputLED2pin, HIGH);      // turn the LED on 
    Serial.println(F("outputLED2pin is ON "));  
  }
  else
  {
    digitalWrite(outputLED2pin, LOW);      // turn the LED off 
    Serial.println(F("outputLED2pin is OFF "));  
  }  
  
}





/*
  Since HighSWRlimitIOT is READ_WRITE variable, onHighSWRlimitIOTChange() is
  executed every time a new value is received from IoT Cloud.
*/
void onHighSWRlimitIOTChange()  {
  // Add your code here to act upon HighSWRlimitIOT change
  
  highSWRlimit = highSWRlimitIOT;         //updates high SWR alarm limit from IOT dashboard
  
  Serial.print(F("The highSWRlimit is now: "));  Serial.println(highSWRlimit); 
  
  
  if ( highSWRlimit == 0 )                // if set to zero at dashboard then Alarm is disabled
  {
    highSWRenable = false;
    Serial.println(F("highSWRenable is OFF "));  
  }
  else
  {
    highSWRenable = true;
    Serial.println(F("highSWRenable is ON "));  
  }
  
  
}
