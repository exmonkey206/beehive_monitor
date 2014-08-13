#include <DHT.h>


//Version 4 of the beehive monitor

//**************************************************
//Marc Curtis 2014 - @exmonkey - exmonkey@gmail.com - 
//Uses Sleep_n0m1 Library - Noah Shibley, Michael Grant, NoMi Design Ltd. http://n0m1.com  https://github.com/n0m1/Sleep_n0m1 
//


#include <GSM.h>                  // Importing all appropriate libraries
#include <HttpClient.h>
#include <Xively.h>
//#include <DHT22.h>
#include <DHT.h>
#include <Sleep_n0m1.h>



#define PINNUMBER ""                       // SIM PIN (not required for GiffGaff
#define GPRS_APN "giffgaff.com"           // Access Point Name
#define GPRS_LOGIN "giffgaff"            // Username (phone number)
#define GPRS_PASSWORD "password"        // Password (phone number)
// Setup a DHT22 nstance
#define DHTTYPE DHT22   // DHT 22  (AM2302)
#define DHTPIN 5
DHT dht(DHTPIN, DHTTYPE);
//#define DHT22_PIN 4
//DHT22 myDHT22(DHT22_PIN);

float temperature = 0; 
float humidity = 0;
int wait = 225;
boolean notConnected = true;
unsigned long time;
int mins;
boolean working;

//sleep stuff
Sleep sleep;
unsigned long sleepTime; //how long you want the arduino to sleep


//charger shield stuff
const int analogInPin = A0;    // Analog input pin that the VBAT pin is attached to
int BatteryValue = 0;          // value read from the VBAT pin
double outputValue = 0;        // variable for voltage calculation

char myTempStream[] = "temperature";            // Set stream name (need to match Xively name)
char myHumidityStream[] = "humidity";           // Set 2nd stream name
char myBatteryValue[] = "battery";              // Set 3rd stream name

//GSM shield objects
GSMClient client;
GPRS gprs;
GSM gsmAccess;

XivelyDatastream datastreams[] = {              // Create the datasterams
  XivelyDatastream(myTempStream, strlen(myTempStream), DATASTREAM_FLOAT),
  XivelyDatastream(myHumidityStream, strlen(myHumidityStream), DATASTREAM_FLOAT),
  XivelyDatastream(myBatteryValue, strlen(myBatteryValue), DATASTREAM_FLOAT),

};


#include "passwords.h"  //unique Xively API key replace with below
//#define FEED_ID XXXXXXXXXXXX
//char xivelyKey[] = "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX";    // Set the Xively API key

XivelyFeed feed(FEED_ID, datastreams, 3);       // Creating the feed, defining three datastreams
XivelyClient xivelyclient(client);              // Telling Xively to use the correct client


void setup(void) {                              // Connecting to network, initiating GPRS connection
  Serial.begin(9600); 
  sleepTime = 360000;//1 hour 10000;//7200000; //how long to sleep for - 2 hours
  boolean notConnected = true; //set connected value for GSM shield
  boolean working = false;
  dht.begin();

}


void loop(void) {

  if(working==false){
    working = true;
    Serial.print("allow the sensor to warm up");
    delay(5000);

    Serial.print("\nVoltage: ");
    //get data from the charger shield
    BatteryValue = analogRead(analogInPin);            
    // Calculate the battery voltage value
    outputValue = (float(BatteryValue)*5)/1023*2;
    Serial.print(outputValue);
    Serial.print("\n---------\n");

    //connect the GSM to GPRS
    startConnection();
  }


}



// Get the data from the DHT222 sensor
void get_data(){
  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)

  boolean gotReading = 0;

  while (gotReading==0) {
    humidity = dht.readHumidity();
    temperature = dht.readTemperature();
    // Check if any reads failed and exit early (to try again).
    if (isnan(humidity) || isnan(temperature) ){
      Serial.println("Failed to read from DHT sensor!");
      delay(1000);
    }
    else{
      gotReading = 1;
    }
  }

  // Compute heat index
  // Must send in temp in Fahrenheit!
  //float hi = dht.computeHeatIndex(f, h);

  Serial.print("Humidity: "); 
  Serial.print(humidity);
  Serial.print(" %\t");
  Serial.print("Temperature: "); 
  Serial.print(temperature);
  Serial.print(" *C ");
  processData();
}


//connect to GPRS
void startConnection(){

  while (notConnected) {
    digitalWrite(3,HIGH);                       // Enable the RX pin
    if(gsmAccess.begin(PINNUMBER)==GSM_READY){
      delay(3000);
      if(gprs.attachGPRS(GPRS_APN, GPRS_LOGIN, GPRS_PASSWORD)==GPRS_READY){
        notConnected = false;
        Serial.println("connected...");  
        //Get the sensor data
        get_data();
      }
    }
    else{
      Serial.println("loop");  
      delay(1000);
    }
  }
}


void closeConnection(){
  Serial.println("closing connection");  
  while(notConnected==false){
    if(gsmAccess.shutdown()){
      delay(1000);
      digitalWrite(3,LOW);                    // Disable the RX pin
      notConnected = true;
      Serial.println("GSM connection closed");

      Serial.println("Entering Sleep Mode");  
      working = false;
      sleep.pwrDownMode(); //set sleep mode
      sleep.sleepDelay(sleepTime); //sleep for: sleepTime
    }
    else{
      delay(1000);
    }
  }




}

void processData(){
  Serial.println("process data"); 



  //add the results to the xively stream
  datastreams[0].setFloat(temperature); 
  datastreams[1].setFloat(humidity);
  datastreams[2].setFloat(outputValue);

  int ret = xivelyclient.put(feed, xivelyKey);  // Send to Xively
  Serial.println("Ret: "); 
  Serial.println(ret); 

  if(ret == 200){
    Serial.println(ret);      
    delay(1000);
  }
  else{                                    
    delay(1000);
    Serial.println(ret);
  }

  delay(5000);
  Serial.println("call close connecttion");  

  closeConnection();

}




