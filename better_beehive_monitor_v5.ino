
//Version 4 of the beehive monitor

//**************************************************
//Marc Curtis 2014 - @exmonkey - exmonkey@gmail.com - 
//Uses Sleep_n0m1 Library - Noah Shibley, Michael Grant, NoMi Design Ltd. http://n0m1.com  https://github.com/n0m1/Sleep_n0m1 
//

          
#include <GSM.h>                  // Importing all appropriate libraries
#include <HttpClient.h>
#include <Xively.h>
#include <DHT22.h>
#include <Sleep_n0m1.h>



#define PINNUMBER ""                       // SIM PIN (not required for GiffGaff
#define GPRS_APN "giffgaff.com"           // Access Point Name
#define GPRS_LOGIN "giffgaff"            // Username (phone number)
#define GPRS_PASSWORD "password"        // Password (phone number)
// Setup a DHT22 nstance
#define DHT22_PIN 4
DHT22 myDHT22(DHT22_PIN);
double temperature = 0; 
double humidity = 0;
int wait = 225;
boolean notConnected = true;
unsigned long time;
int mins;


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

while (notConnected) {

    if(gsmAccess.begin(PINNUMBER)==GSM_READY){
      delay(3000);
      if(gprs.attachGPRS(GPRS_APN, GPRS_LOGIN, GPRS_PASSWORD)==GPRS_READY){
        notConnected = false;
      }
    }
    else{
      delay(1000);
    }
  }
}


void loop(void) {
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

  //Get the sensor data
  get_data();

  //add the results to the xively stream
  datastreams[0].setFloat(temperature); 
  datastreams[1].setFloat(humidity);
  datastreams[2].setFloat(outputValue);

  int ret = xivelyclient.put(feed, xivelyKey);  // Send to Xively
  if(ret == 200){
Serial.println(ret);      
      delay(100);
    }else{                                    
      delay(100);
    Serial.println(ret);
    }
    Serial.println("call close connecttion");  
  closeConnection();
}



// Get the data from the DHT222 sensor
void get_data(){
  DHT22_ERROR_t errorCode;

  // The sensor can only be read from every 1-2s, and requires a minimum
  // 2s warm-up after power-on.

  errorCode = myDHT22.readData();
  switch(errorCode)
  {
  case DHT_ERROR_NONE:
    //got some value - pass it to data function
    processData(myDHT22.getTemperatureC(),myDHT22.getHumidity());

    break;
  case DHT_ERROR_CHECKSUM:
    Serial.print("check sum error ");
    Serial.print(myDHT22.getTemperatureC());
    Serial.print("C ");
    Serial.print(myDHT22.getHumidity());
    Serial.println("%");
    break;
  case DHT_BUS_HUNG:
    Serial.println("BUS Hung ");
    break;
  case DHT_ERROR_NOT_PRESENT:
    Serial.println("Not Present ");
    break;
  case DHT_ERROR_ACK_TOO_LONG:
    Serial.println("ACK time out ");
    break;
  case DHT_ERROR_SYNC_TIMEOUT:
    Serial.println("Sync Timeout ");
    break;
  case DHT_ERROR_DATA_TIMEOUT:
    Serial.println("Data Timeout ");
    break;
  case DHT_ERROR_TOOQUICK:
    Serial.println("Polled to quick ");
    break;
  }
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
    }
    else{
      delay(1000);
    }
  }

  Serial.println("Entering Sleep Mode");  
  sleep.pwrDownMode(); //set sleep mode
  sleep.sleepDelay(sleepTime); //sleep for: sleepTime


}

void processData(double t,double h){
  temperature = t;
  humidity = h;
  Serial.println(t);
  Serial.println(h);

}



