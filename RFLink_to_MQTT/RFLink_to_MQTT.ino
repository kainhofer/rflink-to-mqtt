/*
Link between the RF reciever/transmitter project based on an arduino mega, RFLink and MQTT
Link to RFLink: http://www.nemcon.nl/blog2/download
This includes wiring and device compatability for RFLink. 
This is used as per the RFLink instructions except that 
 an ESP8266 is used for the serial communication instead of a computer via USB


RFLink is designed to be used by a computer using USB
This bridge uses and ESP8266 as the interface, which then encodes the recieved RF data and publishes it as JSON to an MQTT server

RF433 data is recieved by the RFLink and passed on to the ESP8266
The ESP8266 packages it up into a JSON statement and publishes it ot the MQTT broker
format on the MQTT broker is: (Recieved RF codes)

Topic: RF/name_of_device/ID_of_device     - this tries to have a unique topic per name/id combo. 
    Note - the name and ID are as determined by the RFLink program - it may not be the label printed on the device sending data!

Payload example: {"raw":"20;B3;DKW2012;ID=004c;TEMP=00eb;HUM=3f;WINSP=0000;WINGS=0000;RAIN=0003;WINDIR=0008;\r","TEMP":23.50,"HUM":3,"WINSP":"0000","WINGS":"0000","RAIN":3,"WINDIR":8}
see RFLink documentation for more details: http://www.nemcon.nl/blog2/protref

Sending commands to the RFLink example: (how to send codes to the RFLink)
Topic: RF/command
Payload: 10;Eurodomest;02d0f2;06;ALLON\n
    Note that the \n on the end is critical at the moment. Otherwise the ESP will either crash or the RFLink will ignore



I inculde the raw data in full for debugging, then the split out components form RFink
There is some conversions made to make the data more useable:
    Temp is converted to a float from hex
    Wind direction is converted to a compass heading
    etc - details in the parseData function below


Requirements:
Arduino mega 2560 and RF reciever and or transmitter (see RFLink for recommended devices and wiring)
ESP8266 device (I have used a node MCU V1.0)
Arduino libraries: SoftwareSerial.h and ArduinoJson.h
an MQTT broker runnning. Tested with Mosquitto on a raspberry Pi 2

Optional: 
Somethig to read and react to the MQTT measurements
I am using Home Assistant, also running on the same Pi : https://home-assistant.io/


Setup:
1) Confirm RFLink working on its own through your USB port - then you know you are getting data recieved etc
    Collect some data form your RF433 devices on the system monitor screen and save them for step 2

2) Set sketch to be in test mode (testmode = true). 
   Load this firmware onto the ESP8266 and run in test mode (so it listens to the PC over USB, not the RFLink)
    Input some data as recieved in step 1. Check that the ESP connects to your system and publishes topics as expected

3) Load updated firmware - setting testmode to false (see section for user inputs below)
    Wire the ESP to the RFLink ESP d5 & d6  to Mega 0 & 1.
    Check your mqtt broker for recieved data being published.
 
4) Setup your home automation to react to the published data and pubish commands to the RFLink 
      
To Do:
1) ESP will sometimes crash with unexpected output from RFLink. 
    I think it is when you get messages starting with 20 but not with as as many semicolon delimited fields as expected.
    Currently, I ensure that the Mega (RFLink) is up before restarting the ESP.

2) Tidy up the callback behaviour for sending data to the RFLink - data is fickle and if you do not terminate with \n it will crash the ESP
  
  
Phil Wilson December 2016

*/


//============
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
// if having problems with larger payloads, increase #define MQTT_MAX_PACKET_SIZE 128 in PubSubClient.h to a larger value before compiling
// to allow larger payloads - needed if you have a weather station (because I also include raw data in the json payload in case you need it for deugging) 
#include <PubSubClient.h>


WiFiClient espClient;
PubSubClient client(espClient);

//#define TESTMODE false
#define DEBUG_SERIAL 1
#define DEBUG_MQTT   2
#define DEBUG_NONE   0
#define DEBUG_TYPE   DEBUG_SERIAL   // Valid: DEBUG_SERIAL, DEBUG_MQTT, DEBUG_NONE
#define USE_SOFTWARE_SERIAL TRUE // Comment this out to use the hardware serial for communication


// Setup the serial device to use for communication (software or hardware serial
#if USE_SOFTWARE_SERIAL
  #include <SoftwareSerial.h>
  // Key User Configuration here:
  // SoftwareSerial swSer(14, 12, false, 256); // d5 & d6 on the nodu MC v1.0
  SoftwareSerial swSer(1, 2, false, 256); // d5 & d6 on the nodu MC v1.0
  #define SERIAL swSer
#else
  #define SERIAL Serial
#endif


const char* ssid = "opentools"; // network SSID for ESP8266 to connect to
const char* password = "reinhold"; // password for the network above
const char* mqtt_server = "10.0.0.112"; // address of the MQTT server that we will communicte with
const char* mqtt_user = "openhabian";
const char* mqtt_pass = "reinhold";
#ifdef TESTMODE
  char* client_name = "RFLink-TEST"; // production version client name for MQTT login - must be unique on your system
#else
  char* client_name = "RFLink"; // production version client name for MQTT login - must be unique on your system
#endif
#define MQTT_PREFIX "/sensors/rflink/"
const char* mqtt_prefix = MQTT_PREFIX;
const char* mqtt_willTopic = MQTT_PREFIX "status";
const char* mqtt_rawTopic = MQTT_PREFIX "raw";
const char* mqtt_commandTopic = MQTT_PREFIX "command";  // command topic ESP will subscribe to and pass as commands to the RFLink
const char* mqtt_debugTopic = MQTT_PREFIX "debug";      // MQTT topic to send debug messages to
boolean willRetain = true;
const char* willMessage = "offline" ;



// some testing switches
boolean enableMQTT = true; // if false, do not transmit MQTT codes - for testing really



// ******************************************************************



// ArduinoJson credits - used for building JSON to post to MQTT

// Copyright Benoit Blanchon 2014-2016
// MIT License
//
// Arduino JSON library
// https://github.com/bblanchon/ArduinoJson
#include <ArduinoJson.h>


// Depending on the definition of DEBUG_TYPE, we provide the functions 
// rflinkDebug and rflinkDebugln to print out debug output either to the 
// hardware serial console or the MQTT broker
#if DEBUG_TYPE == DEBUG_SERIAL
  #define rflinkDebug Serial.print
  #define rflinkDebugln Serial.println


#elif DEBUG_TYPE == DEBUG_MQTT
  // Print all debug output to the MQTT broker
  String debugMsg = "";

  void rflinkDebug() {}
  void rflinkDebugln() {}

  void rflinkDebug(String msg) { rflinkDebug(msg.c_str()); }
  void rflinkDebugln(String msg) { rflinkDebugln(msg.c_str()); }

  void rflinkDebug(const char*msg) { debugMsg += msg; }
  void rflinkDebugln(const char*msg) {
    debugMsg += msg;
    // If we are not connected to MQTT, we cannot print any debug output!!!
    if (client.connected()) {
      client.publish(mqtt_debugTopic, debugMsg.c_str());
      debugMsg = "";
    } else {
      debugMsg += "\n";
    }
  }

#else 
  // No debug type set
  void rflinkDebug() {}
  void rflinkDebugln() {}
  void rflinkDebug(String) {}
  void rflinkDebugln(String) {}
  void rflinkDebug(const char*) {}
  void rflinkDebugln(const char*) {}

#endif




const byte numChars = 254;
char receivedChars[numChars];
char tempChars[numChars];        // temporary array for use when parsing

// variables to hold the parsed data

char messageFromPC[numChars];// = {0};
char RFName[30]; // name of protocol from RFLINK
char RFID[30]; //ID from RFLINK ie ID=XXXX;
char RFData[numChars]; //the rest from RFLINK - will include one or many pieces of data
char RFDataTemp[numChars]; //temporary area for processing RFData - when we look for temp and convert etc


const float TempMax = 100.0; // max temp - if we get a value greater than this, ignore it as an assumed error
const int HumMax = 101; // max hum - if we get a value greater than this, ignore it as an assumed error



boolean newData = false;

void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  rflinkDebugln();
  rflinkDebug("Connecting to ");
  rflinkDebugln(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    rflinkDebug(".");
  }

  rflinkDebugln();
  rflinkDebug("WiFi connected. ");
  rflinkDebug("IP address: ");
  rflinkDebugln(WiFi.localIP().toString());
}

void callback(char* topic, byte* payload, unsigned int length) {
  payload[length] = '\0'; // terminate payload
  char mqtt_payload[numChars];
  strcpy(mqtt_payload, (char*)payload);
  
  rflinkDebug("Command coming in!: "); // got someting
  if (mqtt_payload[length-1] != '\n') {
    mqtt_payload[length] = '\n';
    mqtt_payload[length+1] = '\0';
  }
  rflinkDebugln(mqtt_payload); // got someting
  SERIAL.println(mqtt_payload);   // send data to the RFLink      
    
//  if(strncmp(strPayloadTrimmed,"10",2) == 0) { // starts with 10
//    rflinkDebugln("got a command - test result: ");
//    strPayloadTrimmed2.remove(strPayloadTrimmed2.length()-1,1);
//    rflinkDebugln(strPayloadTrimmed2.c_str());  
////    SERIAL.print(strPayload);   // snd data to the RFLink      
//  }
}


void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    rflinkDebug("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(client_name, mqtt_user, mqtt_pass, mqtt_willTopic, 0, willRetain, willMessage)) {
      rflinkDebugln("connected");
      // Once connected, update status to online - will Message will drop in if we go offline ...
      client.publish(mqtt_willTopic,"online",true); 
          
      client.subscribe(mqtt_commandTopic);// subscribe to the command topic - will listen here for comands to the RFLink
      rflinkDebugln("\nRFLINK serial listener started\n\nExample data  20;0D;UPM_Esic;ID=test;TEMP=00df;HUM=67;BAT=OK;\nExample data  20;B3;DKW2012;ID=test;TEMP=00eb;HUM=3f;WINSP=008c;WINGS=00cd;RAIN=0003;WINDIR=0008;");
      rflinkDebugln();
      
    } else {
      rflinkDebug("failed, rc=");
      rflinkDebug(String(client.state()));
      rflinkDebugln(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void setup() {
  // JSON parsing library setup
  DynamicJsonBuffer  jsonBuffer;

  // Create the root of the object tree.
  //
  // It's a reference to the JsonObject, the actual bytes are inside the
  // JsonBuffer with all the other nodes of the object tree.
  // Memory is freed when jsonBuffer goes out of scope.
  JsonObject& root = jsonBuffer.createObject();
  
  Serial.begin(57600);
  SERIAL.begin(57600); // this is the baud rate of the RF LINK
    // expected format examples: - will treat as 5 components (Packet Count ignored).
    // 20;02;Imagintronix;ID=0001;TEMP=00dc;HUM=88; 
    // 20;0D;UPM_Esic;ID=0001;TEMP=00df;HUM=67;BAT=OK;
   
    // Node [nn]: 20 - means message from system - proceed if prefix is 20. Other values are 10 fofr sent message and 11 for recursive
    // Packet Count [hh]: next is packet count - ignore (2 digit hexdecimal)
    // Name [text]:  Name of the protocol used
    // ID [ID=text]: ID - proceed if ID indentified. If not there, is not a recieved code, just a message from system
    // Data [text]: remainder is data from the sensor - concateate & send as a JSON data block
 
    setup_wifi(); 
    client.setServer(mqtt_server, 1883);
    client.setCallback(callback);
  
    rflinkDebugln("\nRFLINK serial listener started");
    rflinkDebugln("Example data  20;0D;UPM_Esic;ID=test;TEMP=00df;HUM=67;BAT=OK;");
    rflinkDebugln("Example data  20;B3;DKW2012;ID=test;TEMP=00eb;HUM=3f;WINSP=008c;WINGS=00cd;RAIN=0003;WINDIR=0008;");
    rflinkDebugln("");

  // Hostname defaults to esp8266-[ChipID]
  ArduinoOTA.setHostname(client_name);

  // No authentication by default
  // ArduinoOTA.setPassword((const char *)"123");

  ArduinoOTA.onStart([]() {
    Serial.println("OTA Start");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();
  rflinkDebugln("oTA Ready");
  rflinkDebug("IP address: ");
  rflinkDebugln(WiFi.localIP().toString().c_str());
// end OTA stuff in setup
}



//============

void recvWithStartEndMarkers() {
  static byte ndx = 0;
  char endMarker = '\n';
  char rc;

  while (SERIAL.available() > 0 && newData == false) {
    rc = SERIAL.read();
  
    if (rc != endMarker) {
      receivedChars[ndx] = rc;
      ndx++;
      if (ndx >= numChars) {
        ndx = numChars - 1;
      }
    } else {
      receivedChars[ndx] = '\0'; // terminate the string
      ndx = 0;
      newData = true;
    }
  }
}

//============

float hextofloat(char* hexchars) {return float(strtol(hexchars,NULL,16));}
int hextoint(char* hexchars) {return strtol(hexchars,NULL,16);}

void parseData(char *msg) {      // split the data into its parts
  DynamicJsonBuffer  jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();
//  root["raw"] = receivedChars; // copy the raw data to the json in case we need to debug
  root["raw"] = msg; // copy the raw data to the json in case we need to debug

  rflinkDebug("Message received: ");
  rflinkDebugln(msg);
  client.publish(mqtt_rawTopic, msg);

  float tmpfloat = 0.0; // temporary float used in tests
  int tmpint = 0;       // temporary int used in tests
  char * strtokIndx;    // this is used by strtok() as an index
  strtokIndx = strtok(msg,"\n");  // Consider only the first line received (i.e. replace \n by \0)
  strtokIndx = strtok(msg,";");   // get the first part - the string

  // Message needs to start with "20":
  if (strcmp(strtokIndx, "20") == 0 ) { // 20 means a message recieved from RFLINK - this is what we are interested in breaking up for use
    // IGNORE the next token (running packet counter)
    strtokIndx = strtok(NULL, ";");
    strcpy(RFData, strtokIndx);     // copy remainder of block to RFData 
    // Next token is the family
    if (strtokIndx != NULL) {
      strtokIndx = strtok(NULL, ";");
    }
    if (strtokIndx != NULL) {
      strcpy(RFName, strtokIndx); // copy name to RFName
      root["Family"] = RFName;
    }
    // Ignore certain messages (i.e. confirmations of 10;... commands, debug output
    if (strcmp(RFName, "OK")==0 || strcmp(RFName, "RFDEBUG=")==0 || strcmp(RFName, "DEBUG")==0 || strcmp(RFName, "Slave")==0 ||strcmp(RFName, "PONG")==0 ) {
      return;
    }
    RFID[0] = '\0';

    strtokIndx = strtok(NULL, ";");
    // Read each command pair 
    while (strtokIndx != 0) {
      // Split the command in two values
      char* separator = strchr(strtokIndx, '=');
      if (separator != 0) {
        // Actually split the string in 2: replace '=' with 0
        *separator = 0;
        String NamePart = strtokIndx;
        ++separator;
        
        if (NamePart == "ID") {
          root[NamePart] = separator;
          strcpy(RFID, separator);
         
        } else if (NamePart == "TEMP") { // test if it is TEMP, which is HEX
          int tmpval = hextoint(separator);
          float neg = 1.0;
          if (tmpval & 0x8000) {
            neg = -1.0;
            tmpval = tmpval & 0x7FFF;
          }
          float tmpfloat = float(tmpval)*0.1; // divide by 10 - using multiply as it is faster than divide
//          float tmpfloat = hextofloat(separator)*0.1;
//          if (tmpfloat < TempMax) { //test if we are inside the maximum test point - if not, assume spurious data
            root.set<float>(NamePart, tmpfloat*neg); // passed spurious test - add data to root
//          } 
        } /// end of TEMP block
        else if (NamePart == "HUM") { // test if it is HUM, which is int
          if (strcmp(RFName,"DKW2012") == 0 ) { // digitech weather station - assume it is a hex humidity, not straight int
            tmpint = hextoint(separator);
          } else {
            tmpint = atoi(separator); // end of setting tmpint to the value we want to use & test
          }
          if (tmpint > 0 and tmpint < HumMax) { //test if we are inside the maximum test point - if not, assume spurious data
            root.set<int>(NamePart, tmpint);  // passed the test - add the data to rot, otherwise it will not be added as spurious
          }
        }  // end of HUM block                
        else if (NamePart == "RAIN" || NamePart == "WINSP" || NamePart == "WINGS") { // Handle all HEX data fields:
          root.set<float>(NamePart, hextofloat(separator)*0.10 );
        } 
        else {// check if an int, add as int, else add as text
          char *ptr;
          long val = strtol(separator, &ptr, 10);
          if (*ptr != '\0') { // not an integer, further characters following
            root[NamePart] = separator; // do normal string add
          } else {
            root.set<long>(NamePart, val); // do int add
          }
        }
      }
   
      // Find the next command in input string
      strtokIndx = strtok(NULL, ";");
    }

  } else { // not a 20 code- something else
    rflinkDebugln("Received serial command that is not a 20 code "); 
    strcpy(RFData, strtokIndx); // copy all of it to RFData 
    strcpy(RFName, "unknown");
    strcpy(RFID, "");
  }

//  client.publish("RF/" + RFName + "-" + RFID , root );


  String MQTTTopic = mqtt_prefix;
  MQTTTopic += String(RFName);
  MQTTTopic += "/" ;
  MQTTTopic += String(RFID);
  MQTTTopic += '\0';
  size_t lenM = MQTTTopic.length(); // returns length of the json     
  size_t sizeM = lenM + 1;

  char MQTTTopicConst[lenM];
  MQTTTopic.toCharArray(MQTTTopicConst,sizeM) ;
    
  // place the json data into variable 'json' for publishing over MQTT
  size_t len = root.measureLength(); // returns length of the json 
  size_t size = len+1;
  char json[size];
  root.printTo(json,size);

  rflinkDebug(MQTTTopicConst);
  rflinkDebug("   ");
  rflinkDebugln(json);
  client.publish(MQTTTopicConst, json);

}

//============

void showParsedData() {
  rflinkDebug("Got something : ");
  rflinkDebugln(receivedChars);
  // mqtt structure
}


//============

void loop() {
  recvWithStartEndMarkers();
  if (newData == true) {
    strcpy(tempChars, receivedChars);
    // this temporary copy is necessary to protect the original data
    //   because strtok() used in parseData() replaces the commas with \0
    parseData(tempChars);
    //showParsedData(receivedChars);
    newData = false;
  }
  if (!client.connected() and enableMQTT ) {
    reconnect();
  }
  client.loop();

  // listen for OTA reprogramming
  ArduinoOTA.handle(); 
}


