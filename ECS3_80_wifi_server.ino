
#include "crypto_diffie_hellman.h"

#include <SPI.h>
#include <WiFiST.h>
#include <WiFiServerST.h>
#include <WiFiClientST.h>

#include <Wire.h>
#include <M24SR.h>
/*
 * WiFiWebServer
 *
 * A simple web server that shows the value of the analog input pins
 * using the WiFi module ISM43362-M3G-L44.
 *
 * This example is written for a network using WPA encryption.
 * For WEP or WPA, change the Wifi.begin() call accordingly.
 *Configure SPI3:
 * MOSI: PC12
 * MISO: PC11
 * SCLK: PC10

Configure WiFi:
 * SPI         : SPI3
 * Cs          : PE0
 * Data_Ready  : PE1
 * reset       : PE8
 * wakeup      : PB13
 */



SPIClass SPI_3(PC12, PC11, PC10);
WiFiClass WiFi(&SPI_3, PE0, PE1, PE8, PB13);
void printWifiStatus();
int dataRead(char *c);


const int READ_BUFFER = 100;             // Size of reading buffer in WiFi device
uint8_t buf_read[READ_BUFFER]= {0};      // Reading buffer
int8_t read_index = READ_BUFFER;         // Reading index in buf_read

WiFiClient client;

char ssid[] = "rddl";      // your network SSID (name)
char pass[] = "code&riddle";   // your network password
int keyIndex = 0;                 // your network key Index number (needed only for WEP)

int WiFistatus = WL_IDLE_STATUS;
WiFiServer server(80);

/*  NFC M24SR Module Conifguration */

#define SerialPort      Serial
#define I2C2_SCL        PB10
#define I2C2_SDA        PB11
#define M24SR_ADDR      0xAC
#define GPO_PIN         PE4
#define RF_DISABLE_PIN  PE2

TwoWire dev_i2c(I2C2_SDA, I2C2_SCL);
M24SR nfcTag(M24SR_ADDR, &dev_i2c, NULL, GPO_PIN, RF_DISABLE_PIN);
/*Modbus Configuration*/

#include "ModbusMaster.h"

#define connection_error_led 13

#define BAUD 9600
#define TIMEOUT 100000
#define POLLING 10  // the scan rate

// If the packets internal retry register matches
// the set retry count then communication is stopped
// on that packet. To re-enable the packet you must
// set the "connection" variable to true.
#define RETRY_COUNT 10

// used to toggle the receive/transmit pin on the driver
#define TX_ENABLE_PIN 3 

#define NUM_REGISTERS 2//70

String display_packet(Packet* packet, char* parameter, char* unit);

// This is the easiest way to create new packets
// Add as many as you want. TOTAL_NO_OF_PACKETS
// is automatically updated.
enum
{
  PACKET1,
  PACKET2,
  PACKET3,
  PACKET4,
  // leave this last entry
  TOTAL_NO_OF_PACKETS
};

// Create an array of Packets for modbus_update()
Packet packets[TOTAL_NO_OF_PACKETS];

// Create a packetPointer to access each packet
// individually. This is not required you can access
// the array explicitly. E.g. packets[PACKET1].id = 2;
// This does become tedious though...
packetPointer packet1 = &packets[PACKET1];
packetPointer packet2 = &packets[PACKET2];
packetPointer packet3 = &packets[PACKET3];
packetPointer packet4 = &packets[PACKET4];

// The data from the PLC will be stored
// in the regs array
unsigned int regs[31];
unsigned int single_reg_1 = 0x1;
unsigned int regs_2[6];
//------------------------------end-of-modbus-stuff


void setup() {
  Serial1.begin(115200);
  //Serial2.begin(9600);
  
  pinMode(connection_error_led, OUTPUT);
  
  packet1->id = 1;
  packet1->function = READ_HOLDING_REGISTERS;
  packet1->address = 4119;
  packet1->no_of_registers = 4;
  packet1->register_array = &regs[0];


  packet2->id = 1;
  packet2->function = READ_HOLDING_REGISTERS;
  packet2->address = 4267;
  packet2->no_of_registers = 2;
  packet2->register_array = &regs[4];
  
  packet3->id = 1;
  packet3->function = READ_HOLDING_REGISTERS;
  packet3->address = 4303;
  packet3->no_of_registers = 2;
  packet3->register_array = &regs[6];

  packet4->id = 1;
  packet4->function = READ_HOLDING_REGISTERS;
  packet4->address = 4279;
  packet4->no_of_registers = 2;
  packet4->register_array = &regs[8];
  
  // Initialize communication settings etc...
  modbus_configure(BAUD, TIMEOUT, POLLING, RETRY_COUNT, TX_ENABLE_PIN, packets, TOTAL_NO_OF_PACKETS);

  
  // Initialize serial communication:

  
  while (!Serial1) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  
  

  Serial1.println("----------------");

  // Initialize the WiFi module:
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial1.println("WiFi module not detected");
    // don't continue:
    while (true);
  }

  // Print firmware version:
  String fv = WiFi.firmwareVersion();
  Serial1.print("Firwmare version: ");
  Serial1.println(fv);

  if (fv != "C3.5.2.3.BETA9") {
    Serial1.println("Please upgrade the firmware");
  }

  // attempt to connect to Wifi network:
  while (WiFistatus != WL_CONNECTED) {
    Serial1.print("Attempting to connect to WiFi network ");
    Serial1.println(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    WiFistatus = WiFi.begin(ssid, pass);

    // wait 10 seconds for connection:
    delay(10000);
  }

  // you're connected now, so print out the status
  Serial1.println("Connected.\nNetwork information:");
  printWifiStatus();
  server.begin();      // start the web server on port 80
  Serial1.println("---------Diffie Hellman---------");
}


void loop() {
  
  unsigned int connection_status = modbus_update(packets);
  
  client = server.available();       // listen for incoming clients

  if (client) {  // if a new client is connetcted,
    char c = 0x35;
    Serial1.println("New client connected");
    // an http request ends with a blank line
    boolean currentLineIsBlank = true;
    while (client.connected()) {
        connection_status = modbus_update(packets);     
        // Serial1.print("connection_status: ");
        // Serial1.println(connection_status);
      
        // Serial1.print("TOTAL_NO_OF_PACKETS: ");
        // Serial1.println(TOTAL_NO_OF_PACKETS);
        
        if (connection_status != TOTAL_NO_OF_PACKETS) {
          digitalWrite(connection_error_led, HIGH);
          // You could re-enable the connection by:
          //packets[connection_status].connection = true;
        }
        else
          digitalWrite(connection_error_led, LOW);
        
        // update the array with the counter data
        //regs[3] = packet1->requests;
        //regs[4] = packet1->successful_requests;
        //regs[5] = packet1->total_errors;
        
        //printPacketStatus();
        
        delay(10);
        dataRead(&c);
      // if you've received a newline character and the line is blank
      // the http request has ended, so you can send a reply
      if (c == '\n' && currentLineIsBlank) {
        // send a standard http response header
        client.println("HTTP/1.1 200 OK");
        client.println("Content-Type: text/html");
        client.println("Connection: close");  // the connection will be closed after completion of the response
        //client.println("Refresh: 10");        // refresh the page automatically every 10 seconds
        client.println();
        client.println("<!DOCTYPE HTML>");
        client.println("<html>");
        client.println("Diffie Hellman: ");
        // Serial Print the values
        for (int i = 0; i < 32; i++){ 
           uint8_t* tmp = secretASM();
           Serial.print(tmp[i],HEX);
           client.print(tmp[i],HEX);
        }
        Serial.println();
        client.println("<br />");    
        client.println(display_packet(packet1, "Consumption", "kWh"));
        client.println(display_packet(packet4, "Current", "A"));
        client.println(display_packet(packet2, "Voltage", "V"));
        client.println(display_packet(packet3, "Frequency", "Hz"));
        client.println("<br />");
        client.println("</html>");
        break;
      }

      if (c == '\n') {
        // you're starting a new line
        currentLineIsBlank = true;
      } else if (c != '\r') {
        // you've received a character on the current line
        currentLineIsBlank = false;
      }
    }
    // give the web browser time to receive the data
    delay(1);
    Serial1.println("---------");
    Serial1.print("connection status: ");
    Serial1.println(connection_status);
    Serial1.println("---------");
    Serial1.println(display_packet(packet1, "Consumption", "kWh"));
    Serial1.println(display_packet(packet4, "Current", "A"));
    Serial1.println(display_packet(packet2, "Voltage", "V"));
    Serial1.println(display_packet(packet3, "Frequency", "Hz"));
    // close the connection:
    client.stop();
    server.begin();
    Serial1.println("Client disconnected");
    }
}

void printWifiStatus() {
  // print the SSID of the network you're connected to:
  Serial1.print("SSID: ");
  Serial1.println(WiFi.SSID());

  // print the IP address of your WiFi module:
  IPAddress ip = WiFi.localIP();
  Serial1.print("IP Address: ");
  Serial1.println(ip);
  
  // print the MAC address of your WiFi module:
  byte mac[6];
  WiFi.macAddress(mac);
  Serial1.print("MAC address: ");
  for (uint8_t i = 0; i < 6; i++) {
    if (mac[i] < 0x10) {
      Serial1.print("0");
    }
    Serial1.print(mac[i], HEX);
    if (i != 5) {
      Serial1.print(":");
    } else {
      Serial1.println();
    }

    
  }

  // print the received signal strength (RSSI):
  int32_t rssi = WiFi.RSSI();
  Serial1.print("Signal strength (RSSI):");
  Serial1.print(rssi);
  Serial1.println(" dBm");

  // print where to go in a browser:
  Serial1.print("To see this page in action, open a browser to http://");
  Serial1.println(ip);
}

/*
 * Read data in the WiFi device. Reading byte by byte in very slow, so this function allows to
 * read READ_BUFFER byte in the device and return byte by byte the result.
 */
int dataRead(char *c) {
  if (read_index == READ_BUFFER) {        // No data available in the current buffer
    client.read(buf_read, READ_BUFFER);
    read_index = 0;
  }
  *c = buf_read[read_index];
  read_index++;

  if (*c != '\0') {
    return 1;
  } else {
    return 0;
  }
}

String display_packet(Packet* packet, char* parameter, char* unit){
  String message;
  //Serial1.println(packet->no_of_registers);
  /*
  for( int i=0 ; i< (packet->no_of_registers) ;i++){
    Serial1.print("reg array[");
    Serial1.print(i+1);
    Serial1.print("]:");
    Serial1.println(packet->register_array[i]);
  }*/
  if(packet->no_of_registers == 2){
    unsigned long x = packet->register_array[0];
    x =  (x<<16) | packet->register_array[1] ;
    int x1 = x/10000;
    int x2 = x%10000;
    //Serial1.println(x1);
    //Serial1.println(x2);
    //sprintf(message, "%s: %d.%d %s" , parameter , x1 ,  x2 , unit);  
    //Serial1.println((x ) | packet->register_array[0]); 
    message = String(parameter) + ": " + String(x1) + "." + String(x2) + " " + String(unit);
    
  }
  if(packet->no_of_registers == 4){
    unsigned long x = packet->register_array[0];
    x =  (x<<16) | packet->register_array[1] * 1000000;
    unsigned long xx = packet->register_array[2];
    xx =  (xx<<16) | packet->register_array[3] ;
    int x1 = xx/10000;
    int x2 = xx%10000;
    //Serial1.println(x1);
    //Serial1.println(x2);
    //sprintf(message, "%s: %lu%d.%d %s" , parameter , x , x1,   x2 , unit);  
    message = String(parameter) + ": " + String(x) + String(x1) + "." + String(x2) + " " + String(unit);
  }
  return message;
  
  
}

