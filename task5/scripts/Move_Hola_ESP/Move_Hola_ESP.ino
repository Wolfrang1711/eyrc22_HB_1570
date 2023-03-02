// Team ID:    [ eYRC#HB#1570 ]
// Author List:  [ Pratik ]

#include <AccelStepper.h>
#include <Ticker.h>
#include <WiFi.h>
#include <WiFiUdp.h>
 
// set WiFi credentials
#define WiFi_SSID "Pratik"
#define WiFi_Password "1234567890"
#define UDP_Port 44444

// declaring motors step and direction pins
AccelStepper front_stepper(1, 13, 16);
AccelStepper right_stepper(1, 12,14);
AccelStepper left_stepper(1, 26, 25);

// initializing ticker 
Ticker interrupt;

// intiliazing UDP
WiFiUDP udp;

// initializing variables
char packet[255];
const char s[2] = ",";
char *fragment;
int i=0;
int velocity_array[255];
int v_left, v_forward, v_right;
 
void setup() 
{
  // setup serial port
  Serial.begin(115200);
  Serial.println();
 
  // begin WiFi
  WiFi.begin(WiFi_SSID, WiFi_Password);
 
  // connecting to WiFi
  Serial.print("Connecting to ");
  Serial.print(WiFi_SSID);
  
  // loop continuously while WiFi is not connected
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
 
  // connected to WiFi
  Serial.println();
  Serial.print("Connected! IP address: ");
  Serial.println(WiFi.localIP());

  // begin listening to UDP port
  udp.begin(UDP_Port);
  Serial.print("Listening on UDP port ");
  Serial.println(UDP_Port);

  // initializing interrupt to call the loop after given interval
  interrupt.attach_ms(1, move_hola);

  // setting max speed for 3 steppers
  front_stepper.setMaxSpeed(1000.0);
  left_stepper.setMaxSpeed(1000.0);
  right_stepper.setMaxSpeed(1000.0);
 
}

void move_hola(void)
{
  // setting speed to 3 steppers  
  front_stepper.setSpeed(v_forward);
  left_stepper.setSpeed(v_left);
  right_stepper.setSpeed(v_right);
  
}

void loop() 
{
  
  // running 3 stepper motors
  front_stepper.runSpeed();
  left_stepper.runSpeed();
  right_stepper.runSpeed();
  
  // If packet received
  int packet_size = udp.parsePacket();
  if (packet_size) 
  {
    // read packet and store it
    int packet_length = udp.read(packet, 255);
    if (packet_length > 0)
    {
      packet[packet_length] = 0;
    }
   
    // get the first fragment 
    fragment = strtok(packet, s);
    
    //walk through other fragments 
    while( fragment != NULL ) 
    {
      // convert the fragment into integer and store them in an velocity_arrayay
      int value = atoi(fragment);
      velocity_array[i++]= int(value);
      fragment = strtok(NULL, s);
      
    }

    // assign velocity_arrayay values to respective wheel velocity
    v_forward = velocity_array[0];
    v_left = velocity_array[1];
    v_right = velocity_array[2];    

    // resetting loop
    i = 0;    
    
  } 

}
