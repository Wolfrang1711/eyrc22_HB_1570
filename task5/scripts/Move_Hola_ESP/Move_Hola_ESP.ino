// Team ID:    [ eYRC#HB#1570 ]
// Author List:  [ Pratik ]

#include <AccelStepper.h>
#include <Ticker.h>
#include <WiFi.h>
#include <WiFiUdp.h>
 
// Set WiFi credentials
#define WIFI_SSID "Redmi Note 10S"
#define WIFI_PASS "12345678"
#define UDP_PORT 44444

AccelStepper front_stepper(1, 13, 16);
AccelStepper right_stepper(1, 12,14);
AccelStepper left_stepper(1, 25, 26);

Ticker ticker;

// UDP
WiFiUDP UDP;
char packet[255];
char reply[] = "Packet received!";
const char s[2] = ",";
char *token;
int i=0;
float arr[255];

float v_left, v_forward, v_right;
 
void setup() 
{
  // Setup serial port
  Serial.begin(115200);
  Serial.println();
 
  // Begin WiFi
  WiFi.begin(WIFI_SSID, WIFI_PASS);
 
  // Connecting to WiFi...
  Serial.print("Connecting to ");
  Serial.print(WIFI_SSID);
  // Loop continuously while WiFi is not connected
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
 
  // Connected to WiFi
  Serial.println();
  Serial.print("Connected! IP address: ");
  Serial.println(WiFi.localIP());

  // Begin listening to UDP port
  UDP.begin(UDP_PORT);
  Serial.print("Listening on UDP port ");
  Serial.println(UDP_PORT);

  // Initializing timer to call the loop after given interval
  ticker.attach_ms(1, move_hola);

  // Setting max speed for 3 steppers
  front_stepper.setMaxSpeed(1000.0);
  left_stepper.setMaxSpeed(1000.0);
  right_stepper.setMaxSpeed(1000.0);
 
}

void move_hola(void)
{
  // Setting speed to 3 steppers  
  front_stepper.setSpeed(v_forward);
  left_stepper.setSpeed(v_left);
  right_stepper.setSpeed(v_right);
  
}


void loop() 
{
  
  // If packet received...
  int packetSize = UDP.parsePacket();
  if (packetSize) 
  {
//    Serial.print("Received packet! Size: ");
//    Serial.println(packetSize);/ 
    
    int len = UDP.read(packet, 255);
    if (len > 0)
    {
      packet[len] = 0;
    }
//    Serial.print("Packet/ received: ");
//    Serial.println(packet);    
   
    /* get the first token */
    token = strtok(packet, s);
    
    /* walk through other tokens */
    while( token != NULL ) {

      float value = atof(token);
//      Serial.println(value);

      arr[i]= float(value);
          
      token = strtok(NULL, s);

      i+=1; 

      if(i>3)
      {
        i=0;
      }
      
    }
    
    v_right = arr[0];
    v_forward = arr[1];
    v_left = arr[2];

  } 

  
    // Running 3 stepper motors
    front_stepper.runSpeed();
    left_stepper.runSpeed();
    right_stepper.runSpeed();

}
