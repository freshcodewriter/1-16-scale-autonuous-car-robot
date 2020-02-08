#include <WiFi.h>
#include <WiFiUdp.h>

#define Pot1 32 //declare the port 1 for slider pot 1
#define Pot2 33 //declare the port 2 for slider pot 2
#define Switch 13 //decalre the port for switch
#define WeaponPin1 36 //declare the port 1 for slider pot 1
#define WeaponPin2 39 //declare the port 2 for slider pot 2
#define Switch2 14 //decalre the port for switch

WiFiUDP udp;  //degine local wifiudp

const char* ssid = "Hogwarts";
const char* password = "Slytherin";
WiFiServer server(80);

WiFiUDP UDPTestServer; //degine target wifiudp
unsigned int udpTargetPort = 2100; //define target port of sending
unsigned int udpLocalPort = 2200; //define local port of receiving
const int UDP_PACKET_SIZE = 20; //define udp packet size
char udpBuffer[UDP_PACKET_SIZE]; //define udpBuffer of sending
IPAddress ipTarget(192, 168, 1, 186); // change to your target esp32 IP ADDRESS
IPAddress ipLocal(192, 168, 1, 189);  // change to your local esp32 IP ADDRESS
byte packetBuffer[UDP_PACKET_SIZE + 1]; //define packetBuffer of recieving
unsigned int annieADC; //degine global int annieADC that recieved
unsigned int annieSwitch; //degine global int annieSwitch that recieved
unsigned int result = 0; //degine global int result
int current = 0; //degine global int current
int previous = 0; //degine global int previous

void setup() {
  Serial.begin(115200);  //set up serial in upload spped of 115200
  pinMode(LED_BUILTIN, OUTPUT); //set up the onboard led pin
  Serial.println("Connecting to ");  Serial.println(ssid); //print on the serial monitor of the wifi of connecting

  WiFi.mode(WIFI_AP); //set up AP mode
  WiFi.softAP(ssid, password); //set the wifi ssid and passward
  delay(100); //delay for 100 ms
  WiFi.softAPConfig(ipLocal, IPAddress(192, 168, 1, 1), IPAddress(255, 255, 255, 0)); //set local IP and router IP
  UDPTestServer.begin(udpTargetPort); // begin to link to target port
  udp.begin(udpLocalPort); // begin to link to local port

  IPAddress myIP = WiFi.softAPIP(); //save myIP as local IP address
  server.begin(); //begin the WiFi server
  Serial.println("WiFi connected as "); Serial.print(myIP); //print connecting information
  packetBuffer[UDP_PACKET_SIZE] = 0; //set packetBuffer as 0 first

  pinMode(Pot1, INPUT); //set up the switch Pin on
  pinMode(Pot2, INPUT); //set up the off board LED Pin on
  pinMode(Switch, INPUT); //set up the off board LED Pin on
  pinMode(WeaponPin1, INPUT); //set up the switch Pin on
  pinMode(WeaponPin2, INPUT); //set up the off board LED Pin on
  pinMode(Switch2, INPUT); //set up the off board LED Pin on

}


void fncUdpSend(int numb1, int numb2, int numb3, int numb4, int numb5, int numb6) // sending subroutine
{

  udpBuffer[0] = numb1 ; //first index of udpBuffer as the first number accept, which is the ADC number of speed of right wheel
  udpBuffer[1] = numb2 ; //second index of udpBuffer as the second number accept, which is the ADC number of speed of left wheel
  udpBuffer[2] = numb3 ; //second index of udpBuffer as the third number accept, which is the switch status
  udpBuffer[3] = numb4 ; //first index of udpBuffer as the fourth number accept, which is the ADC number of speed of right wheel
  udpBuffer[4] = numb5 ; //second index of udpBuffer as the fifth number accept, which is the ADC number of speed of left wheel
  udpBuffer[5] = numb6 ; //second index of udpBuffer as the sixth number accept, which is the switch status


  UDPTestServer.beginPacket(ipTarget, udpTargetPort); //set up the packet to the target port
  UDPTestServer.printf("%s\n", udpBuffer); //put the infomation into the packet
  UDPTestServer.endPacket(); //end the packet

  delay(100); //delay for 100 ms


}


void loop() {
  int xJoystick = map(analogRead(Pot1),0,4095,1,255); //map the ADC number of joystick in x
  int yJoystick = map(analogRead(Pot2),0,4095,1,255); //map the ADC number of joystick in y
  int back = 1; //set the car switch as open status
  int weaponAngle1 = map(analogRead(WeaponPin1),0,4095,0, 200); //map the ADC number for weapon 1
  int weaponAngle2 = map(analogRead(WeaponPin2),0,4095,0, 200); //map the ADC number for weapon 2
  int weaponOn = 1; //set the weapon switch as open status
  if(digitalRead(Switch)){ 
    back = 2; //if the car swithc is close, set the number as 2
  }
   if(digitalRead(Switch2)){ 
    weaponOn = 2; //if weapon the swithc is close, set the number as 2
  }
  Serial.println(back);  
  Serial.println(xJoystick);
  Serial.println(yJoystick);
  Serial.println(weaponOn); 
  Serial.println(weaponAngle1);
  Serial.println(weaponAngle2);
  Serial.println("");

  
  fncUdpSend(xJoystick, yJoystick, back, weaponAngle1, weaponAngle2, weaponOn); //send the speeds of right and left wheel and the on/off status to the car
}
