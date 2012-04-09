/*********************
Arduino gierobot

**********************/

// genutzte Libary's
#include <Wire.h>
#include <Adafruit_MCP23017.h>
#include <Adafruit_RGBLCDShield.h>
#include <math.h>  // Berechnung des Aufnahmewinkels
#include <Servo.h> // F¸r die Servo Steuerung
#include <NewSoftSerial.h>
#include <inttypes.h>
#include <avr/pgmspace.h>
#include <Max3421e.h>
#include <Max3421e_constants.h>
#include <Max_LCD.h>
#include <Usb.h>
#include <ptp.h>
#include <canoneos.h>
#include <MemoryFree.h>

//Objekte
Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();
NewSoftSerial servo_controller(2,3);

// Defines
#define RED 0x1
#define YELLOW 0x3
#define GREEN 0x2
#define TEAL 0x6
#define BLUE 0x4
#define VIOLET 0x5
#define WHITE 0x7
//#define SERVOPIN_gier  9
//#define SERVOPIN_pitch 8
#define SERVO_RESET_PIN 4
#define VMIN 10
#define VMAX 180
#define HMIN 0
#define HMAX 360

#define DEV_ADDR        1

// Canon EOS 400D
#define DATA_IN_EP      1
#define DATA_OUT_EP     2
#define INTERRUPT_EP    3
#define CONFIG_NUM      1

int menu_pos = 0;
int menu_max = 4;
int gier_winkel = 360;
int pitch_winkel = 0;
int gier_min = 0;
int gier_max = 360;
int pitch_min = 90;
int pitch_max = 90;
int anzahl_bilder_soll = 0;
int anzahl_bilder_ist = 0;
int brennweite = 17;
float sensor[2] ;    		// breite des Sensors in MM 
float bildwinkel[2]; 		//Sichtfeld des Objektivs
int schrittweite;   // tbd
int verschlusszeit=10;

boolean debug_on = true;

char* giermenu [] ={"Pano starten",
		   "HDR starten",
		   "Brennweite einstellen",
		   "Horz. Winkel einstellen",
		   "Vert. Winkel einstellen"};


// Eventbasierte Camerasteuerung
class CamStateHandlers : public PTPStateHandlers
{
      bool stateConnected;
    
public:
      CamStateHandlers() : stateConnected(false) {};
      
      virtual void OnDeviceDisconnectedState(PTP *ptp);
      virtual void OnDeviceInitializedState(PTP *ptp);
} CamStates;

CanonEOS  Eos(DEV_ADDR, DATA_IN_EP, DATA_OUT_EP, INTERRUPT_EP, CONFIG_NUM, &CamStates);

void CamStateHandlers::OnDeviceDisconnectedState(PTP *ptp)
{
    if (stateConnected)
    {
        stateConnected = false;
        Notify(PSTR("Camera disconnected\r\n"));
    }
}

void CamStateHandlers::OnDeviceInitializedState(PTP *ptp)
{
    if (!stateConnected)
        stateConnected = true;

    //uint16_t rc = Eos.Capture();
    
    //if (rc != PTP_RC_OK)
    //    Message(PSTR("Error: "), rc);
    
    delay(5000);
}

// Hilffunktionen für Berechnung
void berechne_bildzahl()
{
  int bilder_pitch;
  float temp = ceil(360 / bildwinkel[0]);
  float temp_zeile=ceil(180 / bildwinkel[0]);
  bilder_pitch =  (int)ceil(temp*temp_zeile);
  anzahl_bilder_soll = bilder_pitch;
}

void setze_sensor(float p_breite, int p_hoehe)
{
  	sensor[0] = p_breite;
	sensor[1] = p_hoehe;
	berechne_bildwinkel();
}

void berechne_bildwinkel()
{
   if (debug_on) Serial.println("Bildwinkel berechnen.");
   
     bildwinkel[0]= 2*atan(sensor[0]/(2*brennweite))*180/M_PI;
     bildwinkel[1]= 2*atan(sensor[1]/(2*brennweite))*180/M_PI;
     Serial.print( "Bildwinkel = "); Serial.println( bildwinkel[0]);
   
}


// Hilfsfunktionen für die Bewegung
void foto()
{
  delay(10); //Warten bis die Kammera still steht
  
  if (debug_on) Serial.println("Foto!"); 
  //uint16_t rc = Eos.Capture(); // Foto machen
  
  delay(verschlusszeit+500); // Warten bis alles fertig
  anzahl_bilder_ist=anzahl_bilder_ist++;
  
}



void gier(int pstart, int pende)
{
  int serv_pos=pstart; 
  //gierservo.write(serv_pos);
  int temp;
  
  put(0,map(serv_pos, 0,360 , 500 , 5000));
  foto();
  zeige_fortschritt();
  while (serv_pos<pende)
   {    
     serv_pos=serv_pos+bildwinkel[0];
     if (serv_pos>pende) serv_pos=pende;
     if (debug_on) Serial.println(serv_pos);
    // gierservo.write(serv_pos);
    put(0,map(serv_pos,0,360,500,5000));
     foto();
     zeige_fortschritt();
   }  
}



void pitch(int tstart, int tende, int pstart, int pende)
{
  anzahl_bilder_ist=0;
  berechne_bildzahl();
  Serial.println(bildwinkel[0]);
  lcd.setBacklight(RED);
  int serv_pos=tstart; 
  //pitchservo.write(serv_pos);
  put(1,map(serv_pos,0,360,500,5000));
  gier(pstart, pende);
  while (serv_pos<tende)
   {
     serv_pos=serv_pos+bildwinkel[1];
     if (serv_pos>tende) serv_pos=tende;
     if (debug_on) Serial.println(serv_pos);
     //pitchservo.write(serv_pos);
     put(1,map(serv_pos,0,360,500,5000));
     gier(pstart, pende);
   } 
   lcd.setBacklight(GREEN); 
   zeichne_menu();
   servoOff(0);
   servoOff(1);
}

// Funktionen für den Servo

        void put(unsigned char servo, unsigned int angle){
       //servo is the servo number (typically 0-7)
       //angle is the absolute position from 500 to 5500

       //Send a Pololu Protocol command
       servo_controller.print(0x80,BYTE); //start byte
       servo_controller.print(0x01,BYTE); //device id
       servo_controller.print(0x04,BYTE); //command number
       servo_controller.print(servo,BYTE); //servo number
       //Convert the angle data into two 7-bit bytes
       servo_controller.print(((angle>>7)&0x3f),BYTE); //data1
       servo_controller.print((angle&0x7f),BYTE); //data2
    }

    void servoOff(unsigned char servo){//turns off a servo
       //(servo will go limp until next position command)
       //servo is the servo number (typically 0-7)

       //Send a Pololu Protocol command
       servo_controller.print(0x80,BYTE);//start byte
       servo_controller.print(0x01,BYTE);//device id
       servo_controller.print(0x00,BYTE);//command number
       servo_controller.print(servo,BYTE);//servo number
       servo_controller.print(0x0f,BYTE);//data1 (turn servo off, keep full range)
    }

    void servoSetSpeed(unsigned char servo, unsigned char speedcmd){
       //servo is the servo number (typically 0-7)
       //speed is servo speed (1=slowest, 127=fastest)
       //set speed to zero to turn off speed limiting
       
       speedcmd=speedcmd&0x7f;//take only lower 7 bits of the speed
       
       //Send a Pololu Protocol command
       servo_controller.print(0x80,BYTE);//start byte
       servo_controller.print(0x01,BYTE);//device id
       servo_controller.print(0x01,BYTE);//command number
       servo_controller.print(servo,BYTE);//servo number
       servo_controller.print(speedcmd,BYTE);//data1
    }

//Funktionen für das LCD Menu

void menu_hoch()
{
  if (menu_pos < menu_max)
    menu_pos = menu_pos++;
  else
    menu_pos = 0;
}

void menu_runter()
{
  if (menu_pos > 0)
    menu_pos =  menu_pos--;
  else
    menu_pos = menu_max;
}

int menu_nachfolger()
{
  if (menu_pos < menu_max)
    	return menu_pos+1;
  else
    return  0;
}

void brennweite_hoch()
{
 
  brennweite = brennweite++;


}

void brennweite_runter()
{
  if (brennweite > 0)
    brennweite =  brennweite--;
  else
    brennweite = 0;
}



void zeichne_menu()
{
   lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(">");
  lcd.setCursor(2,0);
  lcd.print(giermenu[menu_pos]);
  lcd.setCursor(2,1);
  lcd.print(giermenu[menu_nachfolger()]);
}

int einstellen(char* pbezeichner, int* paktuell, int pmin, int pmax)
{
  boolean ende = false;
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(pbezeichner);
  while (!ende)
  {
    
  }
}

void brennweite_einstellen()
{
  lcd.setBacklight(YELLOW);
  boolean ende = false;
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Brennweite: ");
  lcd.print(brennweite);
  lcd.setCursor(1,1);
  lcd.print("aendern zurueck<");
  while (!ende)
  {
      
    uint8_t buttons = lcd.readButtons();
    if (buttons) {
      if (buttons & BUTTON_UP) {
        for(int i=0;i<10;i++) brennweite_hoch();
      }
      if (buttons & BUTTON_DOWN) {
        for(int i=0;i<10;i++) brennweite_runter();
      }
      if (buttons & BUTTON_LEFT) {
        brennweite_runter();
      }
      if (buttons & BUTTON_RIGHT) {
        brennweite_hoch();
      }
      if (buttons & BUTTON_SELECT) {
        //lcd.print("SELECT ");
        ende = true;
      }
      boolean ende = false;
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Brennweite: ");
      lcd.print(brennweite);
      lcd.setCursor(1,1);
      lcd.print("aendern zurueck<");
      
  }
  }
  berechne_bildwinkel();
  lcd.setBacklight(GREEN);
  zeichne_menu();
}

void zeige_fortschritt()
{
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Bild: ");
  lcd.print(anzahl_bilder_ist); 
  lcd.setCursor(0,1);
  lcd.print("von:  ");
  lcd.print(anzahl_bilder_soll); 
}

void setup() {
  Serial.begin(9600);
  
  lcd.begin(16, 2);
  lcd.print("Starte Panorobot");
  lcd.setBacklight(GREEN);
  servo_controller.begin(9600);
  berechne_bildwinkel();
  setze_sensor(22.5, 15);
  // Print a message to the LCD. We track how long it takes since
  // this library has been optimized a bit and we're proud of it :)
  Eos.Setup();
  pinMode(SERVO_RESET_PIN, OUTPUT);
  delay(1000) ;
  digitalWrite(SERVO_RESET_PIN,HIGH);
  delay(1000);
  digitalWrite(SERVO_RESET_PIN,LOW);
  delay(1000);
  zeichne_menu();
}

uint8_t i=0;
void loop() {
  uint8_t buttons = lcd.readButtons();
  if (buttons) {
    if (buttons & BUTTON_UP) {
      menu_runter();
    }
    if (buttons & BUTTON_DOWN) {
      menu_hoch();
    }
   /* if (buttons & BUTTON_LEFT) {
      lcd.print("LEFT ");
      lcd.setBacklight(GREEN);
    }
    if (buttons & BUTTON_RIGHT) {
      lcd.print("RIGHT ");
      lcd.setBacklight(TEAL);
    }*/
    if (buttons & BUTTON_SELECT) {
      //lcd.print("SELECT ");
      switch(menu_pos) {
        case 0: pitch(pitch_min,pitch_max,gier_min,gier_max); break;
        case 1: break;
        case 2: brennweite_einstellen(); break;
        case 3: break;
        
        
      }
    }
    zeichne_menu();
  }
}
