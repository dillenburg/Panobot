/* Capture command demo */
#include <inttypes.h>
#include <avr/pgmspace.h>

//#include <Spi.h>
#include <Max3421e.h>
#include <Max3421e_constants.h>
#include <Max_LCD.h>
#include <Usb.h>
#include <math.h>  // Berechnung des Aufnahmewinkels
#include <ptp.h>
#include <canoneos.h>
#include <Servo.h> // F¸r die Servo Steuerung

#include <MemoryFree.h>

#define DEV_ADDR        1

// Canon EOS 400D
#define DATA_IN_EP      1
#define DATA_OUT_EP     2
#define INTERRUPT_EP    3
#define CONFIG_NUM      1

#define SERVOPIN_PAN  9
#define SERVOPIN_TILT 8
#define AUSLOESERPIN  13


// Variablen

int brennweite = 17;  	// Brennweite in MM
float sensor[2] ;    		// breite des Sensors in MM 
float bildwinkel[2]; 		//Sichtfeld des Objektivs
int schrittweite;   // tbd
int verschlusszeit=1000;

boolean debug_on = true;

//Objekte

Servo panservo;  				// create servo object to control a servo 
Servo tiltservo;

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

    uint16_t rc = Eos.Capture();
    
    if (rc != PTP_RC_OK)
        Message(PSTR("Error: "), rc);
    
    delay(5000);
}



void setze_brennweite(int pbrennweite)

{

	brennweite=pbrennweite;

	berechne_bildwinkel();

}



void setze_sensor(float p_breite, int p_hoehe)

{

	sensor[0] = p_breite;

	sensor[1] = p_hoehe;

	berechne_bildwinkel();

}



void berechne_bildwinkel()

{

	 if (debug_on) Serial.println("Bildwinkel berechnen");

   for(int i=0;i++;i<2)

   {

     bildwinkel[i]= 2*atan(sensor[i]/(2*brennweite))*180/M_PI;

     if (debug_on) {Serial.print( "Bildwinkel = "); Serial.println((int) bildwinkel[i]);}

   }

}



void foto()

{

  delay(1000); //Warten bis die Kammera still steht

  digitalWrite(AUSLOESERPIN, HIGH);

  if (debug_on) Serial.println("Foto!"); 

  uint16_t rc = Eos.Capture(); // Foto machen

  delay(verschlusszeit+1000); // Warten bis alles fertig

  digitalWrite(AUSLOESERPIN, LOW);

}



void pan(int pstart, int pende)

{

  int serv_pos=pstart; 

  panservo.write(serv_pos);

  foto();

  while (serv_pos<pende)

   {    

     serv_pos=serv_pos+bildwinkel[0];

     if (serv_pos>pende) serv_pos=pende;

     if (debug_on) Serial.println(serv_pos);

     panservo.write(serv_pos);

     foto();

   }  

}



void tilt(int tstart, int tende, int pstart, int pende)

{

  int serv_pos=tstart; 

  tiltservo.write(serv_pos);

  pan(pstart, pende);

  while (serv_pos<tende)

   {

     serv_pos=serv_pos+bildwinkel[1];

     if (serv_pos>tende) serv_pos=tende;

     if (debug_on) Serial.println(serv_pos);

     tiltservo.write(serv_pos);

     pan(pstart, pende);

   }  

}

void setup() 
{
  Serial.begin( 115200 );
  Serial.println("Start");
  panservo.attach(SERVOPIN_PAN);  // attaches the servo on pin 9 to the servo object 
  panservo.write(0);
  tiltservo.attach(SERVOPIN_TILT);  // attaches the servo on pin 8 to the servo object 
  tiltservo.write(0);
  pinMode(AUSLOESERPIN, OUTPUT);
  digitalWrite(AUSLOESERPIN, LOW);
  setze_sensor(22.5, 15);
  setze_brennweite(200);
  Eos.Setup();
  delay( 200 );
}

void loop() 
{
   tilt(0,0,20,160);  

   while(1==1) //Ende

   {

   

   } 
}

