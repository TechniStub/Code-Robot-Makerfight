#include <SPI.h>
#include "RF24.h"
#include "printf.h"

/** Brochage de la radio nRF24L01+ sur le Joystick Shield
 *  Signal  nRF pin     Arduino pin
 *  GND     1           GND
 *  VCC     2           3V3
 *  CE      3           9
 *  CSn     4           10
 *  SCK     5           13
 *  MOSI    6           11
 *  MISO    7           12
 *  IRQ     8           -
 */
const byte nRF_CE = 9;
const byte nRF_CSn = 10;
RF24 nRF(nRF_CE, nRF_CSn);

/* Configuration des adresses pour la communication radio
 *  Ces adresses doivent être définies identiques entre la
 *  manette et le robot pour que la communication fonctionne...
 */
/* Adresse de la radio montée sur le robot */
const byte nRF_robot_address[6] = "VT0RX";
/* Adresse de la radio montée sur la télécommande */
const byte nRF_joystick_address[6] = "VT0TX";

/* Valeur des axes et boutons de la manette
 *  Voir le code du robot pour le détail des champs...
 */
struct joystick_state {
  byte  buttons;
  int   axis_x;
  int   axis_y;
} joystate;

const byte button_mask_up = 0x01;
const byte button_mask_right = 0x02;
const byte button_mask_down = 0x04;
const byte button_mask_left = 0x08;
const byte button_mask_start = 0x10;
const byte button_mask_select = 0x20;
const byte button_mask_joystick = 0x40;

const int low_battery_led = A2;

void read_joystick()
{
  /* Lecture des boutons du joystick
   *  Les boutons sont attribués de la façon suivante :
   *  Bouton    Broche Arduino  Broche Atmega
   *  ---------------------------------------
   *  UP        2               PD2
   *  RIGHT     3               PD3
   *  DOWN      4               PD4
   *  LEFT      5               PD5
   *  START     6               PD6
   *  SELECT    7               PD7
   *  JOYSTICK  8               PB0
   *  
   *  Comme les 6 premiers boutons sont situés sur le même port,
   *  on lit la valeur de celui-ci et on décale de deux bits
   *  vers la droite, comme ça le bouton UP se retrouve sur le 
   *  bit 0, le bouton RIGHT sur le bit 1, et ainsi de suite.
   *  Les boutons sont actifs à l'état bas (reliés à GND), ainsi
   *  on inverse la valeur du port pour convertir en logique
   *  positive. Ainsi, le bit correspondant à un bouton est mis à
   *  1 dans <buttons> si le bouton est pressé
   */
  joystate.buttons = (~PIND >> 2) & 0x7F;
  /* Le bouton situé sous le joystick est relié à la broche 0
   *  du port B. On le traite donc séparément des précédents.
   *  Si la broche physique est à 1, on met le bit correspondant
   *  à 0 dans <buttons>, car le bouton n'est pas pressé.
   */
  if (PINB & 0x01)
    joystate.buttons &= ~button_mask_joystick;
  /* Lecture de la position des sticks analogiques */
  /* Axe horizontal du joystick */
  joystate.axis_x = analogRead(A0);
  /* Axe vertical du joystick */
  joystate.axis_y = analogRead(A1);
}

void setup() {
  Serial.begin(115200);
  Serial.println("nRF24L01+ Joystick");
  printf_begin();
  
  /* Initialisation des ports du joystick
   *  Ports en entrées, activer les pull-ups
   */
  DDRD &= 3;
  PORTD |= 0xFC;
  DDRB &= 0xFE;
  PORTB |= 0x01;

  digitalWrite(low_battery_led, LOW);
  pinMode(low_battery_led, OUTPUT);
  
  /* Initialisation de la radio nRF24L01 */
  nRF.begin();
  nRF.enableAckPayload();
  nRF.openWritingPipe(nRF_robot_address);
  nRF.openReadingPipe(1, nRF_joystick_address);
  nRF.printDetails();
}

void loop() {
  /* Lecture de l'état du joystick */
  read_joystick();

  /* Ecrit l'état du joystick dans la console, pour info */
  Serial.print("X = ");
  Serial.print(joystate.axis_x);
  Serial.print("\tY = ");
  Serial.print(joystate.axis_y);
  Serial.print("\tButtons = ");
  if (joystate.buttons & button_mask_up) Serial.print("UP ");
  if (joystate.buttons & button_mask_right) Serial.print("RIGHT ");
  if (joystate.buttons & button_mask_down) Serial.print("DOWN ");
  if (joystate.buttons & button_mask_left) Serial.print("LEFT ");
  if (joystate.buttons & button_mask_start) Serial.print("START ");
  if (joystate.buttons & button_mask_select) Serial.print("SELECT ");
  if (joystate.buttons & button_mask_joystick) Serial.print("JOYSTICK ");
  
  /* Envoie l'état du joystick au robot
   *  Si le robot ne répond pas dans le délai imparti,
   *  affiche un message d'erreur dans la console
   */
  if (!nRF.write(&joystate, sizeof(struct joystick_state))) {
    Serial.println(F("(Not Ack'd from bot)"));
  }
  else
  {
    if (nRF.isAckPayloadAvailable())
    {
      short batt_raw;
      float batt_volt;
      nRF.read(&batt_raw, sizeof(batt_raw));
      batt_volt = batt_raw * 16.0 * 1.0806 / 1024;
      digitalWrite(low_battery_led, (batt_volt < 12.8));
      Serial.print("(");
      Serial.print(batt_volt);
     /* Serial.print(".");
      Serial.print((batt_volt*100 - (int)(batt_volt)*100));*/
      Serial.println("V)");
    }
    else
      Serial.println("(OK)");
  }

  /* Attente de 20ms avant le prochain envoi */
  delay(20);
}
