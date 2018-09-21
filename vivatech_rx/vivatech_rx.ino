#include <SPI.h>
#include "RF24.h"
#include "printf.h"

/* Brochage de la radio nRF24L01+ */
const byte nRF_CE = 9;
const byte nRF_CSn = 10;
RF24 nRF(nRF_CE, nRF_CSn);

/* Adresse de la radio montée sur le robot */
const byte nRF_robot_address[6] = "VT0RX";
/* Adresse de la radio montée sur la télécommande */
const byte nRF_joystick_address[6] = "VT0TX";

/* Valeur des axes et boutons de la manette */
struct joystick_state {
  /* <buttons> stocke l'état des 7 boutons de la manette dans 1 octet.
   *  chaque bit de <buttons> est mis à 1 si le bouton correspondant est pressé
   *  Par exemple, pour vérifier si le bouton du haut est pressé, on utilise :
   *  if (joystate.buttons & button_mask_up)
   *  {
   *    ...
   *  }
   */
  byte  buttons;
  /* <axis_x> est la valeur analogique de l'axe horizontal du joystick
   *  La valeur est comprise entre 0 et 1023
   *  0    : stick à gauche
   *  1023 : stick à gauche
   *  ~512 : stick au centre
   */
  int   axis_x;
  /* <axis_y> est la valeur analogique de l'axe vertical du joystick
   *  La valeur est comprise entre 0 et 1023
   *  0    : stick en bas
   *  1023 : stick en haut
   *  ~512 : stick au centre
   */
  int   axis_y;
} joystate;

/* Masques binaires pour décoder la valeur <buttons> de <joystate> */
const byte button_mask_up = 0x01;         /* Bouton haut ou A */
const byte button_mask_right = 0x02;      /* Bouton droit ou B */
const byte button_mask_down = 0x04;       /* Bouton bas ou C */
const byte button_mask_left = 0x08;       /* Bouton gauche ou D */
const byte button_mask_start = 0x10;      /* Bouton start ou E */
const byte button_mask_select = 0x20;     /* Bouton select ou F */
const byte button_mask_joystick = 0x40;   /* Bouton sous le stick */

/* Classe permettant de piloter simplement un moteur relié via un pont en H */
class motor {
  private:
  byte pin_A, pin_B, pin_PWM;
  public:
  /* Initialise une instance de la classe <moteur>
   *  <pin_A> correspond à la commande du moteur dans un sens
   *  <pin_B> correspond à la commande du moteur dans l'autre sens
   *  <pin_PWM> est le signal de modulation de largeur d'impulsion
   *    et permet de faire varier la vitesse du moteur
   * Pour la connexion à un pont chinois à base de BTS7960, le cablage 
   * est le suivant :
   *  <pin_A> et <pin_B> sont respectivement reliés à L_PWM et R_PWM
   *    (ou R_PWM et L_PWM si on souhaite inverser le sens de rotation)
   *  <pin_PWM> est reliée à L_EN et R_EN mis ensemble
   */
  motor(byte pin_A, byte pin_B, byte pin_PWM){
    this->pin_A = pin_A;
    this->pin_B = pin_B;
    this->pin_PWM = pin_PWM;
  }

  /* Initialise les broches de commande du moteur en tant que sorties
   *  Cette fonction est de préférence appelée dans setup()
   */
  void begin() {
    pinMode(pin_A, OUTPUT);
    pinMode(pin_B, OUTPUT);
    pinMode(pin_PWM, OUTPUT);
    stop();
  }

  /* Arrête le moteur
   *  Les deux branches du pont sont déconnectées, le moteur est en roue libre  
   */
  void stop() {
    digitalWrite(pin_A, 0);
    digitalWrite(pin_B, 0);
    digitalWrite(pin_PWM, 0);
  }


  /* Définit la vitesse de rotation du moteur
   *  <value> est un entier compris entre -255 et +255
   *  Si <value> dépasse ces valeurs, <value> sera borné à -255 ou +255
   */
  void setPower(int value) {
    if (value > 0) {
        digitalWrite(pin_A, HIGH);
        digitalWrite(pin_B, LOW);
        analogWrite(pin_PWM, min(value, 255));
    } else if (value < 0) {
        digitalWrite(pin_A, LOW);
        digitalWrite(pin_B, HIGH);
        analogWrite(pin_PWM, min(-value, 255));
    } else {
        digitalWrite(pin_A, LOW);
        digitalWrite(pin_B, LOW);
        analogWrite(pin_PWM, 0);
    }
  }
};

/* Filtre passe-bas IIR du premier ordre */
class filter {
  public:
  float input;
  float output;
  float coeff;
  long last_filter_time;
  int filter_period;
  
  filter()
  {
    input = 0.0;
    output = 0.0;
    coeff = 1.0;
    last_filter_time = 0;
  }
  filter (float _coeff) : filter()
  {
    coeff = _coeff;
  }
  filter (float _coeff, int _filter_period) : filter(_coeff)
  {
    filter_period = _filter_period;
  }
  bool shouldUpdate()
  {
    return millis() - last_filter_time > filter_period;
  }
  float update()
  {
    output = output * coeff + input * (1-coeff);
    last_filter_time = millis();
    return output;
  }
  float reset()
  {
    output = input;
    return output;
  }
};

/* Moteur gauche
 *  pin_A sur broche 2
 *  pin_B sur broche 4
 *  pin_PWM sur broche 3
 */
motor left_motor(2,4,3);
/* Moteur droit
 *  pin_A sur broche 7
 *  pin_B sur broche 8
 *  pin_PWM sur broche 6
 */
motor right_motor(7,8,6);
/* Moteur arme
 *  pin_A sur broche A0
 *  pin_B sur broche A1
 *  pin_PWM sur broche 5
 */
motor weapon_motor(A0,A1,5);
/* Filtre passe-bas du moteur d'arme */
filter weapon_filter(0.95,10);
/* Vitesse maximale du moteur d'arme */
const int weapon_motor_max_pwr = 128;

const int relay_1_pin = A2;
const int relay_2_pin = A3;
const int batt_volt_pin = A4;

void setup() {
  Serial.begin(115200);
  Serial.println("nRF24L01+ Robot");
  printf_begin();

  /* Initialisation de la radio nRF24L01 */
  nRF.begin();
  nRF.setAutoAck(1);
  nRF.enableAckPayload();
  nRF.openWritingPipe(nRF_joystick_address);
  nRF.openReadingPipe(1, nRF_robot_address);
  nRF.printDetails();
  nRF.startListening();

  /* Réglage de la fréquence PWM du moteur gauche
   *  Le timer2 par défaut fonctionne à 31250 Hz
   *  on le reconfigure à 62500 Hz pour que la fréquence PWM
   *  sur la pin 3 soit identique à celle des pins 5 et 6
   */
  TCCR2B = TCCR2B & 0b11111000 | 0x03; 
  
  /* Initialisation des commandes de moteur */
  left_motor.begin();
  right_motor.begin();
  weapon_motor.begin();

  /* Initialisation des relais */
  digitalWrite(relay_1_pin, LOW);
  digitalWrite(relay_2_pin, LOW);
  pinMode(relay_1_pin, OUTPUT);
  pinMode(relay_2_pin, OUTPUT);

  /* Référence de tension 1.1V pour mesure de tension batterie */
  analogReference(INTERNAL);
}

/* Crée une zone morte sur <inval>
 *  si <inval> est compris entre -<thres> et +<thres>, renvoie 0
 *  sinon, retranche <thres> à <inval> (ajoute si <inval> < 0)
 */
int deadZone(int inval, int thres)
{
    if (inval > thres) {
        return inval - thres;
    } else if (inval < -thres) {
        return inval + thres;
    } else {
        return 0;
    }
}

void loop() {
  /* Date à laquelle on a reçu le dernier message de la télécommande.
   *  Cette valeur est en millisecondes depuis le démarrage de l'Arduino
   *  (obtenu avec la fonction millis())
   */
  static uint32_t last_joystick_time = 0;

  /* Teste si l'on a reçu un message provenant de la télécommande */
  while (nRF.available()) {
    /* Oui, alors on extrait la structure de donnée du message reçu */
    nRF.read(&joystate, sizeof(struct joystick_state));
    /* Lecture de la tension de batterie et envoi par liaison radio */
    short batt_raw = analogRead(batt_volt_pin);
    nRF.writeAckPayload (1, &batt_raw, sizeof(batt_raw)); 
    
    /* Ecrit l'état de la manette dans la console, pour info */
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
    Serial.println("");

    /* Met à jour la date de dernière bonne réception */
    last_joystick_time = millis();

    /* Vitesse d'avance/recul du robot
     *  Définie à partir de l'axe vertical du joystick
     *  On retire 512 à <axis_y> pour avoir 0 au centre
     *  On divise par 2 pour avoir au maximum +/- 256
     */
    int throttle = deadZone(joystate.axis_y - 512, 16) / 2;
    /* Vitesse de rotation du robot
     *  Définie à partir de l'axe horizontal du joystick
     *  On retire 512 à <axis_x> pour avoir 0 au centre
     *  On divise par 3 pour limiter la vitesse de rotation du robot
     */
    int steering = deadZone(joystate.axis_x - 512, 16) / 3;

    /* Définit la vitesse de rotation des moteurs gauche et droit
     *  Le signal <throttle> est une action commune 
     *  Le signal <steering> est une action différentielle
     */
    left_motor.setPower(throttle + steering);
    right_motor.setPower(throttle - steering);

    /* Commandes des relais */
    digitalWrite(relay_1_pin, joystate.buttons & button_mask_left);
    digitalWrite(relay_2_pin, joystate.buttons & button_mask_right);

    /* Commande du moteur d'arme */
    if (joystate.buttons & button_mask_up)
      weapon_filter.input = weapon_motor_max_pwr;
    else if (joystate.buttons & button_mask_down)
      weapon_filter.input = -weapon_motor_max_pwr;
    else
      weapon_filter.input = 0;
  }

  if (weapon_filter.shouldUpdate())
  {
    weapon_filter.update();
    weapon_motor.setPower(weapon_filter.output);
  }

  /* Si l'on n'a pas reçu de message de la télécommande depuis plus de 500 ms */
  if (millis() - last_joystick_time > 500)
  {
    Serial.println("No joystick data !");
    
    /* On arrête les moteurs */
    left_motor.stop();
    right_motor.stop();
    weapon_motor.stop();

    /* On remet le filtre à zero */
    weapon_filter.input = 0;
    weapon_filter.reset();
    
    /* On coupe les relais */
    digitalWrite(relay_1_pin, LOW);
    digitalWrite(relay_2_pin, LOW);
    
    delay(100);
  }
}
