#include <LiquidCrystal.h>
#include <Adafruit_MAX31856.h>
#include <avr/interrupt.h>
#define CUISSON_BISCUIT 0;
#define CUISSON_EMAIL 1;
// Use software SPI: CS, DI, DO, CLK
Adafruit_MAX31856 maxthermo = Adafruit_MAX31856(2, 11, 12, 13);
// use hardware SPI, just pass in the CS pin
//Adafruit_MAX31856 maxthermo = Adafruit_MAX31856(3);
// use hardware SPI, pass in the CS pin and using SPI1
//Adafruit_MAX31856 maxthermo = Adafruit_MAX31856(10, &SPI1);

// states
const byte INITIAL = 0;
const byte CUISSON_EN_COURS = 1;
const byte CUISSON_REFROISDISSEMENT = 2;
const byte CUISSON_TERMINEE = 3;
const byte CHOIX_TEMP = 4;

// menus
const byte MENU_INITIAL_1 = 0;
const byte MENU_INITIAL_2 = 1;
const byte MENU_INTERRUPTION_1 = 2;
const byte MENU_INTERRUPTION_2 = 3;
const byte TEMP_EMAIL = 4;
const byte MENU_REPRENDRE_1 = 5;
const byte MENU_REPRENDRE_2 = 6;
const byte EN_COURS = 7;
const byte STOP = 8;
const byte EN_COURS_EMAIL = 9;
const byte MENU_REPRENDRE_CF_1 = 10;
const byte MENU_REPRENDRE_CF_2 = 11;
const byte TEMP_BISCUIT = 12;
const byte LANCER_CUISSON_1 = 13;
const byte ARRET_CUISSON = 14;
const byte LANCER_CUISSON_EMAIL = 15;
const byte ARRET_CUISSON_EMAIL = 16;

const byte ACTION_MENU = 0;
const byte ACTION_UP = 1;
const byte ACTION_DOWN = 2;
const byte ACTION_SELECT = 3;
const byte NEXT_STATE = 4;

byte Menu[17][5];

const char *MenuItems[][2] = {
  { ">Biscuit", " Email  " },  //0
  { " Biscuit", ">Email  " },
  { ">Continuer      ", " Stopper        " },
  { " Continuer      ", ">Stopper        " },
  { " Temp Email: ", "+-" },
  { ">Reprendre      ", " Stopper        " },  //5
  { " Reprendre      ", ">Stopper        " },
  { " CUISSON       ", " EN COURS        " },
  { " CUISSON       ", " STOPPEE       " },
  { "CUISSON EMAIL  ", "En coursM11     " },
  { ">CUISSON EMAIL  ", "Stopper M12     " },  //10
  { " CUISSON EMAIL", ">Stopper  M12     " },
  { "Temp Biscuit   ", "+-" },
  { "DEPART CUISSON ?", "VALID = SELECT " },
  { "ARRET CUISSON ?", "VALID = SELECT " },  //14
  { "DEPART CUISSON ?", "VALID = SELECT " },
  { "ARRET CUISSON ?", "VALID = SELECT " }  //14
};

byte ETAT_MENU;
byte STATE;


const int L1 = 3;  // commande relais / LED

LiquidCrystal lcd(8, 9, 4, 5, 6, 7);
// Données des mesures
double temperatureMoyenne;
double temperatureActuelle;
typedef struct {
  int t0; // temerature de depart
  int t1; // temperature cible
  unsigned long pente; // pente
  int duree; //
  bool parametrable; 
} segment;

// temps
const unsigned long TSec = 1000;
unsigned long Num_ms, Num_sec, Num_min;
unsigned long Num_heur, Num_jour, Temps_ms;
unsigned long t;

// indicateurs
boolean cuissonTerminee;


// constantes de correction
// basse temperature
double Kp_basse_temperature = 0,014696416;      //2;
float Ki_basse_temperature = 919,662;  // 0.00038
double Kd_basse_temperature = 229,9155;  // 10000
//haute temperature
double Kp_haute_temperature =  0,014696416;      //2;
float Ki_haute_temperature = 919,662;  // 0.00038
double Kd_haute_temperature = 229,9155;  // 10000

double dt = 0.25;      // période échantillonage = 0.25s à rapprocher du timer 
double integration = 0.;
double derivation = 0.;
double erreur_n_1 = 0;
// constantes de système
const int TEMPERATURE_SEUIL_PID = 600; // seuil de température pour changement paramètres PID
const int AMPLITUDE_MAX = 1050;   // température max do four (température de travail)
int nbEchantillons = 120;  // base de codage pour une valeur de consigne

// cuisson faïence
int userDefinedMaxTemp = 1030;
const int DEFAULT_TEMPERATURE_MAX_EMAIL = 950;
const int DEFAULT_TEMPERATURE_MAX_BISCUIT = 910;
const int NB_TYPES = 2;
const int NB_PHASES = 6;
segment courbe[NB_TYPES][NB_PHASES] = {
  {
    // cuisson faience
    // pentes de chauffe en °C / heure
    { 0, 100, 100, 0 , false},    // 0 à 100° à 100°C/heure
    { 100, 100, 0, 1, false },    // 100° pendant 10 minutes
    { 100, 400, 100, 0, false },  // 105 à 400° à 100°C/heure
    { 400, 400, 0, 15, false },   // 400° pendant 15 min
    { 400, 1030, 150, 0, true},   // 400 à 1030° à 150°C/heure
    { 1030, 1030, 0, 30, true},   // 1030° pendant 30 min
  },
  {
    // cuisson email
    // pentes de chauffe en °C / heure
    { 0, 100, 100, 0, false },     // 0 à 100° à 100°C/heure
    { 105, 105, 0, 10, false },    // 95 à 105° pendant 10 minutes
    { 105, 400, 100, 0, false },   // 105 à 400° à 100°C/heure
    { 400, 400, 0, 15, false },    // 400° pendant 15 min
    { 400, 1030, 150, 0, true },  // 400 à 1030° à 150°C/heure
    { 1030, 1030, 0, 30, true },  // 1030° pendant 30 min
  },

};

int phaseEnCours, typeCuisson;
boolean initialisation;
float consigneInitiale = 0;  // consigne initiale
float dureePhase;
float tInit;         // temps à l'ilit de la phase
float tDecalePhase;  //décalage de phase (température non atteinte)
double consigne, erreur, CorrectionP, commande;

// affichage caratère spéciaux
byte degre[8] = {
  0b00100,
  0b01010,
  0b01010,
  0b00100,
  0b00000,
  0b00000,
  0b00000,
  0b00000
};
bool cycleTermine;  // flag de fin d'une période
// fonctions
/**
 *  calcul de l'erreur entre la consigne et la mesure
 */
float calculeErreur(float consigne, float mesure) {
  return consigne - mesure;
}
/**
 * Correction statique PID
 */
float correctionProportionnelle(float erreur, float mesure) {
  if (mesure < TEMPERATURE_SEUIL_PID) {
    return erreur * Kp_basse_temperature;
  } 
    return erreur * Kp_haute_temperature;
}

float correctionIntegral(float erreur, float integration, float mesure) {
   if (mesure < TEMPERATURE_SEUIL_PID) {
     return Ki_basse_temperature * dt * nbEchantillons * erreur + integration;
   }
  return Ki_haute_temperature * dt * nbEchantillons * erreur + integration;
}

float correctionDerive(float erreur_n_1, float erreur, float mesure) {
  if (mesure < TEMPERATURE_SEUIL_PID) {
    return (erreur - erreur_n_1) * Kd_basse_temperature * dt / (nbEchantillons);  
  } else {
    return (erreur - erreur_n_1) * Kd_haute_temperature * dt / (nbEchantillons);  
  }
  
}

/* calcul de la consigne
 *  phase index de la phase
 *  t en seconde 
*/
float calculeConsigne(int phase, float t) {
  float pente = courbe[typeCuisson][phase].pente;
  float temperatureInitiale = courbe[typeCuisson][phase].t0;
  float maxTemp = courbe[typeCuisson][phase].t1;
  bool parametrable = courbe[typeCuisson][phase].parametrable;
  float consigne;

  if (parametrable) {
    maxTemp = userDefinedMaxTemp;
  }
  if (pente == 0) {
    return maxTemp;
  }  
  // phase départ cuisson
  if (phase == 0) {
    consigne = pente * t / 3600 + consigneInitiale;
    //if (consigne > maxTemp) {
    //  consigne = maxTemp;
    //}
    return consigne;
  }
  // phases cuisson
  // si température initiale inférieure à la température actuelle, on part de la température actuelle
  if (temperatureInitiale < temperatureMoyenne) { 
    temperatureInitiale = temperatureMoyenne;
  }
  consigne = pente * t / 3600 + temperatureInitiale;
  //if (consigne > maxTemp) {
  //  consigne = maxTemp;
  //}
  return consigne;
  
}

float calculRatio(float consigne, int amplitude, int nbEchantillon) {
  float ratio = consigne * nbEchantillon / amplitude;
  if (ratio > nbEchantillons) {
    return nbEchantillons;
  } else if (ratio < 0) {
    return 0;
  }
  return ratio;
}
/** 
 *  Calcul du changement de phase 
 *  consigne : température de consigne
 *  phase : index de la phase
 *  t : durée écoulée de la phase
 */
boolean changePhase(float consigne, float mesure, int phase, float t, float &tDecalage) {
  if (courbe[typeCuisson][phase].pente == 0) {
    // si on a pas atteint la temperature
    if (mesure * 1.05 < courbe[typeCuisson][phase].t0) {
      tDecalage = t;
      return false;
    }
    return t - tDecalage > courbe[typeCuisson][phase].duree * 60;
  } else {
    float maxTemp = courbe[typeCuisson][phase].t1;
    if (courbe[typeCuisson][phase].parametrable) {
      maxTemp = userDefinedMaxTemp;
    }
    return maxTemp <= consigne;
  }
}
/* Affichage horloge */
void displayTime(unsigned long heure, unsigned long minutes, unsigned long secondes) {
  lcd.setCursor(0, 1);
  lcd.print("Total: ");
  lcd.setCursor(8, 1);
  lcd.print(Num_heur);
  lcd.print(":");
  if (Num_min < 10) {
    lcd.print("0");
  }
  lcd.print(Num_min);
  lcd.print(":");
  if (Num_sec < 10) {
    lcd.print("0");
  }
  lcd.print(Num_sec);
  lcd.print("s");
  return;
}
void displayPhase(int phase, unsigned long duree) {
  lcd.setCursor(0, 1);
  lcd.print("Ph:");
  lcd.print(phase);
  int Heure = (duree / 3600) % 60;
  int Min = (duree / 60) % 60;
  int Sec = duree % 60;
  lcd.setCursor(8, 1);
  lcd.print(Heure);
  lcd.print(":");
  if (Min < 10) {
    lcd.print("0");
  }
  lcd.print(Min);
  lcd.print(":");
  if (Sec < 10) {
    lcd.print("0");
  }
  lcd.print(Sec);
  lcd.print("s");
  return;
}
/* affichage de la température */
void displayTemperature(float temp, float consigne) {

  lcd.setCursor(0, 0);
  lcd.print(temp);
  lcd.write((byte)0);
  lcd.setCursor(10, 0);
  lcd.print(consigne);
  lcd.write((byte)0);
}
void displayGoal(float temp, float goal) {
  lcd.setCursor(0, 0);
  lcd.print(temp);
  lcd.write((byte)0);
  lcd.setCursor(8, 0);
  lcd.print("->");
  lcd.print((int)goal);
  lcd.write((byte)0);
}
void displayFinCuisson() {
  lcd.setCursor(0, 1);
  lcd.print("Cuisson terminee");
}
/*
 * lecture de la température du thermocouple
 */
float readTemp(Adafruit_MAX31856 &maxTh, float temp) {
  maxTh.triggerOneShot();
  delay(200);
  if (maxTh.conversionComplete()) {
    return maxTh.readThermocoupleTemperature();
  } else {
    uint8_t fault = maxTh.readFault();
    if (fault) {
      if (fault & MAX31856_FAULT_CJRANGE) Serial.println("Cold Junction Range Fault");
      if (fault & MAX31856_FAULT_TCRANGE) Serial.println("Thermocouple Range Fault");
      if (fault & MAX31856_FAULT_CJHIGH) Serial.println("Cold Junction High Fault");
      if (fault & MAX31856_FAULT_CJLOW) Serial.println("Cold Junction Low Fault");
      if (fault & MAX31856_FAULT_TCHIGH) Serial.println("Thermocouple High Fault");
      if (fault & MAX31856_FAULT_TCLOW) Serial.println("Thermocouple Low Fault");
      if (fault & MAX31856_FAULT_OVUV) Serial.println("Over/Under Voltage Fault");
      if (fault & MAX31856_FAULT_OPEN) Serial.println("Thermocouple Open Fault");
    }
    Serial.println(fault);
    Serial.println("Conversion not complete!");
  }
  return temp;
}
/**
 * Calcul de la température moyenne
 * 
*/
double mesures[16]; // tableau des mesures de températures aggrégées

float calculTemperatureMoyenne(float temperature) {
  byte i=0;
  float totalTemp = 0;
  for(i=0; i<15; i++) {
    mesures[15-i] = mesures[14-i];
    totalTemp += mesures[15-i];
  }
  mesures[0] = temperature;
  totalTemp += temperature;
  return totalTemp/16;
}
/**
 * Gestion du menu
 */
int adc_key_in = 0;
#define btnRIGHT 0
#define btnUP 1
#define btnDOWN 2
#define btnLEFT 3
#define btnSELECT 4
#define btnNONE 5

int read_LCD_buttons() {
  adc_key_in = analogRead(0);  // read the value from the sensor
  // my buttons when read are centered at these valies: 0, 144, 329, 504, 741
  // we add approx 50 to those values and check to see if we are close
  if (adc_key_in > 1000) return btnNONE;  // We make this the 1st option for speed reasons since it will be the most likely result
  // For V1.1 us this threshold
  // if (adc_key_in < 50)   return btnRIGHT;
  // if (adc_key_in < 250)  return btnUP;
  // if (adc_key_in < 450)  return btnDOWN;
  // if (adc_key_in < 650)  return btnLEFT;
  // if (adc_key_in < 850)  return btnSELECT;

  // For V1.0 comment the other threshold and use the one below:

  if (adc_key_in < 50) return btnRIGHT;
  if (adc_key_in < 195) return btnUP;
  if (adc_key_in < 380) return btnDOWN;
  if (adc_key_in < 555) return btnLEFT;
  if (adc_key_in < 790) return btnSELECT;

  return btnNONE;  // when all others fail, return this...
}

// create two variables to store the current menu option
// and the previous menu option
int menuOption = 0;
int prevMenuOption = 2;

bool menuEnCours = 0;

void setup() {
  pinMode(11, INPUT);
  pinMode(13, OUTPUT);
  pinMode(L1, OUTPUT);  //L1 est une broche de sortie

  Menu[MENU_INITIAL_1][ACTION_MENU] = MENU_INITIAL_1;
  Menu[MENU_INITIAL_1][ACTION_UP] = MENU_INITIAL_1;
  Menu[MENU_INITIAL_1][ACTION_DOWN] = MENU_INITIAL_2;
  Menu[MENU_INITIAL_1][ACTION_SELECT] = TEMP_BISCUIT;
  Menu[MENU_INITIAL_1][NEXT_STATE] = INITIAL;

  Menu[MENU_INITIAL_2][ACTION_MENU] = MENU_INITIAL_1;
  Menu[MENU_INITIAL_2][ACTION_UP] = MENU_INITIAL_1;
  Menu[MENU_INITIAL_2][ACTION_DOWN] = MENU_INITIAL_2;
  Menu[MENU_INITIAL_2][ACTION_SELECT] = TEMP_EMAIL;
  Menu[MENU_INITIAL_2][NEXT_STATE] = CHOIX_TEMP;

  Menu[MENU_INTERRUPTION_1][ACTION_MENU] = MENU_INTERRUPTION_1;
  Menu[MENU_INTERRUPTION_1][ACTION_UP] = MENU_INTERRUPTION_1;
  Menu[MENU_INTERRUPTION_1][ACTION_DOWN] = MENU_INTERRUPTION_2;
  Menu[MENU_INTERRUPTION_1][ACTION_SELECT] = EN_COURS;
  Menu[MENU_INTERRUPTION_1][NEXT_STATE] = CUISSON_EN_COURS;

  Menu[MENU_INTERRUPTION_2][ACTION_MENU] = MENU_INTERRUPTION_1;
  Menu[MENU_INTERRUPTION_2][ACTION_UP] = MENU_INTERRUPTION_1;
  Menu[MENU_INTERRUPTION_2][ACTION_DOWN] = MENU_INTERRUPTION_2;
  Menu[MENU_INTERRUPTION_2][ACTION_SELECT] = ARRET_CUISSON;
  Menu[MENU_INTERRUPTION_2][NEXT_STATE] = CUISSON_REFROISDISSEMENT;

  Menu[TEMP_EMAIL][ACTION_MENU] = TEMP_EMAIL;
  Menu[TEMP_EMAIL][ACTION_UP] = TEMP_EMAIL;
  Menu[TEMP_EMAIL][ACTION_DOWN] = TEMP_EMAIL;
  Menu[TEMP_EMAIL][ACTION_SELECT] = LANCER_CUISSON_EMAIL;
  Menu[TEMP_EMAIL][NEXT_STATE] = CHOIX_TEMP;  //TODO

  Menu[MENU_REPRENDRE_1][ACTION_MENU] = MENU_REPRENDRE_1;
  Menu[MENU_REPRENDRE_1][ACTION_UP] = MENU_REPRENDRE_1;
  Menu[MENU_REPRENDRE_1][ACTION_DOWN] = MENU_REPRENDRE_2;
  Menu[MENU_REPRENDRE_1][ACTION_SELECT] = EN_COURS;
  Menu[MENU_REPRENDRE_1][NEXT_STATE] = CUISSON_EN_COURS;  //8

  Menu[MENU_REPRENDRE_2][ACTION_MENU] = MENU_REPRENDRE_1;
  Menu[MENU_REPRENDRE_2][ACTION_UP] = MENU_REPRENDRE_1;
  Menu[MENU_REPRENDRE_2][ACTION_DOWN] = MENU_REPRENDRE_2;
  Menu[MENU_REPRENDRE_2][ACTION_SELECT] = STOP;
  Menu[MENU_REPRENDRE_2][NEXT_STATE] = CUISSON_REFROISDISSEMENT;  //8

  Menu[EN_COURS][ACTION_MENU] = MENU_INTERRUPTION_1;
  Menu[EN_COURS][ACTION_UP] = EN_COURS;
  Menu[EN_COURS][ACTION_DOWN] = EN_COURS;
  Menu[EN_COURS][ACTION_SELECT] = EN_COURS;
  Menu[EN_COURS][NEXT_STATE] = CUISSON_EN_COURS;  //8

  Menu[STOP][ACTION_MENU] = MENU_REPRENDRE_1;
  Menu[STOP][ACTION_UP] = STOP;
  Menu[STOP][ACTION_DOWN] = STOP;
  Menu[STOP][ACTION_SELECT] = STOP;
  Menu[STOP][NEXT_STATE] = CUISSON_REFROISDISSEMENT;  //8

  Menu[EN_COURS_EMAIL][ACTION_MENU] = MENU_REPRENDRE_1;  //TODO
  Menu[EN_COURS_EMAIL][ACTION_UP] = EN_COURS_EMAIL;
  Menu[EN_COURS_EMAIL][ACTION_DOWN] = EN_COURS_EMAIL;
  Menu[EN_COURS_EMAIL][ACTION_SELECT] = EN_COURS_EMAIL;
  Menu[EN_COURS_EMAIL][NEXT_STATE] = CUISSON_EN_COURS;  //8

  Menu[MENU_REPRENDRE_CF_1][ACTION_MENU] = MENU_REPRENDRE_CF_1;
  Menu[MENU_REPRENDRE_CF_1][ACTION_UP] = MENU_REPRENDRE_CF_1;
  Menu[MENU_REPRENDRE_CF_1][ACTION_DOWN] = MENU_REPRENDRE_CF_2;
  Menu[MENU_REPRENDRE_CF_1][ACTION_SELECT] = EN_COURS;
  Menu[MENU_REPRENDRE_CF_1][NEXT_STATE] = CUISSON_EN_COURS;  //8

  Menu[MENU_REPRENDRE_CF_2][ACTION_MENU] = MENU_REPRENDRE_CF_2;
  Menu[MENU_REPRENDRE_CF_2][ACTION_UP] = MENU_REPRENDRE_CF_1;
  Menu[MENU_REPRENDRE_CF_2][ACTION_DOWN] = MENU_REPRENDRE_CF_2;
  Menu[MENU_REPRENDRE_CF_2][ACTION_SELECT] = STOP;
  Menu[MENU_REPRENDRE_CF_2][NEXT_STATE] = CUISSON_REFROISDISSEMENT;  //8

  Menu[TEMP_BISCUIT][ACTION_MENU] = MENU_INITIAL_1;
  Menu[TEMP_BISCUIT][ACTION_UP] = TEMP_BISCUIT;
  Menu[TEMP_BISCUIT][ACTION_DOWN] = TEMP_BISCUIT;
  Menu[TEMP_BISCUIT][ACTION_SELECT] = LANCER_CUISSON_1;
  Menu[TEMP_BISCUIT][NEXT_STATE] = CHOIX_TEMP;  //8

  Menu[LANCER_CUISSON_1][ACTION_MENU] = MENU_INITIAL_1;
  Menu[LANCER_CUISSON_1][ACTION_UP] = LANCER_CUISSON_1;
  Menu[LANCER_CUISSON_1][ACTION_DOWN] = LANCER_CUISSON_1;
  Menu[LANCER_CUISSON_1][ACTION_SELECT] = EN_COURS;
  Menu[LANCER_CUISSON_1][NEXT_STATE] = CHOIX_TEMP;  //8

  Menu[ARRET_CUISSON][ACTION_MENU] = EN_COURS;
  Menu[ARRET_CUISSON][ACTION_UP] = ARRET_CUISSON;
  Menu[ARRET_CUISSON][ACTION_DOWN] = ARRET_CUISSON;
  Menu[ARRET_CUISSON][ACTION_SELECT] = STOP;
  Menu[ARRET_CUISSON][NEXT_STATE] = CUISSON_EN_COURS;  //8

  Menu[LANCER_CUISSON_EMAIL][ACTION_MENU] = MENU_INITIAL_1;
  Menu[LANCER_CUISSON_EMAIL][ACTION_UP] = LANCER_CUISSON_1;
  Menu[LANCER_CUISSON_EMAIL][ACTION_DOWN] = LANCER_CUISSON_1;
  Menu[LANCER_CUISSON_EMAIL][ACTION_SELECT] = EN_COURS_EMAIL;
  Menu[LANCER_CUISSON_EMAIL][NEXT_STATE] = CHOIX_TEMP;  //8

  Menu[ARRET_CUISSON_EMAIL][ACTION_MENU] = EN_COURS;
  Menu[ARRET_CUISSON_EMAIL][ACTION_UP] = ARRET_CUISSON;
  Menu[ARRET_CUISSON_EMAIL][ACTION_DOWN] = ARRET_CUISSON;
  Menu[ARRET_CUISSON_EMAIL][ACTION_SELECT] = STOP;
  Menu[ARRET_CUISSON_EMAIL][NEXT_STATE] = CUISSON_EN_COURS;  //8


  ETAT_MENU = MENU_INITIAL_1;
  STATE = INITIAL;

  Serial.begin(9600);

  // init du timer et des interruptions
  cli();                    // Désactive l'interruption globale
  bitClear(TCCR2A, WGM20);  // WGM20 = 0
  bitClear(TCCR2A, WGM21);  // WGM21 = 0
  TCCR2B = 0b00000110;      // Clock / 256 soit 16 micro-s et WGM22 = 0
  TIMSK2 = 0b00000001;      // Interruption locale autorisée par TOIE2
  sei();                    // Active l'interruption global


  //init LCD
  lcd.begin(16, 2);
  lcd.createChar(0, degre);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Initialisation");
  lcd.setCursor(0, 1);
  lcd.print("en cours...");

  if (!maxthermo.begin()) {
    Serial.println("Could not initialize thermocouple.");
    while (1) delay(10);
  }
  maxthermo.setThermocoupleType(MAX31856_TCTYPE_K);
  //Serial.print("Thermocouple type: ");
  switch (maxthermo.getThermocoupleType()) {
    //case MAX31856_TCTYPE_B: Serial.println("B Type"); break;
    //case MAX31856_TCTYPE_E: Serial.println("E Type"); break;
    //case MAX31856_TCTYPE_J: Serial.println("J Type"); break;
    case MAX31856_TCTYPE_K: Serial.println("K Type"); break;
    //case MAX31856_TCTYPE_N: Serial.println("N Type"); break;
    //case MAX31856_TCTYPE_R: Serial.println("R Type"); break;
    //case MAX31856_TCTYPE_S: Serial.println("S Type"); break;
    //case MAX31856_TCTYPE_T: Serial.println("T Type"); break;
    //case MAX31856_VMODE_G8: Serial.println("Voltage x8 Gain mode"); break;
    //case MAX31856_VMODE_G32: Serial.println("Voltage x8 Gain mode"); break;
    default: Serial.println("Unknown"); break;
  }

  maxthermo.setConversionMode(MAX31856_ONESHOT_NOWAIT);
  phaseEnCours = 0;
  typeCuisson = CUISSON_BISCUIT;  // faience
  cuissonTerminee = false;
  initialisation = true;
  // temps initial
  tInit = millis() / TSec;
  tDecalePhase = 0;
  // test LED et relais
  digitalWrite(L1, HIGH);  //allumer L1
  delay(250);
  digitalWrite(L1, LOW);  // éteindre L1
  cycleTermine = true;
  Serial.println("erreur;correctionP;Integration;Derivation;commande;mesure;ratio;consigne");
}

byte varCompteurTimer = 0;  // La variable compteur
byte compteurEchantillon = 0;
float ratio;
bool stopCuisson = false;
// Routine d'interruption
ISR(TIMER2_OVF_vect) {
  //15625*16µs
  // on fait partir le timer de 6 pour qu'il compte 250 avant déborder
  // il déborde ainsi toutes les 4 ms
  //TCNT2 = 256 - 250;               // Timer CoNTrole2 250 x 16 µS = 4 ms
  TCNT2 = 256 - 125;                  //125*16µs 2ms
  if (varCompteurTimer++ > 125) {  // 62 * 4 ms = 248 ms //125* 4*s = 500ms
    varCompteurTimer = 0;

    if (compteurEchantillon++ >= nbEchantillons - 1) {
      compteurEchantillon = 0;
      cycleTermine = true;
      if (STATE != CUISSON_REFROISDISSEMENT && STATE != CUISSON_TERMINEE) {
        if (STATE == EN_COURS_EMAIL) {
          consigne = calculeConsigne(phaseEnCours, dureePhase);
        } else {
          consigne = calculeConsigne(phaseEnCours, dureePhase);
        }
        // calcul de la consigne

        // calcul de la commande
        erreur = calculeErreur(consigne, temperatureMoyenne);
        derivation = correctionDerive(erreur_n_1, erreur, temperatureMoyenne);
        erreur_n_1 = erreur;
        CorrectionP = correctionProportionnelle(erreur, temperatureMoyenne);
        integration = correctionIntegral(erreur, integration, temperatureMoyenne);
        commande = integration + CorrectionP + derivation;
        ratio = round(calculRatio(commande, AMPLITUDE_MAX, nbEchantillons));
      } else {
        ratio = 0;
      }
    }
    switch (STATE) {
      case CUISSON_EN_COURS:
        if (compteurEchantillon < ratio) {
          digitalWrite(L1, HIGH);
        } else {
          digitalWrite(L1, LOW);  
        }
        break;
      default:
        digitalWrite(L1, LOW); 
        break; 
    }
  }
}

/*****************************************
 * 
 */
int previousBtn = 0;

void checkMenu() {
  // read the values of the push buttons

  int button1State = read_LCD_buttons();
  bool menuChanged = false;
  // check if the first button is pressed  
  switch (button1State)  // depending on which button was pushed, we perform an action
  {
    case btnRIGHT:
      menuOption = ACTION_MENU;
      break;
    case btnLEFT:  // touche menu
      menuOption = ACTION_MENU;
      break;
    case btnUP:
      menuOption = ACTION_UP;
      break;
    case btnDOWN:
      menuOption = ACTION_DOWN;
      break;
    case btnSELECT:
      {
        menuOption = ACTION_SELECT;
        break;
      }
    case btnNONE:
      {
        break;
      }
  }
  if (previousBtn != button1State) {
    menuChanged = true;
    previousBtn = button1State;
  }
  //  if (Serial.available() > 0) {
  //    // read the character from the input console
  //    char input = Serial.read();
  //    // check the value of the character and update the menu option
  //    if (input == 'm') {
  //      menuOption = 0;
  //    } else if (input == '1' && prevMenuOption == 0) {
  //      menuOption = 1;
  //    } else if (input == '2' && prevMenuOption == 0) {
  //      menuOption = 2;
  //    }
  //  }
  // check if the menu option has changed
  if (menuChanged && previousBtn == btnNONE) {
    menuChanged = false;
    menuEnCours = 1;
    // update the LCD to show the selected option
    lcd.clear();
    switch (menuOption) {
      case ACTION_SELECT:
        ETAT_MENU = Menu[ETAT_MENU][ACTION_SELECT];
        if (ETAT_MENU == TEMP_EMAIL) {
          typeCuisson = CUISSON_EMAIL;
          userDefinedMaxTemp = DEFAULT_TEMPERATURE_MAX_EMAIL;
        } else if (ETAT_MENU == TEMP_BISCUIT) {
          typeCuisson = CUISSON_BISCUIT;
          userDefinedMaxTemp = DEFAULT_TEMPERATURE_MAX_BISCUIT;
        }
        STATE = Menu[ETAT_MENU][NEXT_STATE];
        switch (STATE) {
          case CUISSON_EN_COURS:
            stopCuisson = false;
            cuissonTerminee = false;
            tInit = millis() / TSec;
            break;
          case CUISSON_REFROISDISSEMENT:
          case CUISSON_TERMINEE:
            stopCuisson = true;
            cuissonTerminee = true;
            tInit = millis() / TSec;
            break;
        }

        break;
      case ACTION_DOWN:
        ETAT_MENU = Menu[ETAT_MENU][ACTION_DOWN];
        switch (ETAT_MENU) {
          case TEMP_BISCUIT:
          case TEMP_EMAIL:
            if (userDefinedMaxTemp > 700) userDefinedMaxTemp -= 10;
        }
        break;
      case ACTION_UP:
        ETAT_MENU = Menu[ETAT_MENU][ACTION_UP];
        switch (ETAT_MENU) {
          case TEMP_BISCUIT:
          case TEMP_EMAIL:
            if (userDefinedMaxTemp < 1100) userDefinedMaxTemp += 10;
        }
        break;
      case ACTION_MENU:
        ETAT_MENU = Menu[ETAT_MENU][ACTION_MENU];
        break;
    }
  }

  prevMenuOption = menuOption;
}

/*****************************************
 * Boucle principale
 */
void loop() {
  checkMenu();
  temperatureActuelle = readTemp(maxthermo, temperatureActuelle);
  temperatureMoyenne= calculTemperatureMoyenne(temperatureActuelle);
  if (initialisation == true) {
    byte i;
    for (i=0; i<16; i++) {
      mesures[i] = temperatureActuelle;
    }
    temperatureMoyenne= calculTemperatureMoyenne(temperatureActuelle);
    consigneInitiale = temperatureMoyenne + 10; // départ immédiat
    initialisation = false;
  }

  switch (STATE) {
    case INITIAL:
      menuEnCours = 1; 
      
      lcd.setCursor(10, 0);   
      lcd.print(temperatureActuelle);
      lcd.setCursor(15, 1);   
      lcd.print("-"); break;
    case CUISSON_EN_COURS:
      menuEnCours = 0;
      //lcd.print("*"); break;
    case CUISSON_REFROISDISSEMENT:
      menuEnCours = 0;
      lcd.setCursor(15, 1);
      lcd.print("_"); break;
    case CUISSON_TERMINEE:
      menuEnCours = 0;
      lcd.setCursor(15, 1);
      lcd.print("x"); break;
    case CHOIX_TEMP:
      menuEnCours = 1;
      lcd.setCursor(15, 1);
      lcd.print("C");
      lcd.setCursor(4, 1);
      lcd.print(userDefinedMaxTemp); break;
  }
  if (menuEnCours) {
    lcd.setCursor(0, 0);
    lcd.print(MenuItems[ETAT_MENU][0]);
    lcd.setCursor(0, 1);
    lcd.print(MenuItems[ETAT_MENU][1]);
    
  } else {
    if (STATE == CUISSON_EN_COURS || STATE == STOP || STATE == CUISSON_REFROISDISSEMENT || STATE == CUISSON_TERMINEE) {
      // Lecture de l'horloge interne en ms
      Temps_ms = millis();  // 2^32 secondes = 49.71 jours
      // Calcul des secondes
      t = Temps_ms / TSec;

      dureePhase = t - tInit;
      Num_sec = t % 60;

      // Calcul des minutes
      Num_min = (Temps_ms / (TSec * 60)) % 60;
      // Calcul des heures
      Num_heur = (Temps_ms / (TSec * 3600)) % 60;

      
      if (STATE != CUISSON_REFROISDISSEMENT && STATE != CUISSON_TERMINEE && stopCuisson == false) {
        // calcul de la consigne
        if (STATE == CUISSON_EN_COURS) {
          consigne = calculeConsigne(phaseEnCours, dureePhase);
          //Serial.println(consigne);
        } else {
          consigne = userDefinedMaxTemp;
        }
        // affichage température
        lcd.clear();
        if (Num_sec % 10 > 5) {
          // affichage temps écoulé
          displayTime(Num_heur, Num_min, Num_sec);
          displayTemperature(temperatureMoyenne, consigne);
        } else {
          // affichage temps écoulé
          if (STATE == CUISSON_EN_COURS) {
            displayPhase(phaseEnCours, dureePhase);
            float maxTemp = courbe[typeCuisson][phaseEnCours].t1;
            if (courbe[typeCuisson][phaseEnCours].parametrable) {
              maxTemp = userDefinedMaxTemp;
            }
            displayGoal(temperatureMoyenne, maxTemp);
          } else { //TODO
            displayGoal(temperatureMoyenne, userDefinedMaxTemp);
            displayTime(Num_heur, Num_min, Num_sec);
          }
        }

        if (cycleTermine == true) {
          cycleTermine = false;
          Serial.println();
          Serial.print(erreur);
          Serial.print(";");
          Serial.print(CorrectionP);
          Serial.print(";");
          Serial.print(integration);
          Serial.print(";");
          Serial.print(derivation);
          Serial.print(";");
          Serial.print(commande);
          Serial.print(";");
          Serial.print(temperatureMoyenne);
          Serial.print(";");
          Serial.print(ratio);
          Serial.print(";");
          Serial.print(consigne);
          Serial.print(";");
          Serial.print(dureePhase);
          Serial.print(";");
        }
        if (STATE == CUISSON_EN_COURS) {
          if (changePhase(consigne, temperatureMoyenne, phaseEnCours, dureePhase, tDecalePhase)) {
            tInit = t;
            tDecalePhase = 0;
            phaseEnCours++;
            if (phaseEnCours >= NB_PHASES) {
              cuissonTerminee = true;
            }
          }
        }
      } else {
        lcd.clear();
        displayTemperature(temperatureMoyenne, consigne);
        displayFinCuisson();
        ratio = 0;
        if (cycleTermine == true) {
          cycleTermine = false;
          Serial.println();
          Serial.print(";");
          Serial.print(";");
          Serial.print(";");
          Serial.print(";");
          Serial.print(";");
          Serial.print(temperatureMoyenne);
          Serial.print(";");
          Serial.print(ratio);
          Serial.print(";");
          Serial.print(dureePhase);
          Serial.print(";");
        }
      }
    }
  }
}
