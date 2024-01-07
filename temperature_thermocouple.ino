#include <LiquidCrystal.h>
#include <Adafruit_MAX31856.h>
#include <avr/interrupt.h>
const int CUISSON_BISCUIT = 0;
const int CUISSON_EMAIL = 1;
const int CUISSON_CONSIGNE_FIXE = 2;
// Use software SPI: CS, DI, DO, CLK
Adafruit_MAX31856 maxthermo = Adafruit_MAX31856(2, 11, 12, 13);
// use hardware SPI, just pass in the CS pin
//Adafruit_MAX31856 maxthermo = Adafruit_MAX31856(3);
// use hardware SPI, pass in the CS pin and using SPI1
//Adafruit_MAX31856 maxthermo = Adafruit_MAX31856(10, &SPI1);

// next Menu
const byte NEXT_MENU = 0;
const byte NEXT_STATE = 1;
// states
const byte INITIAL = 0;
const byte CHOIX_TEMP = 1;
const byte CUISSON_EN_COURS = 2;
const byte CUISSON_REFROISDISSEMENT = 3;
const byte CUISSON_TERMINEE = 4;


// menus
const byte MENU_INITIAL_1 = 0;
const byte MENU_INITIAL_2 = 1;
const byte CONSIGNE_FIXE = 2;

const byte TEMP_EMAIL = 0;
const byte TEMP_BISCUIT = 1;
const byte TEMP_CONSIGNE_FIXE = 2;
const byte LANCER_CUISSON_1 = 3;

const byte EN_COURS = 0;
const byte MENU_INTERRUPTION_1 = 1;
const byte MENU_INTERRUPTION_2 = 2;

const byte STOP = 0;
const byte MENU_REPRENDRE_1 = 1;
const byte MENU_REPRENDRE_2 = 2;

const byte ACTION_MENU = 0;
const byte ACTION_UP = 1;
const byte ACTION_DOWN = 2;
const byte ACTION_SELECT = 3;
//const byte ACTION_NONE = 4;

byte Menu[5][4][4][2];

const char *MenuItems[][4][2] = {
  // INITIAL
  // INITIAL
  {
    { ">Cuisson Biscuit", " Cuisson Email  " },
    { " Cuisson Biscuit", ">Cuisson Email  " },
    { " Cuisson Email", ">Consigne fixe  " },
  },
  // CHOIX_TEMP
  {
    { "Temp Email   ", "+-" },
    { "Temp Biscuit   ", "+-" },
    { "T Consigne Fixe", "+-" },
    { "DEPART CUISSON ?", "VALID = SELECT " },
  },
  // CUISSON_EN_COURS
  {
    { "      ", "         " },
    { ">Continuer      ", " Stopper        " },
    { " Continuer      ", ">Stopper        " },
  },
  // CUISSON_REFROIDISSEMENT
  {
    { " CUISSON       ", " STOPPEE        " },
    { ">Reprendre      ", " Stopper        " },
    { " Reprendre      ", ">Stopper        " },
  },
  // CUISSON_TERMINNEE
  {
    { ">Reprendre      ", " Stopper        " },
    { " Reprendre      ", ">Stopper        " },
    { " CUISSON       ", " EN COURS        " },
  }
};

byte ETAT_MENU;
byte STATE;


const int L1 = 3;  // commande relais / LED

LiquidCrystal lcd(8, 9, 4, 5, 6, 7);
// Données des mesures
double temperatureMoyenne;
double temperatureActuelle;
typedef struct {
  int t0;               // temerature de depart
  int t1;               // temperature cible
  unsigned long pente;  // pente
  int duree;            //
  bool parametrable;
} segment;

// temps
const unsigned long TSec = 1000;
unsigned long Num_ms, Num_sec, Num_min;
unsigned long Num_heur, Num_jour, Temps_ms;
unsigned long timeInSecond;

// constantes de correction établies par étude de la réponse indicielle
// retard apparent : t1 = 459.831 secondes
// a = 86.68 °C
//
typedef struct {
  float Kp;
  float Ki;
  float Kd;
} pid;

pid pids[3] = {
  { 25, 250, 15000 },  // hautes temp > 400
  { 25, 250, 15000 },  // intermediate <400
  { 25, 250, 15000 },  // basse temp < 100
};
// // basse temperature
// float Kp_basse_temperature = 8; // 0.014696416
// float Ki_basse_temperature = 919.662;      // 1/Ki is used after
// float Kd_basse_temperature = 229.9155;    // 10000
// //haute temperature
// float Kp_haute_temperature = 10 ; // 0.014696416
// float Ki_haute_temperature = 919.662;       // 1/Ki is used after
// float Kd_haute_temperature = 229.9155;     //

float dt = 0.125;  // période échantillonage = 0.125s à rapprocher du timer
float integration = 0.;
float derivation = 0.;
float erreur_n_1 = 0;
// constantes de système
const int TEMPERATURE_SEUIL_PID = 600;  // seuil de température pour changement paramètres PID
const int AMPLITUDE_MAX = 1100;         // température max do four (température de travail)
int nbEchantillons = 120;               // base de codage pour une valeur de consigne

// cuisson faïence
float userDefinedMaxTemp = 1030;  // température paramétrable
float pente;                      // pente de la courbe de chauffe
float temperatureInitiale;        // T0 de la courbe de chauffe
float temperatureFinale;          // T1 de la courbe de chauffe
int seuil = 2;                    // °C seuil de tolérance température atteinte
const int DEFAULT_TEMPERATURE_MAX_EMAIL = 950;
const int DEFAULT_TEMPERATURE_MAX_BISCUIT = 910;
const int NB_TYPES = 3;
const int NB_PHASES = 6;
segment courbe[NB_TYPES][NB_PHASES] = {
  {
    // cuisson faience
    // pentes de chauffe en °C / heure
    { 0, 100, 100, 0, false },    // 0 à 100° à 100°C/heure
    { 100, 100, 0, 1, false },    // 100° pendant 10 minutes
    { 100, 400, 100, 0, false },  // 105 à 400° à 100°C/heure
    { 400, 400, 0, 15, false },   // 400° pendant 15 min
    { 400, 1030, 150, 0, true },  // 400 à 1030° à 150°C/heure
    { 1030, 1030, 0, 30, true },  // 1030° pendant 30 min
  },
  {
    // cuisson email
    // pentes de chauffe en °C / heure
    { 0, 100, 100, 0, false },    // 0 à 100° à 100°C/heure
    { 100, 100, 0, 10, false },   // 95 à 105° pendant 10 minutes
    { 100, 400, 100, 0, false },  // 105 à 400° à 100°C/heure
    { 400, 400, 0, 15, false },   // 400° pendant 15 min
    { 400, 1030, 150, 0, true },  // 400 à 1030° à 150°C/heure
    { 1030, 1030, 0, 30, true },  // 1030° pendant 30 min
  },
  {
    // consigne fixe
    { 900, 900, 0, 10, true },  // 900 paramétrable
    { 0, 0, 0, 0, false },  // pas de chauffe
    { 0, 0, 0, 0, false },  // pas de chauffe
    { 0, 0, 0, 0, false },  // pas de chauffe
    { 0, 0, 0, 0, false },  // pas de chauffe
    { 0, 0, 0, 0, false },  // pas de chauffe
   
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

/* calcul de la consigne
 *  @param pente  pente de la phase
 *  @param tInit  température initiale
 *  @param tEnd   température finale (cible)
 *  @param time   temps écoulé depuis le début de la phase en seconde
*/
float calculeConsigne(float pente, float tInit, float tEnd, float time) {
  if (pente == 0) {
    return tEnd;
  }
  // phases cuisson
  return pente * time / 3600 + tInit;
}

float calculRatio(float commande) {
  float ratio = commande * nbEchantillons / AMPLITUDE_MAX;
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
 *  time : durée écoulée de la phase
 */
boolean changePhase(float consigne, float mesure, int phase, float time, float &tDecalage) {
  //Serial.println(typeCuisson);
  if (courbe[typeCuisson][phase].pente == 0) {
    // si on a pas atteint la temperature
    if ((mesure + seuil) < courbe[typeCuisson][phase].t0) {
      tDecalage = time;
      return false;
    }
    return time - tDecalage > courbe[typeCuisson][phase].duree * 60;
  } else {
    float maxTemp = courbe[typeCuisson][phase].t1;
    if (courbe[typeCuisson][phase].parametrable) {
      maxTemp = userDefinedMaxTemp;
    }
    return (maxTemp <= consigne) || ((mesure - seuil) > maxTemp);
  }
}

/**
 * paramètres pour la nouvelle phase
*/
void setPhaseParameters(int typeCuisson, int phase) {
  pente = courbe[typeCuisson][phase].pente;
  temperatureInitiale = courbe[typeCuisson][phase].t0;
  if (temperatureInitiale < temperatureMoyenne) {
    // on part de la temp moyenne si > temperature initiale
    temperatureInitiale = temperatureMoyenne;
  }

  temperatureFinale = courbe[typeCuisson][phase].t1;
  if (courbe[typeCuisson][phase].parametrable /*&& temperatureFinale > userDefinedMaxTemp*/) {
    temperatureFinale = userDefinedMaxTemp;  // cible si temp paramétrable
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
      if (fault & MAX31856_FAULT_CJRANGE) Serial.println(F("Cold Junction Range Fault"));
      if (fault & MAX31856_FAULT_TCRANGE) Serial.println(F("Thermocouple Range Fault"));
      if (fault & MAX31856_FAULT_CJHIGH) Serial.println(F("Cold Junction High Fault"));
      if (fault & MAX31856_FAULT_CJLOW) Serial.println(F("Cold Junction Low Fault"));
      if (fault & MAX31856_FAULT_TCHIGH) Serial.println(F("Thermocouple High Fault"));
      if (fault & MAX31856_FAULT_TCLOW) Serial.println(F("Thermocouple Low Fault"));
      if (fault & MAX31856_FAULT_OVUV) Serial.println(F("Over/Under Voltage Fault"));
      if (fault & MAX31856_FAULT_OPEN) Serial.println(F("Thermocouple Open Fault"));
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
double mesures[16];  // tableau des mesures de températures aggrégées

float calculTemperatureMoyenne(float temperature) {
  byte i = 0;
  float totalTemp = 0;
  for (i = 0; i < 15; i++) {
    mesures[15 - i] = mesures[14 - i];
    totalTemp += mesures[15 - i];
  }
  mesures[0] = temperature;
  totalTemp += temperature;
  return totalTemp / 16;
}

double derivees[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };  // tableau des mesures de températures aggrégées
/**
 * lissage de la dérivée
 * N = 10 échantillons
*/
float filtreDerivee(float derivee) {
  byte i = 0;
  float totalTemp = 0;
  for (i = 0; i < 5; i++) {
    derivees[5 - i] = derivees[4 - i];
    totalTemp += derivees[5 - i];
  }
  derivees[0] = derivee;
  totalTemp += derivee;
  return totalTemp / 5;
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
byte menuOption = 0;
byte prevMenuOption = 2;

bool menuEnCours = 0;

void setupMenu() {
  Menu[INITIAL][MENU_INITIAL_1][ACTION_MENU][NEXT_MENU] = MENU_INITIAL_1;
  Menu[INITIAL][MENU_INITIAL_1][ACTION_MENU][NEXT_STATE] = INITIAL;
  Menu[INITIAL][MENU_INITIAL_1][ACTION_UP][NEXT_MENU] = MENU_INITIAL_1;
  Menu[INITIAL][MENU_INITIAL_1][ACTION_UP][NEXT_STATE] = INITIAL;
  Menu[INITIAL][MENU_INITIAL_1][ACTION_DOWN][NEXT_MENU] = MENU_INITIAL_2;
  Menu[INITIAL][MENU_INITIAL_1][ACTION_DOWN][NEXT_STATE] = INITIAL;
  Menu[INITIAL][MENU_INITIAL_1][ACTION_SELECT][NEXT_MENU] = TEMP_BISCUIT;
  Menu[INITIAL][MENU_INITIAL_1][ACTION_SELECT][NEXT_STATE] = CHOIX_TEMP;

  Menu[INITIAL][MENU_INITIAL_2][ACTION_MENU][NEXT_MENU] = MENU_INITIAL_1;
  Menu[INITIAL][MENU_INITIAL_2][ACTION_MENU][NEXT_STATE] = INITIAL;
  Menu[INITIAL][MENU_INITIAL_2][ACTION_UP][NEXT_MENU] = MENU_INITIAL_1;
  Menu[INITIAL][MENU_INITIAL_2][ACTION_UP][NEXT_STATE] = INITIAL;
  Menu[INITIAL][MENU_INITIAL_2][ACTION_DOWN][NEXT_MENU] = CONSIGNE_FIXE;
  Menu[INITIAL][MENU_INITIAL_2][ACTION_DOWN][NEXT_STATE] = INITIAL;
  Menu[INITIAL][MENU_INITIAL_2][ACTION_SELECT][NEXT_MENU] = TEMP_EMAIL;
  Menu[INITIAL][MENU_INITIAL_2][ACTION_SELECT][NEXT_STATE] = CHOIX_TEMP;

  Menu[INITIAL][CONSIGNE_FIXE][ACTION_MENU][NEXT_MENU] = MENU_INITIAL_1;
  Menu[INITIAL][CONSIGNE_FIXE][ACTION_MENU][NEXT_STATE] = INITIAL;
  Menu[INITIAL][CONSIGNE_FIXE][ACTION_UP][NEXT_MENU] = MENU_INITIAL_2;
  Menu[INITIAL][CONSIGNE_FIXE][ACTION_UP][NEXT_STATE] = INITIAL;
  Menu[INITIAL][CONSIGNE_FIXE][ACTION_DOWN][NEXT_MENU] = CONSIGNE_FIXE;
  Menu[INITIAL][CONSIGNE_FIXE][ACTION_DOWN][NEXT_STATE] = INITIAL;
  Menu[INITIAL][CONSIGNE_FIXE][ACTION_SELECT][NEXT_MENU] = TEMP_CONSIGNE_FIXE;
  Menu[INITIAL][CONSIGNE_FIXE][ACTION_SELECT][NEXT_STATE] = CHOIX_TEMP;

  Menu[CHOIX_TEMP][TEMP_EMAIL][ACTION_MENU][NEXT_MENU] = MENU_INITIAL_1;
  Menu[CHOIX_TEMP][TEMP_EMAIL][ACTION_MENU][NEXT_STATE] = INITIAL;
  Menu[CHOIX_TEMP][TEMP_EMAIL][ACTION_UP][NEXT_MENU] = TEMP_EMAIL;
  Menu[CHOIX_TEMP][TEMP_EMAIL][ACTION_UP][NEXT_STATE] = CHOIX_TEMP;
  Menu[CHOIX_TEMP][TEMP_EMAIL][ACTION_DOWN][NEXT_MENU] = TEMP_EMAIL;
  Menu[CHOIX_TEMP][TEMP_EMAIL][ACTION_DOWN][NEXT_STATE] = CHOIX_TEMP;
  Menu[CHOIX_TEMP][TEMP_EMAIL][ACTION_SELECT][NEXT_MENU] = LANCER_CUISSON_1;
  Menu[CHOIX_TEMP][TEMP_EMAIL][ACTION_SELECT][NEXT_STATE] = CHOIX_TEMP;

  Menu[CHOIX_TEMP][TEMP_BISCUIT][ACTION_MENU][NEXT_MENU] = MENU_INITIAL_1;
  Menu[CHOIX_TEMP][TEMP_BISCUIT][ACTION_MENU][NEXT_STATE] = INITIAL;
  Menu[CHOIX_TEMP][TEMP_BISCUIT][ACTION_UP][NEXT_MENU] = TEMP_BISCUIT;
  Menu[CHOIX_TEMP][TEMP_BISCUIT][ACTION_UP][NEXT_STATE] = CHOIX_TEMP;
  Menu[CHOIX_TEMP][TEMP_BISCUIT][ACTION_DOWN][NEXT_MENU] = TEMP_BISCUIT;
  Menu[CHOIX_TEMP][TEMP_BISCUIT][ACTION_DOWN][NEXT_STATE] = CHOIX_TEMP;
  Menu[CHOIX_TEMP][TEMP_BISCUIT][ACTION_SELECT][NEXT_MENU] = LANCER_CUISSON_1;
  Menu[CHOIX_TEMP][TEMP_BISCUIT][ACTION_SELECT][NEXT_STATE] = CHOIX_TEMP;

  Menu[CHOIX_TEMP][TEMP_CONSIGNE_FIXE][ACTION_MENU][NEXT_MENU] = MENU_INITIAL_1;
  Menu[CHOIX_TEMP][TEMP_CONSIGNE_FIXE][ACTION_MENU][NEXT_STATE] = INITIAL;
  Menu[CHOIX_TEMP][TEMP_CONSIGNE_FIXE][ACTION_UP][NEXT_MENU] = TEMP_CONSIGNE_FIXE;
  Menu[CHOIX_TEMP][TEMP_CONSIGNE_FIXE][ACTION_UP][NEXT_STATE] = CHOIX_TEMP;
  Menu[CHOIX_TEMP][TEMP_CONSIGNE_FIXE][ACTION_DOWN][NEXT_MENU] = TEMP_CONSIGNE_FIXE;
  Menu[CHOIX_TEMP][TEMP_CONSIGNE_FIXE][ACTION_DOWN][NEXT_STATE] = CHOIX_TEMP;
  Menu[CHOIX_TEMP][TEMP_CONSIGNE_FIXE][ACTION_SELECT][NEXT_MENU] = LANCER_CUISSON_1;
  Menu[CHOIX_TEMP][TEMP_CONSIGNE_FIXE][ACTION_SELECT][NEXT_STATE] = CHOIX_TEMP;

  Menu[CHOIX_TEMP][LANCER_CUISSON_1][ACTION_MENU][NEXT_MENU] = MENU_INITIAL_1;
  Menu[CHOIX_TEMP][LANCER_CUISSON_1][ACTION_MENU][NEXT_STATE] = INITIAL;
  Menu[CHOIX_TEMP][LANCER_CUISSON_1][ACTION_UP][NEXT_MENU] = LANCER_CUISSON_1;
  Menu[CHOIX_TEMP][LANCER_CUISSON_1][ACTION_UP][NEXT_STATE] = CHOIX_TEMP;
  Menu[CHOIX_TEMP][LANCER_CUISSON_1][ACTION_DOWN][NEXT_MENU] = LANCER_CUISSON_1;
  Menu[CHOIX_TEMP][LANCER_CUISSON_1][ACTION_DOWN][NEXT_STATE] = CHOIX_TEMP;
  Menu[CHOIX_TEMP][LANCER_CUISSON_1][ACTION_SELECT][NEXT_MENU] = EN_COURS;
  Menu[CHOIX_TEMP][LANCER_CUISSON_1][ACTION_SELECT][NEXT_STATE] = CUISSON_EN_COURS;

  Menu[CUISSON_EN_COURS][EN_COURS][ACTION_MENU][NEXT_MENU] = MENU_INTERRUPTION_1;
  Menu[CUISSON_EN_COURS][EN_COURS][ACTION_MENU][NEXT_STATE] = CUISSON_EN_COURS;
  Menu[CUISSON_EN_COURS][EN_COURS][ACTION_UP][NEXT_MENU] = MENU_INTERRUPTION_1;
  Menu[CUISSON_EN_COURS][EN_COURS][ACTION_UP][NEXT_STATE] = CUISSON_EN_COURS;
  Menu[CUISSON_EN_COURS][EN_COURS][ACTION_DOWN][NEXT_MENU] = MENU_INTERRUPTION_1;
  Menu[CUISSON_EN_COURS][EN_COURS][ACTION_DOWN][NEXT_STATE] = CUISSON_EN_COURS;
  Menu[CUISSON_EN_COURS][EN_COURS][ACTION_SELECT][NEXT_MENU] = MENU_INTERRUPTION_1;
  Menu[CUISSON_EN_COURS][EN_COURS][ACTION_SELECT][NEXT_STATE] = CUISSON_EN_COURS;

  Menu[CUISSON_EN_COURS][MENU_INTERRUPTION_1][ACTION_MENU][NEXT_MENU] = MENU_INTERRUPTION_1;
  Menu[CUISSON_EN_COURS][MENU_INTERRUPTION_1][ACTION_MENU][NEXT_STATE] = CUISSON_EN_COURS;
  Menu[CUISSON_EN_COURS][MENU_INTERRUPTION_1][ACTION_UP][NEXT_MENU] = MENU_INTERRUPTION_1;
  Menu[CUISSON_EN_COURS][MENU_INTERRUPTION_1][ACTION_UP][NEXT_STATE] = CUISSON_EN_COURS;
  Menu[CUISSON_EN_COURS][MENU_INTERRUPTION_1][ACTION_DOWN][NEXT_MENU] = MENU_INTERRUPTION_2;
  Menu[CUISSON_EN_COURS][MENU_INTERRUPTION_1][ACTION_DOWN][NEXT_STATE] = CUISSON_EN_COURS;
  Menu[CUISSON_EN_COURS][MENU_INTERRUPTION_1][ACTION_SELECT][NEXT_MENU] = EN_COURS;
  Menu[CUISSON_EN_COURS][MENU_INTERRUPTION_1][ACTION_SELECT][NEXT_STATE] = CUISSON_EN_COURS;

  Menu[CUISSON_EN_COURS][MENU_INTERRUPTION_2][ACTION_MENU][NEXT_MENU] = MENU_INTERRUPTION_1;
  Menu[CUISSON_EN_COURS][MENU_INTERRUPTION_2][ACTION_MENU][NEXT_STATE] = CUISSON_EN_COURS;
  Menu[CUISSON_EN_COURS][MENU_INTERRUPTION_2][ACTION_UP][NEXT_MENU] = MENU_INTERRUPTION_1;
  Menu[CUISSON_EN_COURS][MENU_INTERRUPTION_2][ACTION_UP][NEXT_STATE] = CUISSON_EN_COURS;
  Menu[CUISSON_EN_COURS][MENU_INTERRUPTION_2][ACTION_DOWN][NEXT_MENU] = MENU_INTERRUPTION_2;
  Menu[CUISSON_EN_COURS][MENU_INTERRUPTION_2][ACTION_DOWN][NEXT_STATE] = CUISSON_EN_COURS;
  Menu[CUISSON_EN_COURS][MENU_INTERRUPTION_2][ACTION_SELECT][NEXT_MENU] = STOP;
  Menu[CUISSON_EN_COURS][MENU_INTERRUPTION_2][ACTION_SELECT][NEXT_STATE] = CUISSON_REFROISDISSEMENT;

  Menu[CUISSON_REFROISDISSEMENT][STOP][ACTION_MENU][NEXT_MENU] = MENU_REPRENDRE_1;
  Menu[CUISSON_REFROISDISSEMENT][STOP][ACTION_MENU][NEXT_STATE] = CUISSON_REFROISDISSEMENT;
  Menu[CUISSON_REFROISDISSEMENT][STOP][ACTION_UP][NEXT_MENU] = STOP;
  Menu[CUISSON_REFROISDISSEMENT][STOP][ACTION_UP][NEXT_STATE] = CUISSON_REFROISDISSEMENT;
  Menu[CUISSON_REFROISDISSEMENT][STOP][ACTION_DOWN][NEXT_MENU] = STOP;
  Menu[CUISSON_REFROISDISSEMENT][STOP][ACTION_DOWN][NEXT_STATE] = CUISSON_REFROISDISSEMENT;
  Menu[CUISSON_REFROISDISSEMENT][STOP][ACTION_SELECT][NEXT_MENU] = STOP;
  Menu[CUISSON_REFROISDISSEMENT][STOP][ACTION_SELECT][NEXT_STATE] = CUISSON_REFROISDISSEMENT;

  Menu[CUISSON_REFROISDISSEMENT][MENU_REPRENDRE_1][ACTION_MENU][NEXT_MENU] = MENU_REPRENDRE_1;
  Menu[CUISSON_REFROISDISSEMENT][MENU_REPRENDRE_1][ACTION_MENU][NEXT_STATE] = CUISSON_REFROISDISSEMENT;
  Menu[CUISSON_REFROISDISSEMENT][MENU_REPRENDRE_1][ACTION_UP][NEXT_MENU] = MENU_REPRENDRE_1;
  Menu[CUISSON_REFROISDISSEMENT][MENU_REPRENDRE_1][ACTION_UP][NEXT_STATE] = CUISSON_REFROISDISSEMENT;
  Menu[CUISSON_REFROISDISSEMENT][MENU_REPRENDRE_1][ACTION_DOWN][NEXT_MENU] = MENU_REPRENDRE_2;
  Menu[CUISSON_REFROISDISSEMENT][MENU_REPRENDRE_1][ACTION_DOWN][NEXT_STATE] = CUISSON_REFROISDISSEMENT;
  Menu[CUISSON_REFROISDISSEMENT][MENU_REPRENDRE_1][ACTION_SELECT][NEXT_MENU] = EN_COURS;
  Menu[CUISSON_REFROISDISSEMENT][MENU_REPRENDRE_1][ACTION_SELECT][NEXT_STATE] = CUISSON_EN_COURS;

  Menu[CUISSON_REFROISDISSEMENT][MENU_REPRENDRE_2][ACTION_MENU][NEXT_MENU] = MENU_REPRENDRE_1;
  Menu[CUISSON_REFROISDISSEMENT][MENU_REPRENDRE_2][ACTION_MENU][NEXT_STATE] = CUISSON_REFROISDISSEMENT;
  Menu[CUISSON_REFROISDISSEMENT][MENU_REPRENDRE_2][ACTION_UP][NEXT_MENU] = MENU_REPRENDRE_1;
  Menu[CUISSON_REFROISDISSEMENT][MENU_REPRENDRE_2][ACTION_UP][NEXT_STATE] = CUISSON_REFROISDISSEMENT;
  Menu[CUISSON_REFROISDISSEMENT][MENU_REPRENDRE_2][ACTION_DOWN][NEXT_MENU] = MENU_REPRENDRE_2;
  Menu[CUISSON_REFROISDISSEMENT][MENU_REPRENDRE_2][ACTION_DOWN][NEXT_STATE] = CUISSON_REFROISDISSEMENT;
  Menu[CUISSON_REFROISDISSEMENT][MENU_REPRENDRE_2][ACTION_SELECT][NEXT_MENU] = STOP;
  Menu[CUISSON_REFROISDISSEMENT][MENU_REPRENDRE_2][ACTION_SELECT][NEXT_STATE] = CUISSON_REFROISDISSEMENT;

  Menu[CUISSON_TERMINEE][STOP][ACTION_MENU][NEXT_MENU] = MENU_REPRENDRE_1;
  Menu[CUISSON_TERMINEE][STOP][ACTION_MENU][NEXT_STATE] = CUISSON_REFROISDISSEMENT;
  Menu[CUISSON_TERMINEE][STOP][ACTION_UP][NEXT_MENU] = STOP;
  Menu[CUISSON_TERMINEE][STOP][ACTION_UP][NEXT_STATE] = CUISSON_REFROISDISSEMENT;
  Menu[CUISSON_TERMINEE][STOP][ACTION_DOWN][NEXT_MENU] = STOP;
  Menu[CUISSON_TERMINEE][STOP][ACTION_DOWN][NEXT_STATE] = CUISSON_REFROISDISSEMENT;
  Menu[CUISSON_TERMINEE][STOP][ACTION_SELECT][NEXT_MENU] = STOP;
  Menu[CUISSON_TERMINEE][STOP][ACTION_SELECT][NEXT_STATE] = CUISSON_REFROISDISSEMENT;

  Menu[CUISSON_TERMINEE][MENU_REPRENDRE_1][ACTION_MENU][NEXT_MENU] = MENU_REPRENDRE_1;
  Menu[CUISSON_TERMINEE][MENU_REPRENDRE_1][ACTION_MENU][NEXT_STATE] = CUISSON_TERMINEE;
  Menu[CUISSON_TERMINEE][MENU_REPRENDRE_1][ACTION_UP][NEXT_MENU] = MENU_REPRENDRE_1;
  Menu[CUISSON_TERMINEE][MENU_REPRENDRE_1][ACTION_UP][NEXT_STATE] = CUISSON_TERMINEE;
  Menu[CUISSON_TERMINEE][MENU_REPRENDRE_1][ACTION_DOWN][NEXT_MENU] = MENU_REPRENDRE_2;
  Menu[CUISSON_TERMINEE][MENU_REPRENDRE_1][ACTION_DOWN][NEXT_STATE] = CUISSON_TERMINEE;
  Menu[CUISSON_TERMINEE][MENU_REPRENDRE_1][ACTION_SELECT][NEXT_MENU] = EN_COURS;
  Menu[CUISSON_TERMINEE][MENU_REPRENDRE_1][ACTION_SELECT][NEXT_STATE] = CUISSON_EN_COURS;

  Menu[CUISSON_TERMINEE][MENU_REPRENDRE_2][ACTION_MENU][NEXT_MENU] = MENU_REPRENDRE_1;
  Menu[CUISSON_TERMINEE][MENU_REPRENDRE_2][ACTION_MENU][NEXT_STATE] = CUISSON_TERMINEE;
  Menu[CUISSON_TERMINEE][MENU_REPRENDRE_2][ACTION_UP][NEXT_MENU] = MENU_REPRENDRE_1;
  Menu[CUISSON_TERMINEE][MENU_REPRENDRE_2][ACTION_UP][NEXT_STATE] = CUISSON_TERMINEE;
  Menu[CUISSON_TERMINEE][MENU_REPRENDRE_2][ACTION_DOWN][NEXT_MENU] = MENU_REPRENDRE_2;
  Menu[CUISSON_TERMINEE][MENU_REPRENDRE_2][ACTION_DOWN][NEXT_STATE] = CUISSON_TERMINEE;
  Menu[CUISSON_TERMINEE][MENU_REPRENDRE_2][ACTION_SELECT][NEXT_MENU] = STOP;
  Menu[CUISSON_TERMINEE][MENU_REPRENDRE_2][ACTION_SELECT][NEXT_STATE] = CUISSON_TERMINEE;
}

void setup() {
  pinMode(11, INPUT);
  pinMode(13, OUTPUT);
  pinMode(L1, OUTPUT);  //L1 est une broche de sortie
  ETAT_MENU = MENU_INITIAL_1;
  STATE = INITIAL;
  setupMenu();
  Serial.begin(19200);
  // init du timer et des interruptions
  cli();                    // Désactive l'interruption globale
  bitClear(TCCR2A, WGM20);  // WGM20 = 0
  bitClear(TCCR2A, WGM21);  // WGM21 = 0
  //TCCR2B = 0b00000001;      // Clock
  //TCCR2B = 0b00000010;      // Clock / 8 soit 0.5 micro-s et WGM22 = 0
  //TCCR2B = 0b00000011;      // Clock / 32 soit 2 micro-s et WGM22 = 0
  //TCCR2B = 0b00000100;      // Clock / 64 soit 4 micro-s et WGM22 = 0
  TCCR2B = 0b00000101;  // Clock / 128 soit 4 micro-s et WGM22 = 0
  //TCCR2B = 0b00000110;      // Clock / 256 soit 16 micro-s et WGM22 = 0
  //TCCR2B = 0b00000111;      // Clock / 1024 soit 64 micro-s et WGM22 = 0
  TIMSK2 = 0b00000001;  // Interruption locale autorisée par TOIE2
  sei();                // Active l'interruption global


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
    case MAX31856_TCTYPE_K: Serial.println(F("K Type")); break;
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
  initialisation = true;
  // temps initial
  tInit = millis() / TSec;
  tDecalePhase = 0;
  // test LED et relais
  digitalWrite(L1, HIGH);  //allumer L1
  delay(250);
  digitalWrite(L1, LOW);  // éteindre L1
  cycleTermine = true;
  Serial.println(F("erreur;correctionP;Integ;Deriv;D_filtree;commande;mesure;ratio;consigne;dureePhase"));
}

byte varCompteurTimer = 0;  // La variable compteur
byte compteurEchantillon = 0;
float ratio;
bool stopCuisson = false;
float derivationFiltree = 0.0;
// Routine d'interruption
// période échantillonge 15 secondes
ISR(TIMER2_OVF_vect) {

  TCNT2 = 256 - 125;  //125*8µs => débordement à 1ms

  if (varCompteurTimer++ > 125) {  // 125* 1ms = 125ms
    varCompteurTimer = 0;

    if (compteurEchantillon++ >= nbEchantillons - 1) {
      compteurEchantillon = 0;
      cycleTermine = true;
      if (STATE != CUISSON_REFROISDISSEMENT && STATE != CUISSON_TERMINEE) {
        // calcul de la consigne
        consigne = calculeConsigne(pente, temperatureInitiale, temperatureFinale, dureePhase);
        // calcul de la commande

        // TODO adapter les coefficient en fonction de la commande possible (plus agressif au début)
        erreur = consigne - temperatureMoyenne;

        float kp, ki, kd;
        if (temperatureMoyenne < 100) {
          kp = pids[2].Kp;
          ki = pids[2].Ki;
          kd = pids[2].Kd;
        } else if (temperatureMoyenne < 400) {
          kp = pids[1].Kp;
          ki = pids[1].Ki;
          kd = pids[1].Kd;
        } else {
          kp = pids[0].Kp;
          ki = pids[0].Ki;
          kd = pids[0].Kd;
        }
        // terme proportionnel

        CorrectionP = erreur * kp;

        // calcul du terme intégral
        // Ki = 1/Ti

        integration = dt * nbEchantillons * erreur / ki + integration;
        if (integration > 1000) integration = 1000;
        if (integration < -1000) integration = -1000;
        // calcul du terme dérivé
        if (temperatureMoyenne < TEMPERATURE_SEUIL_PID) {
          derivation = (erreur - erreur_n_1) * kd * dt / (nbEchantillons);
        } else {
          derivation = (erreur - erreur_n_1) * kd * dt / (nbEchantillons);
        }
        // lissage terme dérivé
        derivationFiltree = filtreDerivee(derivation);
        erreur_n_1 = erreur;
        commande = integration + CorrectionP + derivationFiltree;
        if (commande > AMPLITUDE_MAX) commande = AMPLITUDE_MAX;
        if (commande < 0) commande = 0;
        ratio = round(calculRatio(commande));
      } else {
        ratio = 0;
      }
    }
    switch (STATE) {
      case CUISSON_EN_COURS:
        // start with 0 to avoid wrong temperature measure
        if (compteurEchantillon > (nbEchantillons - ratio)) {
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

void debugState(byte state, byte menu) {
  switch (state) {
    case INITIAL:
      Serial.println(F("State: INITIAL"));
      switch (menu) {
        case MENU_INITIAL_1:
          Serial.println(F("Menu: MENU_INITIAL_1"));
          break;
        case MENU_INITIAL_2:
          Serial.println(F("Menu: MENU_INITIAL_2"));
          break;
        case CONSIGNE_FIXE:
          Serial.println(F("Menu: CONSIGNE_FIXE"));
          break;
      }
      break;
    case CHOIX_TEMP:
      Serial.println(F("State: CHOIX_TEMP"));
      switch (menu) {
        case TEMP_EMAIL:
          Serial.println(F("Menu: TEMP_EMAIL"));
          break;
        case TEMP_BISCUIT:
          Serial.println(F("Menu: TEMP_BISCUIT"));
          break;
        case TEMP_CONSIGNE_FIXE:
          Serial.println(F("Menu: TEMP_CONSIGNE_FIXE"));
          break;
        case LANCER_CUISSON_1:
          Serial.println(F("Menu: LANCER_CUISSON_1"));
          break;
      }
      break;
    case CUISSON_EN_COURS:
      Serial.println(F("State: CUISSON_EN_COURS"));
      switch (menu) {
        case EN_COURS:
          Serial.println(F("Menu: EN_COURS"));
          break;
        case MENU_INTERRUPTION_1:
          Serial.println(F("Menu: MENU_INTERRUPTION_1"));
          break;
        case MENU_INTERRUPTION_2:
          Serial.println(F("Menu: MENU_INTERRUPTION_2"));
          break;
      }
      break;
    case CUISSON_REFROISDISSEMENT:
      Serial.println(F("State: CUISSON_REFROISDISSEMENT"));
      switch (menu) {
        case STOP:
          Serial.println(F("Menu: STOP"));
          break;
        case MENU_REPRENDRE_1:
          Serial.println(F("Menu: MENU_REPRENDRE_1"));
          break;
        case MENU_REPRENDRE_2:
          Serial.println(F("Menu: MENU_REPRENDRE_2"));
          break;
      }
      break;
    case CUISSON_TERMINEE:
      Serial.println(F("State: CUISSON_TERMINEE"));
      switch (menu) {
        case STOP:
          Serial.println(F("Menu: STOP"));
          break;
        case MENU_REPRENDRE_1:
          Serial.println(F("Menu: MENU_REPRENDRE_1"));
          break;
        case MENU_REPRENDRE_2:
          Serial.println(F("Menu: MENU_REPRENDRE_2"));
          break;
      }
      break;
  }
}
/**
* 
*/
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
      menuOption = ACTION_SELECT;
      break;
    case btnNONE:
      break;
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
    Serial.println(F("menu changed"));
    menuChanged = false;
    menuEnCours = 1;
    // update the LCD to show the selected option
    lcd.clear();
    byte previous_menu;
    byte previous_state;
    switch (menuOption) {
      case ACTION_SELECT:
        previous_state = STATE;
        previous_menu = ETAT_MENU;
        ETAT_MENU = Menu[STATE][ETAT_MENU][ACTION_SELECT][NEXT_MENU];
        STATE = Menu[STATE][previous_menu][ACTION_SELECT][NEXT_STATE];
        debugState(STATE, ETAT_MENU);

        switch (STATE) {
          case CHOIX_TEMP:
            switch(ETAT_MENU) {
              case TEMP_EMAIL:
                typeCuisson = CUISSON_EMAIL;
                userDefinedMaxTemp = DEFAULT_TEMPERATURE_MAX_EMAIL;
                break;
              case TEMP_BISCUIT:
                typeCuisson = CUISSON_BISCUIT;
                userDefinedMaxTemp = DEFAULT_TEMPERATURE_MAX_BISCUIT;
                break;
              case TEMP_CONSIGNE_FIXE:
                typeCuisson = CUISSON_CONSIGNE_FIXE;
                userDefinedMaxTemp = DEFAULT_TEMPERATURE_MAX_BISCUIT;
                break;                
            }
          case CUISSON_EN_COURS:
            stopCuisson = false;
            if (previous_state == CHOIX_TEMP && previous_menu == LANCER_CUISSON_1) {
              Serial.print("typeCuisson ");
              Serial.println(typeCuisson);
              setPhaseParameters(typeCuisson, phaseEnCours);
              tInit = millis() / TSec;
            }
            break;
          case CUISSON_REFROISDISSEMENT:
          case CUISSON_TERMINEE:
            stopCuisson = true;
            tInit = millis() / TSec;
            break;
        }
        break;
      case ACTION_DOWN:
        //Serial.println(F("Down"));
        previous_menu = ETAT_MENU;
        ETAT_MENU = Menu[STATE][ETAT_MENU][ACTION_DOWN][NEXT_MENU];
        STATE = Menu[STATE][previous_menu][ACTION_DOWN][NEXT_STATE];
        switch (ETAT_MENU) {
          case TEMP_BISCUIT:
          case TEMP_EMAIL:
          case TEMP_CONSIGNE_FIXE:
            if (userDefinedMaxTemp > 700) userDefinedMaxTemp -= 10;
        }
        break;
      case ACTION_UP:
        previous_menu = ETAT_MENU;
        ETAT_MENU = Menu[STATE][ETAT_MENU][ACTION_UP][NEXT_MENU];
        STATE = Menu[STATE][previous_menu][ACTION_UP][NEXT_STATE];
        switch (ETAT_MENU) {
          case TEMP_BISCUIT:
          case TEMP_EMAIL:
          case TEMP_CONSIGNE_FIXE:
            if (userDefinedMaxTemp < 1100) userDefinedMaxTemp += 10;
        }
        break;
      case ACTION_MENU:
        previous_menu = ETAT_MENU;
        ETAT_MENU = Menu[STATE][ETAT_MENU][ACTION_MENU][NEXT_MENU];
        STATE = Menu[STATE][previous_menu][ACTION_MENU][NEXT_STATE];
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
  temperatureMoyenne = calculTemperatureMoyenne(temperatureActuelle);
  if (initialisation == true) {
    byte i;
    for (i = 0; i < 16; i++) {
      mesures[i] = temperatureActuelle;
    }
    temperatureMoyenne = calculTemperatureMoyenne(temperatureActuelle);
    consigneInitiale = temperatureMoyenne + 10;  // départ immédiat

    initialisation = false;
  }

  switch (STATE) {
    case INITIAL:
      menuEnCours = 1;
      lcd.setCursor(10, 0);
      lcd.print(temperatureActuelle);
      lcd.setCursor(15, 1);
      lcd.print("-");
      break;
    case CUISSON_EN_COURS:
      if (ETAT_MENU == EN_COURS) {
        menuEnCours = 0;
      } else {
        menuEnCours = 1;
      }
      break;
    case CUISSON_REFROISDISSEMENT:
      if (ETAT_MENU == STOP) {
        menuEnCours = 0;
        lcd.setCursor(15, 1);
        lcd.print("_");
      } else {
        menuEnCours = 1;
      }
      break;
    case CUISSON_TERMINEE:
      menuEnCours = 0;
      lcd.setCursor(15, 1);
      lcd.print("x");
      break;
    case CHOIX_TEMP:
      menuEnCours = 1;
      lcd.setCursor(15, 1);
      lcd.print("C");
      lcd.setCursor(4, 1);
      lcd.print(userDefinedMaxTemp);
      break;
  }
  if (menuEnCours) {
    lcd.setCursor(0, 0);
    lcd.print(MenuItems[STATE][ETAT_MENU][0]);
    lcd.setCursor(0, 1);
    lcd.print(MenuItems[STATE][ETAT_MENU][1]);

  } else {
    if (STATE == CUISSON_EN_COURS || STATE == STOP || STATE == CUISSON_REFROISDISSEMENT || STATE == CUISSON_TERMINEE) {
      // Lecture de l'horloge interne en ms
      Temps_ms = millis();  // 2^32 secondes = 49.71 jours
      // Calcul des secondes
      timeInSecond = Temps_ms / TSec;

      dureePhase = timeInSecond - tInit;
      Num_sec = timeInSecond % 60;
      // Calcul des minutes
      Num_min = (Temps_ms / (TSec * 60)) % 60;
      // Calcul des heures
      Num_heur = (Temps_ms / (TSec * 3600)) % 60;

      if (STATE != CUISSON_REFROISDISSEMENT && STATE != CUISSON_TERMINEE && stopCuisson == false) {
        // calcul de la consigne
        if (STATE != CUISSON_EN_COURS) {
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
            if (!menuEnCours) {
              displayPhase(phaseEnCours, dureePhase);
              displayGoal(temperatureMoyenne, temperatureFinale);
            }

          } else {  //TODO
            if (!menuEnCours) {
              displayGoal(temperatureMoyenne, temperatureFinale);
              displayTime(Num_heur, Num_min, Num_sec);
            }
          }
        }

        if (cycleTermine == true) {
          cycleTermine = false;
          Serial.println();
          Serial.print(erreur);
          Serial.print(F(";"));
          Serial.print(CorrectionP);
          Serial.print(F(";"));
          Serial.print(integration);
          Serial.print(F(";"));
          Serial.print(derivation);
          Serial.print(F(";"));
          Serial.print(derivationFiltree);
          Serial.print(F(";"));
          Serial.print(commande);
          Serial.print(F(";"));
          Serial.print(temperatureMoyenne);
          Serial.print(F(";"));
          Serial.print(ratio);
          Serial.print(F(";"));
          Serial.print(consigne);
          Serial.print(F(";"));
          Serial.print(dureePhase);
          Serial.print(F(";"));
        }
        if (STATE == CUISSON_EN_COURS) {
          Serial.println(sizeof(courbe[0]) / (sizeof(courbe[0][0])));
          Serial.println(sizeof(courbe[1]) / (sizeof(courbe[1][0])));
          Serial.println(sizeof(courbe[2]) / (sizeof(courbe[2][0])));
          if (changePhase(consigne, temperatureMoyenne, phaseEnCours, dureePhase, tDecalePhase)) {
            tInit = timeInSecond;
            tDecalePhase = 0;
            phaseEnCours++;
            //derivation = 0;
            //CorrectionP = 0;
            //integration = 0;
             
            if (phaseEnCours >= NB_PHASES) {
              STATE = CUISSON_REFROISDISSEMENT;
              ETAT_MENU = STOP;
            } else {
              setPhaseParameters(typeCuisson, phaseEnCours);
            }
          }
        }
      } else {
        lcd.clear();
        displayTemperature(temperatureMoyenne, consigne);
        if (temperatureMoyenne >= 100) {
          STATE = CUISSON_REFROISDISSEMENT;
          lcd.setCursor(0, 1);
          lcd.print(F("Refroisissement "));
        } else {
          STATE = CUISSON_TERMINEE;
          lcd.setCursor(0, 1);
          lcd.print(F("Cuisson terminee"));
        }

        ratio = 0;
        if (cycleTermine == true) {
          cycleTermine = false;
          Serial.println();
          Serial.print(F(";"));
          Serial.print(F(";"));
          Serial.print(F(";"));
          Serial.print(F(";"));
          Serial.print(F(";"));
          Serial.print(temperatureMoyenne);
          Serial.print(F(";"));
          Serial.print(ratio);
          Serial.print(F(";"));
          Serial.print(dureePhase);
          Serial.print(F(";"));
        }
      }
    }
  }
}
