#include <LiquidCrystal.h>
#include <Adafruit_MAX31856.h>
#include <avr/interrupt.h>
#define CUISSON_FAIENCE 0;
#define CUISSON_EMAIL 1;
// Use software SPI: CS, DI, DO, CLK
Adafruit_MAX31856 maxthermo = Adafruit_MAX31856(2, 11, 12, 13);
// use hardware SPI, just pass in the CS pin
//Adafruit_MAX31856 maxthermo = Adafruit_MAX31856(3);
// use hardware SPI, pass in the CS pin and using SPI1
//Adafruit_MAX31856 maxthermo = Adafruit_MAX31856(10, &SPI1);

const int L1 = 3; // commande relais / LED
//const int rs = 8, en = 9, d4 = 4, d5 = 5, d6 = 6, d7 = 7;
//LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);
// Données des mesures
double Input;

typedef struct {
  int t0;
  int t1;
  unsigned long pente;
  int duree;
} segment;

// temps
const unsigned long TSec=1000;
unsigned long Num_ms, Num_sec,Num_min; 
unsigned long Num_heur,Num_jour, Temps_ms;
unsigned long t;
// indicateurs
boolean cuissonTerminee;
const int NB_TYPES = 2;
const int NB_PHASES =6;


// constantes de correction
double Kp = 7; //2;
float Ki = 0.0006; // 0.00038
double Kd = 10000; // 10000
double dt = 1; // période échantillonage = 1s
double integration = 0.;
double derivation = 0.;
double erreur_n_1 =0;
// constantes de système
double amplitude = 1100; // puissance electrique
int nbEchantillons = 120; // base de codage pour une valeur de consigne

// cuisson faïence 
segment courbe[NB_TYPES][NB_PHASES] = {
  {
    // cuisson faience
    // pentes de chauffe en °C / heure
    {  0, 100, 100,  0},   // 0 à 100° à 100°C/heure 
    {100, 100,   0, 10},   // 95 à 105° pendant 10 minutes
    {100, 400, 100,  0},    // 105 à 400° à 100°C/heure
    {400, 400,   0, 15},    // 400° pendant 15 min 
    {400, 1030,150,  0},   // 400 à 1030° à 150°C/heure
    {1030,1030,  0, 30},  // 1030° pendant 30 min 
  },
  {
    // cuisson email
    // pentes de chauffe en °C / heure
    {  0, 100,  100,  0},   // 0 à 100° à 100°C/heure 
    {105, 105,   0, 10},   // 95 à 105° pendant 10 minutes
    {105, 400,  100, 0},    // 105 à 400° à 100°C/heure
    {400, 400,   0, 15},    // 400° pendant 15 min 
    {400, 1030, 150, 0},   // 400 à 1030° à 150°C/heure
    {1030,1030,  0, 30},  // 1030° pendant 30 min 
  },
  
};

int phaseEnCours, typeCuisson;
boolean initialisation;
float consigneInitiale = 0; // consigne initiale
float dureePhase;
float tInit; // temps à l'ilit de la phase
float tDecalePhase; //décalage de phase (température non atteinte)
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
bool cycleTermine; // flag de fin d'une période 
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
float correctionProportionnelle(float erreur) {
  return erreur * Kp;
}

float correctionIntegral(float erreur, float integration) {
  return Ki * dt * nbEchantillons *  erreur + integration;
}

float correctionDerive(float erreur_n_1, float erreur) {
  return (erreur - erreur_n_1) * Kd * dt / (nbEchantillons);
}

/* calcul de la consigne
 *  phase index de la phase
 *  t en seconde 
*/
float calculeConsigne(int phase, float t) {
  if (courbe[typeCuisson][phase].pente == 0) {
    return courbe[typeCuisson][phase].t0;  
  } else {
    if(phase == 0) {
      return courbe[typeCuisson][phase].pente*t/3600 + consigneInitiale;
    }
    return courbe[typeCuisson][phase].pente*t/3600 + courbe[typeCuisson][phase].t0;
  }
}

float calculRatio(float consigne, int amplitude, int nbEchantillon) {
  float ratio = consigne*nbEchantillon/amplitude;
  if (ratio > nbEchantillons) {
     return nbEchantillons;
  } else if (ratio < 0) {
    return 0;
  }
  return ratio;
}
/** 
 *  Calcul du changement de phase 
 *  temperature : température de consigne
 *  phase : index de la phase
 *  t : durée écoulée de la phase
 */
boolean changePhase(float consigne, float mesure, int phase, float t, float &tDecalage) {
  if (courbe[typeCuisson][phase].pente == 0) {
    // si on a pas atteint la temperature
    if (mesure*1.05 < courbe[typeCuisson][phase].t0) {
      tDecalage = t;
      return false;
    }
    return t-tDecalage > courbe[typeCuisson][phase].duree*60;  
  } else {
    return courbe[typeCuisson][phase].t1 < consigne;
  }
}
/* Affichage horloge */ 
void displayTime(unsigned long heure, unsigned long minutes,unsigned long secondes) {
  lcd.setCursor(0,1);
  lcd.print("Total: ");
  lcd.setCursor(8,1);
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
  lcd.setCursor(0,1);
  lcd.print("Ph:");
  lcd.print(phase); 
  int Heure=(duree/3600)%60;
  int Min=(duree/60)%60;
  int Sec=duree%60;
  lcd.setCursor(8,1);
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
    
  lcd.setCursor(0,0);
  lcd.print(temp); 
  lcd.write((byte)0);
  lcd.setCursor(10,0);
  lcd.print(consigne); 
  lcd.write((byte)0);
}
void displayGoal(float temp, float goal) {
  lcd.setCursor(0,0);
  lcd.print(temp); 
  lcd.write((byte)0);
  lcd.setCursor(8,0);
  lcd.print("->"); 
  lcd.print((int)goal); 
  lcd.write((byte)0);
}
void displayFinCuisson() {
  lcd.setCursor(0,1);
  lcd.print("Cuisson terminee");
}
/*
 * lecture de la température du thermocouple
 */
float readTemp(Adafruit_MAX31856 &maxTh, float temp){
  maxTh.triggerOneShot();
  delay(200);
  if (maxTh.conversionComplete()) {
    return maxTh.readThermocoupleTemperature();
  } else {
    uint8_t fault = maxTh.readFault();
    if (fault) {
      if (fault & MAX31856_FAULT_CJRANGE) Serial.println("Cold Junction Range Fault");
      if (fault & MAX31856_FAULT_TCRANGE) Serial.println("Thermocouple Range Fault");
      if (fault & MAX31856_FAULT_CJHIGH)  Serial.println("Cold Junction High Fault");
      if (fault & MAX31856_FAULT_CJLOW)   Serial.println("Cold Junction Low Fault");
      if (fault & MAX31856_FAULT_TCHIGH)  Serial.println("Thermocouple High Fault");
      if (fault & MAX31856_FAULT_TCLOW)   Serial.println("Thermocouple Low Fault");
      if (fault & MAX31856_FAULT_OVUV)    Serial.println("Over/Under Voltage Fault");
      if (fault & MAX31856_FAULT_OPEN)    Serial.println("Thermocouple Open Fault");
    }
    Serial.println(fault);
    Serial.println("Conversion not complete!");
  }
  return temp;
}
/**
 * Gestion du menu
 */
 
int adc_key_in = 0;
#define btnRIGHT  0
#define btnUP     1
#define btnDOWN   2
#define btnLEFT   3
#define btnSELECT 4
#define btnNONE   5

int read_LCD_buttons()
{
 adc_key_in = analogRead(0);      // read the value from the sensor 
 // my buttons when read are centered at these valies: 0, 144, 329, 504, 741
 // we add approx 50 to those values and check to see if we are close
 if (adc_key_in > 1000) return btnNONE; // We make this the 1st option for speed reasons since it will be the most likely result
 // For V1.1 us this threshold
// if (adc_key_in < 50)   return btnRIGHT;  
// if (adc_key_in < 250)  return btnUP; 
// if (adc_key_in < 450)  return btnDOWN; 
// if (adc_key_in < 650)  return btnLEFT; 
// if (adc_key_in < 850)  return btnSELECT;  

 // For V1.0 comment the other threshold and use the one below:

 if (adc_key_in < 50)   return btnRIGHT;  
 if (adc_key_in < 195)  return btnUP; 
 if (adc_key_in < 380)  return btnDOWN; 
 if (adc_key_in < 555)  return btnLEFT; 
 if (adc_key_in < 790)  return btnSELECT;   



 return btnNONE;  // when all others fail, return this...
}

// create two variables to store the current menu option
// and the previous menu option
int menuOption = 0;
int prevMenuOption = 2;
// 0 = non commencé
#define INITIAL 0
// 1 = en cours
#define EN_COURS 1
// 2 = stoppé
#define STOP 2
// consigne fixe
#define CONSIGNE_SELECT 3
#define CONSIGNE_EN_COURS 4
#define STOP_CONSIGNE 5
int consigneCustomTemp = 50;


int etatCuisson = INITIAL; 
bool menuEnCours = 0; 


 
void setup() {  
  pinMode(11,INPUT);
  pinMode(13,OUTPUT);
  pinMode(L1, OUTPUT); //L1 est une broche de sortie


  Serial.begin(9600);

  // init du timer et des interruptions
  cli(); // Désactive l'interruption globale
  bitClear (TCCR2A, WGM20); // WGM20 = 0
  bitClear (TCCR2A, WGM21); // WGM21 = 0 
  TCCR2B = 0b00000110; // Clock / 256 soit 16 micro-s et WGM22 = 0
  TIMSK2 = 0b00000001; // Interruption locale autorisée par TOIE2
  sei(); // Active l'interruption global

  
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
  switch (maxthermo.getThermocoupleType() ) {
    case MAX31856_TCTYPE_B: Serial.println("B Type"); break;
    case MAX31856_TCTYPE_E: Serial.println("E Type"); break;
    case MAX31856_TCTYPE_J: Serial.println("J Type"); break;
    case MAX31856_TCTYPE_K: Serial.println("K Type"); break;
    case MAX31856_TCTYPE_N: Serial.println("N Type"); break;
    case MAX31856_TCTYPE_R: Serial.println("R Type"); break;
    case MAX31856_TCTYPE_S: Serial.println("S Type"); break;
    case MAX31856_TCTYPE_T: Serial.println("T Type"); break;
    case MAX31856_VMODE_G8: Serial.println("Voltage x8 Gain mode"); break;
    case MAX31856_VMODE_G32: Serial.println("Voltage x8 Gain mode"); break;
    default: Serial.println("Unknown"); break;
  }

  maxthermo.setConversionMode(MAX31856_ONESHOT_NOWAIT);
  phaseEnCours = 0;
  typeCuisson = CUISSON_FAIENCE; // faience
  cuissonTerminee=false;
  initialisation=true;
  // temps initial
  tInit = millis()/TSec;
  tDecalePhase = 0;
  // test LED et relais
  digitalWrite(L1, HIGH); //allumer L1
  delay(250);
  digitalWrite(L1, LOW); // éteindre L1
  cycleTermine=true;
  Serial.println("erreur;correctionP;Integration;Derivation;commande;mesure;ratio");
}

byte varCompteurTimer = 0; // La variable compteur
byte compteurEchantillon = 0;
float ratio;
bool stopCuisson = false;
// Routine d'interruption
ISR(TIMER2_OVF_vect) {
  TCNT2 = 256 - 250; // 250 x 16 µS = 4 ms
  if (varCompteurTimer++ > 250) { // 500 * 4 ms = 1000 ms 
    varCompteurTimer = 0;
        
    if (compteurEchantillon++ >= nbEchantillons-1) {
      compteurEchantillon = 0;
      cycleTermine = true; 
      if (cuissonTerminee == false) {
        if (etatCuisson == CONSIGNE_EN_COURS) {
          consigne = consigneCustomTemp;
        } else {
          consigne = calculeConsigne(phaseEnCours, dureePhase);  
        }
        // calcul de la consigne
        
        // calcul de la commande
        erreur = calculeErreur(consigne,Input);
        derivation = correctionDerive(erreur_n_1, erreur); 
        erreur_n_1 = erreur;
        CorrectionP = correctionProportionnelle(calculeErreur(consigne,Input));   
        integration = correctionIntegral(erreur, integration);
        commande = integration + CorrectionP + derivation;
        ratio = calculRatio(commande, amplitude, nbEchantillons); 
      } else {
        ratio = 0;
      }
    }
    if (etatCuisson == EN_COURS  || etatCuisson == CONSIGNE_EN_COURS  || etatCuisson == STOP) {
      if(compteurEchantillon < ratio && cuissonTerminee == false) {
        digitalWrite(L1, HIGH); //allumer L1 
      } else {
        digitalWrite(L1, LOW); //éteindre L1
      }
      Serial.print(".");
    }
  }
}

/*****************************************
 * 
 */
 
#define T_MENU  0
#define T_UP     1
#define T_DOWN   2
#define T_SELECT   3
int previousBtn = 0;

 void checkMenu() {
  // read the values of the push buttons
  
  int button1State = read_LCD_buttons();
  bool menuChanged = false;
  // check if the first button is pressed
  Serial.println(button1State);
  switch (button1State)               // depending on which button was pushed, we perform an action
  {
    case btnRIGHT:
      menuOption = T_MENU;
      break;
    case btnLEFT: // touche menu
      menuOption = T_MENU; 
      break;
    case btnUP:
      menuOption = T_UP; 
      break;
    case btnDOWN:
      menuOption = T_DOWN;
      break;
    case btnSELECT:
      {
      menuOption = T_SELECT;
      break;
      }
    case btnNONE:
      {
      break;
      }
  }
  if(previousBtn != button1State) {
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
  if (menuChanged && previousBtn == btnNONE ) {
    menuChanged = false;
    menuEnCours = 1;
    // update the LCD to show the selected option
    lcd.clear();
    switch (menuOption) {
      case T_MENU:
        switch(etatCuisson) {
          case INITIAL:
            lcd.setCursor(0, 0);
            lcd.print(">Cuisson Biscuit");
            lcd.setCursor(0, 1);
            lcd.print(" Consigne Fixe");
            break;
          case EN_COURS:
            lcd.setCursor(0, 0);
            lcd.print(">Continuer");
            lcd.setCursor(0, 1);
            lcd.print(" Stopper");
            break;
          case STOP:
            lcd.setCursor(0, 0);
            lcd.print(">Reprendre");
            lcd.setCursor(0, 1);
            lcd.print(" Menu Principal");
            break;
          case CONSIGNE_EN_COURS:
            lcd.setCursor(0, 0);
            lcd.print(">Continuer");
            lcd.setCursor(0, 1);
            lcd.print(" Stopper");
            break;
         case STOP_CONSIGNE:
            lcd.setCursor(0, 0);
            lcd.print(">Reprendre");
            lcd.setCursor(0, 1);
            lcd.print(" Menu Principal");
            break;
        }
//        prevMenuOption = menuOption;
        break;
      case T_UP:
        switch(etatCuisson) {
          case INITIAL:
            lcd.setCursor(0, 0);
            lcd.print(">Cuisson Biscuit");
            lcd.setCursor(0, 1);
            lcd.print(" Consigne Fixe");
            break;
          case EN_COURS:
            lcd.setCursor(0, 0);
            lcd.print(">Continuer");
            lcd.setCursor(0, 1);
            lcd.print(" Stopper");
            break;
          case STOP:
            lcd.setCursor(0, 0);
            lcd.print(">Reprendre");
            lcd.setCursor(0, 1);
            lcd.print(" Menu Principal");
            break;
          case CONSIGNE_SELECT:
            lcd.setCursor(0, 0);
            lcd.print("Consigne fixe");
            lcd.setCursor(0, 1);
            lcd.print("Temp: ");
            if(consigneCustomTemp < 1050) {
              consigneCustomTemp += 50;
            }
            lcd.print(consigneCustomTemp);
            break;
          case CONSIGNE_EN_COURS:
            lcd.setCursor(0, 0);
            lcd.print(">Continuer");
            lcd.setCursor(0, 1);
            lcd.print(" Stopper");
            break;
          case STOP_CONSIGNE:
            lcd.setCursor(0, 0);
            lcd.print(">Reprendre");
            lcd.setCursor(0, 1);
            lcd.print(" Menu Principal");
            break;
        }
        break;
      case T_DOWN:
        switch(etatCuisson) {
          case INITIAL:
            lcd.setCursor(0, 0);
            lcd.print(" Cuisson Biscuit");
            lcd.setCursor(0, 1);
            lcd.print(">Consigne Fixe");
            break;
          case EN_COURS:
            lcd.setCursor(0, 0);
            lcd.print(" Continuer");
            lcd.setCursor(0, 1);
            lcd.print(">Stopper");
            break;
          case STOP:
            lcd.setCursor(0, 0);
            lcd.print(" Reprendre");
            lcd.setCursor(0, 1);
            lcd.print(">Menu Principal");
            break;
          case CONSIGNE_SELECT:
            lcd.setCursor(0, 0);
            lcd.print("Consigne fixe");
            lcd.setCursor(0, 1);
            lcd.print("Temp: ");
            if(consigneCustomTemp > 0) {
              consigneCustomTemp -= 50;
            }
            lcd.print(consigneCustomTemp);
            break;
          case CONSIGNE_EN_COURS:
            lcd.setCursor(0, 0);
            lcd.print(" Continuer");
            lcd.setCursor(0, 1);
            lcd.print(">Stopper");
            break;
          case STOP_CONSIGNE:
            lcd.setCursor(0, 0);
            lcd.print(" Reprendre");
            lcd.setCursor(0, 1);
            lcd.print(">Menu Principal");
            break;
        }
//        prevMenuOption = menuOption;
        break;
      case T_SELECT: // select
        switch(etatCuisson) {
            case INITIAL:
              if (prevMenuOption == T_DOWN) {// option Consigne Fixe 
                lcd.print("Consigne fixe");
                etatCuisson = CONSIGNE_SELECT;
                lcd.setCursor(0, 1);
                lcd.print("Temp: ");
                lcd.print(consigneCustomTemp);
              } else {
                lcd.print("Depart cuisson"); // option cuisson biscuit
                stopCuisson = false;
                cuissonTerminee = false;
                etatCuisson = EN_COURS;
                menuEnCours = 0;
                tInit = millis()/TSec;
              }
              break;
            case EN_COURS:
              if (prevMenuOption == T_UP) {// option Continuer 
                lcd.setCursor(0, 0);
                lcd.print("Reprise cuisson");
                stopCuisson = false;
                menuEnCours = 0;
              } else if (prevMenuOption == T_DOWN) { //Option stopper
                lcd.setCursor(0, 0);
                lcd.print("Stop cuisson");
                cuissonTerminee = true;
                stopCuisson = true;
                menuEnCours = 0;
                etatCuisson = STOP;
              }
              break;
            case STOP:
              if (prevMenuOption == T_UP) {// option Reprendre 
                lcd.setCursor(0, 0);
                lcd.print("Reprise cuisson");
                stopCuisson = false;
                cuissonTerminee = false;
                etatCuisson = EN_COURS;
                menuEnCours = 0;
              } else if (prevMenuOption == T_DOWN) { //Option Menu Principal
                lcd.setCursor(0, 0);
                lcd.print(">Cuisson Biscuit");
                lcd.setCursor(0, 1);
                lcd.print(" Consigne Fixe");
                etatCuisson = INITIAL;
                //menuEnCours = 0;
              } else {
                lcd.setCursor(0, 0);
                lcd.print("Reprise cuisson");
                stopCuisson = false;
                cuissonTerminee = false;
                etatCuisson = EN_COURS;
                menuEnCours = 0;
              }
              break;  
            case CONSIGNE_SELECT:
              lcd.setCursor(0, 0);
              lcd.print("Consigne fixe");
              lcd.setCursor(0, 1);
              lcd.print("Temp: ");
              etatCuisson = CONSIGNE_EN_COURS;
              stopCuisson = false;
              cuissonTerminee = false;
              menuEnCours = 0;
              tInit = millis()/TSec;
              break;
            case CONSIGNE_EN_COURS:
              if (prevMenuOption == T_UP) {// option Continuer 
                lcd.setCursor(0, 0);
                lcd.print("Reprise cuisson");
                stopCuisson = false;
                menuEnCours = 0;
              } else if (prevMenuOption == T_DOWN) { //Option stopper
                lcd.setCursor(0, 0);
                lcd.print("Stop cuisson");
                cuissonTerminee = true;
                stopCuisson = true;
                menuEnCours = 0;
                etatCuisson = STOP_CONSIGNE;
              }
              break;
            case STOP_CONSIGNE:
              if (prevMenuOption == T_UP) {// option Reprendre 
                lcd.setCursor(0, 0);
                lcd.print("Reprise cuisson");
                stopCuisson = false;
                cuissonTerminee = false;
                etatCuisson = CONSIGNE_EN_COURS;
                menuEnCours = 0;
              } else if (prevMenuOption == T_DOWN) { //Option Menu Principal
                lcd.setCursor(0, 0);
                lcd.print(">Cuisson Biscuit");
                lcd.setCursor(0, 1);
                lcd.print(" Consigne Fixe");
                etatCuisson = INITIAL;
                //menuEnCours = 0;
              } else {
                lcd.setCursor(0, 0);
                lcd.print("Reprise cuisson");
                stopCuisson = false;
                cuissonTerminee = false;
                etatCuisson = CONSIGNE_EN_COURS;
                menuEnCours = 0;
              }
              break;  
        }
        break;        
      default:
        break;  
        
    }
    // update the previous menu option
    prevMenuOption = menuOption;
  }
 }
 
/*****************************************
 * Boucle principale
 */
void loop() {
  checkMenu();
  if (menuEnCours != 1) {
   if (etatCuisson == EN_COURS || etatCuisson == STOP || etatCuisson == CONSIGNE_EN_COURS || etatCuisson == STOP_CONSIGNE) {
    // Lecture de l'horloge interne en ms
    Temps_ms=millis();  // 2^32 secondes = 49.71 jours  
    // Calcul des secondes  
    t = Temps_ms/TSec;
   
    dureePhase = t-tInit;
    Num_sec= t%60;
    
    // Calcul des minutes  
    Num_min= (Temps_ms/(TSec*60))%60;
    // Calcul des heures  
    Num_heur= (Temps_ms/(TSec*3600))%60;
  
    Input = readTemp(maxthermo, Input);
    if (initialisation == true) {
      consigneInitiale = Input;
      initialisation = false;
    }
    if (cuissonTerminee == false && stopCuisson == false) {
      // calcul de la consigne
      if (etatCuisson == EN_COURS) {
        consigne = calculeConsigne(phaseEnCours, dureePhase);
      } else {
        consigne = consigneCustomTemp;
      }
      // affichage température
      lcd.clear();
      if (Num_sec%10 > 5) {
        // affichage temps écoulé    
        displayTime(Num_heur, Num_min, Num_sec); 
        displayTemperature(Input, consigne);   
      } else {
        // affichage temps écoulé       
        if (etatCuisson == EN_COURS) {
          displayPhase(phaseEnCours,dureePhase);
          displayGoal(Input, courbe[typeCuisson][phaseEnCours].t1);
        } else {
          displayGoal(Input, consigneCustomTemp);
          displayTime(Num_heur, Num_min, Num_sec); 
        }
        
      }
       
      if (cycleTermine == true) {
        cycleTermine = false; 
        Serial.println();
        Serial.print(erreur);Serial.print(";");
        Serial.print(CorrectionP);Serial.print(";");
        Serial.print(integration);Serial.print(";");
        Serial.print(derivation);Serial.print(";");
        Serial.print(commande);Serial.print(";");
        Serial.print(Input);Serial.print(";");
        Serial.print(ratio);Serial.print(";");
      }
      if (etatCuisson == EN_COURS) {
        if(changePhase(consigne,Input, phaseEnCours, dureePhase, tDecalePhase)) {
          tInit=t;
          tDecalePhase=0;
          phaseEnCours++;
          if (phaseEnCours >= NB_PHASES) {
            cuissonTerminee = true;
          }
        }
      }
    } else {
      lcd.clear();
      displayTemperature(Input, consigne);
      displayFinCuisson();
      ratio=0;
      if (cycleTermine == true) {
        cycleTermine = false; 
        Serial.println();
        Serial.print(";");
        Serial.print(";");
        Serial.print(";");
        Serial.print(";");
        Serial.print(";");
        Serial.print(Input);Serial.print(";");
        Serial.print(ratio);Serial.print(";");
      }
    }
   }
  }
}
