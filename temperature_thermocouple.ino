#include <LiquidCrystal.h>

#include <Adafruit_MAX31856.h>
// Use software SPI: CS, DI, DO, CLK
//Adafruit_MAX31856 maxthermo = Adafruit_MAX31856(10, 11, 12, 13);
// use hardware SPI, just pass in the CS pin
Adafruit_MAX31856 maxthermo = Adafruit_MAX31856(10);
// use hardware SPI, pass in the CS pin and using SPI1
//Adafruit_MAX31856 maxthermo = Adafruit_MAX31856(10, &SPI1);

const int L1 = 2; // commande relais / LED
const int rs = 7, en = 8, d4 = 6, d5 = 5, d6 = 4, d7 = 3;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

// Données des mesures
typedef struct {
  float temperature;
} Mesure;

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
const int NB_PHASES =6;

// constantes de correction
double Kproportionel = 6; //2;
float Kintegral = 0.0006; // 0.00038
double Kderive = 1000; // 10000
double dt = 1; // période échantillonage = 1s
double integration = 0.;
double derivation = 0.;
double erreur_n_1 =0;
// constantes de système
double amplitude = 1100; // puissance electrique
int nbEchantillons = 120; // base de codage pour un valeur de consigne


segment courbe[NB_PHASES] = {
  // pentes de chauffe en °C / heure
  {  0, 100,  80,  0},   // 0 à 95° à 80°C/heure 
  {100, 100,   0, 10},   // 95 à 105° pendant 10 minutes
  {100, 400,  80, 0},    // 105 à 400° à 80°C/heure
  {400, 400,   0, 15},    // 400° pendant 15 min 
//  {400, 1030,150, 0},   // 400 à 1030° à 150°C/heure
//  {1030,1030,  0, 30},  // 1030° pendant 30 min
};
int phaseEnCours;
boolean initialisation;
float consigneInitiale = 20; // consigne initiale
float dureePhase;
float tInit; // temps à l'ilit de la phase
float tDecalePhase; //décalage de phase (température non atteinte)
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
  return erreur * Kproportionel;
}

float correctionIntegral(float erreur, float integration) {
  return Kintegral * dt * erreur + integration;
}

float correctionDerive(float erreur_n_1, float erreur) {
  return (erreur - erreur_n_1) * Kderive * dt / (nbEchantillons);
}
/* calcul de la consigne
 *  phase index de la phase
 *  t en seconde 
*/
float calculeConsigne(int phase, float t) {
  if (courbe[phase].pente == 0) {
    return courbe[phase].t0;  
  } else {
    if(phase == 0) {
      return courbe[phase].pente*t/3600 + consigneInitiale;
    }
    return courbe[phase].pente*t/3600 + courbe[phase].t0;
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
boolean changePhase(float temperature, int phase, float t, float &tDecalage) {
  if (courbe[phase].pente == 0) {
    // si on a pas atteint la temperature
    if (temperature < courbe[phase].t0) {
      tDecalage = t;
      return false;
    }
    return t-tDecalage > courbe[phase].duree*60;  
  } else {
    return courbe[phase].t1 < temperature;
  }
}
/* Affichage horloge */ 
void displayTime(unsigned long heure, unsigned long minutes,unsigned long secondes) {
  lcd.setCursor(0,1);
  lcd.print("Time : ");
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
  lcd.clear();  
  lcd.setCursor(0,0);
  lcd.print(temp); 
  lcd.write((byte)0);
  lcd.setCursor(10,0);
  lcd.print(consigne); 
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
  phaseEnCours=0;
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
  Serial.println("erreur;correctionP;Intégration;Derivation;commande;mesure");
}

byte varCompteurTimer = 0; // La variable compteur
byte compteurEchantillon = 0;
float ratio;


// Routine d'interruption
ISR(TIMER2_OVF_vect) {
  TCNT2 = 256 - 250; // 250 x 16 µS = 4 ms
  if (varCompteurTimer++ > 250) { // 500 * 4 ms = 1000 ms 
    varCompteurTimer = 0;
    
    if (compteurEchantillon++ > nbEchantillons) {
      compteurEchantillon = 0;
      cycleTermine = true;
    }
    if(compteurEchantillon < ratio && cuissonTerminee == false) {
        digitalWrite(L1, HIGH); //allumer L1 
      } else {
        digitalWrite(L1, LOW); //éteindre L1
      }
    Serial.print(".");
//    Serial.print("compteurEchantillon ");
//    Serial.println(compteurEchantillon);
  }
}
 
/*****************************************
 * Boucle principale
 */
void loop() {
  double consigne;
  double erreur;
  Mesure mesure;
  double CorrectionP;
  double commande;
 
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

  mesure.temperature = readTemp(maxthermo, mesure.temperature);

  if (cuissonTerminee == false) {
    // calcul de la consigne
    consigne = calculeConsigne(phaseEnCours, dureePhase);
    // affichage température
    displayTemperature(mesure.temperature, consigne);
    if (Num_sec%10 > 5) {
      // affichage temps écoulé    
      displayTime(Num_heur, Num_min, Num_sec);    
    } else {
      // affichage temps écoulé
      displayPhase(phaseEnCours,dureePhase);
    }
     
    // calcul de la commande
    if (cycleTermine == true) {
      erreur = calculeErreur(consigne,mesure.temperature);
      derivation = correctionDerive(erreur_n_1, erreur); 
      erreur_n_1 = erreur;
      CorrectionP = correctionProportionnelle(calculeErreur(consigne,mesure.temperature));   
      integration = correctionIntegral(erreur, integration);
      commande = integration + CorrectionP + derivation;
      ratio = calculRatio(commande, amplitude, nbEchantillons); 
      cycleTermine = false; 
      Serial.println();
      Serial.print(erreur);Serial.print(";");
      Serial.print(CorrectionP);Serial.print(";");
      Serial.print(integration);Serial.print(";");
      Serial.print(derivation);Serial.print(";");
      Serial.print(commande);Serial.print(";");
      Serial.println(mesure.temperature);
    }
    
    if(changePhase(consigne,phaseEnCours, dureePhase, tDecalePhase)) {
      tInit=t;
      tDecalePhase=0;
      phaseEnCours++;
      if (phaseEnCours >= NB_PHASES) {
        cuissonTerminee = true;
      }
    }
  } else {
    displayTemperature(mesure.temperature, consigne);
    displayFinCuisson();
    //consigne=0;//
    ratio=0;
    Serial.print("Fin de cuisson ; Temperature: ");
    Serial.println(mesure.temperature);
  }
  
}
