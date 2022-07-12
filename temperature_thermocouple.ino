#include <LiquidCrystal.h>

#include <Adafruit_MAX31856.h>
// Use software SPI: CS, DI, DO, CLK
//Adafruit_MAX31856 maxthermo = Adafruit_MAX31856(10, 11, 12, 13);
// use hardware SPI, just pass in the CS pin
Adafruit_MAX31856 maxthermo = Adafruit_MAX31856(10);
// use hardware SPI, pass in the CS pin and using SPI1
//Adafruit_MAX31856 maxthermo = Adafruit_MAX31856(10, &SPI1);
double pulse,frequency,capacitance,inductance;

const int rs = 7, en = 8, d4 = 6, d5 = 5, d6 = 4, d7 = 3;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

// déclaration de fonctions
void displayTime(unsigned long heure, unsigned long minutes,unsigned long secondes);
void displayTemperature(float temp);
void displayPhase(int phase, unsigned long duree);
void displayFinCuisson();
boolean marche(float consigne, float mesure); // chauffe on/off en fonction de la consigne et de la température mesurée
float setConsigne(int phase);
boolean changePhase(float temperature, int phase, float t);

// Données des mesures
typedef struct {
  int profondeur;  // profondeur mesurée
  int volume;      // volume mesuré
  int sonde;       // valeur renvoyée par la sonde
  int etalon;      // mesure étalon
  float tension;
} MyData;

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
boolean cuissonTerminee;
const int NB_PHASES =6;
segment courbe[NB_PHASES] = {
  // pentes de chauffe en °C / heure
  {  0, 100,  80,  0},   // 0 à 95° à 80°C/heure 
  {100, 100,   0, 10},   // 95 à 105° pendant 10 minutes
  {100, 400,  80, 0},    // 105 à 400° à 80°C/heure
  {400, 400,   0, 15},    // 400° pendant 15 min 
  {400, 1030,150, 0},   // 400 à 1030° à 150°C/heure
  {1030,1030,  0, 30},  // 1030° pendant 30 min
};
int phaseEnCours;
boolean initialisation;
float temperatureInitiale = 0;
float dureePhase;
float tInit; // temps à l'ilit de la phase
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

// fonctions
/* consigne température */
boolean marche(float consigne, float mesure) {
  return consigne < mesure;
}
/* calcul de la consigne
 *  phase index de la phase
 *  t en seconde 
*/
float setConsigne(int phase, float t) {
  if (courbe[phase].pente == 0) {
    return courbe[phase].t0;  
  } else {
    if(phase == 0) {
      return courbe[phase].pente*t/3600 + temperatureInitiale;
    }
    return courbe[phase].pente*t/3600 + courbe[phase].t0;
  }
}
/** 
 *  Calcul du changement de phase 
 *  temperature : température de consigne
 *  phase : index de la phase
 *  t : durée écoulée de la phase
 */
boolean changePhase(float temperature, int phase, float t) {
  if (courbe[phase].pente == 0) {
    return courbe[phase].duree*60 < t;  
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

void setup() {
  pinMode(11,INPUT);
  pinMode(13,OUTPUT);
  
  Serial.begin(9600);
  //init LCD
  lcd.begin(16, 2);
  lcd.createChar(0, degre);
  
  if (!maxthermo.begin()) {
    Serial.println("Could not initialize thermocouple.");
    while (1) delay(10);
  }
   maxthermo.setThermocoupleType(MAX31856_TCTYPE_K);
   Serial.print("Thermocouple type: ");
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
  tInit=0;
}



void loop() {
  float consigne;
  MyData message;
  maxthermo.triggerOneShot();
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
  
  delay(250);
  
  if (maxthermo.conversionComplete()) {
    message.tension = maxthermo.readThermocoupleTemperature();
    if(initialisation == true) {
      initialisation = false;
      tInit = millis()/TSec;
      temperatureInitiale = message.tension;
      Serial.print("Temperature initiale: ");
      Serial.println(temperatureInitiale);
    }
//    Serial.println(message.tension);  
  } else {
    Serial.println("Conversion not complete!");
  } 

  if (cuissonTerminee == false) {
    // calcul de la consigne
    consigne = setConsigne(phaseEnCours, dureePhase);

    // affichage température
    displayTemperature(message.tension, consigne);
    if (Num_sec%10 > 5) {
      // affichage temps écoulé    
      displayTime(Num_heur, Num_min, Num_sec);    
    } else {
      // affichage temps écoulé
      displayPhase(phaseEnCours,dureePhase);
    }
    Serial.print("température consigne: "); 
    Serial.println(consigne); 
    if(changePhase(consigne,phaseEnCours, dureePhase)) {
      tInit=t;
      phaseEnCours++;
      if (phaseEnCours >= NB_PHASES) {
        cuissonTerminee = true;
      }
    }
  } else {
    displayTemperature(message.tension, consigne);
    displayFinCuisson();
    consigne=0;
  }
   delay(250);
}
