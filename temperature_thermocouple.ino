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

// Données des mesures
typedef struct {
  int profondeur;  // profondeur mesurée
  int volume;      // volume mesuré
  int sonde;       // valeur renvoyée par la sonde
  int etalon;      // mesure étalon
  float tension;
} MyData;

// temps
const unsigned long TSec=1000;
unsigned long Num_ms, Num_sec,Num_min; 
unsigned long Num_heur,Num_jour, Temps_ms;

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
}

void loop() {
  MyData message;
  maxthermo.triggerOneShot();
  // Lecture de l'horloge interne en ms
  Temps_ms=millis();  // 2^32 secondes = 49.71 jours  
  // Calcul des secondes  
  Num_sec= (Temps_ms/TSec)%60;
  // Calcul des minutes  
  Num_min= (Temps_ms/(TSec*60))%60;
  // Calcul des heures  
  Num_heur= (Temps_ms/(TSec*3600))%60;

  
  delay(250);
  byte taille_message = sizeof(MyData);
   
  if (maxthermo.conversionComplete()) {
    message.tension = maxthermo.readThermocoupleTemperature();
    Serial.println(message.tension);  
  } else {
    Serial.println("Conversion not complete!");
  } 
// affichage température
    lcd.clear();  
    lcd.setCursor(0,0);
    lcd.write((byte)0);
    lcd.print("C : ");
    lcd.setCursor(11,0);
    lcd.print(message.tension); 
    
// affichage temps écoulé
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
    
   delay(250);
}
