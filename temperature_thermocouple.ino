//#include <VirtualWire.h>
//#include <VirtualWire_Config.h>
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


void setup() {
  pinMode(11,INPUT);
  pinMode(13,OUTPUT);
  //vw_set_rx_pin(12);//connect the receiver data pin to pin 12
  //vw_setup(4000);  // speed of data transfer in bps, maxes out at 10000
  //vw_rx_start();       // Start the receiver PLL running
  Serial.begin(9600);
  lcd.begin(16, 2);
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
  lcd.setCursor(15,1);
  lcd.print("_");
  delay(250);
  byte taille_message = sizeof(MyData);
   
  if (maxthermo.conversionComplete()) {
    message.tension = maxthermo.readThermocoupleTemperature();
    Serial.println(message.tension);  
  } else {
    Serial.println("Conversion not complete!");
  } 
//  if (vw_get_message((byte *)&message, &taille_message)) // if we get a message that we recognize on this buffer...
//  {
//    Serial.println("------------------------------------");
//    Serial.print("Profondeur:  ");Serial.print(message.profondeur);Serial.println(" cm");
//    Serial.print("Volume:      ");Serial.print(message.volume);Serial.println(" litres");
//    Serial.print("Valeur Sonde:");Serial.println(message.sonde);
//    Serial.print("Etalon:      ");Serial.println(message.etalon);
//    Serial.print("Tension:     ");Serial.print(message.tension);Serial.println(" v");
//    lcd.clear();  
//    lcd.setCursor(0,0);
//    lcd.print("Profondeur: ");
//    lcd.setCursor(12,0);
//    lcd.print(message.profondeur); 
//    lcd.setCursor(0,1);
//    lcd.print("Volume: ");
//    lcd.setCursor(8,1);
//    lcd.print(message.volume);
//   }
//   else {
    lcd.clear();  
    lcd.setCursor(0,0);
    lcd.print("Temp.: ");
    lcd.setCursor(11,0);
    lcd.print(message.tension); 
//    Serial.println("Pas de signaml recu");
//    lcd.setCursor(15,1);
//    lcd.print(".");
//   }
   delay(250);
}
