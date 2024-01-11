#include <Arduino.h>

// Configuración
#define LED_PIN 13
#define BUZZER_PIN 8
#define MOTION_SENSOR_PIN 12
#define ALARM_STANDBY_TIME 3000

// Modos del sistema
enum Modes
{
  OFF,
  ON,
  ALARM,
  STANDBY,
  TIME,
  NUM_MODES
};

// Definicioón de la estructura de tono del Buzzer
struct ToneSetting
{
  int frequency;
  unsigned long duration;
};

class Updater
{
protected:
  const unsigned long interval;
  unsigned long previousMillis;

public:
  Updater(unsigned long interval) : interval(interval), previousMillis(0) {}

  void update()
  {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval)
    {
      previousMillis = currentMillis;
      performUpdate();
    }
  }
  virtual void performUpdate() = 0;
};

// LEDBlinker
// Definición de la clase LEDBlinker que hereda de Updater
// La clase LEDBlinker se encarga de manejar el estado del LED según el modo del sistema.
class LEDBlinker : public Updater
{
private:
  int ledPin;
  Modes mode = Modes::OFF;
  static void (*updateFunctions[static_cast<int>(Modes::NUM_MODES)])(int); // Array de punteros a función para actualizar el LED

public:
  LEDBlinker(int pin, unsigned long interval) : Updater(interval), ledPin(pin)
  {
    pinMode(ledPin, OUTPUT);
    // Asigna funciones a cada modo
    updateFunctions[static_cast<int>(Modes::OFF)] = &LEDBlinker::turnOff;
    updateFunctions[static_cast<int>(Modes::ON)] = &LEDBlinker::turnOn;
    updateFunctions[static_cast<int>(Modes::STANDBY)] = &LEDBlinker::toggle;
    updateFunctions[static_cast<int>(Modes::TIME)] = &LEDBlinker::toggle;
    updateFunctions[static_cast<int>(Modes::ALARM)] = &LEDBlinker::toggle;
  };

  // Implementación específica de la actualización para el LEDBlinker
  void performUpdate() override
  {
    // Llama a la función correspondiente al modo actual
    updateFunctions[static_cast<int>(mode)](ledPin);
  }
  // Funciones estáticas para actualizar el LED
  static void turnOff(int pin) // Función estática para apagar el LED
  {
    digitalWrite(pin, LOW); // Apaga el LED
  }
  static void turnOn(int pin) // Función estática para encender el LED
  {
    digitalWrite(pin, HIGH); // Enciende el LED
  }
  static void toggle(int pin) // Función estática para cambiar el estado del LED
  {
    digitalWrite(pin, !digitalRead(pin)); // Cambia el estado del LED
  }
  void changeMode(Modes newMode) // Cambia el modo del LEDBlinker
  {
    mode = newMode; // Cambia el modo
  }
};
// Inicializa el arreglo de punteros a función para actualizar el LED en cada modo del sistema
void (*LEDBlinker::updateFunctions[static_cast<int>(Modes::NUM_MODES)])(int) = {nullptr};
class Buzzer : public Updater
{
private:
  int pin;                             // Pin del buzzer
  Modes mode = Modes::OFF;             // Modo del sistema
  ToneSetting toneSettings[NUM_MODES]; // Arreglo de ajustes de tono para cada modo

public:
  Buzzer(int pin, unsigned long interval) : Updater(interval), pin(pin) // Constructor
  {
    pinMode(pin, OUTPUT); // Configura el pin del buzzer como salida
    // Configura los ajustes de tono para cada modo
    toneSettings[OFF] = {0, 0};         // Frecuencia y duración del tono para el modo OFF
    toneSettings[ON] = {0, 0};          // Frecuencia y duración del tono para el modo ON
    toneSettings[ALARM] = {1000, 0};    // Frecuencia y duración del tono para el modo ALARM
    toneSettings[STANDBY] = {500, 250}; // Frecuencia y duración del tono para el modo STANDBY
    toneSettings[TIME] = {500, 250};    // Frecuencia y duración del tono para el modo DEACTIVATE_TIME
  };

  void performUpdate() override // Actualiza el estado del buzzer
  {
    ToneSetting setting = toneSettings[mode]; // Obtiene los ajustes de tono para el modo actual
    if (setting.frequency == 0)               // Si la frecuencia es 0
    {
      noTone(pin); // Apaga el buzzer
    }
    else // Si la frecuencia no es 0
    {
      tone(pin, setting.frequency, setting.duration); // Enciende el buzzer con los ajustes de tono
    }
  }
  void changeMode(Modes newMode) // Cambia el modo del buzzer
  {
    mode = newMode; // Cambia el modo
  }

  static void beep()
  {
    tone(BUZZER_PIN, 800, 200);
    delay(100);
  }
  static void tripleBeep()
  {
    delay(100);
    for (int i = 0; i < 3; i++)
    {
      tone(BUZZER_PIN, 1000, 100);
      delay(200);
    }
  }
};
// ALARM
// Definición de la clase Alarm que hereda de Updater
// La clase Alarm se encarga de manejar el estado de la alarma
// y de cambiar el modo del sistema.
class Alarm : public Updater
{
private:
  Modes mode = Modes::OFF;                              // Modo de la alarma
  bool isRunning = false;                               // Estado de la alarma
  bool isActivated = false;                             // Estado de la activación de la alarma
  const unsigned long standByTime = ALARM_STANDBY_TIME; // Tiempo de espera para activar la alarma
  unsigned long activationTime = 0;                     // Tiempo en el que se ha activado la alarma (en millis)

public:
  Alarm(unsigned long interval) : Updater(interval), isRunning(false) {}

  void performUpdate() override // Actualiza el estado de la alarma
  {
    Serial.println("ALARMA UPDATE");

    if (activationTime == 0) // Si no se ha activado la alarma
      return;                // No hace nada
                             // Si estamos en STANDBY y el tiempo se ha excedido, cambiamos a DEACTIVATE_TIME
    if (mode == Modes::STANDBY && (millis() - activationTime > standByTime))
    {
      mode = Modes::ON;
      Serial.println("Cambiando a ON");
      // Reiniciar el contador para DEACTIVATE_TIME
      return;
    }
    // Si estamos en DEACTIVATE_TIME y el tiempo se ha excedido, cambiamos a ALARM
    if (mode == Modes::TIME && (millis() - activationTime > standByTime))
    {
      mode = Modes::ALARM;
      Serial.println("ALARMA ACTIVADA: Tiempo de desactivación excedido");
      return;
    }
    // mode = Modes::ON;   // Cambia el modo del sistema a ON
    // return;             // Termina la función
  }
  Modes getAlarmMode() // Devuelve el modo de la alarma
  {
    return mode; // Devuelve el modo
  }
  // bool getAlarmState() // Devuelve el estado de la alarma
  // {
  //   return isRunning; // Devuelve el estado
  // }

  void alarm() // Hace 'Sonar' la alarma (cambia el modo del sistema a ALARM)
  {
    if (isActivated == false) // Si la alarma no está activada
      return;                 // No hace nada
    // if (mode == Modes::ALARM) // Si la alarma ya está sonando
    //   return;                 // No hace nada
    if (mode == Modes::ON)
    {                     // Si la alarma está activada y no está sonando
      mode = Modes::TIME; // Cambia el modo del sistema a DEACTIVATE_TIME
      Serial.println("ALARMA SONANDO - ESPERANDO A QUE SE DESACTIVE !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
      delay(100);
      Serial.println(millis());
      activationTime = millis(); // Guarda tiempo de la activación;
      // isRunning = true;          // Cambia el estado de la alarma a sonando
      return; // Termina la función
    }
    if (!isRunning) // Si la alarma no está sonando
    {
     
      mode = Modes::ALARM; // Cambia el modo del sistema a ALARM
      Serial.println("HA ENTRADO ALGUIEN");
      Serial.println("ALARMA SONANDO - EVIA UN EMAIL MAIILLLLLLLLLLLLLLLLLLL");
      isRunning = true; // Cambia el estado de la alarma a sonando
      return;           // Termina la función
    }
    isRunning = true; // Cambia el estado de la alarma a sonando en los otros casos
    Serial.println("ALARMA SONANDO");
  }
  void deactivate() // Desactiva la alarma
  {
    isRunning = false;   // Cambia el estado de la alarma a apagado
    isActivated = false; // Cambia el estado de la alarma a apagado
    activationTime = 0;  // Reinicia el tiempo de espera
    Buzzer::tripleBeep();
    mode = Modes::OFF; // Cambia el modo del sistema a OFF
    Serial.println("DESACTIVAR");
  }
  void activate() // Activa la alarma
  {
    isActivated = true;        // Cambia el estado de la alarma a activado
    activationTime = millis(); // Guarda tiempo de la activación
    Buzzer::beep();
    mode = Modes::STANDBY; // Cambia el modo del sistema a STANDBY
    Serial.println("ACTIVAR ALARMA");
  }
};

// SENSOR DE MOVIMIENTO
// Definición de la clase MotionSensor que se encarga de leer el estado del sensor de movimiento
// y devolver el estado del mismo.
class MotionSensor
{
private:
  int motionSensorPin;
  bool motionState = digitalRead(motionSensorPin);

public:
  MotionSensor(int pin)
  {
    motionSensorPin = pin;
    pinMode(motionSensorPin, INPUT);
  }

  bool motionDetected()
  {
    motionState = digitalRead(motionSensorPin);
    return motionState;
  }
};

// BUZZER
// Definición de la clase Buzzer que hereda de Updater y se encarga de manejar el estado del buzzer
// según el modo del sistema.

// KEYPAD
// Definición de la clase Keypad que se encarga de leer el estado del teclado numérico
class Keypad
{
private:
  int keypadPin;
  bool keypadState = false;
  bool lastKeypadState = false;
  String password = "AAA";
  String keypadValue = "";

public:
  Keypad(int pin)
  {
    keypadPin = pin;
    pinMode(keypadPin, INPUT_PULLUP);
  }

  bool keypadPressed()
  {
    keypadState = !digitalRead(keypadPin); // Leer el estado actual del botón

    // Comprobar si el botón fue soltado antes de esta presión
    if (keypadState && !lastKeypadState)
    {
      lastKeypadState = keypadState; // Actualizar el estado anterior
      keypadValue += 'A';            // Agregar el valor del botón al valor total
      Serial.println(keypadValue);   // Imprimir el valor total
      Buzzer::beep();                // Hacer sonar el buzzer como beep
      return true;                   // Botón presionado
    }
    else
    {
      lastKeypadState = keypadState; // Actualizar el estado anterior
      return false;                  // Botón no presionado o ya estaba presionado
    }
  }
  bool isPasswordCorrect()
  {
    if (keypadValue == password) // Comprobar si el valor total es igual a la contraseña
    {
      keypadValue = ""; // Reiniciar el valor del teclado

      return true; // Contraseña correcta retorna true
    }
    // Si el valor total es diferente a la contraseña
    return false; // Contraseña incorrecta retorna false
  }
};

// Variables globales

int globalInterval = 100;

LEDBlinker ledBlinker(LED_PIN, 300);          // Crea un objeto LEDBlinker que es actualizable cada 300 ms
Alarm alarma(1000);                           // Crea un objeto Alarm que se actualizable cada 1 segundo
MotionSensor motionSensor(MOTION_SENSOR_PIN); // Crea un objeto MotionSensor
Buzzer buzzer(BUZZER_PIN, 1000);              // Crea un objeto Buzzer que es actualizable cada 1 segundo
Keypad keypad(6);                             // Crea un objeto Keypad

// Modes mode = Modes::OFF; // Modo del sistema inicial establecido en OFF

// SETUP
// Función de configuración que se ejecuta una vez al inicio del programa
void setup()
{
  Serial.begin(9600);       // Inicializa el puerto serial
  pinMode(5, INPUT_PULLUP); // Botón de activación de la alarma
  // pinMode(6, INPUT_PULLUP); // Botón de desactivación de la alarma

  delay(1000); // Espera 1 segundo antes de empezar el loop
}

// LOOP
// Función que se ejecuta continuamente
void loop()
{
  Serial.println(millis());
  Modes mode = alarma.getAlarmMode();    // Actualiza el modo del sistema despues de tiempo de espera
  bool activateAlarma = !digitalRead(5); // Botón de activación de la alarma cambiar por boton del teclado

  Serial.println(mode);
  //  Activa la alarma
  if (activateAlarma)
  {
    // mode = "STANDBY";
    alarma.activate();
  }

  // Si la alarma está activada y se detecta movimiento
  if (mode == Modes::ON && motionSensor.motionDetected())
  {
    // mode = "ALARM";
    alarma.alarm();
  }

  keypad.keypadPressed(); // Botón de desactivación de la alarma cambiar el teclado numerico

  // Si la contraseña es correcta desactiva la alarma
  if (keypad.isPasswordCorrect())
  {
    alarma.deactivate(); // Desactiva la alarma
  }

  alarma.update();             // Actualiza el estado de la alarma empezando por el tiempo de espera
  buzzer.changeMode(mode);     // Cambia el modo del BUZZER
  ledBlinker.changeMode(mode); // Cambia el modo del LEDBlinker
  buzzer.update();             // Actualiza el estado del BUZZER
  ledBlinker.update();         // Actualiza el estado del LED

  Serial.println("End Of Loop");
  // delay(500); // Espera 100 ms antes de volver a ejecutar el loop
}
