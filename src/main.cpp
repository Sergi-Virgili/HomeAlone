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
    updateFunctions[static_cast<int>(Modes::ALARM)] = &LEDBlinker::toggle;
  };

  // Implementación específica de la actualización para el LEDBlinker
  void performUpdate() override
  {
    // Llama a la función correspondiente al modo actual
    updateFunctions[static_cast<int>(mode)](ledPin);
  }
  // Funciones estáticas para actualizar el LED
  static void turnOff(int pin)
  {
    digitalWrite(pin, LOW);
  }
  static void turnOn(int pin)
  {
    digitalWrite(pin, HIGH);
  }
  static void toggle(int pin)
  {
    digitalWrite(pin, !digitalRead(pin));
  }
  void changeMode(Modes newMode)
  {
    mode = newMode;
  }
};
// Inicializa el arreglo de punteros a función para actualizar el LED en cada modo del sistema
void (*LEDBlinker::updateFunctions[static_cast<int>(Modes::NUM_MODES)])(int) = {nullptr};
// ALARM

class Alarm : public Updater
{
private:
  Modes mode = OFF;
  bool isRunning;
  bool isActivated = false;
  const unsigned long standByTime = ALARM_STANDBY_TIME;
  unsigned long activationTime = 0;

public:
  Alarm(unsigned long interval) : Updater(interval), isRunning(false) {}

  void performUpdate() override
  {
    if (activationTime == 0)
      return;
    if (millis() - activationTime > standByTime)
    {
      activationTime = 0;
      mode = Modes::ON;
      return;
    }
  }
  Modes getAlarmMode()
  {
    return mode;
  }
  bool getAlarmState()
  {
    return isRunning;
  }

  void alarm()
  {
    if (isActivated == false)
      return;
    if (!isRunning)
    {
      mode = Modes::ALARM;
      Serial.println("HA ENTRADO ALGUIEN");
      Serial.println("ALARMA SONANDO - EVIA UN EMAIL");
      isRunning = true;
      return;
    }
    isRunning = true;
    Serial.println("ALARMA SONANDO");
  }
  void deactivate()
  {
    isRunning = false;
    isActivated = false;
    activationTime = 0;
    mode = Modes::OFF;
    Serial.println("DESACTIVAR");
  }
  void activate()
  {
    isActivated = true;
    activationTime = millis();
    mode = Modes::STANDBY;
    Serial.println("ACTIVAR ALARMA");
  }
};

// SENSOR DE MOVIMIENTO

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

class Buzzer : public Updater
{
private:
  int pin;
  Modes mode = OFF;
  ToneSetting toneSettings[NUM_MODES];

public:
  Buzzer(int pin, unsigned long interval) : Updater(interval), pin(pin)
  {
    pinMode(pin, OUTPUT);
    // Configura los ajustes de tono para cada modo
    toneSettings[OFF] = {0, 0};
    toneSettings[ON] = {0, 0}; // Ajustar si necesario
    toneSettings[ALARM] = {1000, 0};
    toneSettings[STANDBY] = {500, 250};
  };

  void performUpdate() override
  {
    ToneSetting setting = toneSettings[mode];
    if (setting.frequency == 0)
    {
      noTone(pin);
    }
    else
    {
      tone(pin, setting.frequency, setting.duration);
    }
  }
  void changeMode(Modes newMode)
  {
    mode = newMode;
  }
};

int globalInterval = 100;

LEDBlinker ledBlinker(LED_PIN, 300); // Crea un objeto LEDBlinker que se actualizable cada 300 ms
Alarm alarma(1000);                  // Crea un objeto Alarm que se actualizable cada 1 segundo
MotionSensor motionSensor(MOTION_SENSOR_PIN);
Buzzer buzzer(BUZZER_PIN, 1000);

Modes mode = Modes::OFF; // Modo del sistema OFF, ON, BLINK
void setup()
{
  Serial.begin(9600);       // Inicializa el puerto serial
  pinMode(5, INPUT_PULLUP); // Botón de activación de la alarma
  pinMode(6, INPUT_PULLUP); // Botón de desactivación de la alarma

  delay(1000); // Espera 1 segundo antes de empezar el loop
}

void loop()
{
  bool activateAlarma = !digitalRead(5);    // Botón de activación de la alarma cambiar por boton del teclado
  bool desactivateAlarma = !digitalRead(6); // Botón de desactivación de la alarma cambiar el teclado numerico

  Serial.println(mode);
  //  Activa la alarma
  if (activateAlarma)
  {
    // mode = "STANDBY";
    alarma.activate();
  }
  // Desactiva la alarma
  if (desactivateAlarma)
  {
    // mode = "OFF";
    alarma.deactivate();
  }

  // Si la alarma está activada y se detecta movimiento
  if (mode == Modes::ON && motionSensor.motionDetected())
  {
    // mode = "ALARM";
    alarma.alarm();
  }

  alarma.update();              // Actualiza el estado de la alarma empezando por el tiempo de espera
  buzzer.changeMode(mode);      // Cambia el modo del BUZZER
  ledBlinker.changeMode(mode);  // Cambia el modo del LEDBlinker
  buzzer.update();              // Actualiza el estado del BUZZER
  ledBlinker.update();          // Actualiza el estado del LED
  mode = alarma.getAlarmMode(); // Actualiza el modo del sistema despues de tiempo de espera
}
