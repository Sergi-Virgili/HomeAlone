#include <Arduino.h>
//
class Updater
{
protected:
  unsigned long previousMillis;
  const unsigned long interval;

public:
  Updater(unsigned long interval) : previousMillis(0), interval(interval) {}

  virtual void update()
  {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval)
    {
      previousMillis = currentMillis;
      performUpdate(); // Llama a la función que se debe implementar en la clase derivada
    }
  }

  // Función virtual pura que las clases derivadas deben implementar
  virtual void performUpdate() = 0;
};

class LEDBlinker : public Updater
{
private:
  int ledPin;

public:
  LEDBlinker(int pin, unsigned long interval) : Updater(interval), ledPin(pin)
  {
    pinMode(ledPin, OUTPUT);
  }

  // Implementación específica de la actualización para el LEDBlinker
  void performUpdate() override
  {
    digitalWrite(ledPin, !digitalRead(ledPin)); // Cambia el estado del LED
  }
};

// ALARM

class Alarm : public Updater
{
private:
  bool alarmState;
  bool isActivated = true;

public:
  Alarm(unsigned long interval) : Updater(interval), alarmState(false) {}

  void performUpdate() override
  {
    // alarmState = !alarmState;
  }

  bool getAlarmState()
  {
    return alarmState;
  }

  void alarm()
  {
    if (!isActivated)
      return;
    if (!alarmState)
    {
      Serial.println("ALARMA SONANDO - EVIA UN EMAIL");
    }
    alarmState = true;
  }
};

// SENSOR DE MOVIMIENTO

class MotionSensor : public Updater
{
private:
  int motionSensorPin;
  bool motionState;

public:
  MotionSensor(int pin, unsigned long interval) : Updater(interval), motionSensorPin(pin), motionState(true)
  {
    pinMode(motionSensorPin, INPUT);
  }

  void performUpdate() override
  {
    motionState = digitalRead(motionSensorPin);
  }

  bool getMotionState()
  {
    return motionState;
  }
};

LEDBlinker ledBlinker(13, 300); // Crea un objeto LEDBlinker que parpadea cada 1000 ms
Alarm alarm(1000);
MotionSensor motionSensor(2, 1000);

void setup()
{
  Serial.begin(9600);
  // Configuración inicial si es necesaria
}

void loop()
{
  ledBlinker.update(); // Actualiza el estado del LED
  // Aquí puedes actualizar otros objetos que hereden de Updater
  Serial.println(alarm.getAlarmState());
  if (motionSensor.getMotionState())
  {
    alarm.alarm();
  }

  alarm.update();
}
