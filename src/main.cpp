#include <Arduino.h>

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
  String mode = "OFF";

public:
  LEDBlinker(int pin, unsigned long interval) : Updater(interval), ledPin(pin)
  {
    pinMode(ledPin, OUTPUT);
  };

  // Implementación específica de la actualización para el LEDBlinker
  void performUpdate() override
  {
    if (mode == "OFF")
    {
      digitalWrite(ledPin, LOW);
      return;
    }
    if (mode == "ON")
    {
      digitalWrite(ledPin, HIGH);
      return;
    }
    if (mode == "STANDBY")
    {
      digitalWrite(ledPin, !digitalRead(ledPin)); // Cambia el estado del LED
      return;
    }
  }
  void changeMode(String newMode)
  {
    mode = newMode;
  }
};

// ALARM

class Alarm : public Updater
{
private:
  String mode = "OFF";
  bool alarmState;
  bool isActivated = false;
  const unsigned long standByTime = 3000;
  unsigned long activationTime = 0;

public:
  Alarm(unsigned long interval) : Updater(interval), alarmState(false) {}

  void performUpdate() override
  {
    if (activationTime == 0)
      return;
    if (millis() - activationTime > standByTime)
    {
      activationTime = 0;
      mode = "ON";
      return;
    }
    // alarmState = !alarmState;
  }
  String getAlarmMode()
  {
    return mode;
  }
  bool getAlarmState()
  {
    return alarmState;
  }

  void alarm()
  {
    if (isActivated == false)
      return;
    if (!alarmState)
    {
      mode = "ALARM";
      Serial.println("HA ENTRADO ALGUIEN");
      Serial.println("ALARMA SONANDO - EVIA UN EMAIL");
      alarmState = true;
      return;
    }
    alarmState = true;
    Serial.println("ALARMA SONANDO");
  }
  void deactivate()
  {
    alarmState = false;
    isActivated = false;
    activationTime = 0;
    mode = "OFF";
    Serial.println("DESACTIVAR");
  }
  void activate()
  {
    isActivated = true;
    activationTime = millis();
    mode = "STANDBY";
    Serial.println("ACTIVAR ALARMA");
  }
  bool isActivatedAlarm()
  {
    return isActivated;
  }
};

// SENSOR DE MOVIMIENTO

class MotionSensor : public Updater
{
private:
  bool isActivated = false;
  int motionSensorPin;
  bool motionState;

public:
  MotionSensor(int pin, unsigned long interval) : Updater(interval), motionSensorPin(pin), motionState(false)
  {
    pinMode(motionSensorPin, INPUT);
  }

  void performUpdate() override
  {
    setMotionState(!getMotionState());
  }

  bool getMotionState()
  {
    return motionState;
  }
  void setMotionState(bool state)
  {
    motionState = state;
    if (motionState)
      Serial.println("DETECTADO MOVIMIENTO");
    if (!motionState)
      Serial.println("NO HAY MOVIMIENTO");
  }
  // void activate()
  // {
  //   isActivated = true;
  //   Serial.println("ACTIVAR SENSOR MOVIMIENTO");
  // }
  // void deactivate()
  // {
  //   isActivated = false;
  //   Serial.println("DESACTIVAR SENSOR MOVIMIENTO");
  // }
};

class Buzzer : public Updater
{
private:
  int pin;
  String mode = "OFF";

public:
  Buzzer(int pin, unsigned long interval) : Updater(interval), pin(pin)
  {
    pinMode(pin, OUTPUT);
  };

  void performUpdate() override
  {
    if (mode == "OFF" || mode == "ON")
    {
      noTone(pin);
      return;
    }
    if (mode == "ALARM")
    {
      tone(pin, 1000);
      return;
    }
    if (mode == "STANDBY")
    {
      tone(pin, 500, 250);
      return;
    }
  }
  void changeMode(String newMode)
  {

    mode = newMode;
  }
};

int globalInterval = 100;

LEDBlinker ledBlinker(13, 300); // Crea un objeto LEDBlinker que parpadea cada 1000 ms
Alarm alarma(1000);
MotionSensor motionSensor(2, 1000);
Buzzer buzzer(8, 1000);

String mode = "OFF"; // Modo del sistema OFF, ON, BLINK
void setup()
{
  Serial.begin(9600);
  pinMode(5, INPUT_PULLUP); // Botón de activación de la alarma
  pinMode(12, INPUT);       // Pin de prueba de movimiento
  pinMode(6, INPUT_PULLUP); // Botón de desactivación de la alarma
  delay(1000);
  // Configuración inicial si es necesaria
}

void loop()
{
  bool activateAlarma = !digitalRead(5);
  bool desactivateAlarma = !digitalRead(6);
  bool motionFaker = digitalRead(12);

  //  Activa la alarma
  if (activateAlarma)
  {
    mode = "STANDBY";
    alarma.activate();
  }
  // Desactiva la alarma
  if (desactivateAlarma)
  {
    mode = "OFF";
    alarma.deactivate();
  }
  // Salta el detector de movimiento
  if (motionFaker)
  {
    motionSensor.update();
    if (mode == "ON")
    {
      mode = "ALARM";
      alarma.alarm();
    };
  }
  alarma.update();
  buzzer.changeMode(mode);      // Cambia el modo del BUZZER
  ledBlinker.changeMode(mode);  // Cambia el modo del LEDBlinker
  buzzer.update();              // Actualiza el estado del BUZZER
  ledBlinker.update();          // Actualiza el estado del LED
  mode = alarma.getAlarmMode(); // Actualiza el modo del sistema despues de tiempo de espera
}
