
#include <Arduino.h>

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