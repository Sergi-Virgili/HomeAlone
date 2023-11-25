#include <Arduino.h>
#include "global/Updater.cpp"

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
