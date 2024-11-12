#include <math.h>
#include <Wire.h>
#include "rgb_lcd.h"

const int ROTARY_ANGLE_SENSOR = A1;
const int MANUAL_TEMP_SENSOR = A3;
const float ADC_REF = 5.0; //reference voltage of ADC is 5v.If the Vcc switch on the seeeduino
                    //board switches to 3V3, the ADC_REF should be 3.3
const float GROVE_VCC = 5.0; //VCC of the grove interface is normally 5v
const float FULL_ANGLE = 300.0; //full value of the rotary angle is 300 degrees

const float B = 4275;               // B value of the thermistor
const float R0 = 100000;            // R0 = 100k
const int pinTempSensor = A0;     // Grove - Temperature Sensor connect to A0

rgb_lcd lcd;

const int colorR = 0;
const int colorG = 255;
const int colorB = 0;

#if defined(ARDUINO_ARCH_AVR)
#define debug  Serial
#elif defined(ARDUINO_ARCH_SAMD) ||  defined(ARDUINO_ARCH_SAM)
#define debug  SerialUSB
#else
#define debug  Serial
#endif

void setup()
{
    Serial.begin(9600);

    pinMode(MANUAL_TEMP_SENSOR, INPUT);

    // set up the LCD's number of columns and rows:
    lcd.begin(16, 2);
    
    lcd.setRGB(colorR, colorG, colorB);

}

void loop()
{
    // Temperature reading
    int a = analogRead(pinTempSensor);

    float R = 1023.0 / a - 1.0;
    R = R0 * R;

    float temperature = 1.0 / (log(R / R0) / B + 1 / 298.15) - 273.15; // Convert to temperature via datasheet

    // manual breadboard sensor temperature reading
    float voltage;
    int sensor_value = analogRead(MANUAL_TEMP_SENSOR);
    voltage = (float)sensor_value * ADC_REF / 1023.0;

    // Send temperature and target to serial port on the same line separated by a tab
    Serial.print(temperature);
    Serial.print(",");
    Serial.println(voltage);

    // Display both values on the LCD
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Temp: ");
    lcd.print(temperature);
    lcd.print(" C");

    lcd.setCursor(0, 1);
    lcd.print("Voltage: ");
    lcd.print(voltage);
    lcd.print(" V");

    delay(100);
}
