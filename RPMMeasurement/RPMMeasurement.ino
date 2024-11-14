#include <math.h>
#include <Wire.h>

const int ROTARY_ANGLE_SENSOR = A1;
const float ADC_REF = 5.0; //reference voltage of ADC is 5v.If the Vcc switch on the seeeduino
                    //board switches to 3V3, the ADC_REF should be 3.3
const float GROVE_VCC = 5.0; //VCC of the grove interface is normally 5v
const float FULL_ANGLE = 300.0; //full value of the rotary angle is 300 degrees

const float B = 4275;               // B value of the thermistor
const float R0 = 100000;            // R0 = 100k
const int pinTempSensor = A0;     // Grove - Temperature Sensor connect to A0
const int MANUAL_TEMP_SENSOR = A3;
const int FAN_PWM_PIN = 11;
const int FAN_TACHO = A4;
const float MY_VCC = 5.0;
const float MY_B = 4600;
const float lower_threshold_diff = 4.0;

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

    pinMode(ROTARY_ANGLE_SENSOR, INPUT);
    // pinMode(FAN_PWM_PIN, OUTPUT);
    pinMode(FAN_TACHO, INPUT);

    // analogWrite(FAN_PWM_PIN, 0);

}

void loop()
{
    for (int k = 0; k < 256; k++) {

    analogWrite(FAN_PWM_PIN, k);
    int j = 0;
    bool state = true;
    unsigned long starttime = micros();
    
    for (int i = 0; i < 5000; i++) {
      // My own temp sensor
      float temp_voltage;
      int temp_sensor_value = analogRead(MANUAL_TEMP_SENSOR);
      temp_voltage = (float)temp_sensor_value * ADC_REF / 1023.0;
      float temp_r = (MY_VCC / temp_voltage - 1.0) * R0;
      float my_temperature = 1.0 / (log(temp_r / R0) / MY_B + 1 / 298.15) - 273.15; // Convert to temperature via datasheet
    
      // Temperature reading
      int a = analogRead(pinTempSensor);
  
      float R = 1023.0 / a - 1.0;
      R = R0 * R;
  
      float temperature = 1.0 / (log(R / R0) / B + 1 / 298.15) - 273.15; // Convert to temperature via datasheet
  
      // Target temperature reading
      float voltage;
      int sensor_value = analogRead(ROTARY_ANGLE_SENSOR);
      voltage = (float)sensor_value * ADC_REF / 1023.0;
      float degrees = (voltage * FULL_ANGLE) / GROVE_VCC;
      float target;
      target = 20.0 + (degrees * 20.0 / FULL_ANGLE);
  
      float lower_target = target - lower_threshold_diff;
  
      int pwm_speed = (int)min(255.0, max(0.0, 255.0 * (my_temperature - lower_target) / lower_threshold_diff));
      // analogWrite(FAN_PWM_PIN, pwm_speed);

      float fan_tacho_voltage = (float)analogRead(FAN_TACHO) * ADC_REF / 1023.0;

      if (state != (fan_tacho_voltage > 2.5)) {
        state = !state;
        j++;
      }
      
      if (i == 4999) {

        float rpm = (float)j * 60000000.0 / 4.0 / (float)(micros() - starttime); // two pulses per revolution, and each edge is counted, so divide by 4
        
        // Send temperature and target to serial port on the same line separated by a tab
        Serial.print(target);
        Serial.print(",");
        Serial.print(my_temperature);
        Serial.print(",");
        Serial.print(temp_voltage);
        Serial.print(",");
        Serial.print(k);
        Serial.print(",");
        Serial.print(fan_tacho_voltage);
        Serial.print(",");
        Serial.println(rpm);
      }
  
      delay(1);
    };
    };
}
