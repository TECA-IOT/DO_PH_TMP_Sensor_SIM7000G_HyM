
#include "Arduino.h"

//#include "WProgram.h"


#include "do_grav.h"
#include <EEPROM.h>

#define EEPROM_SIZE 1024
#define ESP32

Gravity_DO::Gravity_DO(uint8_t pin){
	this->pin = pin;
    this->EEPROM_offset = (pin) * EEPROM_SIZE_CONST;
    //to lay the calibration parameters out in EEPROM we map their locations to the analog pin numbers
    //we assume a maximum size of EEPROM_SIZE_CONST for every struct we're saving  
}

bool Gravity_DO::begin(){
    
        Serial.println("BeginDO...");
        EEPROM.begin(EEPROM_SIZE);
    
	//if((EEPROM.read(this->EEPROM_offset) == magic_char)
    //&& (EEPROM.read(this->EEPROM_offset + sizeof(uint8_t)) == GRAV_DO)){
		Serial.println("Data_EEPROM_DO...");
        EEPROM.get(this->EEPROM_offset,Do);
		return true;
    //}
	//return false;
}

float Gravity_DO::read_voltage() {
    float voltage_mV = 0;
    for (int i = 0; i < volt_avg_len; ++i) {
      voltage_mV += analogRead(this->pin) / 4095.0 * 3300.0;
    }
    voltage_mV /= volt_avg_len;
    return voltage_mV;
}

float Gravity_DO::read_do_percentage(float voltage_mV) {
    return voltage_mV * 100.0 / this->Do.full_sat_voltage;
}

float Gravity_DO::cal() {
    this->Do.full_sat_voltage = read_voltage();
        Serial.println("Grabando_DO");
    EEPROM.put(this->EEPROM_offset,Do);
        Serial.println(read_voltage());
        Serial.println(EEPROM_offset);
           
        EEPROM.commit(); 
        Serial.println("Commit");
    
}

float Gravity_DO::cal_clear() {
    this->Do.full_sat_voltage = DEFAULT_SAT_VOLTAGE;
    EEPROM.put(this->EEPROM_offset,Do);
    
        EEPROM.commit(); 
    
}

float Gravity_DO::read_do_percentage() {
  return(read_do_percentage(read_voltage()));
}
