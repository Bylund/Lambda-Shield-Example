/*
    Example code compatible with the Lambda Shield for Arduino and GUI frontend.
    
    Copyright (C) 2018 - 2019 Bylund Automotive AB
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    
    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    
    Contact information of author:
    http://www.bylund-automotive.com/
    
    info@bylund-automotive.com
    
    Version history:
    2018-01-20        v1.0.0        First release.
    2019-04-19        v1.0.1        Published on Github.
    2019-06-26        v1.1.0        Adjusted PID regulation of heater.
    2019-07-18        v1.2.0        Code optimization.
*/

//Define included headers.
#include <ArduinoBLE.h>
#include <SPI.h>

//Define BLE service.
BLEService LambdaService("C664F3C0-81B3-4E7B-A2F6-EA96181A2E71");
BLEUnsignedCharCharacteristic OxygenLevelChar("C83C027E-F105-4E0D-AA8F-801AC553D5C1", BLERead | BLENotify);

//Define CJ125 registers used.
#define           CJ125_IDENT_REG_REQUEST             0x4800        /* Identify request, gives revision of the chip. */
#define           CJ125_DIAG_REG_REQUEST              0x7800        /* Dignostic request, gives the current status. */
#define           CJ125_INIT_REG1_REQUEST             0x6C00        /* Requests the first init register. */
#define           CJ125_INIT_REG2_REQUEST             0x7E00        /* Requests the second init register. */
#define           CJ125_INIT_REG1_MODE_CALIBRATE      0x569D        /* Sets the first init register in calibration mode. */
#define           CJ125_INIT_REG1_MODE_NORMAL_V8      0x5688        /* Sets the first init register in operation mode. V=8 amplification. */
#define           CJ125_INIT_REG1_MODE_NORMAL_V17     0x5689        /* Sets the first init register in operation mode. V=17 amplification. */
#define           CJ125_DIAG_REG_STATUS_OK            0x28FF        /* The response of the diagnostic register when everything is ok. */
#define           CJ125_DIAG_REG_STATUS_NOPOWER       0x2855        /* The response of the diagnostic register when power is low. */
#define           CJ125_DIAG_REG_STATUS_NOSENSOR      0x287F        /* The response of the diagnostic register when no sensor is connected. */
#define           CJ125_INIT_REG1_STATUS_0            0x2888        /* The response of the init register when V=8 amplification is in use. */
#define           CJ125_INIT_REG1_STATUS_1            0x2889        /* The response of the init register when V=17 amplification is in use. */
#define           hardwareId                          0x01          /* */

//Define pin assignments.
#define           LED_STATUS_POWER                    7             /* Pin used for power the status LED, indicating we have power. */
#define           LED_STATUS_HEATER                   6             /* Pin used for the heater status LED, indicating heater activity. */
#define           CJ125_NSS_PIN                       5             /* Pin used for chip select in SPI communication. */
#define           HEATER_OUTPUT_PIN                   3             /* Pin used for the PWM output to the heater circuit. */
#define           UB_ANALOG_INPUT_PIN                 2             /* Analog input for power supply.*/
#define           UR_ANALOG_INPUT_PIN                 1             /* Analog input for temperature.*/
#define           UA_ANALOG_INPUT_PIN                 0             /* Analog input for lambda.*/

//Define adjustable parameters.       
#define           SERIAL_RATE                         10            /* Serial refresh rate in HZ (1-100)*/            
#define           UBAT_MIN                            150           /* Minimum voltage (ADC value) on Ubat to operate */

//Global variables.
int adcValue_UA = 0;                                                /* ADC value read from the CJ125 UA output pin */ 
int adcValue_UR = 0;                                                /* ADC value read from the CJ125 UR output pin */
int adcValue_UB = 0;                                                /* ADC value read from the voltage divider caluclating Ubat */
int adcValue_UR_Optimal = 175;                                      /* UR ADC value stored when CJ125 is in calibration mode, optimal temperature */
int HeaterOutput = 0;                                               /* Current PWM output value (0-255) of the heater output pin */
int CJ125_Status = 0;                                               /* Latest stored DIAG registry response from the CJ125 */
int serial_counter = 0;                                             /* Counter used to calculate refresh rate on the serial output */
int HeaterStatus = 0;                                               /* Defines the heater status for the GUI front-end */


//PID regulation variables.
int dState;                                                         /* Last position input. */
int iState;                                                         /* Integrator state. */
const int iMax = 250;                                               /* Maximum allowable integrator state. */
const int iMin = -250;                                              /* Minimum allowable integrator state. */
const float pGain = 120;                                            /* Proportional gain. Default = 120*/
const float iGain = 0.8;                                            /* Integral gain. Default = 0.8*/
const float dGain = 10;                                             /* Derivative gain. Default = 10*/

//Function for transfering SPI data to the CJ125.
uint16_t COM_SPI(uint16_t TX_data) {

  //Set chip select pin low, chip in use.
  digitalWrite(CJ125_NSS_PIN, LOW);

  //Transmit and receive.
  byte highByte = SPI.transfer(TX_data >> 8);
  byte lowByte = SPI.transfer(TX_data & 0xff);

  //Set chip select pin high, chip not in use.
  digitalWrite(CJ125_NSS_PIN, HIGH);

  //Assemble response in to a 16bit integer and return the value.
  uint16_t Response = (highByte << 8) + lowByte;
  return Response;
  
}

//Temperature regulating software (PID).
int CalculateHeaterOutput(int input) {
  
  //Calculate error term.
  int error = adcValue_UR_Optimal - input;
  
  //Set current position.
  int position = input;
  
  //Calculate proportional term.
  float pTerm = -pGain * error;
  
  //Calculate the integral state with appropriate limiting.
  iState += error;
  
  if (iState > iMax) iState = iMax;
  if (iState < iMin) iState = iMin;
  
  //Calculate the integral term.
  float iTerm = -iGain * iState;
  
  //Calculate the derivative term.
  float dTerm = -dGain * (dState - position);
  dState = position;
  
  //Calculate regulation (PI).
  int RegulationOutput = pTerm + iTerm + dTerm;
  
  //Set maximum heater output (full power).
  if (RegulationOutput > 255) RegulationOutput = 255;
  
  //Set minimum heater value (cooling).
  if (RegulationOutput < 0.0) RegulationOutput = 0;

  //Return calculated PWM output.
  return RegulationOutput;
  
}

//Function to read inputs and update values.
void UpdateInputs() {

  //Update analog inputs.
  adcValue_UA = analogRead(UA_ANALOG_INPUT_PIN);
  adcValue_UR = analogRead(UR_ANALOG_INPUT_PIN);
  adcValue_UB = analogRead(UB_ANALOG_INPUT_PIN);

  //Update CJ125 status.
  CJ125_Status = COM_SPI(CJ125_DIAG_REG_REQUEST);
  
}

//Function to transfer current values to app.
void UpdateBLEOutput() {

  //Only update if app is connected.
  if (BLE.connected()) {

    //Calculate checksum.
    uint16_t checksum = HeaterStatus + hardwareId + CJ125_Status + adcValue_UA + adcValue_UR + adcValue_UB;

    //Assembled data.
    String txString = (String)HeaterStatus;
    txString += ",";
    txString += (String)hardwareId;
    txString += ",";
    txString += (String)CJ125_Status;
    txString += ",";
    txString += (String)adcValue_UA;
    txString += ",";
    txString += (String)adcValue_UR;
    txString += ",";
    txString += (String)adcValue_UB;
    txString += ",";
    txString += (String)checksum;
    txString += "\n";

    //Update character.
    for (int i = 0; i < txString.length(); i++) {
      OxygenLevelChar.writeValue(txString.charAt(i));
    }
    
  } else {
    
    //Poll for BLE radio events and handle them.
    BLE.poll();
    
  }
  
}

void setup() {
  
  //Set up serial communication.
  Serial.begin(9600);
  
  //Set up SPI.
  SPI.begin();  /* Note, SPI will disable the bult in LED*/
  SPI.setClockDivider(SPI_CLOCK_DIV128);
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE1);
  
  //Initialize BLE.
  BLE.begin();
  BLE.setDeviceName("Arduino");
  BLE.setLocalName("Lambda Sensor");
  BLE.setAdvertisedService(LambdaService);
  LambdaService.addCharacteristic(OxygenLevelChar);
  BLE.addService(LambdaService);
  BLE.advertise();

  //Set up digital output pins.
  pinMode(CJ125_NSS_PIN, OUTPUT);  
  pinMode(LED_STATUS_POWER, OUTPUT);
  pinMode(LED_STATUS_HEATER, OUTPUT);
  pinMode(HEATER_OUTPUT_PIN, OUTPUT);

}

void loop() {

  //Update Analog Values.
  UpdateInputs();

  //Update front ends.
  UpdateBLEOutput();
  
  //Turn off heating.
  if (adcValue_UB < UBAT_MIN) {
    analogWrite(HEATER_OUTPUT_PIN, 0);

    //Output LED's.
    digitalWrite(LED_STATUS_POWER, LOW);  
    digitalWrite(LED_STATUS_HEATER, LOW);

  }
  
  //Heat sensor.
  if (adcValue_UR >= 250 && adcValue_UB >= UBAT_MIN && CJ125_Status == CJ125_DIAG_REG_STATUS_OK) {
    
    //Output LED's.
    digitalWrite(LED_STATUS_POWER, HIGH);  
    digitalWrite(LED_STATUS_HEATER, LOW); 

    /* Heat up sensor. This is described in detail in the datasheet of the LSU 4.9 sensor with a 
     * condensation phase and a ramp up face before going in to PID control. */

    //Update heater status to heating.
    HeaterStatus = 1;
    
    //Calculate supply voltage.
    float SupplyVoltage = (((float)adcValue_UB / 1023 * 5) / 10000) * 110000;
    
    //Condensation phase, 2V for 5s.
    int CondensationPWM = (2 / SupplyVoltage) * 255;
    analogWrite(HEATER_OUTPUT_PIN, CondensationPWM);
    
    int t = 0;
    while (t < 500 && adcValue_UB > UBAT_MIN) {
    
      //Update Values.
      UpdateInputs();
    
      //Update frontends.
      UpdateBLEOutput();

      //Increment.
      t += 1;
      delay(10);
      
    }

    //Ramp up phase, +0.4V / s until 100% PWM from 8.5V.
    float UHeater = 8.5;
    while (UHeater < 13.0 && adcValue_UB > UBAT_MIN && adcValue_UR > adcValue_UR_Optimal) {
  
      //Update Values.
      UpdateInputs();
  
      //Update frontends.
      UpdateBLEOutput();
      
      //Set heater output during ramp up.
      CondensationPWM = (UHeater / SupplyVoltage) * 255;
        
      if (CondensationPWM > 255) CondensationPWM = 255; /*If supply voltage is less than 13V, maximum is 100% duty cycle*/
  
      analogWrite(HEATER_OUTPUT_PIN, CondensationPWM);
  
      //Increment Voltage.
      UHeater += 0.004;
      delay(10);
      
    }
    
  }

  //Adjust PWM output by calculated PID regulation.
  if (adcValue_UR < 250 && adcValue_UB >= UBAT_MIN && CJ125_Status == CJ125_DIAG_REG_STATUS_OK) {
    
    //Calculate and set new heater output.
    HeaterOutput = CalculateHeaterOutput(adcValue_UR);
    analogWrite(HEATER_OUTPUT_PIN, HeaterOutput);

    //Output LED's.
    digitalWrite(LED_STATUS_POWER, HIGH);  
    digitalWrite(LED_STATUS_HEATER, HIGH);
    
    //Update heater status.
    HeaterStatus = 2;
    
  }

}
