/*
    Example code compatible with the Lambda Shield for Arduino.
    
    Copyright (C) 2017 Bylund Automotive AB
    
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

    Revision history:
    2017-12-30        Rev. 1        First release to GitHub.
*/

//Define included headers.
#include <SPI.h>

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

//Define pin assignments.
#define           CJ125_NSS_PIN                       10            /* Pin used for chip select in SPI communication. */
#define           LED_STATUS_POWER                    7             /* Pin used for power the status LED, indicating we have power. */
#define           LED_STATUS_HEATER                   6             /* Pin used for the heater status LED, indicating heater activity. */
#define           HEATER_OUTPUT_PIN                   5             /* Pin used for the PWM output to the heater circuit. */
#define           UB_ANALOG_INPUT_PIN                 2             /* Analog input for power supply.*/
#define           UR_ANALOG_INPUT_PIN                 1             /* Analog input for temperature.*/
#define           UA_ANALOG_INPUT_PIN                 0             /* Analog input for lambda.*/

//Define adjustable parameters.
#define           SERIAL_RATE                         1             /* Serial refresh rate in HZ (1-100)*/                           
#define           UBAT_MIN                            550           /* Minimum voltage (ADC value) on Ubat to operate */

//Global variables.
int adcValue_UA = 0;                                                /* ADC value read from the CJ125 UA output pin */ 
int adcValue_UR = 0;                                                /* ADC value read from the CJ125 UR output pin */
int adcValue_UB = 0;                                                /* ADC value read from the voltage divider caluclating Ubat */
int adcValue_UA_Optimal = 0;                                        /* UA ADC value stored when CJ125 is in calibration mode, λ=1 */ 
int adcValue_UR_Optimal = 0;                                        /* UR ADC value stored when CJ125 is in calibration mode, optimal temperature */
int HeaterOutput = 0;                                               /* Current PWM output value (0-255) of the heater output pin */
int serial_counter = 0;                                             /* Counter used to calculate refresh rate on the serial output */
int CJ125_Status = 0;                                               /* Latest stored DIAG registry response from the CJ125 */

//PID regulation variables.
int dState;                                                         /* Last position input. */
int iState;                                                         /* Integrator state. */
const int iMax = 255;                                               /* Maximum allowable integrator state. */
const int iMin = 0;                                                 /* Minimum allowable integrator state. */
const float pGain = 120;                                           /* Proportional gain. Default = 120*/
const float iGain = 0.8;                                            /* Integral gain. Default = 0.8*/
const float dGain = 10;                                            /* Derivative gain. Default = 10*/

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
int Heater_PID_Control(int input) {
  
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

//Calculate Oxygen Content.
float Calculate_Oxygen(int Input_ADC) {
  
    //Calculate CJ125 Voltage.
    float CJ125_UA = (float)Input_ADC / 1023 * 5.0;

    //Calculate pump current acc. to BOSCH LSU 4.9 Technical Product Information Y 258 E00 015e.
    float LAMBDA_IP = 1000 * (CJ125_UA -1.5) / (61.9 * 17); /* V=17 */

    //Calculate oxygen content by linear approximation.
    const float k = 0.2095/2.54;
    float LAMBDA_O2 = LAMBDA_IP * k;

    //Return value.
    return LAMBDA_O2;
    
}

//Calculate Lambda.
float Calculate_Lambda(int Input_ADC) {
  
    //Calculate Oxygen Content.
    float LAMBDA_O2 = Calculate_Oxygen(Input_ADC);

    //Calculate Lambda value acc. to BOSCH LSU 4.9 Technical Product Information Y 258 E00 015e.
    float LAMBDA_VALUE = (LAMBDA_O2 / 3 + 1) / (1 - 4.77 * LAMBDA_O2);

    //Return value.
    return LAMBDA_VALUE;
    
}

//Function to set up device for operation.
void setup() {
  
  //Set up serial communication.
  Serial.begin(9600);

  //Set up SPI.
  SPI.begin();  /* Note, SPI will disable the bult in LED*/
  SPI.setClockDivider(SPI_CLOCK_DIV128);
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE1);
  
  //Set up digital output pins.
  pinMode(CJ125_NSS_PIN, OUTPUT);  
  pinMode(LED_STATUS_POWER, OUTPUT);
  pinMode(LED_STATUS_HEATER, OUTPUT);
  pinMode(HEATER_OUTPUT_PIN, OUTPUT);

  //Set initial values.
  digitalWrite(CJ125_NSS_PIN, HIGH);
  digitalWrite(LED_STATUS_POWER, LOW);
  digitalWrite(LED_STATUS_HEATER, LOW);
  analogWrite(HEATER_OUTPUT_PIN, 0); /* PWM is off before we know power status.*/

  //Start of operation. (Test LED's).
  Serial.print("Device reset.\n\r");
  digitalWrite(LED_STATUS_POWER, HIGH);
  digitalWrite(LED_STATUS_HEATER, HIGH);
  delay(200);
  digitalWrite(LED_STATUS_POWER, LOW);
  digitalWrite(LED_STATUS_HEATER, LOW);

  //Start main function.
  start();

}

void start() {
  
  //Wait until everything is ready. Read CJ125 multiple times with delay in between to let it initialize. Otherwise responds OK.
  int n = 0;
  while (adcValue_UB < UBAT_MIN || CJ125_Status != CJ125_DIAG_REG_STATUS_OK || n < 9) {
    
    //Read CJ125 diagnostic register from SPI.
    CJ125_Status = COM_SPI(CJ125_DIAG_REG_REQUEST);

    //Read input voltage.
    adcValue_UB = analogRead(UB_ANALOG_INPUT_PIN);

    //Delay and increment counter.
    delay(100);
    n++;
    
  }

  //Start of operation. (Start Power LED).
  Serial.print("Device ready.\n\r");
  digitalWrite(LED_STATUS_POWER, HIGH);

  //Store calibrated optimum values.
  Serial.print("Reading calibration data.\n\r");

  //Set CJ125 in calibration mode.
  COM_SPI(CJ125_INIT_REG1_MODE_CALIBRATE);

  //Let values settle.
  delay(500);

  //Store optimal values before leaving calibration mode.
  adcValue_UA_Optimal = analogRead(UA_ANALOG_INPUT_PIN);
  adcValue_UR_Optimal = analogRead(UR_ANALOG_INPUT_PIN);
  
  //Set CJ125 in normal operation mode.
  //COM_SPI(CJ125_INIT_REG1_MODE_NORMAL_V8);  /* V=0 */
  COM_SPI(CJ125_INIT_REG1_MODE_NORMAL_V17);  /* V=1 */

  //Present calibration data:
  Serial.print("UA_Optimal (λ = 1.00): ");
  Serial.print(adcValue_UA_Optimal);
  Serial.print(" (λ = ");
  Serial.print(Calculate_Lambda(adcValue_UA_Optimal), 2);
  Serial.print(")\n\r");
  Serial.print("UR_Optimal: ");
  Serial.print(adcValue_UR_Optimal);
  Serial.print("\n\r");
  
  /* Heat up sensor. This is described in detail in the datasheet of the LSU 4.9 sensor with a 
   * condensation phase and a ramp up face before going in to PID control. */
  Serial.print("Heating sensor.\n\r");    

  //Calculate supply voltage.
  float SupplyVoltage = (((float)adcValue_UB / 1023 * 5) / 49900) * 149900;

  //Condensation phase, 2V for 5s.
  int CondensationPWM = (2 / SupplyVoltage) * 255;
  analogWrite(HEATER_OUTPUT_PIN, CondensationPWM);

  int t = 0;
  while (t < 5 && analogRead(UB_ANALOG_INPUT_PIN) > UBAT_MIN) {

    //Flash Heater LED in condensation phase.
    digitalWrite(LED_STATUS_HEATER, HIGH);  
    delay(500);
          
    digitalWrite(LED_STATUS_HEATER, LOW);
    delay(500);

    t += 1;
    
  }

  //Ramp up phase, +0.4V / s until 100% PWM from 8.5V.
  float UHeater = 8.5;
  while (UHeater < 13.0 && analogRead(UB_ANALOG_INPUT_PIN) > UBAT_MIN) {

    //Set heater output during ramp up.
    CondensationPWM = (UHeater / SupplyVoltage) * 255;
      
    if (CondensationPWM > 255) CondensationPWM = 255; /*If supply voltage is less than 13V, maximum is 100% PWM*/

    analogWrite(HEATER_OUTPUT_PIN, CondensationPWM);

    //Flash Heater LED in condensation phase.
    digitalWrite(LED_STATUS_HEATER, HIGH);
    delay(500);
      
    digitalWrite(LED_STATUS_HEATER, LOW);
    delay(500);

    //Increment Voltage.
    UHeater += 0.4;
      
  }

  //Heat until temperature optimum is reached or exceeded (lower value is warmer).
  while (analogRead(UR_ANALOG_INPUT_PIN) > adcValue_UR_Optimal && analogRead(UB_ANALOG_INPUT_PIN) > UBAT_MIN) {

    //Flash Heater LED in condensation phase.
    digitalWrite(LED_STATUS_HEATER, HIGH);
    delay(500);
      
    digitalWrite(LED_STATUS_HEATER, LOW);
    delay(500);

  }

  //Heating phase finished, hand over to PID-control. Turn on LED and turn off heater.
  digitalWrite(LED_STATUS_HEATER, HIGH);
  analogWrite(HEATER_OUTPUT_PIN, 0);
  
}

//Infinite loop.
void loop() {

  //Update CJ125 diagnostic register from SPI.
  CJ125_Status = COM_SPI(CJ125_DIAG_REG_REQUEST);

  //Update analog inputs.
  adcValue_UA = analogRead(UA_ANALOG_INPUT_PIN);
  adcValue_UR = analogRead(UR_ANALOG_INPUT_PIN);
  adcValue_UB = analogRead(UB_ANALOG_INPUT_PIN);

  //Adjust PWM output by calculated PID regulation.
  if (adcValue_UR < 500 || adcValue_UR_Optimal != 0 || adcValue_UB > UBAT_MIN) {
    
    //Calculate and set new heater output.
    HeaterOutput = Heater_PID_Control(adcValue_UR);
    analogWrite(HEATER_OUTPUT_PIN, HeaterOutput);
    
  } else {
    
    //Turn off heater if we are not in PID control.
    HeaterOutput = 0;
    analogWrite(HEATER_OUTPUT_PIN, HeaterOutput);
    
  }

  //If power is lost, "reset" the device.
  if (adcValue_UB < UBAT_MIN) {

    //Indicate low power.
    Serial.print("Low power.\n");

    //Turn of status LEDs.
    digitalWrite(LED_STATUS_POWER, LOW);
    digitalWrite(LED_STATUS_HEATER, LOW);

    //Re-start() and wait for power.
    start();
    
  }
  
  //Display on serial port at defined rate. Comma separate values, readable by frontends.
  if ( (100 / SERIAL_RATE) ==  serial_counter) {

    //Reset counter.
    serial_counter = 0;

    //Calculate Lambda Value.
    float LAMBDA_VALUE = Calculate_Lambda(adcValue_UA);
    
    //Calculate Oxygen Content.
    float OXYGEN_CONTENT = Calculate_Oxygen(adcValue_UA) * 100;

    //Display information if no errors is reported.
    if (CJ125_Status == CJ125_DIAG_REG_STATUS_OK) {
      
      //Output values.
      Serial.print("Measuring, CJ125: 0x");
      Serial.print(CJ125_Status, HEX);
      Serial.print(", UA_ADC: ");
      Serial.print(adcValue_UA);
      Serial.print(", UR_ADC: ");
      Serial.print(adcValue_UR);
      Serial.print(", UBat_ADC: ");
      Serial.print(adcValue_UB);
      
      //Display Lambda value unless sensor is in air.
      Serial.print(", Lambda: ");
      if (OXYGEN_CONTENT < 20) Serial.print(LAMBDA_VALUE, 2);
      if (OXYGEN_CONTENT >= 20) Serial.print("Air");
      
      //Display oxygen content.
      Serial.print(", Oxygen: ");
      Serial.print(OXYGEN_CONTENT, 2);
      Serial.print("%\n\r");
      
    } else {
      
      //Error handling.
      switch(CJ125_Status) {

        case CJ125_DIAG_REG_STATUS_NOPOWER:
          Serial.print("Error, CJ125: 0x");
          Serial.print(CJ125_Status, HEX);
          Serial.print(" (No Power)\n\r");
        break;
  
        case CJ125_DIAG_REG_STATUS_NOSENSOR:
          Serial.print("Error, CJ125: 0x");
          Serial.print(CJ125_Status, HEX);
          Serial.print(" (No Sensor)\n\r");
        break;

        default:
          Serial.print("Error, CJ125: 0x");
          Serial.print(CJ125_Status, HEX);
          Serial.print("\n\r");
        
        }
        
    }

  }

  //Increment serial output counter and delay for next cycle. The PID requires to be responsive but we don't need to flud the serial port.
  serial_counter++;
  delay(10);

}
