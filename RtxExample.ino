// This example is derived from the UK High Altitude Society: https://ukhas.org.uk/guides:linkingarduinotontx2. Big thanks to them, exceptionally help. The following is an effective rewrite for my own understanding.
#define RADIOPIN 9
#define ENABLE_RADIO_PIN 8
#include <string.h>
#include <util/crc16.h>

char datastring[150];

double modifiedMap(double x, double in_min, double in_max, double out_min, double out_max)
{
  double temp = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  temp = (int) (4 * temp + .5);
  return (double) temp / 4;
}


uint8_t voltageOfArduino = 5; // The Arduino working voltage is 5v
uint8_t voltageOfNtx2 = 3; // NTX has a MAX TX pin voltage of 3v
uint16_t maximumFrequencyDeviationInHz = 6000; // The NTX will specify the maximum frequency deviation it is capable of, in my case 6Khz, or 6000 hz
uint16_t desiredFrequencyShiftInHz = 425; // For us to generate two seperate tones for FSK, we need to select the shift, which will later become our 'bandwidth' you will select to tune into the transmission
double voltageIncrementRequiredForOneHz = (double)voltageOfNtx2 / (double)maximumFrequencyDeviationInHz; // Lets calculate the voltage required for each Hz
double voltageRequiredForDesiredFrequencyShift = voltageIncrementRequiredForOneHz * desiredFrequencyShiftInHz; // Multiple that by the desired shift, aka 'bandwidth'

double voltageForHighState = voltageOfNtx2; // Lets select a voltage for the HIGH state. This may as well be the max voltage of the NTX
double voltageForLowState = voltageForHighState - voltageRequiredForDesiredFrequencyShift; // The low voltage will need to be 500hz below. So we subtract the amount of voltage required for the shift.

// The Arduino uses a mechanism called PWM to generate a square wave form. It effectively does this by turning on/off a certain number of times a cycle. We can use this to "fake" a voltage.
uint8_t pwmDutyForNtxVoltage = modifiedMap(voltageOfNtx2, 0, voltageOfArduino, 0, 255);  // First lets work out the maximum PWM duty value which would generate the NTX voltage (3v). Remember the Arduino is 5 volts, we need to fake at most 3 volts otherwise we risk damaging the transmitter.

uint8_t pwmDutyRequiredForHighState = modifiedMap(voltageForHighState, 0, voltageOfNtx2, 0, pwmDutyForNtxVoltage); // Now we calculate the PWM value for "ON"
uint8_t pwmDutyRequiredForLowState = modifiedMap(voltageForLowState, 0, voltageOfNtx2, 0, pwmDutyForNtxVoltage); // and conversly for "OFF".
// Again, we are trying to generate two tones, separated by the frequency shift defined earlier. Frequency Key Shifting (FSK) works by alternating these two tones to represent binary, our 1 and 0's.

uint16_t desiredBaudRate = 50; // Number of bits per second
uint16_t delayRequiredForBaudRateInMilliseconds = ((double)1 / (double)desiredBaudRate) * 1000;


void setup()
{
  TCCR1B = TCCR1B & 0b11111000 | 0x01;
  pinMode(RADIOPIN, OUTPUT);
  pinMode(ENABLE_RADIO_PIN, OUTPUT);
  digitalWrite(ENABLE_RADIO_PIN, LOW); // Disable Radio

  Serial.begin(9600);
  Serial.print("Voltage Of Arduino: ");
  Serial.println(voltageOfArduino);

  Serial.print("Voltage Of Ntx: ");
  Serial.println(voltageOfNtx2);

  Serial.print("Maximum FSK Deviation in Hz: ");
  Serial.println(maximumFrequencyDeviationInHz);

  Serial.print("Voltage required for 1Hz: ");
  printDouble(voltageIncrementRequiredForOneHz, 5);

  Serial.print("Desired Frequency Shift: ");
  Serial.println(desiredFrequencyShiftInHz);

  Serial.print("Voltage required for desired frequency shift: ");
  printDouble(voltageRequiredForDesiredFrequencyShift, 3);

  Serial.print("Voltage for HIGH state: ");
  printDouble(voltageForHighState, 3);

  Serial.print("Voltage for LOW state: ");
  printDouble(voltageForLowState, 3);

  Serial.print("Maximum PWM duty value for NTX max voltage: ");
  Serial.println(pwmDutyForNtxVoltage);

  Serial.print("PWM Value for HIGH state: ");
  Serial.println(pwmDutyRequiredForHighState);

  Serial.print("PWM Value for LOW state: ");
  Serial.println(pwmDutyRequiredForLowState);

  Serial.print("Desired Baud Rate: ");
  Serial.println(desiredBaudRate);

  Serial.print("Time in milliseconds per bit: ");
  Serial.println(delayRequiredForBaudRateInMilliseconds);

}


String msg;
void loop() {

  while (!Serial.available());

  while (Serial.available()) {
    msg = Serial.readString();
  }

  Serial.println(msg);

  msg.toCharArray(datastring, msg.length());


 unsigned int CHECKSUM = gps_CRC16_checksum(datastring); // Calculates the checksum for this datastring
 char checksum_str[6];
 sprintf(checksum_str, "*%04X\n", CHECKSUM);
 strcat(datastring,checksum_str);
  
  for (int i = 0; i < 3; i++) {
    rtty_txstring (datastring);
  }
}

uint16_t gps_CRC16_checksum (char *string) {
    size_t i;
    uint16_t crc;
    uint8_t c;
 
    crc = 0xFFFF;
 
    // Calculate checksum ignoring the first two $s
    for (i = 2; i < strlen(string); i++) {
        c = string[i];
        crc = _crc_xmodem_update (crc, c);
    }
 
    return crc;
}

 
void rtty_txstring (char * string) {

  digitalWrite(ENABLE_RADIO_PIN, HIGH); // Disable Radio
  /* Simple function to sent a char at a time to
  ** rtty_txbyte function.
  ** NB Each char is one byte (8 Bits)
  */

  char c;
  c = *string++;

  while ( c != '\0') {
    rtty_txbyte (c);
    c = *string++;
  }

  digitalWrite(ENABLE_RADIO_PIN, LOW); // Disable Radio
}


void rtty_txbyte (char c) {
  /* Simple function to sent each bit of a char to
  ** rtty_txbit function.
  ** NB The bits are sent Least Significant Bit first
  **
  ** All chars should be preceded with a 0 and
  ** proceed with a 1. 0 = Start bit; 1 = Stop bit
  **
  */

  int i;

  rtty_txbit (0); // Start bit

  // Send bits for for char LSB first

  for (i = 0; i < 7; i++) { // Change this here 7 or 8 for ASCII-7 / ASCII-8
    if (c & 1) rtty_txbit(1);
    else rtty_txbit(0);

    c = c >> 1;
  }
  rtty_txbit (1); // Stop bit
  rtty_txbit (1); // Stop bit
}

void rtty_txbit (int bit) {
  if (bit) {
    // high
    analogWrite(RADIOPIN, pwmDutyRequiredForHighState);
  }
  else {
    // low
    analogWrite(RADIOPIN, pwmDutyRequiredForLowState);
  }

  delay(delayRequiredForBaudRateInMilliseconds);
}




void printDouble( double val, byte precision) {
  // prints val with number of decimal places determine by precision
  // precision is a number from 0 to 6 indicating the desired decimial places
  // example: printDouble( 3.1415, 2); // prints 3.14 (two decimal places)

  Serial.print (int(val));  //prints the int part
  if ( precision > 0) {
    Serial.print("."); // print the decimal point
    unsigned long frac;
    unsigned long mult = 1;
    byte padding = precision - 1;
    while (precision--)
      mult *= 10;

    if (val >= 0)
      frac = (val - int(val)) * mult;
    else
      frac = (int(val) - val ) * mult;
    unsigned long frac1 = frac;
    while ( frac1 /= 10 )
      padding--;
    while (  padding--)
      Serial.print("0");
    Serial.print(frac, DEC) ;
  }
  Serial.print('\n');
}



