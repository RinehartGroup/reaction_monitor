#include <Wire.h>
#include <RTClib.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_ADS1X15.h>
#include <Adafruit_BME280.h>
#include <math.h>
#include <AltSoftSerial.h>
#include <SensorModbusMaster.h>
#include <SPI.h>
#include <SD.h>

bool CALIBRATE = false;

RTC_DS3231 rtc;

#define updateInterval 1000 // 1000 ms for actual use, 10000 ms for calibration
unsigned long lastUpdate = 0;

#define BUTTON_PIN 2
#define RECORDING_LED 3
int ledState = LOW;
int buttonState;
int lastButtonState = HIGH;
bool recording = false;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;

#define flow1 A0 // model 100-3, range 20-100 mL/min, SN 46747
#define flow2 A1 // model 100-5, range 100-500 mL/min, SN 47499
#define FLOW1_CAL_SLOPE 20.103417415227547
#define FLOW1_CAL_INTERCEPT 0.10905233849315721
#define FLOW2_CAL_SLOPE 96.6787554876885
#define FLOW2_CAL_INTERCEPT 9.410192784882554

Adafruit_ADS1115 oxy;
Adafruit_ADS1115 oxyTemp;
Adafruit_BME280 bme;

#define OXY_RES 2.0 * 0.512 / 65536.0      // GAIN_EIGHT
#define OXY_TEMP_RES 2.0 * 4.096 / 65536.0 // GAIN_ONE

#define PCAL1 99.99825        // kPa
#define HCAL1 4.75015         // %, dry calibration
#define OXY1_VCAL 0.052950695 // V
#define OXY1_V0 0.001456225   // V
#define OXY1_TCAL 22.02595    // °C
#define PCAL2 100.044045      // kPa
#define HCAL2 100.0           // %, wet calibration
#define OXY2_VCAL 0.04897935  // V
#define OXY2_V0 0.00153704    // V
#define OXY2_TCAL 23.3594     // °C

float OXY1_CAL[5] = {PCAL1, HCAL1, OXY1_VCAL, OXY1_V0, OXY1_TCAL}; // SN 6089
float OXY2_CAL[5] = {PCAL2, HCAL2, OXY2_VCAL, OXY2_V0, OXY2_TCAL}; // SN 6238

// input variables
DateTime now;
int16_t oxy1_adc, oxy1_temp_adc, oxy2_adc, oxy2_temp_adc;
int16_t flow1_adc, flow2_adc;
float bme_temp, pressure, humidity;
// calculated variables
float oxy1_percent, oxy1_temp, oxy2_percent, oxy2_temp;
float flow1_mLmin, flow2_mLmin;

// LCD display
LiquidCrystal_I2C lcd_omega(0x26, 16, 2);
LiquidCrystal_I2C lcd_gas(0x27, 16, 2);

/*
Modbus communication with Omega PID controller
pins:
altsoftserial: Tx: 9, Rx: 8

Temperature Controller Settings:
CoSH on
C-SL rtU
C-no 2
bPS 9600
LEn 8
PrtY nonE
StoP 1
*/

#define OMEGA_ADDRESS 0x02
#define MODBUS_BAUD 9600

AltSoftSerial modbusSerial; // uses 8N1
modbusMaster omega;

// Omega parameters
#define READ_REGISTER 0x03
#define PV_ADDR 0x1000
#define SV_ADDR 0x1001
#define CTRL_METHOD_ADDR 0x1005
#define OUT_ADDR 0x1012
#define PID_NUM_ADDR 0x101C
#define PID_SV_ADDR 0x101D
#define P_ADDR 0x1009
#define I_ADDR 0x100A
#define D_ADDR 0x100B

float omega_pv = 0.0, omega_sv = 0.0, omega_pid_sv = 0.0;
int omega_ctrl_method = 0, omega_out = 0, omega_pid_num = 0, omega_p = 0, omega_i = 0, omega_d = 0;
bool omega_connected = false;

/* SD card
pins:
    - MISO, 50
    - MOSI, 51
    - SCK, 52
    - SS, 53
*/
#define SD_CS 53
String fileName = "rxn00001.csv";

void setup()
{
    Serial.begin(9600);

    rtc.begin();

    pinMode(BUTTON_PIN, INPUT_PULLUP);
    pinMode(RECORDING_LED, OUTPUT);
    digitalWrite(RECORDING_LED, ledState);

    // OXYGEN SENSORS
    oxy.begin(0x48);     // the O2 sensors of the two Apogee S110 sensors
    oxyTemp.begin(0x49); // the temperature sensor of the two Apogee S110 sensors
    oxy.setGain(GAIN_EIGHT);
    oxyTemp.setGain(GAIN_ONE);

    pinMode(flow1, INPUT);
    pinMode(flow2, INPUT);

    // TEMP/PRESSURE/HUMIDITY SENSORS
    bme.begin();

    // LCD DISPLAY
    lcd_omega.init();
    lcd_omega.backlight();
    lcd_omega.clear();
    lcd_gas.init();
    lcd_gas.backlight();
    lcd_gas.clear();

    // MODBUS COMMUNICATION
    modbusSerial.begin(MODBUS_BAUD);
    omega.begin(OMEGA_ADDRESS, modbusSerial);

    omega_pv = omega.uint16FromRegister(READ_REGISTER, PV_ADDR, bigEndian) / 10.0;
    if (omega_pv != 0.0)
    {
        omega_connected = true;
        Serial.println("Omega PID controller connected");
    }
    else
    {
        omega_connected = false;
        Serial.println("Omega PID controller not connected");
    }

    // SD CARD
    bool cardOK = true;
    Serial.print("Initializing SD card...");
    // see if the card is present and can be initialized:
    if (!SD.begin(SD_CS))
    {
        Serial.println("Card failed, or not present");
        cardOK = false;
    }
    if (cardOK)
    {
        Serial.println("card initialized.");
    }
}

void loop()
{
    int buttonReading = digitalRead(BUTTON_PIN);
    if (buttonReading != lastButtonState)
    {
        lastDebounceTime = millis();
    }
    if ((millis() - lastDebounceTime) > debounceDelay)
    {
        if (buttonReading != buttonState)
        {
            buttonState = buttonReading;
            if (buttonState == LOW)
            {
                ledState = !ledState;
                digitalWrite(RECORDING_LED, ledState);
                recording = !recording;
                if (recording == true)
                {
                    fileName = formatFileName();
                    Serial.print("start recording to ");
                    Serial.println(fileName);
                    writeHeader(fileName);
                }
                else if (recording == false)
                {
                    Serial.print("stop recording to ");
                    Serial.println(fileName);
                }
            }
        }
    }
    lastButtonState = buttonReading;

    if ((millis() - lastUpdate) > updateInterval)
    {
        lastUpdate = millis();
        now = rtc.now();

        oxy1_adc = oxy.readADC_Differential_0_1(); // oxy1 sensor: Apogee SO-110, SN 6089
        oxy1_temp_adc = oxyTemp.readADC_Differential_0_1();
        oxy2_adc = oxy.readADC_Differential_2_3(); // oxy2 sensor: Apogee SO-110, SN 6238
        oxy2_temp_adc = oxyTemp.readADC_Differential_2_3();

        bme_temp = bme.readTemperature();
        pressure = bme.readPressure() / 1000.0F;
        humidity = bme.readHumidity();

        oxy1_temp = Convert_Oxy_Temp_adc(oxy1_temp_adc);
        oxy2_temp = Convert_Oxy_Temp_adc(oxy2_temp_adc);
        oxy1_percent = Convert_Oxy_adc(oxy1_adc, oxy1_temp, pressure, 0.0, OXY1_CAL); // assume 0% humidity coming the O2/N2 gas tanks
        oxy2_percent = Convert_Oxy_adc(oxy2_adc, oxy2_temp, pressure, humidity, OXY2_CAL);

        flow1_adc = analogRead(flow1);
        flow2_adc = analogRead(flow2);

        flow1_mLmin = Convert_Flow_adc(flow1_adc, FLOW1_CAL_SLOPE, FLOW1_CAL_INTERCEPT);
        flow2_mLmin = Convert_Flow_adc(flow2_adc, FLOW2_CAL_SLOPE, FLOW2_CAL_INTERCEPT);

        // read Omega controller
        if (omega_connected)
        {
            omega_pv = omega.uint16FromRegister(READ_REGISTER, PV_ADDR, bigEndian) / 10.0;
            omega_sv = omega.uint16FromRegister(READ_REGISTER, SV_ADDR, bigEndian) / 10.0;
            omega_ctrl_method = omega.uint16FromRegister(READ_REGISTER, CTRL_METHOD_ADDR, bigEndian);
            omega_out = omega.uint16FromRegister(READ_REGISTER, OUT_ADDR, bigEndian);
            omega_pid_num = omega.uint16FromRegister(READ_REGISTER, PID_NUM_ADDR, bigEndian);
            omega_pid_sv = omega.uint16FromRegister(READ_REGISTER, PID_SV_ADDR, bigEndian) / 10.0;
            omega_p = omega.uint16FromRegister(READ_REGISTER, P_ADDR, bigEndian);
            omega_i = omega.uint16FromRegister(READ_REGISTER, I_ADDR, bigEndian);
            omega_d = omega.uint16FromRegister(READ_REGISTER, D_ADDR, bigEndian);
        }

        // update LCDs
        lcd_omega.clear();
        lcd_omega.setCursor(0, 0);
        lcd_omega.print(formatDigits(now.hour()));
        lcd_omega.print(":");
        lcd_omega.print(formatDigits(now.minute()));
        lcd_omega.print(":");
        lcd_omega.print(formatDigits(now.second()));
        if (recording)
        {
            lcd_omega.setCursor(11, 0);
            String rxnNum = fileName;
            rxnNum.remove(0, 3);
            rxnNum.remove(rxnNum.length() - 4);
            lcd_omega.print(rxnNum);
        }
        if (CALIBRATE)
        {
            lcd_omega.setCursor(0, 1);
            lcd_omega.print(String(bme_temp, 1));
            lcd_omega.setCursor(5, 1);
            lcd_omega.print(String(pressure, 1));
            lcd_omega.setCursor(12, 1);
            lcd_omega.print(String(humidity, 1));
        }
        else
        {
            lcd_omega.setCursor(0, 1);
            lcd_omega.print(String(omega_pv, 1));
            lcd_omega.setCursor(5, 1);
            lcd_omega.print(String(omega_sv, 1));
            lcd_omega.setCursor(13, 1);
            lcd_omega.print(omega_out);
        }
        lcd_gas.clear();
        lcd_gas.setCursor(0, 0);
        lcd_gas.print("Ox%: ");
        lcd_gas.print(String(oxy1_percent, 1));
        lcd_gas.print(" ");
        lcd_gas.print(String(oxy2_percent, 1));
        lcd_gas.setCursor(0, 1);
        if (CALIBRATE)
        {
            lcd_gas.print(oxy1_adc * OXY_RES, 5);
            lcd_gas.print("  ");
            lcd_gas.print(oxy2_adc * OXY_RES, 5);
        }
        else
        {
            lcd_gas.print("Flow: ");
            lcd_gas.print(String(flow1_mLmin, 1));
            lcd_gas.print(" ");
            lcd_gas.print(String(flow2_mLmin, 1));
        }

        Print_Report();

        if (recording)
        {
            writeData(fileName);
        }
    }
}

void Print_Report()
{
    Serial.print(now.unixtime());
    Serial.print(" ");
    Serial.print(recording);
    Serial.print(" ");
    Serial.print(oxy1_adc * OXY_RES, 6);
    Serial.print(" ");
    Serial.print(oxy1_percent);
    Serial.print(" ");
    Serial.print(oxy1_temp_adc * OXY_TEMP_RES, 6);
    Serial.print(" ");
    Serial.print(oxy1_temp);
    Serial.print(" ");
    Serial.print(oxy2_adc * OXY_RES, 6);
    Serial.print(" ");
    Serial.print(oxy2_percent);
    Serial.print(" ");
    Serial.print(oxy2_temp_adc * OXY_TEMP_RES, 6);
    Serial.print(" ");
    Serial.print(oxy2_temp);
    Serial.print(" ");
    Serial.print(bme_temp);
    Serial.print(" ");
    Serial.print(pressure);
    Serial.print(" ");
    Serial.print(humidity);
    Serial.print(" ");
    Serial.print(flow1_adc * 5.0 / 1024.0, 6);
    Serial.print(" ");
    Serial.print(flow1_mLmin);
    Serial.print(" ");
    Serial.print(flow2_adc * 5.0 / 1024.0, 6);
    Serial.print(" ");
    Serial.print(flow2_mLmin);
    Serial.print(" ");
    Serial.print(omega_pv);
    Serial.print(" ");
    Serial.print(omega_sv);
    Serial.print(" ");
    Serial.print(omega_ctrl_method);
    Serial.print(" ");
    Serial.print(omega_out);
    Serial.print(" ");
    Serial.print(omega_pid_num);
    Serial.print(" ");
    Serial.print(omega_pid_sv);
    Serial.print(" ");
    Serial.print(omega_p);
    Serial.print(" ");
    Serial.print(omega_i);
    Serial.print(" ");
    Serial.print(omega_d);
    Serial.println();
}

// OXYGEN SENSOR FUCTIONS
float Convert_Oxy_adc(int16_t adc, float temp, float pressure, float humidity, float OXY_CAL[])
{
    // OXY_CAL = {PCAL, HCAL, OXY_VCAL, OXY_V0, OXY_TCAL}
    float v = adc * OXY_RES;
    float CF = (0.2095 * OXY_CAL[0]) / (OXY_CAL[2] - OXY_CAL[3]);
    float oxy_percent = CF * (v - OXY_CAL[3]);
    // pressure adjustment
    oxy_percent = oxy_percent * (OXY_CAL[0] / pressure);
    // humidity adjustment
    float sat_vp = 0.61121 * exp((temp * (18.678 - temp / 234.5)) / (257.14 + temp));
    float cal_sat_vp = 0.61121 * exp((OXY_CAL[4] * (18.678 - OXY_CAL[4] / 234.5)) / (257.14 + OXY_CAL[4]));
    float vp = sat_vp * humidity / 100.0;
    float cal_vp = cal_sat_vp * OXY_CAL[2] / 100.0;
    oxy_percent = oxy_percent * ((OXY_CAL[0] + vp - cal_vp) / OXY_CAL[0]);
    // temperature adjustment
    float c1 = -6.949e-2, c2 = 1.422e-3, c3 = -8.213e-7;
    oxy_percent = oxy_percent + c3 * pow(temp, 3) + c2 * pow(temp, 2) + c1 * temp - c3 * pow(OXY_CAL[4], 3) - c2 * pow(OXY_CAL[4], 2) - c1 * OXY_CAL[4];
    return oxy_percent;
}

float sat_vapor_pressure(float temp)
{
    // note: temp in C
    float vp = 0.61121 * exp((temp * (18.678 - temp / 234.5)) / (257.14 + temp));
}

float Convert_Oxy_Temp_adc(int16_t adc)
{
    float v_out = float(adc) * OXY_TEMP_RES;
    float r_therm = v_out * 24900.0 / (3.37 - v_out);
    float a = 1.129241e-3, b = 2.341077e-4, c = 8.775468e-8;
    float log_r = log(r_therm);
    float temp = 1.0 / (a + b * log_r + c * pow(log_r, 3)) - 273.15;
    return temp;
}

// FLOW SENSOR FUCTIONS
float Convert_Flow_adc(int16_t adc, float FLOW_CAL_SLOPE, float FLOW_CAL_INTERCEPT)
{
    float voltage = float(adc) * 5.0 / 1024.0;
    float flow = FLOW_CAL_SLOPE * voltage + FLOW_CAL_INTERCEPT;
    return flow;
}

// Time functions
String formatDigits(int digits)
{
    // utility function for digital clock display: prints preceding colon and leading 0
    String formattedDigits;
    if (digits < 10)
        formattedDigits = String("0" + String(digits));
    else
        formattedDigits = String(digits);
    return formattedDigits;
}

void dateTime(uint16_t *date, uint16_t *time)
{
    // get date and time from RTC
    unsigned int year = now.year();
    byte month = now.month();
    byte day = now.day();
    byte hour = now.hour();
    byte minute = now.minute();
    byte second = now.second();

    *date = FAT_DATE(year, month, day);
    *time = FAT_TIME(hour, minute, second);
}

// File functions
String formatFileName()
{
    String formattedFileName;
    for (int i = 0; i < 99999; i++)
    {
        String name = "rxn00000";
        String strNum = String(i);
        name.remove(name.length() - strNum.length());
        name.concat(strNum + ".csv");
        if (!SD.exists(name))
        {
            formattedFileName = name;
            break;
        }
    }
    return formattedFileName;
}

void writeHeader(String fileName)
{
    SdFile::dateTimeCallback(dateTime);

    File dataFile = SD.open(fileName, FILE_WRITE);
    if (dataFile)
    {
        dataFile.println("File Name," + fileName);
        String date = String(now.year()) + "-" + formatDigits(now.month()) + "-" + formatDigits(now.day()) + " " + formatDigits(now.hour()) + ":" + formatDigits(now.minute()) + ":" + formatDigits(now.second());
        dataFile.println("Date," + date);
        dataFile.println();

        dataFile.println("[DATA]");

        dataFile.print("time (s),");
        dataFile.print("oxy1_voltage (V),");
        dataFile.print("oxy1_O2 (%),");
        dataFile.print("oxy1_temp_voltage (V),");
        dataFile.print("oxy1_temp (C),");
        dataFile.print("oxy2_voltage (V),");
        dataFile.print("oxy2_O2 (%),");
        dataFile.print("oxy2_temp_voltage (V),");
        dataFile.print("oxy2_temp (C),");
        dataFile.print("bme_temp (C),");
        dataFile.print("pressure (kPa),");
        dataFile.print("humidity (%),");
        dataFile.print("flow1_voltage (V),");
        dataFile.print("flow1_flow (mL/min),");
        dataFile.print("flow2_voltage (V),");
        dataFile.print("flow2_flow (mL/min),");
        dataFile.print("PV (C),");
        dataFile.print("SV (C),");
        dataFile.print("Control Method,");
        dataFile.print("Output,");
        dataFile.print("PID profile,");
        dataFile.print("PID SV,");
        dataFile.print("P,");
        dataFile.print("I,");
        dataFile.print("D");
        dataFile.println();

        dataFile.close();
    }
    else
    {
        Serial.println("Error opening file");
    }
}

void writeData(String fileName)
{
    File dataFile = SD.open(fileName, FILE_WRITE);

    if (dataFile)
    {
        dataFile.print(now.unixtime());
        dataFile.print(",");
        dataFile.print(oxy1_adc * OXY_RES, 6);
        dataFile.print(",");
        dataFile.print(oxy1_percent);
        dataFile.print(",");
        dataFile.print(oxy1_temp_adc * OXY_TEMP_RES, 6);
        dataFile.print(",");
        dataFile.print(oxy1_temp);
        dataFile.print(",");
        dataFile.print(oxy2_adc * OXY_RES, 6);
        dataFile.print(",");
        dataFile.print(oxy2_percent);
        dataFile.print(",");
        dataFile.print(oxy2_temp_adc * OXY_TEMP_RES, 6);
        dataFile.print(",");
        dataFile.print(oxy2_temp);
        dataFile.print(",");
        dataFile.print(bme_temp);
        dataFile.print(",");
        dataFile.print(pressure);
        dataFile.print(",");
        dataFile.print(humidity);
        dataFile.print(",");
        dataFile.print(flow1_adc * 5.0 / 1024.0, 6);
        dataFile.print(",");
        dataFile.print(flow1_mLmin);
        dataFile.print(",");
        dataFile.print(flow2_adc * 5.0 / 1024.0, 6);
        dataFile.print(",");
        dataFile.print(flow2_mLmin);
        dataFile.print(",");
        dataFile.print(omega_pv);
        dataFile.print(",");
        dataFile.print(omega_sv);
        dataFile.print(",");
        dataFile.print(omega_ctrl_method);
        dataFile.print(",");
        dataFile.print(omega_out);
        dataFile.print(",");
        dataFile.print(omega_pid_num);
        dataFile.print(",");
        dataFile.print(omega_pid_sv);
        dataFile.print(",");
        dataFile.print(omega_p);
        dataFile.print(",");
        dataFile.print(omega_i);
        dataFile.print(",");
        dataFile.print(omega_d);
        dataFile.println();

        dataFile.close();
    }
    else
    {
        Serial.println("Error opening file");
    }
}