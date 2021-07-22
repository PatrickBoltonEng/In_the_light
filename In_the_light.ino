/*
 * Project In_the_light
 * Description:  Seeed UV Sensor Based on GUVA-S12D SKU 101020043  & Sunlight Sensor SI1145
 * Author:  PJB
 * Date:    5/20/21
 */
#include "Particle.h"
#include "Nextion.h"
#include "math.h"
#include <Wire.h>
#include "SI114X.h"
#include "tsl2561.h"
#include "JsonParserGeneratorRK.h"    //install it JsonParserGeneratorRK

SYSTEM_THREAD(ENABLED);

USARTSerial& nexSerial = Serial1;
#define DEBUG_SERIAL_ENABLE
#define dbSerial Serial

#define UPDATE_INTERVAL 10000  //1 sec = 1000 millis

SI114X SI1145 = SI114X();

TSL2561 tsl(TSL2561_ADDR_0);
		// TSL2561_ADDR_0 (0x29 address with '0', connected to GND)
		// TSL2561_ADDR   (0x39 default address, pin floating)
		// TSL2561_ADDR_1 (0x49 address with '1' connected to VIN)

// TSL sensor related vars
uint16_t tsl_integrationTime;
double tsl_illuminance;
uint32_t tsl_illuminance_int;
bool tsl_autoGainOn;
// TSL execution control var
bool tsl_operational;
// TSL status vars
char tsl_status[21] = "na";
char tsl_autoGain_s[4] = "na";
uint8_t tsl_error_code;
uint8_t tsl_gain_setting;
int tsl_vis, tsl_ifr;
//Si1145
int SI_UVI, SI_VIS, SI_IR;
//GUVA-12SD
float GU_UVI;
//Grove Light Sensor v1.2 with LS06-MØ5
const int LS06_pin= A4;               //Connect the LED Grove A2 on Boron
const int LS_threshold=10;            //The threshold for which the LED should turn on. 
int LS_sensorV;                       //sensor analog read
float LS_ave;                         //average sensor reading
float LS_lux;                         //Resistance in K, 

//timegrab
int hr_i, min_i, ampm_i, weekday_i, month_i, day_i, min_time, min_last;
String ampm_s, weekday_s, month_s;

unsigned long UpdateInterval;

SerialLogHandler logHandler(LOG_LEVEL_INFO);

//page0
NexNumber n000 = NexNumber(0, 1, "n000");       //hour
NexNumber n001 = NexNumber(0, 2, "n001");       //minute
NexText t000 = NexText(0, 3, "t000");           //AM-PM 
NexText t001 = NexText(0, 4, "t001");           //weekday 
NexText t002 = NexText(0, 5, "t002");           //month 
NexNumber n002 = NexNumber(0, 6, "n002");       //day-date

NexNumber n100 = NexNumber(1, 21, "n100");        //Si1145 UVI
NexNumber n101 = NexNumber(1, 22, "n101");        //Si1145 VIS
NexNumber n102 = NexNumber(1, 23, "n102");        //Si1145 IR
NexNumber n103 = NexNumber(1, 24, "n103");        //GUVA-S12D UVI 
NexNumber n104 = NexNumber(1, 28, "n104");        //TSL-2561 Illumination (lux)
NexNumber n105 = NexNumber(1, 32, "n105");        //LS06-MØ5 Illum (lux)

NexTouch *nex_listen_list[] = 
{ 
  NULL
};

void setup()
{ 
  Wire.begin();
  delay(100);
  
  Serial.begin(9600);
  delay(100);

  Serial1.begin(9600);
  delay(100);

  nexInit();
  delay(100);

  while (!SI1145.Begin())
  {
    Log.info("Si1145 is not ready!");
    delay(1000);
  }
  Log.info("Si1145 is ready!");

  TSLsetup();

  //function on the cloud: change sensor exposure settings (mqx 4)
  Particle.function("setExposure", setExposure);

  pinMode(A2, INPUT);  //analog GUVA UV light sensor

  pinMode(A4, INPUT);  //analog LS06-MØ5 VIS-IR light sensor

  Log.info("Setup Complete");

  UpdateInterval = millis();
  min_last=-1;
  min_time=0;
}

void loop()
{
  Time.zone(-7);

  if(Particle.disconnected()){return;}

  if(millis() - UpdateInterval > UPDATE_INTERVAL)
  {
    timegrab();
    n000.setValue(hr_i);
    n001.setValue(min_i);
    t000.setText(ampm_s);
    t001.setText(weekday_s);
    t002.setText(month_s);
    n002.setValue(day_i);
    
    getGUVASensor();
    int GU_UVIi = int(GU_UVI*100.0);  //formatting for display
    //Log.info("GU-UVI: %d", GU_UVIi);

    getSi1145UVVISIR();

    getTSLdata();
    int tsl_illum = int(tsl_illuminance);
    //Log.info("TSL2561_Ill: %f", tsl_illuminance);
    //Log.info("TSL2561_Ill: %d", tsl_illum);
    
    int LS_sum = 0;
    for(int i=0; i<256; i++)
    {
      int LS_sensorV = analogRead(A4); 
      delayMicroseconds(12);
      LS_sum = LS_sum  + LS_sensorV;
    }
    float LS_ave = float(LS_sum)/256.0;
    LS_lux=LS_ave*(3300.0/4095.0)*(3.3/5.0);
    int LS_luxi = int(LS_lux);
    //Log.info("LS Sensor Ave: %f", LS_ave);
    //Log.info("LS Sensor Lux: %f", LS_lux);
    //Log.info("LS Sensor Lux int: %d", LS_luxi);
    

    n100.setValue(SI_UVI);
    n101.setValue(SI_VIS);
    n102.setValue(SI_IR);
    n103.setValue(GU_UVIi);
    n104.setValue(tsl_illum);
    n105.setValue(LS_luxi);



    min_time=Time.minute();
        if((min_time!=min_last)&&(min_time==0||min_time==10||min_time==20||min_time==30||min_time==40||min_time==50))
    {
      createEventPayload(GU_UVIi, SI_UVI, SI_VIS, SI_IR, tsl_illum, LS_luxi);
      min_last = min_time;
      Log.info("Last Update: %d", min_last);
      Log.info(Time.timeStr());
    }
    UpdateInterval = millis();
  }
}

void timegrab()
{
  Time.zone(-7);
  hr_i=Time.hourFormat12();
  min_i=Time.minute();
  ampm_i = Time.isPM();
  if(ampm_i==1){ampm_s=String("PM");}
  else{ampm_s=String("AM");}
  weekday_i=Time.weekday();
  if(weekday_i==1){weekday_s=String("SUN");}
  if(weekday_i==2){weekday_s=String("MON");}
  if(weekday_i==3){weekday_s=String("TUES");}
  if(weekday_i==4){weekday_s=String("WED");}
  if(weekday_i==5){weekday_s=String("THUR");}
  if(weekday_i==6){weekday_s=String("FRI");}
  if(weekday_i==7){weekday_s=String("SAT");}
  month_i=Time.month();
  if(month_i==1){month_s=String("JAN");}
  if(month_i==2){month_s=String("FEB");}
  if(month_i==3){month_s=String("MAR");}
  if(month_i==4){month_s=String("APR");}
  if(month_i==5){month_s=String("MAY");}
  if(month_i==6){month_s=String("JUNE");}
  if(month_i==7){month_s=String("JULY");}
  if(month_i==8){month_s=String("AUG");}
  if(month_i==9){month_s=String("SEPT");}
  if(month_i==10){month_s=String("OCT");}
  if(month_i==11){month_s=String("NOV");}
  if(month_i==12){month_s=String("DEC");}
  day_i=Time.day();
  /*
  Log.info("Hour: %d", hr_i);
  Log.info("Min: %d", min_i);
  Log.info("Is PM: %d", ampm_i);
  Log.info(ampm_s);
  Log.info("Weekday: %d", weekday_i);
  Log.info(weekday_s);
  Log.info("Month: %d", month_i);
  Log.info(month_s);
  Log.info("Day: %d", day_i);
  */
  return;
}

void getGUVASensor()
{
  int gu_sensorvalue;
  int gu_sum=0;
  int gu_sensorreadings=256;

  for(int i=0; i<gu_sensorreadings; i++)
  {
    gu_sensorvalue=analogRead(A2);     //0-3.3V mapped to 0-4095
    delay(10);
    gu_sum = gu_sum + gu_sensorvalue;
  }

  //per GUVA-S12SD datasheet: Photocurrent(nA)=113xUVPower(mW/cm2)
  //per GUVA-S12SD datasheet: Photocurrent(nA)=21xUVIndex+83
  //per Gove Data schematic Photocurrent(nA)= VsensorxR2x10^9/((R1+R2)*R3)), where R1=3.3kOhm, R2=1kOhm, R3=10MOhm for v1.1, 1Mohm for version v1.1b - confirmed v1.1  
  float gu_sensorave = float(gu_sum)/float(gu_sensorreadings);
  float GU_PC = gu_sensorave*(3300.0f/4095.0f)*1000000000.0f*1000.0f/((3300.0f+1000.0f)*10000000.0f);       //nA
  GU_UVI = (GU_PC)/21.0f;
  //Log.info("GUVA SensorAve: %f", gu_sensorave);
  //Log.info("Photocurrent(nA): %f", GU_PC);
  //Log.info("GU_UVI: %f", GU_UVI);
  return;
}

void getSi1145UVVISIR()
{
  SI_UVI=SI1145.ReadUV();
  SI_VIS=SI1145.ReadVisible();
  SI_IR=SI1145.ReadIR();

  //Log.info("SI_UVI: %d", SI_UVI);
  //Log.info("SI_VIS: %d", SI_VIS);
  //Log.info("SI_IR: %d", SI_IR);
  return;
}

int setExposure(String command)
// cloud function to change exposure settings (gain and integration time)
//command is expected to be [gain={0,1,2},integrationTimeSwitch={0,1,2}]
// gain = 0:x1, 1: x16, 2: auto
// integrationTimeSwitch: 0: 14ms, 1: 101ms, 2:402ms
{
    // private vars
    char tsl_gainInput;
    uint8_t tsl_itSwitchInput;
    boolean tsl__setTimingReturn = false;
    // extract gain as char and integrationTime swithc as byte
    tsl_gainInput = command.charAt(0);//we expect 0, 1 or 2
    tsl_itSwitchInput = command.charAt(2) - '0';//we expect 0,1 or 2
    if (tsl_itSwitchInput >= 0 && tsl_itSwitchInput < 3){
      // acceptable integration time value, now check gain value
      if (tsl_gainInput=='0'){
        tsl__setTimingReturn = tsl.setTiming(false,tsl_itSwitchInput,tsl_integrationTime);
        tsl_autoGainOn = false;
      }
      else if (tsl_gainInput=='1') {
        tsl__setTimingReturn = tsl.setTiming(true,tsl_itSwitchInput,tsl_integrationTime);
        tsl_autoGainOn = false;
      }
      else if (tsl_gainInput=='2') {
        tsl_autoGainOn = true;
        // when auto gain is enabled, set starting gain to x16
        tsl__setTimingReturn = tsl.setTiming(true,tsl_itSwitchInput,tsl_integrationTime);
      }
      else{
        // no valid settings, raise error flag
        tsl__setTimingReturn = false;
      }
    }
    else{
      tsl__setTimingReturn = false;
    }

    // setTiming has an error
    if(!tsl__setTimingReturn){
        // set appropriate status variables
        tsl_error_code = tsl.getError();
        strcpy(tsl_status,"CloudSettingsError");
        //disable getting illuminance value
        tsl_operational = false;
        return -1;
    }
    else {
      // all is good
      tsl_operational = true;
      return 0;
    }
}

void TSLsetup()
{
  tsl_error_code = 0;
  tsl_operational = false;
  tsl_autoGainOn = false;

  //connecting to light sensor device
  if (tsl.begin()) {
    strcpy(tsl_status,"tsl2561 found");
  }
  else {
    strcpy(tsl_status,"tsl 2561 not found ");
  }

  // setting the sensor: gain x1 and 101ms integration time
  if(!tsl.setTiming(false,1,tsl_integrationTime))
  {
    tsl_error_code = tsl.getError();
    strcpy(tsl_status,"setTimingError");
    return;
  }

  if (!tsl.setPowerUp())
  {
    tsl_error_code = tsl.getError();
    strcpy(tsl_status,"PowerUPError");
    return;
  }

  // device initialized
  tsl_operational = true;
  strcpy(tsl_status,"initOK");
  return;
}

void getTSLdata()
{
  uint16_t tsl_broadband, tsl_ir;
  // update exposure settings display vars
  if (tsl._gain){tsl_gain_setting = 16;}
  else{tsl_gain_setting = 1;}

  if (tsl_autoGainOn){strcpy(tsl_autoGain_s,"yes");}
  else{strcpy(tsl_autoGain_s,"no");}

  if (tsl_operational)
  {
    // device operational, update status vars
    strcpy(tsl_status,"OK");
    // get raw data from sensor
    if(!tsl.getData(tsl_broadband,tsl_ir,tsl_autoGainOn))
    {
      tsl_error_code = tsl.getError();
      strcpy(tsl_status,"saturated?");
      tsl_operational = false;

    }
    // compute illuminance value in lux
    if(!tsl.getLux(tsl_integrationTime,tsl_broadband,tsl_ir,tsl_illuminance))
    {
      tsl_error_code = tsl.getError();
      strcpy(tsl_status,"getLuxError");
      tsl_operational = false;
    }
    // try the integer based calculation
    if(!tsl.getLuxInt(tsl_broadband,tsl_ir,tsl_illuminance_int))
    {
      tsl_error_code = tsl.getError();
      strcpy(tsl_status,"getLuxIntError");
      tsl_operational = false;
    }
  }
  else
  // device not set correctly
  {
    strcpy(tsl_status,"OperationError");
    tsl_illuminance = -1.0;
    // trying a fix
    // power down the sensor
    tsl.setPowerDown();
    delay(100);
    // re-init the sensor
    if (tsl.begin())
    {
      // power up
      tsl.setPowerUp();
      // re-configure
      tsl.setTiming(tsl._gain,1,tsl_integrationTime);
      // try to go back normal again
      tsl_operational = true;
    }
  }
}

void createEventPayload(int GU_UVIi, int SI_UVI, int SI_VIS, int SI_IR, int tsl_illum, int LS_luxi)
{
  JsonWriterStatic<256> jw;
  {
    JsonWriterAutoObject obj(&jw);
    jw.insertKeyValue("GU_UVIx100", GU_UVIi);
    jw.insertKeyValue("SI_UVIx100", SI_UVI);
    jw.insertKeyValue("SI_VIS", SI_VIS);
    jw.insertKeyValue("SI_IR", SI_IR);
    jw.insertKeyValue("TSL_Ill", tsl_illum);
    jw.insertKeyValue("LS_Ill", LS_luxi);
  }
  Particle.publish("Inthelight", jw.getBuffer(), PRIVATE);
}