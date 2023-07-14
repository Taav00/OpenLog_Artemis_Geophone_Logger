//Query the RTC and put the appropriately formatted (according to settings) 
//string into the passed buffer. timeStringBuffer should be at least 37 chars long
//Code modified by @DennisMelamed in PR #70
void getTimeString(char timeStringBuffer[])
{
  //reset the buffer
  timeStringBuffer[0] = '\0';

  myRTC.getTime();

  if (settings.logDate)
  {
    char rtcDate[12]; // 10/12/2019,
    char rtcDay[3];
    char rtcMonth[3];
    char rtcYear[5];
    if (myRTC.dayOfMonth < 10)
      sprintf(rtcDay, "0%d", myRTC.dayOfMonth);
    else
      sprintf(rtcDay, "%d", myRTC.dayOfMonth);
    if (myRTC.month < 10)
      sprintf(rtcMonth, "0%d", myRTC.month);
    else
      sprintf(rtcMonth, "%d", myRTC.month);
    if (myRTC.year < 10)
      sprintf(rtcYear, "200%d", myRTC.year);
    else
      sprintf(rtcYear, "20%d", myRTC.year);

      sprintf(rtcDate, "%s-%s-%sT", rtcYear, rtcMonth, rtcDay);
    strcat(timeStringBuffer, rtcDate);
  }

  if ((settings.logTime) || ((settings.logDate)))
  {
    char rtcTime[16]; //09:14:37.41, or 09:14:37+00:00,
    int adjustedHour = myRTC.hour;
    if (settings.hour24Style == false)
    {
      if (adjustedHour > 12) adjustedHour -= 12;
    }
    char rtcHour[3];
    char rtcMin[3];
    char rtcSec[3];
    char rtcHundredths[3];
    char timeZoneH[4];
    char timeZoneM[4];
    if (adjustedHour < 10)
      sprintf(rtcHour, "0%d", adjustedHour);
    else
      sprintf(rtcHour, "%d", adjustedHour);
    if (myRTC.minute < 10)
      sprintf(rtcMin, "0%d", myRTC.minute);
    else
      sprintf(rtcMin, "%d", myRTC.minute);
    if (myRTC.seconds < 10)
      sprintf(rtcSec, "0%d", myRTC.seconds);
    else
      sprintf(rtcSec, "%d", myRTC.seconds);
    if (myRTC.hundredths < 10)
      sprintf(rtcHundredths, "0%d", myRTC.hundredths);
    else
      sprintf(rtcHundredths, "%d", myRTC.hundredths);
    if (settings.localUTCOffset >= 0)
    {
      if (settings.localUTCOffset < 10)
        sprintf(timeZoneH, "+0%d", (int)settings.localUTCOffset);
      else
        sprintf(timeZoneH, "+%d", (int)settings.localUTCOffset);
    }
    else
    {
      if (settings.localUTCOffset <= -10)
        sprintf(timeZoneH, "-%d", 0 - (int)settings.localUTCOffset);
      else
        sprintf(timeZoneH, "-0%d", 0 - (int)settings.localUTCOffset);
    }
    int tzMins = (int)((settings.localUTCOffset - (float)((int)settings.localUTCOffset)) * 60.0);
    if (tzMins < 0)
      tzMins = 0 - tzMins;
    if (tzMins < 10)
      sprintf(timeZoneM, ":0%d", tzMins);
    else
      sprintf(timeZoneM, ":%d", tzMins);
    if ((settings.logDate))
    {
      sprintf(rtcTime, "%s:%s:%s%s%s,", rtcHour, rtcMin, rtcSec, timeZoneH, timeZoneM);
      strcat(timeStringBuffer, rtcTime);      
    }
    if (settings.logTime)
    {
      sprintf(rtcTime, "%s:%s:%s.%s,", rtcHour, rtcMin, rtcSec, rtcHundredths);
      strcat(timeStringBuffer, rtcTime);
    }
  }
}

void getDateTime()
{
  dateTime[0] = '\0'; //Clear string contents

  if (settings.logRTC)
  {
    //Decide if we are using the internal RTC or GPS for timestamps
    if (settings.getRTCfromGPS == false)
    {
      myRTC.getTime();

      if (settings.logDate)
      {
        char rtcDate[12]; //10/12/2019,
        if (settings.americanDateStyle == true)
          sprintf(rtcDate, "%02d/%02d/20%02d,", myRTC.month, myRTC.dayOfMonth, myRTC.year);
        else
          sprintf(rtcDate, "%02d/%02d/20%02d,", myRTC.dayOfMonth, myRTC.month, myRTC.year);
        strcat(dateTime, rtcDate);
      }

      if (settings.logTime)
      {
        char rtcTime[13]; //09:14:37.41,
        int adjustedHour = myRTC.hour;
        if (settings.hour24Style == false)
        {
          if (adjustedHour > 12) adjustedHour -= 12;
        }
        sprintf(rtcTime, "%02d:%02d:%02d.%02d,", adjustedHour, myRTC.minute, myRTC.seconds, myRTC.hundredths);
        strcat(dateTime, rtcTime);
      }
    } //end if use RTC for timestamp
    else //Use GPS for timestamp
    {
      if (settings.serialPlotterMode == false) Serial.println("Print GPS Timestamp / not yet implemented");
    }
  }
}

//Read the ADC value as int16_t (2 bytes -32,768 to 32,767)
int16_t gatherADCValue()
{
  node *temp = head;
  while (temp != NULL)
  {
    //If this node successfully begin()'d
    if (temp->online == true)
    {
      openConnection(temp->muxAddress, temp->portNumber); //Connect to this device through muxes as needed

      //Switch on device type to set proper class and setting struct
      switch (temp->deviceType)
      {
        case DEVICE_GPS_UBLOX:
          {
            //SFE_UBLOX_GPS *nodeDevice = (SFE_UBLOX_GPS *)temp->classPtr;
            //struct_uBlox *nodeSetting = (struct_uBlox *)temp->configPtr;
          }
          break;
        case DEVICE_ADC_ADS122C04:
          {
            SFE_ADS122C04 *nodeDevice = (SFE_ADS122C04 *)temp->classPtr;
            struct_ADS122C04 *nodeSetting = (struct_ADS122C04 *)temp->configPtr;
    
            // Union to simplify converting from uint16_t to int16_t
            // without using a cast
            union ADC_conversion_union{
              int16_t INT16;
              uint16_t UINT16;
            } ADC_conversion;
            
            // Read the raw (signed) ADC data
            // The ADC data is returned in the least-significant 24-bits
            uint32_t raw_ADC_data = nodeDevice->readADC();
            nodeDevice->start(); // Start the next conversion
            ADC_conversion.UINT16 = (raw_ADC_data >> 8) & 0xffff; // Truncate to 16-bits (signed)
            return(ADC_conversion.INT16); // Return the signed version
          }
          break;
        case DEVICE_ADS1015:
          {
            ADS1015 *nodeDevice = (ADS1015 *)temp->classPtr;
            struct_ADS1015 *nodeSetting = (struct_ADS1015 *)temp->configPtr;
            
            int16_t channel_0 = nodeDevice->getSingleEndedSigned(0);
            return(channel_0); 
          }
          break;
        default:
          Serial.printf("gatherADCvalues : printDeviceValue unknown device type: %s\n", getDeviceName(temp->deviceType));
          break;
      }
    }
    temp = temp->next;
  }
  return(0);
}

//Configure the ADC for raw measurements
void configureADC()
{
  node *temp = head;
  while (temp != NULL)
  {
    //If this node successfully begin()'d
    if (temp->online == true)
    {
      openConnection(temp->muxAddress, temp->portNumber); //Connect to this device through muxes as needed

      //Switch on device type to set proper class and setting struct
      switch (temp->deviceType)
      {
        case DEVICE_GPS_UBLOX:
          {
            //SFE_UBLOX_GPS *nodeDevice = (SFE_UBLOX_GPS *)temp->classPtr;
            //struct_uBlox *nodeSetting = (struct_uBlox *)temp->configPtr;
          }
          break;
        case DEVICE_ADC_ADS122C04:
          {
            SFE_ADS122C04 *nodeDevice = (SFE_ADS122C04 *)temp->classPtr;
            struct_ADS122C04 *nodeSetting = (struct_ADS122C04 *)temp->configPtr;
    
            nodeDevice->setInputMultiplexer(ADS122C04_MUX_AIN1_AIN0); // Route AIN1 and AIN0 to AINP and AINN
            if (settings.geophoneGain == 128)
            {
              nodeDevice->setGain(ADS122C04_GAIN_128); // Set the gain to 128
              nodeDevice->enablePGA(ADS122C04_PGA_ENABLED); // Enable the Programmable Gain Amplifier
            }
            else if (settings.geophoneGain == 64)
            {
              nodeDevice->setGain(ADS122C04_GAIN_64); // Set the gain to 64
              nodeDevice->enablePGA(ADS122C04_PGA_ENABLED); // Enable the Programmable Gain Amplifier
            }
            else if (settings.geophoneGain == 32)
            {
              nodeDevice->setGain(ADS122C04_GAIN_32); // Set the gain to 32
              nodeDevice->enablePGA(ADS122C04_PGA_ENABLED); // Enable the Programmable Gain Amplifier
            }
            else if (settings.geophoneGain == 16)
            {
              nodeDevice->setGain(ADS122C04_GAIN_16); // Set the gain to 16
              nodeDevice->enablePGA(ADS122C04_PGA_ENABLED); // Enable the Programmable Gain Amplifier
            }
            else if (settings.geophoneGain == 8)
            {
              nodeDevice->setGain(ADS122C04_GAIN_8); // Set the gain to 8
              nodeDevice->enablePGA(ADS122C04_PGA_ENABLED); // Enable the Programmable Gain Amplifier
            }
            else if (settings.geophoneGain == 4)
            {
              nodeDevice->setGain(ADS122C04_GAIN_4); // Set the gain to 4
              nodeDevice->enablePGA(ADS122C04_PGA_DISABLED); // Disable the Programmable Gain Amplifier
            }
            else if (settings.geophoneGain == 2)
            {
              nodeDevice->setGain(ADS122C04_GAIN_2); // Set the gain to 2
              nodeDevice->enablePGA(ADS122C04_PGA_DISABLED); // Disable the Programmable Gain Amplifier
            }
            else
            {
              nodeDevice->setGain(ADS122C04_GAIN_1); // Set the gain to 1
              nodeDevice->enablePGA(ADS122C04_PGA_DISABLED); // Disable the Programmable Gain Amplifier
            }
            nodeDevice->setDataRate(ADS122C04_DATA_RATE_600SPS); // Set the data rate (samples per second) to 600
            nodeDevice->setOperatingMode(ADS122C04_OP_MODE_NORMAL); // Disable turbo mode
            nodeDevice->setConversionMode(ADS122C04_CONVERSION_MODE_SINGLE_SHOT); // Use single shot mode
            nodeDevice->setVoltageReference(ADS122C04_VREF_INTERNAL); // Use the internal 2.048V reference
            nodeDevice->enableInternalTempSensor(ADS122C04_TEMP_SENSOR_OFF); // Disable the temperature sensor
            nodeDevice->setDataCounter(ADS122C04_DCNT_DISABLE); // Disable the data counter (Note: the library does not currently support the data count)
            nodeDevice->setDataIntegrityCheck(ADS122C04_CRC_DISABLED); // Disable CRC checking (Note: the library does not currently support data integrity checking)
            nodeDevice->setBurnOutCurrent(ADS122C04_BURN_OUT_CURRENT_OFF); // Disable the burn-out current
            nodeDevice->setIDACcurrent(ADS122C04_IDAC_CURRENT_OFF); // Disable the IDAC current
            nodeDevice->setIDAC1mux(ADS122C04_IDAC1_DISABLED); // Disable IDAC1
            nodeDevice->setIDAC2mux(ADS122C04_IDAC2_DISABLED); // Disable IDAC2

            if((settings.printDebugMessages == true) && (settings.serialPlotterMode == false))
            {
              nodeDevice->enableDebugging(Serial); //Enable debug messages on Serial
              nodeDevice->printADS122C04config(); //Print the configuration
              nodeDevice->disableDebugging(); //Enable debug messages on Serial
            }
    
            nodeDevice->start(); // Start the first conversion
          }
          break;
        case DEVICE_ADS1015:
          {
            ADS1015 *nodeDevice = (ADS1015 *)temp->classPtr;
            struct_ADS1015 *nodeSetting = (struct_ADS1015 *)temp->configPtr;
            nodeDevice->setSampleRate(ADS1015_CONFIG_RATE_1600HZ);
          }
          break;
        default:
          Serial.printf("configureADC : printDeviceValue unknown device type: %s\n", getDeviceName(temp->deviceType));
          break;
      }
    }
    temp = temp->next;
  }
}

//If certain devices are attached, we need to reduce the I2C max speed
void setMaxI2CSpeed()
{
  uint32_t maxSpeed = 400000; //Assume 400kHz - but beware! 400kHz with no pull-ups can cause issues.

  //Search nodes for Ublox modules
  node *temp = head;
  while (temp != NULL)
  {
    if (temp->deviceType == DEVICE_GPS_UBLOX)
    {
      //Check if i2cSpeed is lowered
      struct_uBlox *sensor = (struct_uBlox*)temp->configPtr;
      if (sensor->i2cSpeed == 100000)
        maxSpeed = 100000;
    }

    temp = temp->next;
  }

  //If user wants to limit the I2C bus speed, do it here
  if (maxSpeed > settings.qwiicBusMaxSpeed)
    maxSpeed = settings.qwiicBusMaxSpeed;

  qwiic.setClock(maxSpeed);
}

//Read the VIN voltage
float readVIN()
{
  // Only supported on >= V10 hardware
#if(HARDWARE_VERSION_MAJOR == 0)
  return(0.0); // Return 0.0V on old hardware
#else
  int div3 = analogRead(PIN_VIN_MONITOR); //Read VIN across a 1/3 resistor divider
  float vin = (float)div3 * 3.0 * 2.0 / 16384.0; //Convert 1/3 VIN to VIN (14-bit resolution)
  vin = vin * 1.021; //Correct for divider impedance (determined experimentally)
  return (vin);
#endif
}
