/* ******************************************************* */
/* I2C code for ADXL345 accelerometer                      */
/* and HMC5843 magnetometer                                */
/* ******************************************************* */

//int AccelAddress = 0x53;
//int CompassAddress = 0x1E;  //0x3C //0x3D;  //(0x42>>1);

#define I2C_MKMAG_ADDRESS 		0x50

#define I2C_MKMAG_CMD_VERSION   	0x01
#define I2C_MKMAG_CMD_READ_MAG     	0x02
#define I2C_MKMAG_CMD_READ_HEADING      0x03

#define I2C_MAXBUFF 16

volatile uint8_t i2cbuff[I2C_MAXBUFF]; 

void I2C_Init()
{
  //Wire.begin();
}

void Bussola_Init()
{
  
  //Wire.beginTransmission(I2C_MKMAG_ADDRESS);
  //Wire.send(I2C_MKMAG_CMD_VERSION);
  ////Wire.endTransmission(); //end transmission

  int i = 0;

/*
  //Wire.beginTransmission(I2C_MKMAG_ADDRESS);
  //Serial.println("BLOC");
  Wire.requestFrom(I2C_MKMAG_ADDRESS, 1);
  //Serial.println("CANTE");
  while(Wire.available())   // ((Wire.available())&&(i<6))
  { 
    i2cbuff[i] = Wire.receive();  // receive one byte
    i++;
  }
  */
  //Wire.endTransmission(); //end transmission


  
}


void Bussola_read()
{
  int i = 0;
  byte buff[6];
  /*
  Wire.beginTransmission(I2C_MKMAG_ADDRESS); 
  Wire.send(I2C_MKMAG_CMD_READ_HEADING);        //sends address to read from
  Wire.endTransmission(); //end transmission
  
  
  //Wire.beginTransmission(I2C_MKMAG_ADDRESS); //start transmission to device
  Wire.requestFrom(I2C_MKMAG_ADDRESS, 6);    // request 6 bytes from device
  
  while(Wire.available())   // ((Wire.available())&&(i<6))
  { 
    buff[i] = Wire.receive();  // receive one byte
    i++;
  }
  //Wire.endTransmission(); //end transmission
  
  if (i==6)  // All bytes received?
  {
    heading = buff[0];
    heading += buff[1] << 8;
  }
  else
  {
    Serial.println("!ERR: Acc data");
  }
  */
}









/*
void Accel_Init()
{
  Wire.beginTransmission(AccelAddress);
  Wire.send(0x2D);  // power register
  Wire.send(0x08);  // measurement mode
  Wire.endTransmission();
  delay(5);
  Wire.beginTransmission(AccelAddress);
  Wire.send(0x31);  // Data format register
  Wire.send(0x08);  // set to full resolution
  Wire.endTransmission();
  delay(5);	
  // Because our main loop runs at 50Hz we adjust the output data rate to 50Hz (25Hz bandwidth)
  Wire.beginTransmission(AccelAddress);
  Wire.send(0x2C);  // Rate
  Wire.send(0x09);  // set to 50Hz, normal operation
  Wire.endTransmission();
  delay(5);
}
*/

/*
// Reads x,y and z accelerometer registers
void Read_Accel()
{
  int i = 0;
  byte buff[6];
  
  Wire.beginTransmission(AccelAddress); 
  Wire.send(0x32);        //sends address to read from
  Wire.endTransmission(); //end transmission
  
  Wire.beginTransmission(AccelAddress); //start transmission to device
  Wire.requestFrom(AccelAddress, 6);    // request 6 bytes from device
  
  while(Wire.available())   // ((Wire.available())&&(i<6))
  { 
    buff[i] = Wire.receive();  // receive one byte
    i++;
  }
  Wire.endTransmission(); //end transmission
  
  if (i==6)  // All bytes received?
    {
    ACC[1] = (((int)buff[1]) << 8) | buff[0];    // Y axis (internal sensor x axis)
    ACC[0] = (((int)buff[3]) << 8) | buff[2];    // X axis (internal sensor y axis)
    ACC[2] = (((int)buff[5]) << 8) | buff[4];    // Z axis
    AN[3] = ACC[0];
    AN[4] = ACC[1];
    AN[5] = ACC[2];
    accel_x = SENSOR_SIGN[3]*(ACC[0]-AN_OFFSET[3]);
    accel_y = SENSOR_SIGN[4]*(ACC[1]-AN_OFFSET[4]);
    accel_z = SENSOR_SIGN[5]*(ACC[2]-AN_OFFSET[5]);
    }
  else
    Serial.println("!ERR: Acc data");
}
*/


/*
void Compass_Init()
{
  Wire.beginTransmission(CompassAddress);
  Wire.send(0x02); 
  Wire.send(0x00);   // Set continouos mode (default to 10Hz)
  Wire.endTransmission(); //end transmission
}
*/

/*
void Read_Compass()
{
  int i = 0;
  byte buff[6];
 
  Wire.beginTransmission(CompassAddress); 
  Wire.send(0x03);        //sends address to read from
  Wire.endTransmission(); //end transmission
  
  //Wire.beginTransmission(CompassAddress); 
  Wire.requestFrom(CompassAddress, 6);    // request 6 bytes from device
  while(Wire.available())   // ((Wire.available())&&(i<6))
  { 
    buff[i] = Wire.receive();  // receive one byte
    i++;
  }
  Wire.endTransmission(); //end transmission
  
  if (i==6)  // All bytes received?
    {
    // MSB byte first, then LSB, X,Y,Z
    magnetom_x = SENSOR_SIGN[6]*((((int)buff[2]) << 8) | buff[3]);    // X axis (internal sensor y axis)
    magnetom_y = SENSOR_SIGN[7]*((((int)buff[0]) << 8) | buff[1]);    // Y axis (internal sensor x axis)
    magnetom_z = SENSOR_SIGN[8]*((((int)buff[4]) << 8) | buff[5]);    // Z axis
    }
  else
    Serial.println("!ERR: Mag data");
}
*/


