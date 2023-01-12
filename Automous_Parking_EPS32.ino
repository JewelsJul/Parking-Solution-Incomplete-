#include <Wire.h>
#include <HCSR04.h>
#include <MPU6500_WE.h>
#define I2C_SLAVE_ADDR 0x04 // 4 in hexadecimal


// the minimum and maximum values here are determined by the amount of bits used by the chosen variable type
// for int, this is either 16-bits or 32-bits
// due to two's complement, the minimum value is -2^(N-1), and the maximum is (2^(N-1))-1; where N is the number of bits

MPU6500_WE myMPU6500 = MPU6500_WE(0x68);
byte triggerPin = 2;
byte echoPin= 13;

void CarSpeedAngle(int x,int y,int z){
    Wire.beginTransmission(0x04); // transmit to device #4    >> X refers to a shift right operator by X bits
  
    Wire.write((byte)((x & 0x0000FF00) >> 8));    // first byte of left, containing bits 16 to 9  
    Wire.write((byte)(x & 0x000000FF));           // second byte of left, containing the 8 LSB - bits 8 to 1
    Wire.write((byte)((y & 0x0000FF00) >> 8));    // first byte of right, containing bits 16 to 9
    Wire.write((byte)(y & 0x000000FF));           // second byte of right, containing the 8 LSB - bits 8 to 1
    Wire.write((byte)((z & 0x0000FF00) >> 8));    // first byte of angle, containing bits 16 to 9
    Wire.write((byte)(z & 0x000000FF));           // second byte of angle, containing the 8 LSB - bits 8 to 1
  
    Wire.endTransmission();   // stop transmitting
    }

void ReversingDistance(){
    Wire.requestFrom(0x04,16);
    int BaseLeftEnco=Wire.read();
    int BaseRightEnco=Wire.read();
    Serial.println(BaseLeftEnco);
    Serial.println(BaseRightEnco);
    delay(200);
    Wire.requestFrom(0x04,16);
    int LeftEnco=Wire.read();
    int RightEnco=Wire.read();
    CarSpeedAngle(-150,-150,90);
    while (LeftEnco!=BaseLeftEnco+10||RightEnco!=BaseRightEnco+10){
      Serial.println(LeftEnco);
      Serial.println(RightEnco);
      Wire.requestFrom(0x04,16);
      LeftEnco=Wire.read();
      RightEnco=Wire.read();
      }
    CarSpeedAngle(0,0,90);
  }

void setup()
{
  Serial.begin(115200);
  Wire.begin();   // join i2c bus (address optional for the master) - on the Arduino NANO the default I2C pins are A4 (SDA), A5 (SCL)

  Serial.println("Hold still\n");
  delay(1000);
  myMPU6500.autoOffsets();
  myMPU6500.enableGyrDLPF();
  myMPU6500.setGyrDLPF(MPU6500_DLPF_6);
  myMPU6500.setSampleRateDivider(5);
  myMPU6500.setGyrRange(MPU6500_GYRO_RANGE_250);
  myMPU6500.setAccRange(MPU6500_ACC_RANGE_2G);
  myMPU6500.enableAccDLPF(true);
  myMPU6500.setAccDLPF(MPU6500_DLPF_6);
  delay(200);
  
  HCSR04.begin(triggerPin,echoPin);     //Tells where the UltraSonic Sensor is connected to on the ESP32
}

void loop()
{
  xyzFloat gValue = myMPU6500.getGValues();
  xyzFloat gyr = myMPU6500.getGyrValues();
  float temp = myMPU6500.getTemperature();
  float resultantG = myMPU6500.getResultantG(gValue);

  Serial.println("Acceleration in g (x,y,z):");
  Serial.println(gValue.x);
  Serial.print("   ");
  Serial.println(gValue.y);
  Serial.print("   ");
  Serial.println(gValue.z);
  Serial.print("Resultant g: ");
  Serial.println(resultantG);
  Serial.println("Gyroscope data in degrees/s: ");
  Serial.println(gyr.x);
  Serial.print("   ");
  Serial.println(gyr.y);
  Serial.print("   ");
  Serial.println(gyr.z);
  Serial.print("Temperature in °C: ");
  Serial.println(temp);
  Serial.println("********************************************");
  delay(1000);

///////////////////////////////////////////////////////////////////////////////
  
  double* distances = HCSR04.measureDistanceCm();
  
  Serial.print("1: ");
  Serial.print(distances[0]);
  Serial.println(" cm");
  
  Serial.println("---");
  delay(250);

////////////////////////////////////////////////////////////////////////////////////
int Turn;
Turn=0;
CarSpeedAngle(255,255,90);
delay(1000);
CarSpeedAngle(0,0,90);
delay(200);
CarSpeedAngle(150,150,45);
while (Turn<=180){
      xyzFloat gyr = myMPU6500.getGyrValues();
      Serial.println(gyr.z);
      Turn=Turn+gyr.z;
      delay(1500);
      }
CarSpeedAngle(0,0,90);
delay(200);
CarSpeedAngle(-100,-100,90);
while (distances[0]>10){
      distances = HCSR04.measureDistanceCm();
      Serial.println(distances[0]);
      }
CarSpeedAngle(0,0,90);

delay(500);                //Method A
Turn=0;
CarSpeedAngle(150,150,135);
while (Turn>=-90){
      xyzFloat gyr = myMPU6500.getGyrValues();
      Serial.println(gyr.z);
      Turn=Turn+gyr.z;
      delay(1500);
      }
CarSpeedAngle(0,0,90);
delay(200);
CarSpeedAngle(-100,-100,90);
while (distances[0]>10){
      distances = HCSR04.measureDistanceCm();
      Serial.println(distances[0]);
      }
CarSpeedAngle(0,0,90);

delay(500);                //Method B
CarSpeedAngle(150,150,135);
while (Turn>=-90){
      xyzFloat gyr = myMPU6500.getGyrValues();
      Serial.println(gyr.z);
      Turn=Turn+gyr.z;
      delay(1500);
      }
delay(200);

ReversingDistance();
}
