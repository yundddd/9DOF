#include <Kalman.h>
#include <Wire.h>
#include <L3G.h>
#include <LSM303.h>
#define TWI_FREQ 400000L

LSM303 compass;
L3G gyro;
float gyroOffSet[3];
int accOffSet[3];


double accG[3];
double gyroSpeed[3];
double mag[3];

uint32_t timer;
uint32_t oldTimer;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  TWBR = ((F_CPU / TWI_FREQ) - 16) / 2;//set I2C to 400KHZ
  pinMode (13,OUTPUT);  // Status LED
  digitalWrite(13,LOW);
  initCompass();
  initgyro();
  delay(100);
  calibrateAll();
  digitalWrite(13,HIGH);
  //done calibrating
 //  timer = micros();
}
void loop(){//do loop as fast as possible 
 // oldTimer=timer;
 // timer = micros();
  
  read_Acc();
  read_Gyro();
  read_Mag();
  
 // Serial.print(oldTimer);Serial.print("  ");Serial.println(timer);
  print_Acc();Serial.print(",");
  print_Gyro();Serial.print(",");  
  print_Mag();//Serial.print(",");  
  Serial.println(" ");
}







void initCompass(){
  compass.init();
  compass.enableDefault(); 
  compass.writeReg(LSM303::CTRL_REG4_A, 0x28); // 8 g full scale: FS = 10; high resolution output mode
}
void initgyro(){
  if (!gyro.init())
  {
    Serial.println("Failed to autodetect gyro type!");
    while (1);
  }
  gyro.writeReg(L3G_CTRL_REG4, 0x20); // 2000 dps full scale, 70mdeg per LSB
  gyro.writeReg(L3G_CTRL_REG1, 0x0F); // normal power mode, all axes enabled, 95 Hz
}
void calibrateAll(){
   compass.m_min = (LSM303::vector<int16_t>){-459, -864, -879};
   compass.m_max = (LSM303::vector<int16_t>){+598, +198, +551};
   for(int i=0;i<64;i++){//calibrate all sensors
     compass.read();
     gyro.read();
     accOffSet[0]+=compass.a.x;
     accOffSet[1]+=compass.a.y;
     accOffSet[2]+=compass.a.z;
     gyroOffSet[0]+=(int)gyro.g.x;
     gyroOffSet[1]+=(int)gyro.g.y;
     gyroOffSet[2]+=(int)gyro.g.z;
     delay(20);
  }
  accOffSet[0]/=64;
  accOffSet[1]/=64;
  accOffSet[2]/=64;
  gyroOffSet[0]/=64;
  gyroOffSet[1]/=64;
  gyroOffSet[2]/=64; 
  
 // accOffSet[0]=-90;
 // accOffSet[1]=11;
//  accOffSet[2]=-128;
}
void read_Acc(){//g
  compass.read();
  accG[0]=((compass.a.x-accOffSet[0])>>4)*0.004;
  accG[1]=((compass.a.y-accOffSet[1])>>4)*0.004;
  accG[2]=((compass.a.z-accOffSet[2])>>4)*0.004;
}
void read_Gyro(){//deg per s
  gyro.read();
  gyroSpeed[0]=(gyro.g.x-gyroOffSet[0])*0.07;
  gyroSpeed[1]=(gyro.g.y-gyroOffSet[1])*0.07;
  gyroSpeed[2]=(gyro.g.z-gyroOffSet[2])*0.07; 
}
void read_Mag(){//guass  x,y 450LSB/Guass, z 400LSB/Guass
  mag[0]=compass.m.x/450.0;
  mag[1]=compass.m.y/450.0;
  mag[2]=compass.m.z/400.0;  
}
void print_Gyro(){
  //Serial.print(" Gyro ");  
  Serial.print(gyroSpeed[0]);Serial.print(",");  
  Serial.print(gyroSpeed[1]);Serial.print(",");
  Serial.print(gyroSpeed[2]);
}
void print_Acc(){
  //Serial.print(" ACC ");
  Serial.print(accG[0]); Serial.print(",");
  Serial.print(accG[1]); Serial.print(",");
  Serial.print(accG[2]);
}
void print_Mag(){
  //Serial.print(" Mag ");
  Serial.print(mag[0]); Serial.print(",");
  Serial.print(mag[1]); Serial.print(",");
  Serial.print(mag[2]);
}
