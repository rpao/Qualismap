//Programa : Teste MPU6050 e NEO-06 GPS
//Baseado no programa original de JohnChi

/*-------------------*
 * Pinagem:          *
 *-------------------*
 * GPS               *
 *10 - GPS Module TX *
 *09 - GPS Module RX *
 *-------------------*
 * MPU               *
 * VCC  5V           *
 * GND GND           *
 * SCL SCL           *
 * SDA SDA           *
 * INT 2             *
 *-------------------*/

#include<Wire.h>
#include <TinyGPS.h>
#include <SoftwareSerial.h>

//Endereco I2C do MPU6050
#define MPU 0x68 

//Variaveis para armazenar valores dos sensores
int AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
int status_transmission = 1;

SoftwareSerial mySerial(10, 11);
TinyGPS gps;

void gpsdump(TinyGPS &gps);
void printFloat(double f, int digits = 2);

void setup(){
  Serial.begin(9600); // inicia a comunicacao serial
  
  pinMode (13, OUTPUT); // Pin 13 tem um LED conectado (o led piscando indica que leituras estao sendo feitas)
  digitalWrite (13, HIGH); // set o LED
  
  //Inicializa o MPU-6050
  Wire.begin();                       // inicia I2C
  Wire.beginTransmission(MPU);        // inicia transmissao para o endereço do MPU
  Wire.write(0x6B);                   // PWR_MGMT_1 register
  Wire.write(0);                      // (tira do sleep mode) inicializa o MPU-6050  
  int error = Wire.endTransmission(true); // fecha a transmissao para o endereço do MPU
  if (error != 0){
    status_transmission = 0;
    Serial.print("Setup - ");
    if (error == 1)
      Serial.println("Erro 01: informacao muito longa.");
    else if (error == 2)
      Serial.println("Erro 02: recebeu um NACK do endereco de transmissao.");
    else if (error == 3)
      Serial.println("Erro 03: recebeu um NACK dos dados de transmissao.");
    else if (error == 4)
      Serial.println("Erro 04:  erro desconhecido.");
  }else{
    Serial.println("Setup: Transmissao finalizada");
  }
}

void loop(){
  if(status_transmission != 0){
    int error;
    digitalWrite (13, LOW);         // apaga o LED no incio do Loop
    
    Wire.beginTransmission(MPU);    // transmite para MPU
    Wire.write(0x3B);               // começa com o registrador 0x3B (ACCEL_XOUT_H)
    error = Wire.endTransmission(false);    // finalisa transmissao
    if (error != 0){
      if (error == 1)
        Serial.println("Erro 01: informacao muito longa.");
      else if (error == 2)
        Serial.println("Erro 02: recebeu um NACK do endereco de transmissao.");
      else if (error == 3)
        Serial.println("Erro 03: recebeu um NACK dos dados de transmissao.");
      else if (error == 4)
        Serial.println("Erro 04:  erro desconhecido.");
    }else{
      Wire.requestFrom(MPU,14,true);  //Solicita os dados do sensor
      
      //Armazena o valor dos sensores nas variaveis correspondentes
      AcX=Wire.read()<<8|Wire.read();  //0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
      AcY=Wire.read()<<8|Wire.read();  //0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
      AcZ=Wire.read()<<8|Wire.read();  //0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
      Tmp=Wire.read()<<8|Wire.read();  //0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
      GyX=Wire.read()<<8|Wire.read();  //0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
      GyY=Wire.read()<<8|Wire.read();  //0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
      GyZ=Wire.read()<<8|Wire.read();  //0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)  
      
      //Imprime valores X, Y, Z do acelerometro
      Serial.print("AcX = "); Serial.print(AcX);
      Serial.print(" AcY = "); Serial.print(AcY);
      Serial.print(" AcZ = "); Serial.print(AcZ);
      
      //Imprime valor da temperatura -- Calcula a temperatura em graus Celsius
      Serial.print(" Tmp = "); Serial.print(Tmp/340.00+36.53);
      
      //Imprime valores X, Y, Z do giroscopio
      Serial.print(" GyX = "); Serial.print(GyX);
      Serial.print(" GyY = "); Serial.print(GyY);
      Serial.print(" GyZ = "); Serial.println(GyZ);
    }  
    //apagar o led
    digitalWrite (13, HIGH); // set o LED
    delay(1000);
    digitalWrite (13, LOW); // set o LED
    delay(4000);
  }
}
