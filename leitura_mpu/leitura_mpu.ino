//Programa : Teste MPU6050 e NEO-06 GPS
//Baseado no programa original de JohnChi

/*-------------------*
 * Pinagem:          *
 *-------------------*
 * GPS               *
 *10 - GPS Module TX *
 *11 - GPS Module RX *
 *-------------------*
 * MPU               *
 * VCC - 3.3V        *
 * GND - GND         *
 * SCL - SCL         *
 * SDA - SDA         *
 * AD0 - GND         *
 * INT - 2           *
 *-------------------*/

#include<Wire.h>
#include <TinyGPS.h>
#include <SoftwareSerial.h>

#define MPU 0x68        //Endereco I2C do MPU6050
#define PWR_MGMT_1 0x6B //PWR_MGMT_1 register
#define AWAKE 0         //Tira do sleep mode  
#define SLEEP 1         //Coloca no sleep mode  

// Codigos de origem do log de sistema
#define SETUP_LOG     0           //Define log de setup (geral)
#define INIT_MPU_LOG 1            //Define log de setup do MPU
#define TRANSMISSION_MPU_LOG  2   //Define log de loop do MPU
#define INIT_GPS_LOG 3            //Define log de setup do GPS
#define TRANSMISSION_GPS_LOG  4   //Define log de loop do GPS

// Codigos de log
#define INIT_SISTEM 0              // inicializacao do sistema
#define MPU_INIT 1                 // inicalizacao de comunicacao com MPU
#define MPU_SUCCESS_INIT 2         // comunicacao efetuada com sucesso
#define MPU_ERROR_INIT 3           // erro de comunicacao com MPU
#define MPU_DATA_TOO_LONG 4        // informacao muito longa
#define MPU_ADDRESS_NACK 5         // endereco de transmissao enviou um nack
#define MPU_DATA_NACK 6            // dados de transmissao geraram um nack
#define MPU_UNKNOW_ERROR 7         // erro desconhecido
#define MPU_TRANSMISSION_SUCCESS 8 // transmissao bem sucedida
#define MPU_TRANSMISSION_ERROR  9  // transmissao mal sucedida

SoftwareSerial mySerial(10, 11);
TinyGPS gps;

//Funçoes do sistema
int get_log(int error); // transformaçao do codigo de erro da MPU para o codigo de log
void print_log(int log_id, int origem, int data_gps = 0); // imprime o log do sistema
void imprime_mpu(int AcX,int AcY,int AcZ,int Tmp,int GyX,int GyY); // imprimir dados do sensor mpu

//Variaveis para armazenar valores dos sensores
int AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
int referencial[7] = {0,0,0,0,0,0,0};
int status_transmission = 1;
int set_referencial = 1;
int qualidade = 0;
int error;
int leitura = 0;

void setup(){
  int cont_try = 0;
    
  Serial.begin(9600); // inicia a comunicacao serial
  
  pinMode (13, OUTPUT); // Pin 13 tem um LED conectado (piscando indica que leituras estao sendo feitas, sem piscar indica erro)
  pinMode (11, OUTPUT); // Pin 13 tem um LED conectado (piscando indica que leituras estao sendo feitas, sem piscar indica erro)
  
  digitalWrite (13, HIGH); // set o LED verde
  digitalWrite (11, HIGH);  // reset LED vermelho

  delay(2000);

  //print_log(INIT_SISTEM, SETUP_LOG);
    
  do{   
    //Inicializa o MPU-6050
    //print_log(MPU_INIT, INIT_MPU_LOG);
    
    Wire.begin();                 // inicia I2C
    Wire.beginTransmission(MPU);  // inicia transmissao para o endereço do MPU
    Wire.write(PWR_MGMT_1);       // PWR_MGMT_1 register
    Wire.write(AWAKE);            // Inicializa o MPU-6050  

    error = Wire.endTransmission(true); // fecha a transmissao para o endereço do MPU
    if (error != 0){
      status_transmission = 0;
      //print_log(get_mpu_log(error), INIT_MPU_LOG);
      cont_try++;
      digitalWrite (13, LOW);   // reset o LED verde      
      delay(5000);
    }else{
       status_transmission = 1;
    }
  }while (status_transmission == 0 && cont_try < 5);
    
  if (status_transmission == 0){ // sucesso
    //print_log(MPU_ERROR_INIT, INIT_MPU_LOG);
    digitalWrite (13, LOW);  // reset o LED verde
    digitalWrite (11, LOW);  // reset LED vermelho
  }else{ //erro
    //print_log(MPU_SUCCESS_INIT, INIT_MPU_LOG);
  }
}

void loop(){  
  if(status_transmission != 0){      
    Wire.beginTransmission(MPU);    // transmite para MPU
    Wire.write(0x3B);               // começa com o registrador 0x3B (ACCEL_XOUT_H)
    error = Wire.endTransmission(false);    // finalisa transmissao
    if (error != 0){
      //print_log(get_mpu_log(error), TRANSMISSION_MPU_LOG);
      digitalWrite (11, HIGH);  // set LED vermelho
      delay(5000);
    }else{
      digitalWrite (11, LOW);  // reset LED vermelho
           
      //print_log(MPU_TRANSMISSION_SUCCESS, TRANSMISSION_MPU_LOG);
      
      Wire.requestFrom(MPU,14,true);  //Solicita os dados do sensor
     
      //Armazena o valor dos sensores nas variaveis correspondentes
      AcX=Wire.read()<<8|Wire.read();  //0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
      AcY=Wire.read()<<8|Wire.read();  //0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
      AcZ=Wire.read()<<8|Wire.read();  //0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
      Tmp=Wire.read()<<8|Wire.read();  //0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
      GyX=Wire.read()<<8|Wire.read();  //0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
      GyY=Wire.read()<<8|Wire.read();  //0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
      GyZ=Wire.read()<<8|Wire.read();  //0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)      

      if (set_referencial == 1){
        referencial[0] = AcX;     
        referencial[1] = AcY;
        referencial[2] = AcZ;
        referencial[3] = Tmp;
        referencial[4] = GyX;
        referencial[5] = GyY;
        referencial[6] = GyZ;
               
        set_referencial = 0;
      }

      AcX -= referencial[0];     
      AcY -= referencial[1];
      AcZ -= referencial[2];
      Tmp -= referencial[3];
      GyX -= referencial[4];
      GyY -= referencial[5];
      GyZ -= referencial[6];

      //imprime_mpu(AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ); // imprime dados obtidos do MPU

      int roll = pitch_roll(AcX, AcY, AcZ);
      int pitch = pitch_roll(AcY, AcX, AcZ);
      int yaw = pitch_roll(AcZ, AcX, AcY);

      Serial.print(roll);
      Serial.print(","); Serial.print(pitch);
      Serial.print(","); Serial.println(yaw);

      //apagar o led
      digitalWrite (13, HIGH); // set o LED
      delay(1000);
      digitalWrite (13, LOW); // set o LED
      delay(4000);
    }  
  }
}

void imprime_mpu(int AcX,int AcY,int AcZ,int Tmp,int GyX,int GyY, int GyZ){
  //Imprime valores X, Y, Z do acelerometro
  Serial.print(AcX);
  Serial.print(" | "); Serial.print(AcY);
  Serial.print(" | "); Serial.print(AcZ);
  
  //Imprime valor da temperatura -- Calcula a temperatura em graus Celsius
  Serial.print(" | "); Serial.print(Tmp/340.00+36.53);
  
  //Imprime valores X, Y, Z do giroscopio
  Serial.print(" | "); Serial.print(GyX);
  Serial.print(" | "); Serial.print(GyY);
  Serial.print(" | "); Serial.println(GyZ);
}

int get_mpu_log(int error){  
  if (error == 1){
    return MPU_DATA_TOO_LONG; //informacao muito longa
  }else if (error == 2){
    return MPU_ADDRESS_NACK; //recebeu um NACK do endereco de transmissao
  }else if (error == 3){
    return MPU_DATA_NACK;    //recebeu um NACK dos dados de transmissao
  }else{
    return MPU_UNKNOW_ERROR; //erro desconhecido.
  }
}

void print_log(int log_id, int origem, int data_gps){
  Serial.print("LOG: "); Serial.print(data_gps); Serial.print(" - "); Serial.print(origem); Serial.print(" - "); Serial.println(log_id);
}

double pitch_roll(double A, double B, double C){
  double DatoA, DatoB, Value;
  DatoA = A;
  DatoB = (B*B) + (C*C);
  DatoB = sqrt(DatoB);
  
  Value = atan2(DatoA, DatoB);
  Value = Value * 180/3.14;
  
  return (int)Value;
}
