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

#define SUCESSO 0
#define FALHA 1
#define STOP 2

#define MPU 0x68        //Endereco I2C do MPU6050
#define PWR_MGMT_1 0x6B //PWR_MGMT_1 register
#define AWAKE 0         //Tira do sleep mode  
#define SLEEP 1         //Coloca no sleep mode
#define ON 1            //Resetar o sistema
#define OFF 0           //Desligar o sistema

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
#define TEMP_HIGH  10               // temperatura do sistema está elevada

//Funçoes do sistema para MPU
int get_log(int error);                                   // transformaçao do codigo de erro da MPU para o codigo de log
void send_log(int log_id, int origem, int data_gps = 0);  // imprime o log do sistema
void send_leitura();                                      // imprimir dados do sensor mpu
void set_referencial(int set);                            // inicializa o ponto referencial de nível
void teste_nivel_acelerometro();                                        // testa magnitude da variaçao lida

// Funçoes gerais
void turn_system(int mode = ON);

//Variaveis para armazenar valores dos sensores (MPU)
int AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;

//Variáveis auxiliares
int referencial[7] = {0,0,0,0,0,0,0}; //ponto de referencia (primeira leitura)
int nivel[3] = {0,0,0};               //variaçao entre leituras
int status_transmissao = 1;           //se é possível transmitir
int status_set_referencial = ON;      //se deve resetar o referencial

void setup(){
  int cont_try = 0;
    
  Serial.begin(9600); // inicia a comunicacao serial
  
  pinMode (13, OUTPUT); // Pin 13 tem um LED conectado (piscando indica que leituras estao sendo feitas, sem piscar indica erro)
  pinMode (11, OUTPUT); // Pin 13 tem um LED conectado (piscando indica que leituras estao sendo feitas, sem piscar indica erro)
  
  digitalWrite (13, HIGH); // set o LED verde
  digitalWrite (11, HIGH); // reset LED vermelho
  delay(1000);             // 1 segundo de espera

  //send_log(INIT_SISTEM, SETUP_LOG);
    
  do{
    
    //send_log(MPU_INIT, INIT_MPU_LOG);
    
    //Inicializa o MPU-6050    
    Wire.begin();                 // inicia I2C
    Wire.beginTransmission(MPU);  // inicia transmissao para o endereço do MPU
    Wire.write(PWR_MGMT_1);       // PWR_MGMT_1 register
    Wire.write(AWAKE);            // Inicializa o MPU-6050  
    status_transmissao = Wire.endTransmission(true);       // fecha a transmissao para o endereço do MPU
    
    if (status_transmissao != SUCESSO){                       // teste de erro na transmissao
      //send_log(get_mpu_log(status_transmissao),INIT_MPU_LOG); // utiliza o código enviado pela UNO para obter o código correspondente
      cont_try++;                                         // contagem de tentativas de transmissao
      
      status_transmissao = FALHA;                             // se houve erro, nao houve transmissao
      
      digitalWrite (13, LOW);                             // reset o LED verde (mantem o vermelho setado)
      delay(5000);                                        // 5 segundos de espera
      
    } else {
       status_transmissao = SUCESSO;                          // transmissao foi realizada com sucesso
    }
  }while (status_transmissao == FALHA && cont_try < 5);       //tenta sempre que a transmissao falhou e foram feitas menos de 5 tentativas
   
  if (status_transmissao == SUCESSO){
    //send_log(MPU_ERROR_INIT, INIT_MPU_LOG);
    digitalWrite (13, LOW);                               // reset o LED verde
    digitalWrite (11, LOW);                               // reset LED vermelho
  }else{                                                  //erro
    //send_log(MPU_SUCCESS_INIT, INIT_MPU_LOG);
  }
}

void loop(){  
  if(status_transmissao == SUCESSO){

    Wire.beginTransmission(MPU);    // transmite para MPU
    Wire.write(0x3B);               // começa com o registrador 0x3B (ACCEL_XOUT_H)
    status_transmissao = Wire.endTransmission(false);    // finalisa transmissao
    
    if (status_transmissao != SUCESSO){
      //send_log(get_mpu_log(status_transmissao), TRANSMISSION_MPU_LOG);
      
      digitalWrite (11, HIGH);  // set LED vermelho
      delay(5000);
      
    }else{
      digitalWrite (11, LOW);  // reset LED vermelho
           
      //send_log(MPU_TRANSMISSION_SUCCESS, TRANSMISSION_MPU_LOG);
      
      Wire.requestFrom(MPU,14,true);  //Solicita os dados do sensor
     
      //Armazena o valor dos sensores nas variaveis correspondentes
      AcX=Wire.read()<<8|Wire.read();  //0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
      AcY=Wire.read()<<8|Wire.read();  //0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
      AcZ=Wire.read()<<8|Wire.read();  //0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
      Tmp=Wire.read()<<8|Wire.read();  //0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
      GyX=Wire.read()<<8|Wire.read();  //0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
      GyY=Wire.read()<<8|Wire.read();  //0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
      GyZ=Wire.read()<<8|Wire.read();  //0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)      

      if ((Tmp/340.00+36.53) > 50){
        //send_log(TEMP_HIGH, TRANSMISSION_MPU_LOG);
        turn_system(OFF);
      }

      set_referencial(status_set_referencial);

      AcX -= referencial[0];     
      AcY -= referencial[1];
      AcZ -= referencial[2];
      Tmp -= referencial[3];
      GyX -= referencial[4];
      GyY -= referencial[5];
      GyZ -= referencial[6];

      teste_nivel_acelerometro();

      send_leitura(); // imprime dados obtidos do MPU

      //apagar o led
      digitalWrite (13, HIGH); // set o LED
      delay(5);//1000);
      digitalWrite (13, LOW); // set o LED
      delay(5);//4000);
    }  
    
  }else{
    
  }
}

void send_leitura(){
  Serial.print(nivel[0]); Serial.print(","); Serial.print(nivel[1]);Serial.print(","); Serial.println(nivel[2]);
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

void send_log(int log_id, int origem, int data_gps){
  Serial.print("LOG: "); Serial.print(data_gps); Serial.print(" - "); Serial.print(origem); Serial.print(" - "); Serial.println(log_id);
}

void turn_system(int mode){
  if (mode == OFF){
    Serial.println("Desligar");
  }else{
    Serial.println("Reset");
  }
}

void set_referencial(int set){
  if (set == ON){
        referencial[0] = AcX;     
        referencial[1] = AcY;
        referencial[2] = AcZ;
        referencial[3] = Tmp;
        referencial[4] = GyX;
        referencial[5] = GyY;
        referencial[6] = GyZ;
               
        status_set_referencial = OFF;
      }
}

void teste_nivel_acelerometro(){
  int i;
  nivel[0] = AcX/100;
  nivel[1] = AcY/1000;
  nivel[2] = AcZ/1000;
}
