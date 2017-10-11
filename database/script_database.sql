
DROP SCHEMA `qualismap`;

CREATE SCHEMA `qualismap` ;

/*
 ______________________________________________________________________
|NOME		 	 		 |COD | SIGNIFICADO                            |
|------------------------|----|----------------------------------------|
|INIT_SISTEM 			 | 00 | inicializacao do sistema               |
|MPU_INIT 				 | 01 | inicalizacao de comunicacao com MPU    |
|MPU_SUCCESS_INIT 		 | 02 | comunicacao efetuada com sucesso       |
|MPU_ERROR_INIT 		 | 03 | erro de comunicacao com MPU			   |
|MPU_DATA_TOO_LONG 		 | 04 | informacao muito longa                 |
|MPU_ADDRESS_NACK 		 | 05 | endereco de transmissao enviou um nack |
|MPU_DATA_NACK 			 | 06 | dados de transmissao geraram um nack   |
|MPU_UNKNOW_ERROR 		 | 07 | erro desconhecido                      |
|MPU_TRANSMISSION_SUCCESS| 08 | transmissao bem sucedida               |
|MPU_TRANSMISSION_ERROR	 | 09 | transmissao mal sucedida               |
|______________________________________________________________________|

Codigos de origem do log de sistema
 ________________________________________
|NOME		 	 	  |COD| SIGNIFICADO  |
|----------------------------------------|
|SETUP_LOG			  |00 |setup (geral) |
|INIT_MPU_LOG 		  |01 |setup do MPU  |
|TRANSMISSION_MPU_LOG |02 |loop do MPU   |
|INIT_GPS_LOG		  |03 |setup do GPS  |
|TRANSMISSION_GPS_LOG |04 |loop do GPS   |
|________________________________________|

*/                                                                      
CREATE TABLE `qualismap`.`log_sistem`(
	ID			INT NOT NULL AUTO_INCREMENT,
	MSG			VARCHAR2(40),
	COD_LOG		NUMBER,
	ORIGEM		VARCHAR2(20),
	TIME_SEND	TIMESTAMP,
	PRIMARY KEY (LOG_ID)
);

CREATE TABLE `qualismap`.`dados_local`(
	latitude DOUBLE PRECISION(8,6),
	longitude DOUBLE PRECISION(8,6),
	data_hora TIMESTAMP,
	temp NUMBER,
	ac_x NUMBER,
	ac_y NUMBER,
	ac_z NUMBER,
	gy_x NUMBER,
	gy_y NUMBER,
	gy_z NUMBER,
	classificacao VARCHAR(2) NOT NULL,
);

/*
drop table DADOS_LIDOS_QUALISMAP;

create table DADOS_LIDOS_QUALISMAP(
	LATITUDE number,
	LONGITUDE number,
	DATA_LEITURA date,
	TEMP number,
	AC_X number,
	AC_Y number,
	AC_Z number,
	GY_X number,
	GY_Y number,
	GY_Z number,
	PRIMARY KEY (LATITUDE, LONGITUDE, DATA_LEITURA)
);
*/