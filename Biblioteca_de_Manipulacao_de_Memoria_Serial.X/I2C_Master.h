/********************************************************************
;	Nome do arquivo:		I2C_Master.h            
;	Data:				    10 de junho de 2010          
;	Versao:		  			1.0                              
;	MPLAB IDE:				v8.20a  
;	Autor:				    Wagner Zanco              
*********************************************************************/
/********************************************************************	
Esta biblioteca contém um conjunto de funções que permitem ao microcontrolador
se comunicar com qualquer dispositivo via I2C. O microcontrolador deve
estar configurado como dispositivo mestre. 
********************************************************************/
#ifndef __I2C_MASTER_H
#define __I2C_MASTER_H
/********************************************************************/
void I2C_INICIA();
char I2C_LIVRE  (void);
void I2C_START (void); 
void I2C_RESTART  (void);
void I2C_TRANSMITE  (unsigned char DADO_I2C);
char I2C_TESTA_ACK  (void);
void I2C_STOP  (void);
int I2C_ESCRITA  (unsigned char END_I2C, unsigned char DADO_I2C);
unsigned char I2C_RECEBE  (void);
void I2C_ACK  (void);
void I2C_NACK  (void);
unsigned char I2C_LEITURA (unsigned char END_I2C);
//********************************************************************
#endif