/********************************************************************
;	Nome do arquivo:		I2C_Master.c            
;	Data:				    10 de maio de 2010          
;	Versao:		  			1.0                              
;	MPLAB IDE:				v8.20a  
;	Autor:				    Wagner Zanco              
*********************************************************************/
/********************************************************************	
Esta biblioteca cont�m um conjunto de fun��es que permitem ao microcontrolador
se comunicar com qualquer dispositivo via I2C. O microcontrolador deve
estar configurado como dispositivo mestre. 
********************************************************************/
#include <xc.h>					//diretiva de compila��o
#include "I2C_Master.h"					//diretiva de compila��o
/********************************************************************/

void I2C_INICIA(){  // faz a configura��o do I2C
    TRISC = 0xff;	//configura pinos SDA e SCL como entrada

	SSPCON1 = 0b00101000;				//configura dispositivo como mestre
										//Habilita porta serial e ativa pinos SDA E SCL <5>
										//ativa modo mestre <3:0>
	SSPADD = 20;						//bit rate de 100kbps a Fosc = 16MHz
	SSPSTAT = 0b10000000;				//controle de inclina��o desativado <7>
										//n�veis de entrada conforme especifica��o I2C <6>
}
char   I2C_LIVRE  (void)				//fun��o I2C_LIVRE
{
	if(SSPSTATbits.R_W) return 0;		//retorna 0 se existe transmiss�o em andamento
		if(SSPCON2 & 0x1F) return 0;	//retorna 0 se existe algum evento de transmiss�o em andamento
			else return 1;  			//retorna 1 se barramento est� livre
}//*********************************************************************
void I2C_START (void) 					//fun��o I2C_START
{
	SSPCON2bits.SEN = 1;				//inicia a condi��o START, � resetada por hardware	
	while (SSPCON2bits.SEN);			//aguarda terminar a condi��o START	
}//*********************************************************************
void   I2C_RESTART  (void)				//fun��o I2C_RESTART
{
	SSPCON2bits.RSEN = 1;				//inicia a condi��o RE-START, � resetada por hardware
	while (SSPCON2bits.RSEN);			//aguarda terminar a condi��o RE-START	
}//*********************************************************************
void I2C_TRANSMITE  (unsigned char DADO_I2C)	//fun��o I2C_TRANSMITE
{
	SSPBUF = DADO_I2C;					//carrega dado a ser transmitido no registrador SSPBUF
	while (SSPSTATbits.BF);				//aguarda buffer esvaziar
	while (SSPSTATbits.R_W);			//aguarda transmiss�o terminar
}//*********************************************************************
char I2C_TESTA_ACK  (void)				//fun��o I2C_TESTA_ACK
{
	if (!SSPCON2bits.ACKSTAT) return 1;	//escravo recebeu dado com sucesso
		else return 0;					//erro na transmiss�o
}//*********************************************************************
void  I2C_STOP  (void)					//fun��o I2C_STOP
{
	SSPCON2bits.PEN = 1;				//inicia a condi��o STOP, resetado automaticamente por hardware
	while (SSPCON2bits.PEN);			//aguarda terminar condi��o STOP
}//*********************************************************************
//esta fun��o transmite um byte via I2C com endere�amento de 7 bits 
int  I2C_ESCRITA  (unsigned char END_I2C, unsigned char DADO_I2C)	//fun��o I2C_ESCRITA
{
char x;									//declara��o de vari�vel local 
	x = I2C_LIVRE ();					//chamada � fun��o com retorno de valor. Verifica se barramento est� livre
	if (x == 0)							//se barramento ocupado, aborta transmiss�o e retorna
	{
		I2C_STOP();						//gera bit STOP
		return -1;						//erro na transmiss�o
	}
	else 								//barramento livre
	{
		I2C_START();					//barramento livre, gera condi��o START
		END_I2C <<= 1;					//rotaciona END_I2C para a esquerda (insere bit R_W para escrita) 
		I2C_TRANSMITE(END_I2C);			//transmite endere�o
		if (!I2C_TESTA_ACK())	 		//se erro na transmiss�o, aborta transmiss�o
		{
			I2C_STOP();					//gera bit STOP
			return -1;					//erro na transmiss�o
 		}
		I2C_TRANSMITE(DADO_I2C);		//transmite dado
		if (!I2C_TESTA_ACK())			//se erro na transmiss�o, aborta transmiss�o
		{
			I2C_STOP();					//gera bit STOP
			return -1;					//erro na transmiss�o
		}
		I2C_STOP();						//gera bit STOP
		return 0;						//transmiss�o feita com sucesso
	}
}//*********************************************************************
unsigned char I2C_RECEBE  (void)		//fun��o I2C_RECEBE
{
unsigned char x;
	SSPCON2bits.RCEN = 1;				//ativa mestre-receptor
	while (SSPCON2bits.RCEN);			//aguarda chegada do dado, resetado por hardware ao final da recep��o
	while (!SSPSTATbits.BF);			//aguarda chegada do dado, fica 1 quando buffer rx estiver cheio
	Nop();   // verificar, necess�rio para a mem�ria serial
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();    
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    
	x = SSPBUF;
	return x;							//retorna dado		
}//*********************************************************************
void I2C_ACK  (void)					//fun��o I2C_ACK
{
	SSPCON2bits.ACKDT = 0;				//carrega bit ACK
	SSPCON2bits.ACKEN = 1;				//inicia seq��ncia ACK		
	while (SSPCON2bits.ACKEN);			//aguarda terminar seq��ncia ACK	
}//*********************************************************************
void I2C_NACK  (void)					//fun��o I2C_NACK
{
	SSPCON2bits.ACKDT = 1;				//carrega bit NACK
	SSPCON2bits.ACKEN = 1;				//inicia seq��ncia ACK		
	while (SSPCON2bits.ACKEN);			//aguarda terminar seq��ncia ACK	
}//*********************************************************************

