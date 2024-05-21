/********************************************************************
;	Nome do arquivo:		I2C_Master.c            
;	Data:				    10 de maio de 2010          
;	Versao:		  			1.0                              
;	MPLAB IDE:				v8.20a  
;	Autor:				    Wagner Zanco              
*********************************************************************/
/********************************************************************	
Esta biblioteca contém um conjunto de funções que permitem ao microcontrolador
se comunicar com qualquer dispositivo via I2C. O microcontrolador deve
estar configurado como dispositivo mestre. 
********************************************************************/
#include <xc.h>					//diretiva de compilação
#include "I2C_Master.h"					//diretiva de compilação
/********************************************************************/

void I2C_INICIA(){  // faz a configuração do I2C
    TRISC = 0xff;	//configura pinos SDA e SCL como entrada

	SSPCON1 = 0b00101000;				//configura dispositivo como mestre
										//Habilita porta serial e ativa pinos SDA E SCL <5>
										//ativa modo mestre <3:0>
	SSPADD = 20;						//bit rate de 100kbps a Fosc = 16MHz
	SSPSTAT = 0b10000000;				//controle de inclinação desativado <7>
										//níveis de entrada conforme especificação I2C <6>
}
char   I2C_LIVRE  (void)				//função I2C_LIVRE
{
	if(SSPSTATbits.R_W) return 0;		//retorna 0 se existe transmissão em andamento
		if(SSPCON2 & 0x1F) return 0;	//retorna 0 se existe algum evento de transmissão em andamento
			else return 1;  			//retorna 1 se barramento está livre
}//*********************************************************************
void I2C_START (void) 					//função I2C_START
{
	SSPCON2bits.SEN = 1;				//inicia a condição START, é resetada por hardware	
	while (SSPCON2bits.SEN);			//aguarda terminar a condição START	
}//*********************************************************************
void   I2C_RESTART  (void)				//função I2C_RESTART
{
	SSPCON2bits.RSEN = 1;				//inicia a condição RE-START, é resetada por hardware
	while (SSPCON2bits.RSEN);			//aguarda terminar a condição RE-START	
}//*********************************************************************
void I2C_TRANSMITE  (unsigned char DADO_I2C)	//função I2C_TRANSMITE
{
	SSPBUF = DADO_I2C;					//carrega dado a ser transmitido no registrador SSPBUF
	while (SSPSTATbits.BF);				//aguarda buffer esvaziar
	while (SSPSTATbits.R_W);			//aguarda transmissão terminar
}//*********************************************************************
char I2C_TESTA_ACK  (void)				//função I2C_TESTA_ACK
{
	if (!SSPCON2bits.ACKSTAT) return 1;	//escravo recebeu dado com sucesso
		else return 0;					//erro na transmissão
}//*********************************************************************
void  I2C_STOP  (void)					//função I2C_STOP
{
	SSPCON2bits.PEN = 1;				//inicia a condição STOP, resetado automaticamente por hardware
	while (SSPCON2bits.PEN);			//aguarda terminar condição STOP
}//*********************************************************************
//esta função transmite um byte via I2C com endereçamento de 7 bits 
int  I2C_ESCRITA  (unsigned char END_I2C, unsigned char DADO_I2C)	//função I2C_ESCRITA
{
char x;									//declaração de variável local 
	x = I2C_LIVRE ();					//chamada à função com retorno de valor. Verifica se barramento está livre
	if (x == 0)							//se barramento ocupado, aborta transmissão e retorna
	{
		I2C_STOP();						//gera bit STOP
		return -1;						//erro na transmissão
	}
	else 								//barramento livre
	{
		I2C_START();					//barramento livre, gera condição START
		END_I2C <<= 1;					//rotaciona END_I2C para a esquerda (insere bit R_W para escrita) 
		I2C_TRANSMITE(END_I2C);			//transmite endereço
		if (!I2C_TESTA_ACK())	 		//se erro na transmissão, aborta transmissão
		{
			I2C_STOP();					//gera bit STOP
			return -1;					//erro na transmissão
 		}
		I2C_TRANSMITE(DADO_I2C);		//transmite dado
		if (!I2C_TESTA_ACK())			//se erro na transmissão, aborta transmissão
		{
			I2C_STOP();					//gera bit STOP
			return -1;					//erro na transmissão
		}
		I2C_STOP();						//gera bit STOP
		return 0;						//transmissão feita com sucesso
	}
}//*********************************************************************
unsigned char I2C_RECEBE  (void)		//função I2C_RECEBE
{
unsigned char x;
	SSPCON2bits.RCEN = 1;				//ativa mestre-receptor
	while (SSPCON2bits.RCEN);			//aguarda chegada do dado, resetado por hardware ao final da recepção
	while (!SSPSTATbits.BF);			//aguarda chegada do dado, fica 1 quando buffer rx estiver cheio
	Nop();   // verificar, necessário para a memória serial
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
void I2C_ACK  (void)					//função I2C_ACK
{
	SSPCON2bits.ACKDT = 0;				//carrega bit ACK
	SSPCON2bits.ACKEN = 1;				//inicia seqüência ACK		
	while (SSPCON2bits.ACKEN);			//aguarda terminar seqüência ACK	
}//*********************************************************************
void I2C_NACK  (void)					//função I2C_NACK
{
	SSPCON2bits.ACKDT = 1;				//carrega bit NACK
	SSPCON2bits.ACKEN = 1;				//inicia seqüência ACK		
	while (SSPCON2bits.ACKEN);			//aguarda terminar seqüência ACK	
}//*********************************************************************

