#include <stdint.h>
#include <xc.h>
#include <stdio.h>
#include "I2C_Master.h"

#pragma config MCLRE = ON, WDT = OFF, OSC = HS

#define _XTAL_FREQ 16000000
#define MEMORY_CAPACITY 32768 // Defina a capacidade em bytes da mem�ria
#define EEPROM_I2C_ADDRESS 0xA0 // Novo endere�o I2C da EEPROM

static uint16_t current_pointer = 0; // Ponteiro associado � mem�ria

/*******************************************
 * Fun��es associadas � comunica��o serial *
 *******************************************/
void initUART(void) {
    TRISCbits.RC6 = 0; // Porta TX como sa�da digital
    TRISCbits.RC7 = 1; // Porta RX como entrada digital
    SPBRG = 34; // Ajuste do gerador de baud rate (115200 bps p/ fosc = 16MHz)
    SPBRGH = 0;
    BAUDCONbits.BRG16 = 1; // Modo de 16 bits
    TXSTAbits.BRGH = 1; 
    TXSTAbits.SYNC = 0; // Modo ass�ncrono
    TXSTAbits.TXEN = 1; // Habilita TX
    RCSTAbits.SPEN = 1; // Serial habilitada
    RCSTAbits.CREN = 1; // Modo cont�nuo para recep��o   
}

void putch(char byte) {
    while (!TXSTAbits.TRMT) {
        continue;
    }
    TXREG = byte;
}

/*******************************************
 * Fun��es para leitura e escrita I2C *
 *******************************************/

// Fun��o para escrita de dados na EEPROM via I2C
void i2c_write(uint16_t address, const uint8_t *data, uint16_t length) {


    for (uint16_t i = 0; i < length; i++) {
        I2C_START();
        I2C_TRANSMITE(EEPROM_I2C_ADDRESS);
        I2C_TRANSMITE((address >> 8) & 0xFF); // Endere�o alto
        I2C_TRANSMITE(address & 0xFF);        // Endere�o baixo
        I2C_TRANSMITE((uint8_t)(data[i]));
        printf("Endereco atual: %u, Leitura [%u] = %u \r\n", address, i, (uint8_t)(data[i])); // Adicione esta linha para imprimir o endere�o atual
        I2C_STOP();
        __delay_ms(20); // Tempo de escrita da EEPROM
        address = 2 + address; // pula de volta para a coluna de memoria de dados e da sequncia embaixo

    }
    
}

// Fun��o para leitura de dados da EEPROM via I2C
void i2c_read(uint16_t address, uint8_t *data, uint16_t length) {
    for (uint16_t i = 0; i < length; i++) {
        I2C_START();
        I2C_TRANSMITE(EEPROM_I2C_ADDRESS);
        I2C_TRANSMITE((address >> 8) & 0xFF); // Endere�o alto
        I2C_TRANSMITE(address & 0xFF);        // Endere�o baixo
        I2C_RESTART();
        I2C_TRANSMITE(EEPROM_I2C_ADDRESS | 1); // Leitura
        data[i] = I2C_RECEBE();
        printf("Endereco atual: %u, Leitura [%u] = %u \r\n", address, i, (uint8_t)(data[i])); // Adicione esta linha para imprimir o endere�o atual
        if (i < length - 1) {
            I2C_ACK();
        } else {
            I2C_NACK();
        }
        I2C_STOP();
        address = 2 + address;// pula de volta para a coluna de memoria de dados e da sequncia embaixo

    }
}

uint8_t serial_memory_read(uint8_t *buffer, uint16_t num_bytes) {

    
    // Verifica se h� espa�o suficiente na mem�ria
    if (num_bytes <= 0) {
        printf("Erro: Tentativa de ler alem da capacidade da memoria\r\n");
        return (uint8_t)-1;
    }

    // Realiza a leitura da mem�ria
    i2c_read(current_pointer, buffer, num_bytes);

    return 0; // Sucesso
}

// Fun��o de Escrita
uint8_t serial_memory_write(const uint8_t *data, uint16_t num_bytes) {
    if (current_pointer + num_bytes > MEMORY_CAPACITY) {
        printf("Erro: Tentativa de escrever alem da capacidade da memoria\r\n");
        return (uint8_t)-1;
    }

    i2c_write(current_pointer, data, num_bytes); // Escrever dados na mem�ria


    return 0; // Sucesso
}

// Defini��es para a fun��o de busca (seek)
#define SEEK_SET 0
#define SEEK_CUR 1
#define SEEK_END 2

// Fun��o de Busca (Seek)
uint8_t serial_memory_seek(int16_t offset, uint8_t origin) {
    uint16_t new_pointer;

    switch (origin) {
        case SEEK_SET:
            new_pointer = (uint16_t)offset;
            break;
        case SEEK_CUR:
            new_pointer = (uint16_t)(current_pointer) + (uint16_t)(offset);
            break;
        case SEEK_END:
            new_pointer = (uint16_t)(MEMORY_CAPACITY) + (uint16_t)(offset);
            break;
        default:
            printf("Erro: Origem invalida\r\n");
            return (uint8_t)-1;
    }

    if (new_pointer >= MEMORY_CAPACITY) {
        printf("Erro: Ponteiro fora dos limites da memoria\r\n");
        return (uint8_t)-1;
    }

    printf("Seek: Posicao alterada para %u\r\n", new_pointer);
    current_pointer = new_pointer;
    return 0; // Sucesso
}
int count_characters(const char *message) {
    int count = 0;
    
    // Percorre a string at� encontrar o caractere nulo '\0'
    for (int i = 0; message[i] != '\0'; ++i) {
        count++;
    }
    
    return count;
}
int main(void) {
    initUART();
    I2C_INICIA(); // Inicializa o I2C

    const char *message = "hola mundo";

    uint8_t read_buffer[12]; // Buffer para ler a mensagem
    

    if (serial_memory_write((const uint8_t *)message, 12) == 0) {
        printf("Sucesso na escrita: %s\r\n", message);
    }

    // Reposiciona o ponteiro no in�cio para ler a mensagem
    if (serial_memory_seek(0, SEEK_SET) == 0) {
        printf("Sucesso no reposicionamento do ponteiro\r\n");
    }

    // L� a mensagem da mem�ria
    if (serial_memory_read(read_buffer, sizeof(read_buffer)) == 0) {
        printf("Sucesso na leitura: %s\r\n", read_buffer);
    }

    // Loop principal do programa (se necess�rio)
    while (1) {
        // C�digo principal
    }

    return 0;
}
