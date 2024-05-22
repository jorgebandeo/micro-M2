#include <stdint.h>
#include <xc.h>
#include <stdio.h>
#include "I2C_Master.h"

#pragma config MCLRE = ON, WDT = OFF, OSC = HS

#define _XTAL_FREQ 16000000
#define MEMORY_CAPACITY 32768 // Defina a capacidade em bytes da memória
#define EEPROM_I2C_ADDRESS 0xA0 // Novo endereço I2C da EEPROM
#define MAX_BUFFER_SIZE 100  // Defina um tamanho máximo adequado para o seu buffer
static uint16_t current_pointer = 0; // Ponteiro associado à memória

/*******************************************
 * Funções associadas à comunicação serial *
 *******************************************/
void initUART(void) {
    TRISCbits.RC6 = 0; // Porta TX como saída digital
    TRISCbits.RC7 = 1; // Porta RX como entrada digital
    SPBRG = 34; // Ajuste do gerador de baud rate (115200 bps p/ fosc = 16MHz)
    SPBRGH = 0;
    BAUDCONbits.BRG16 = 1; // Modo de 16 bits
    TXSTAbits.BRGH = 1; 
    TXSTAbits.SYNC = 0; // Modo assíncrono
    TXSTAbits.TXEN = 1; // Habilita TX
    RCSTAbits.SPEN = 1; // Serial habilitada
    RCSTAbits.CREN = 1; // Modo contínuo para recepção   
}

void putch(char byte) {
    while (!TXSTAbits.TRMT) {
        continue;
    }
    TXREG = byte;
}

/*******************************************
 * Funções para leitura e escrita I2C *
 *******************************************/

// Função para escrita de dados na EEPROM via I2C
void i2c_write(uint16_t address, const uint8_t *data, uint16_t length) {


    for (uint16_t i = 0; i < length; i++) {
        I2C_START();
        I2C_TRANSMITE(EEPROM_I2C_ADDRESS);
        I2C_TRANSMITE((address >> 8) & 0xFF); // Endereço alto
        I2C_TRANSMITE(address & 0xFF);        // Endereço baixo
        if ((uint8_t)(data[i]) == 255 ){
            break;
        }
        I2C_TRANSMITE((uint8_t)(data[i]));
        printf("Endereco atual: %u, Leitura [%u] = %u \r\n", address, i, (uint8_t)(data[i])); // Adicione esta linha para imprimir o endereço atual
        I2C_STOP();
        __delay_ms(20); // Tempo de escrita da EEPROM
        address = 2 + address; // pula de volta para a coluna de memoria de dados e da sequncia embaixo

    }
    return;
}

// Função para leitura de dados da EEPROM via I2C
void i2c_read(uint16_t address, uint8_t *data, uint16_t length) {
    for (uint16_t i = 0; i < length; i++) {
        I2C_START();
        I2C_TRANSMITE(EEPROM_I2C_ADDRESS);
        I2C_TRANSMITE((address >> 8) & 0xFF); // Endereço alto
        I2C_TRANSMITE(address & 0xFF);        // Endereço baixo
        I2C_RESTART();
        I2C_TRANSMITE(EEPROM_I2C_ADDRESS | 1); // Leitura
        data[i] = I2C_RECEBE();
        if ((uint8_t)(data[i]) == 255 ){
            break;
        }
        printf("Endereco atual: %u, Leitura [%u] = %u \r\n", address, i, (uint8_t)(data[i])); // Adicione esta linha para imprimir o endereço atual
        if (i < length - 1) {
            I2C_ACK();
        } else {
            I2C_NACK();
        }
        I2C_STOP();
        address = 2 + address;// pula de volta para a coluna de memoria de dados e da sequncia embaixo

    }
    return;
}

uint8_t serial_memory_read(uint8_t *buffer, uint16_t num_bytes) {

    
    // Verifica se há espaço suficiente na memória
    if (num_bytes <= 0) {
        printf("Erro: Tentativa de ler alem da capacidade da memoria\r\n");
        return (uint8_t)-1;
    }

    // Realiza a leitura da memória
    i2c_read(current_pointer, buffer, num_bytes);

    return 0; // Sucesso
}

// Função de Escrita
uint8_t serial_memory_write(const uint8_t *data, uint16_t num_bytes) {
    if (current_pointer + num_bytes > MEMORY_CAPACITY) {
        printf("Erro: Tentativa de escrever alem da capacidade da memoria\r\n");
        return (uint8_t)-1;
    }

    i2c_write(current_pointer, data, num_bytes); // Escrever dados na memória


    return 0; // Sucesso
}

// Definições para a função de busca (seek)
#define SEEK_SET 0
#define SEEK_CUR 1
#define SEEK_END 2

// Função de Busca (Seek)
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
    
    // Percorre a string até encontrar o caractere nulo '\0'
    for (int i = 0; message[i] != '\0'; ++i) {
        count++;
    }
    
    return count;
}
int main(void) {
    initUART();
    I2C_INICIA(); // Inicializa o I2C

    const char *message = "1028382";
    

    int message_length = count_characters(message);

    if (message_length + 1 > MAX_BUFFER_SIZE) {
        // Tratamento de erro caso a mensagem seja maior que o tamanho máximo do buffer
        printf("Erro: mensagem muito longa.\n");
        return 1;
    }

    // Cria o buffer de leitura com o tamanho máximo definido
    uint8_t read_buffer[MAX_BUFFER_SIZE];
    

    
    
    if (serial_memory_write((const uint8_t *)message, (uint16_t)(message_length)) == 0) {
        printf("Sucesso na escrita: %s\r\n", message);
    }

    // Reposiciona o ponteiro no início para ler a mensagem
    if (serial_memory_seek(0, SEEK_SET) == 0) {
        printf("Sucesso no reposicionamento do ponteiro\r\n");
    }

    // Lê a mensagem da memória
    if (serial_memory_read(read_buffer, sizeof(read_buffer)) == 0) {
        read_buffer[message_length] = '\0'; // Adiciona o caractere nulo no final
        printf("Sucesso na leitura: %s\r\n", read_buffer);
    }

    // Loop principal do programa (se necessário)
    while (1) {
        // Código principal
    }

    return 0;
}
