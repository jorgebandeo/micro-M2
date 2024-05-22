#include <stdint.h>
#include <xc.h>
#include <stdio.h>
#include "I2C_Master.h"

#pragma config MCLRE = ON, WDT = OFF, OSC = HS

// Configuracões do PIC18F4520 para Display
#pragma config OSC = HS
#pragma config FCMEN = OFF
#pragma config IESO = OFF
#pragma config PWRT = OFF
#pragma config BOREN = OFF
#pragma config WDT = OFF
#pragma config MCLRE = ON
#pragma config LVP = OFF
#pragma config XINST = OFF



#define _XTAL_FREQ 16000000
//#define MEMORY_CAPACITY 2 // Defina a capacidade em bytes da memoria
#define MEMORY_CAPACITY 32768 // Defina a capacidade em bytes da memoria
#define EEPROM_I2C_ADDRESS 0xA0 // Novo endereco I2C da EEPROM
#define MAX_BUFFER_SIZE 100  // Defina um tamanho máximo adequado para o seu buffer
static uint16_t current_pointer = 0; // Ponteiro associado a memoria
// Definindo o mapeamento dos segmentos para cada dígito (0-9)

const uint8_t segment_map[13] = {
    0b00111111, // 0
    0b00000110, // 1
    0b01011011, // 2
    0b01001111, // 3
    0b01100110, // 4
    0b01101101, // 5
    0b01111101, // 6
    0b00000111, // 7
    0b01111111, // 8
    0b01101111, // 9
    0b01111001, // E (10)
    0b01010000, // R (11)
    0b01011100  // o (12)

};


/*******************************************
 * Funcões associadas a comunicacao serial *
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
    RCSTAbits.CREN = 1; // Modo contínuo para recepcao   
}

void putch(char byte) {
    while (!TXSTAbits.TRMT) {
        continue;
    }
    TXREG = byte;
}

/*******************************************
 * Funcões para leitura e escrita I2C *
 *******************************************/

// Funcao para escrita de dados na EEPROM via I2C
void i2c_write(uint16_t address, const uint8_t *data, uint16_t length) {
    for (uint16_t i = 0; i < length; i++) {
        I2C_START();
        I2C_TRANSMITE(EEPROM_I2C_ADDRESS);
        I2C_TRANSMITE((address >> 8) & 0xFF); // Endereco alto
        I2C_TRANSMITE(address & 0xFF);        // Endereco baixo
        if ((uint8_t)(data[i]) == 255 ){
            break;
        }
        I2C_TRANSMITE((uint8_t)(data[i]));
        printf("Endereco atual: %u, Leitura [%u] = %u \r\n", address, i, (uint8_t)(data[i])); // Adicione esta linha para imprimir o endereco atual
        I2C_STOP();
        __delay_ms(20); // Tempo de escrita da EEPROM
        address = 2 + address; // pula de volta para a coluna de memoria de dados e da sequencia embaixo
    }
    return;
}

// Funcao para leitura de dados da EEPROM via I2C
void i2c_read(uint16_t address, uint8_t *data, uint16_t length) {
    for (uint16_t i = 0; i < length; i++) {
        I2C_START();
        I2C_TRANSMITE(EEPROM_I2C_ADDRESS);
        I2C_TRANSMITE((address >> 8) & 0xFF); // Endereco alto
        I2C_TRANSMITE(address & 0xFF);        // Endereco baixo
        I2C_RESTART();
        I2C_TRANSMITE(EEPROM_I2C_ADDRESS | 1); // Leitura
        data[i] = I2C_RECEBE();
        if ((uint8_t)(data[i]) == 255 ){
            break;
        }
        printf("Endereco atual: %u, Leitura [%u] = %u \r\n", address, i, (uint8_t)(data[i])); // Adicione esta linha para imprimir o endereco atual
        if (i < length - 1) {
            I2C_ACK();
        } else {
            I2C_NACK();
        }
        I2C_STOP();
        address = 2 + address; // pula de volta para a coluna de memoria de dados e da sequencia embaixo
    }
    return;
}

uint8_t serial_memory_read(uint8_t *buffer, uint16_t num_bytes) {
    // Verifica se há espaco suficiente na memoria
    if (num_bytes <= 0) {
        printf("Erro: Tentativa de ler alem da capacidade da memoria\r\n");
        return (uint8_t)-1;
    }

    // Realiza a leitura da memoria
    i2c_read(current_pointer, buffer, num_bytes);

    return 0; // Sucesso
}

// Funcao de Escrita
uint8_t serial_memory_write(const uint8_t *data, uint16_t num_bytes) {
    if (current_pointer + num_bytes > MEMORY_CAPACITY) {
        printf("Erro: Tentativa de escrever alem da capacidade da memoria\r\n");
        return (uint8_t)-1;
    }

    i2c_write(current_pointer, data, num_bytes); // Escrever dados na memoria

    return 0; // Sucesso
}

// Definicões para a funcao de busca (seek)
#define SEEK_SET 0
#define SEEK_CUR 1
#define SEEK_END 2

// Funcao de Busca (Seek)
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
    
    // Percorre a string ate encontrar o caractere nulo '\0'
    for (int i = 0; message[i] != '\0'; ++i) {
        count++;
    }
    
    return count;
}
void init(void) {
    ADCON1 = 0x06; // Configura todos os pinos como digitais
    TRISD = 0x00; // Configura o PORTD como saída (segmentos)
    //TRISB &= 0x0F; // Configura os pinos RB4 a RB7 como saída (dígitos)
    TRISB = 0x00; // Configura o PORTB como saida
    LATD = 0x00;  // Inicializa o PORTD com 0
    //LATB &= 0x0F; // Inicializa os pinos RB4 a RB7 com 0
    LATB = 0x00;  // Inicializa o PORTB com 0
    TRISBbits.TRISB0 = 1; // Configura o RB0 como entrada para o botao SW1
    TRISBbits.TRISB1 = 1; // Configura o RB0 como entrada para o botao SW1
}

void display_digit(uint8_t value, uint8_t position) {
    if (value < 13) { // Ajuste para incluir os índices de 0 a 12
        LATD = segment_map[value]; // Mapeia o valor para os segmentos do display
    } else {
        LATD = 0x00; // Desliga o display para valores fora do intervalo
    }
    LATB = (LATB & 0x0F) | (1 << (position + 4)); // Ativa o display na posição específica (RB4 a RB7)
    __delay_ms(5); // Pequeno atraso para permitir a visualização do dígito
    LATB &= 0x0F; // Desativa todos os displays
}




void display_sequence(const char* sequence, uint8_t length) {
    for (uint8_t i = 0; i <= length; i++) {
        uint8_t digits[4] = {0xFF, 0xFF, 0xFF, 0xFF}; // Inicializa todos os dígitos como apagados
        for (uint8_t j = 0; j < 5 && (i + j) < length; j++) {
            char ch = sequence[i + j];
            if (ch >= '0' && ch <= '9') {
                digits[j] = (uint8_t)(ch - '0');
            } else if (ch == 'E') {
                digits[j] = 10; // Mapeia para 'E'
            } else if (ch == 'R') {
                digits[j] = 11; // Mapeia para 'R'
            } else if (ch == 'O') {
                digits[j] = 12; // Mapeia para 'O'
            } else {
                digits[j] = 0xFF; // Valor apagado para caracteres não mapeados
            }
        }

        for (uint8_t refresh = 0; refresh < 200; refresh++) { // Multiplexa por um tempo suficiente para que o olho humano não perceba a piscada
            for (uint8_t j = 0; j < 4; j++) {
                if (digits[j] == 0xFF) {
                    LATD = 0x00; // Apaga o dígito se estiver como apagado
                } else {
                    LATD = segment_map[digits[j]];
                }
                LATB = (LATB & 0x0F) | (1 << (j + 4)); // Ativa o display na posição específica (RB4 a RB7)
                __delay_ms(1); // Pequeno atraso para permitir a visualização do dígito
                LATB &= 0x0F; // Desativa todos os displays
            }
        }
    }
}



int main(void) {
    initUART();
    I2C_INICIA(); // Inicializa o I2C
    init();
    int erro_mem = 0;

    const char *message = "1234";
   
   int message_length = count_characters(message);

    if (message_length + 1 > MAX_BUFFER_SIZE) {
        // Tratamento de erro caso a mensagem seja maior que o tamanho máximo do buffer
        printf("Erro: mensagem muito longa.\n");
        erro_mem = 1;
    } 

    // Cria o buffer de leitura com o tamanho máximo definido
    uint8_t read_buffer[MAX_BUFFER_SIZE];
    
    if (serial_memory_write((const uint8_t *)message, (uint16_t)(message_length)) == 0) {
        printf("Sucesso na escrita: %s\r\n", message);
    } else {
        erro_mem = 1;
    }

    // Reposiciona o ponteiro no início para ler a mensagem
    if (serial_memory_seek(0, SEEK_SET) == 0) {
        printf("Sucesso no reposicionamento do ponteiro\r\n");
    } else {
        erro_mem = 1;
    }

    // Le a mensagem da memoria
    if (serial_memory_read(read_buffer, (uint16_t)sizeof(read_buffer)) == 0) {
        read_buffer[message_length] = '\0'; // Adiciona o caractere nulo no final
        printf("Sucesso na leitura: %s\r\n", read_buffer);
    } else {
        erro_mem = 1;
    }
    
   while (1){ 
       if (erro_mem){
            const char* error_message = "ERROR";
            display_sequence(error_message, 5);
       }
        if (PORTBbits.RB0 == 0) {
            __delay_ms(50); // Debounce delay
            if (PORTBbits.RB0 == 0) {
                // Botao pressionado, exibe a sequencia
                display_sequence(message, (uint8_t)message_length);
                while (PORTBbits.RB0 == 0); // Espera o botao ser liberado
            }
        }
        if (PORTBbits.RB1 == 0) {
            __delay_ms(50); // Debounce delay
            if (PORTBbits.RB1 == 0) {
                // Botao pressionado, exibe a sequencia
                display_sequence(read_buffer, (uint8_t)message_length);
                while (PORTBbits.RB1 == 0); // Espera o botao ser liberado
            }
        }
   }
    return 0;
}