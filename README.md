
# Biblioteca de Manipulação de Memória Serial

## Introdução

O uso da memória serial pode ser útil para atender a necessidade de armazenamento de dados em várias aplicações, como dataloggers, sistemas de controle de acesso, etc. Visando facilitar a manipulação da memória serial presente no kit, você deverá implementar um conjunto de 3 funções cujas interfaces e funcionalidades são definidas a seguir.

Para o uso das funções, o usuário deverá declarar a capacidade da memória em bytes através de um define:
```c
#define MEMORY_CAPACITY DDDD
```
onde `DDDD` é a capacidade da memória em bytes.

Além disso, as funções implementadas consideram o conceito de acesso aleatório.

## Funções

### Operação de Leitura

Realiza a leitura de um bloco de bytes a partir da posição atual de um ponteiro associado à memória. Antes de usar a função de leitura, provavelmente será necessário posicionar o ponteiro. A operação de leitura avança automaticamente este ponteiro, permitindo a realização de leituras consecutivas sem a necessidade de reposicionamento do ponteiro.

#### Interface da Função
```c
uint8_t serial_memory_read(uint8_t *buffer, uint16_t num_bytes);
```

#### Parâmetros
- `num_bytes`: Especifica o número de palavras (ou bytes, dependendo da organização da memória) a serem lidas.
- `*buffer`: Um ponteiro para um buffer onde os dados lidos serão armazenados.

#### Retorno
A função retorna `0` em caso de sucesso e `-1` em caso de falha.

#### Condições de Erro
- Se o ponteiro associado à memória atingir o valor máximo (maior que `MEMORY_CAPACITY`), a função retorna `-1`. Neste caso, nenhum dado é retornado no buffer e o ponteiro associado à memória não é alterado.

### Operação de Escrita

Realiza a escrita de um bloco de bytes a partir da posição atual de um ponteiro associado à memória. Antes de usar a função de escrita, provavelmente será necessário posicionar o ponteiro. A operação de escrita avança automaticamente este ponteiro, permitindo a realização de escritas consecutivas sem a necessidade de reposicionamento do ponteiro.

#### Interface da Função
```c
uint8_t serial_memory_write(const uint8_t *data, uint16_t num_bytes);
```

#### Parâmetros
- `num_bytes`: Especifica o número de palavras (ou bytes, dependendo da organização da memória) a serem escritas.
- `*data`: Um ponteiro para os dados a serem escritos na memória.

#### Retorno
A função retorna `0` em caso de sucesso e `-1` em caso de falha.

#### Condições de Erro
- Se o número de palavras a serem escritas exceder a capacidade restante da memória a partir do endereço inicial, a função retorna `-1`.

### Operação de Busca (Seek)

Reposiciona o ponteiro associado à memória.

#### Interface da Função
```c
uint8_t serial_memory_seek(int16_t offset, uint8_t origin);
```

#### Parâmetros
- `offset`: Indica o deslocamento em relação à posição atual na memória. Pode ser um valor positivo ou negativo representado por um tipo de dado adequado (como `int16_t`).
- `origin`: Indica de onde o deslocamento deve começar (início, fim, posição atual). As opções são definidas como:
  ```c
  #define SEEK_SET 0
  #define SEEK_CUR 1
  #define SEEK_END 2
  ```

  - `origin == SEEK_SET`: Este é o valor padrão e indica que o deslocamento deve começar a partir do início da memória. Ou seja, o deslocamento será relativo ao primeiro byte ou palavra na memória.
  - `origin == SEEK_CUR`: Isso significa que o deslocamento deve começar a partir da posição atual na memória. Ou seja, o deslocamento será relativo à posição atual do ponteiro de leitura/escrita na memória.
  - `origin == SEEK_END`: Isso indica que o deslocamento deve começar a partir do final da memória. Ou seja, o deslocamento será relativo ao último byte ou palavra na memória.

#### Retorno
A função retorna a nova posição na memória após a operação de busca em caso de sucesso e `-1` em caso de falha.

#### Condições de Erro
- Se a posição resultante da busca estiver fora dos limites da memória (maior ou igual a `MEMORY_CAPACITY`), a função retorna `-1`.
