# Serial Memory Manipulation Library

## Introduction

The use of serial memory can be beneficial for data storage needs in various applications such as dataloggers, access control systems, etc. To facilitate the manipulation of the serial memory present in the kit, you will need to implement a set of 3 functions, whose interfaces and functionalities are defined below.

For the use of these functions, the user must declare the memory capacity in bytes using a define:
```c
#define MEMORY_CAPACITY DDDD
```
where `DDDD` is the capacity of the memory in bytes.

Additionally, the implemented functions consider the concept of random access.

## Functions

### Read Operation

Reads a block of bytes from the current position of a pointer associated with the memory. Before using the read function, it is likely necessary to position the pointer. The read operation automatically advances the pointer, allowing consecutive reads without the need for repositioning the pointer.

#### Function Interface
```c
uint8_t serial_memory_read(uint8_t *buffer, uint16_t num_bytes);
```

#### Parameters
- `num_bytes`: Specifies the number of words (or bytes, depending on the memory organization) to be read.
- `*buffer`: A pointer to a buffer where the read data will be stored.

#### Return
The function returns `0` on success and `-1` on failure.

#### Error Conditions
- If the pointer associated with the memory reaches the maximum value (greater than `MEMORY_CAPACITY`), the function returns `-1`. In this case, no data is returned in the buffer, and the pointer associated with the memory is not changed.

### Write Operation

Writes a block of bytes from the current position of a pointer associated with the memory. Before using the write function, it is likely necessary to position the pointer. The write operation automatically advances the pointer, allowing consecutive writes without the need for repositioning the pointer.

#### Function Interface
```c
uint8_t serial_memory_write(const uint8_t *data, uint16_t num_bytes);
```

#### Parameters
- `num_bytes`: Specifies the number of words (or bytes, depending on the memory organization) to be written.
- `*data`: A pointer to the data to be written to the memory.

#### Return
The function returns `0` on success and `-1` on failure.

#### Error Conditions
- If the number of words to be written exceeds the remaining capacity of the memory from the initial address, the function returns `-1`.

### Seek Operation

Repositions the pointer associated with the memory.

#### Function Interface
```c
uint8_t serial_memory_seek(int16_t offset, uint8_t origin);
```

#### Parameters
- `offset`: Indicates the displacement relative to the current position in the memory. It can be a positive or negative value represented by an appropriate data type (such as `int16_t`).
- `origin`: Indicates where the displacement should start (beginning, end, current position). The options are defined as:
  ```c
  #define SEEK_SET 0
  #define SEEK_CUR 1
  #define SEEK_END 2
  ```

  - `origin == SEEK_SET`: This is the default value and indicates that the displacement should start from the beginning of the memory. The displacement will be relative to the first byte or word in the memory.
  - `origin == SEEK_CUR`: This means that the displacement should start from the current position in the memory. The displacement will be relative to the current position of the read/write pointer in the memory.
  - `origin == SEEK_END`: This indicates that the displacement should start from the end of the memory. The displacement will be relative to the last byte or word in the memory.

#### Return
The function returns the new position in the memory after the seek operation on success and `-1` on failure.

#### Error Conditions
- If the resulting position of the seek is outside the memory limits (greater than or equal to `MEMORY_CAPACITY`), the function returns `-1`.
