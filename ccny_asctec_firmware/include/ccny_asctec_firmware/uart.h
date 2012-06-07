#ifndef __UART_H
#define __UART_H

#include <inttypes.h>
#include <mav_common/comm_packets.h>

extern void UARTInitialize(unsigned int);
extern void UARTWriteChar(unsigned char);
extern unsigned char UARTReadChar(void);
extern void __putchar(int);
extern void UART_send(char *, unsigned char);
extern void UART_send_ringbuffer(void);
extern int ringbuffer(unsigned char, unsigned char*, unsigned int);

void startAutoBaud(void);
volatile extern char autobaud_in_progress;

inline int writePacket2Ringbuffer(uint8_t descriptor, void * data, uint8_t length);
extern void uart0ISR(void);

extern uint16_t crc16(void *, uint16_t count, uint16_t prev_crc);
extern uint16_t crc_update(uint16_t, uint8_t);

#define RBREAD 0
#define RBWRITE 1
#define RBFREE  2 
#define RINGBUFFERSIZE	384

int UART0_txEmpty(void);
void UART0_rxFlush(void);
void UART0_txFlush(void);
uint8_t UART0_writeFifo(void * data, uint32_t length);

// this has to be 2^n !!!
#define UART0_RX_BUFFERSIZE 512
#define UART0_TX_BUFFERSIZE 512

// not more than 64 different packettypes
#define PACKET_INFO_SIZE 64

typedef struct
{
  uint8_t descriptor;
  void * data;
  uint8_t updated;
} PacketInfo;

extern volatile unsigned int UART_rxPacketCount;
extern volatile unsigned int UART_rxGoodPacketCount;

typedef struct
{
  uint8_t *buffer;
  uint8_t inUse;
  uint32_t bufferSize;
  uint32_t readIdx;
  uint32_t writeIdx;
  uint32_t tmp;
  uint32_t mask;
}volatile Fifo;

extern short uart0_min_tx_buffer;
extern short uart0_min_rx_buffer;

void Fifo_initialize(Fifo * fifo, uint8_t * buffer, uint32_t bufferSize);
inline uint8_t Fifo_writeByte(Fifo * fifo, uint8_t byte);
inline uint8_t Fifo_writeBlock(Fifo * fifo, void *data, uint32_t length);
inline uint8_t Fifo_readByte(Fifo * fifo, uint8_t * byte);
inline uint16_t Fifo_availableMemory(Fifo * fifo);
inline void Fifo_reset(Fifo * fifo);

void parseRxFifo(void);

PacketInfo* registerPacket(uint8_t descriptor, void * data);

#endif //__UART_H
