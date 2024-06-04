/*
 * read_axi_fifo.cpp
 *
 *  Created on: Apr 17, 2024
 *      Author: Nuno Barros
 */

#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <assert.h>
#include <fstream>
#include <chrono>
#include <sstream>
#include <fcntl.h>              // Flags for open()
#include <sys/stat.h>           // Open() system call
#include <sys/types.h>          // Types for open()
#include <unistd.h>             // Close() system call
#include <string.h>             // Memory setting and copying
#include <getopt.h>             // Option parsing
#include <errno.h>              // Error codes
#include <pthread.h>
#include <signal.h>
#include <time.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <ctime>

#include <sys/socket.h>
#include <netinet/in.h>
#include "axis-fifo.h"
#include <inttypes.h>
#include <cstdint>
#include <iostream>
#include <cib_mem.h>
#include <mem_utils.h>

/*----------------------------------------------------------------------------
 * Internal Definitions
 *----------------------------------------------------------------------------*/
#define DEF_DEV_TX "/dev/axis_fifo_0x00000000a0000000"
#define DEF_DEV_RX "/dev/axis_fifo_0x00000000a0000000"
#define MAX_BUF_SIZE_BYTES 1450

//#define printf(fmt, ...) printf("DEBUG %s:%d(): " fmt,  __func__, __LINE__, ##__VA_ARGS__)

struct thread_data
{
  int rc;
};


typedef struct daq_trigger_t
{
  // lsb
  uint32_t pos_m3 : 17;
  uint32_t pos_m2_lsb : 15;
  uint32_t pos_m2_msb : 7;
  uint32_t pos_m1 : 22;
  uint32_t padding : 3;
  uint64_t timestamp;
  // msb
  static const uint32_t mask_m3 = 0x1FFFF;
  static const uint32_t mask_m2_1 = 0xFFFE0000;
  static const uint32_t mask_m2_2 = 0x7F;
  static const uint32_t mask_m1 = 0x1FFFFF80;
  static const uint32_t bitmask_m1 = 0x3FFFFF;


  int32_t get_pos_m1() {return cib::util::cast_to_signed(pos_m1,bitmask_m1);}
  int32_t get_pos_m3() {return cib::util::cast_to_signed(pos_m3,mask_m3);}
  int32_t get_pos_m2()
  {
    uint32_t m2_lsb = pos_m2_lsb;
    uint32_t m2_msb = pos_m2_msb;
    uint32_t m2 = (m2_msb << 15) | pos_m2_lsb;
    // the bitmask is the same
    return cib::util::cast_to_signed(m2,bitmask_m1);
  }
} daq_trigger_t;


//pthread_t write_to_fifo_thread;
pthread_t read_from_fifo_thread;

static volatile bool running = true;
static char _opt_dev_rx[255];
static int readFifoFd;

static void signal_handler(int signal);
static void *read_from_fifo_thread_fn(void *data);
static void quit(void);

/*----------------------------------------------------------------------------
 * Main
 *----------------------------------------------------------------------------*/
int main(int argc, char **argv)
{
  //process_options(argc, argv);
  daq_trigger_t word;
  printf("Size of a trigger word %lu\n",sizeof(daq_trigger_t));
  int rc;

  // Listen to ctrl+c and assert
  signal(SIGINT, signal_handler);
  signal(SIGTERM, signal_handler);
  signal(SIGQUIT, signal_handler);

  /*************/
  /* open FIFO */
  /*************/
  readFifoFd = open(_opt_dev_rx, O_RDONLY | O_NONBLOCK);
  if (readFifoFd < 0)
  {
    printf("Open read failed with error: %s\n", strerror(errno));
    return -1;
  }

  /*****************************/
  /* initialize the fifo core  */
  /*****************************/

  //NFB: For some reason, the reset does not work, causing an
  // The IOCTLS were only implemented in the version 4.2 of the module

  printf("Reset\n");
  rc = ioctl(readFifoFd, AXIS_FIFO_RESET_IP);
  if (rc)
  {
    perror("ioctl");
    return -1;
  }

  /*****************/
  /* start threads */
  /*****************/

  /* start thread listening for ethernet packets */
  //    rc = pthread_create(&write_to_fifo_thread, NULL, write_to_fifo_thread_fn,
  //            (void *)NULL);

  /* start thread listening for fifo receive packets */
  printf("Thread\n");

  rc = pthread_create(&read_from_fifo_thread, NULL, read_from_fifo_thread_fn,(void *)NULL);

  /* perform noops */
  while (running)
  {
    sleep(1);
  }

  printf("SHUTTING DOWN\n");
  //    pthread_join(write_to_fifo_thread, NULL);
  pthread_join(read_from_fifo_thread, NULL);
  //    close(writeFifoFd);
  close(readFifoFd);
  return rc;
}

static void quit(void)
{
  running = false;
}

static void *read_from_fifo_thread_fn(void *data)
{
  ssize_t bytesFifo;
  int packets_rx = 0;
  uint8_t buf[MAX_BUF_SIZE_BYTES];
  uint64_t ts;
  uint32_t occupancy;
  /* shup up compiler */
  (void)data;
  daq_trigger_t *word;

  std::ostringstream fname("");
  std::time_t result = std::time(nullptr);
  fname << "trigger_dump_" << result << ".bin";
  std::ofstream fs(fname.str().c_str(), std::ios::out | std::ios::binary | std::ios::app);

  packets_rx = 0;

  printf("Checking current occupancy: \n");
  int rc = ioctl(readFifoFd,AXIS_FIFO_GET_RX_OCCUPANCY,&occupancy);
  if (rc)
  {
    perror("IOCTL failure checking occupancy\n");
  }
  else
  {
    printf("Claimed FIFO occupancy: %u\n",occupancy);
  }

  printf("Entering loop \n");

  std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
  while (running)
  {
    bytesFifo = read(readFifoFd, buf, MAX_BUF_SIZE_BYTES);
    if (bytesFifo > 0)
    {
      printf("Read bytes from fifo %ld\n",bytesFifo);

      //printf("Read : %s\n\r",buf);
      fs.write(reinterpret_cast<const char*>(buf),bytesFifo);

      word = reinterpret_cast<daq_trigger_t*>(buf);
      printf("TS %" PRIu64 " m1 %i m2 %i m3 %i \n"
             ,word->timestamp, word->get_pos_m1(), word->get_pos_m2(), word->get_pos_m3());
      //ts = *reinterpret_cast<uint64_t*>(&buf);
      //printf("Read : %" PRIu64 "\n",ts);

      packets_rx++;
    }
  }
  std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

  printf("Out of loop\n");
  printf("Closing file \n");
  fs.close();
  std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;
  double data_rate = static_cast<double>(packets_rx)/static_cast<double>(std::chrono::duration_cast<std::chrono::seconds>(end - begin).count());
  std::cout << "Word frequency : " << data_rate << " Hz" <<std::endl;

  printf("Querying some transfer information\n");
  uint32_t pkts_read, bytes_read;
  rc = ioctl(readFifoFd,AXIS_FIFO_GET_RX_PKTS_READ,&pkts_read);
  if (rc)
  {
    perror("IOCTL failure checking AXIS_FIFO_GET_RX_PKTS_READ\n");
  }
  else
  {
    printf("FIFO packets read: %u\n",pkts_read);
  }
  rc = ioctl(readFifoFd,AXIS_FIFO_GET_RX_BYTES_READ,&bytes_read);
  if (rc)
  {
    perror("IOCTL failure checking AXIS_FIFO_GET_RX_BYTES_READ\n");
  }
  else
  {
    printf("FIFO bytes read: %u\n",bytes_read);
  }

  return (void *)0;
}

static void signal_handler(int signal)
{
  switch (signal) {
    case SIGINT:
    case SIGTERM:
    case SIGQUIT:
      running = false;
      break;

    default:
      break;
  }
}
