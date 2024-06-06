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
#include <thread>
#include <sys/socket.h>
#include <netinet/in.h>
#include "axis-fifo.h"
#include <inttypes.h>
#include <cstdint>
#include <iostream>
#include <cib_mem.h>
#include <cib_data_fmt.h>
#include <mem_utils.h>

/*----------------------------------------------------------------------------
 * Internal Definitions
 *----------------------------------------------------------------------------*/
#define DEF_DEV_RX "/dev/axis_fifo_0x00000000a0000000"
#define MAX_BUF_SIZE_BYTES 1450

struct thread_data
{
  int rc;
};


using cib::daq::iols_trigger_t;
pthread_t read_from_fifo_thread;

static volatile bool running = true;
static int readFifoFd;

static void signal_handler(int signal);
static void *read_from_fifo_thread_fn(void *data);

/*----------------------------------------------------------------------------
 * Main
 *----------------------------------------------------------------------------*/
int main(int argc, char **argv)
{
  iols_trigger_t word;
  printf("Size of a trigger word %lu\n",sizeof(iols_trigger_t));
  int rc;

  // Listen to ctrl+c and assert
  signal(SIGINT, signal_handler);
  signal(SIGTERM, signal_handler);
  signal(SIGQUIT, signal_handler);

  /*************/
  /* open FIFO */
  /*************/
  readFifoFd = open(DEF_DEV_RX, O_RDONLY | O_NONBLOCK);
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


  /* start thread listening for fifo receive packets */
  printf("Init rx thread\n");

  rc = pthread_create(&read_from_fifo_thread, NULL, read_from_fifo_thread_fn,(void *)NULL);

  /* perform noops */
  while (running)
  {
    sleep(1);
  }

  printf("SHUTTING DOWN\n");
  pthread_join(read_from_fifo_thread, NULL);
  close(readFifoFd);
  return rc;
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
  iols_trigger_t *word;

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

      fs.write(reinterpret_cast<const char*>(buf),bytesFifo);

      // fun fact: by casting to the structure, we no longer need any care about byte order
      word = reinterpret_cast<iols_trigger_t*>(buf);
      printf("TS %" PRIu64 " m1 %i m2 %i m3 %i \n"
             ,word->timestamp
             ,word->get_pos_m1(), word->get_pos_m2(), word->get_pos_m3());

      packets_rx++;
    }
    else
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }
  std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

  printf("Stepped out of the loop\n");
  printf("Closing the local output file \n");
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
