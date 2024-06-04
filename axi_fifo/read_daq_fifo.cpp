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

//pthread_t write_to_fifo_thread;
pthread_t read_from_fifo_thread;

static volatile bool running = true;
static char _opt_dev_tx[255];
static char _opt_dev_rx[255];
static int writeFifoFd;
static int readFifoFd;

static void signal_handler(int signal);
static void *read_from_fifo_thread_fn(void *data);
static int process_options(int argc, char * argv[]);
static void print_opts();
static void display_help(char * progName);
static void *write_to_fifo_thread_fn(void *data);
static void quit(void);

/*----------------------------------------------------------------------------
 * Main
 *----------------------------------------------------------------------------*/
int main(int argc, char **argv)
{
  process_options(argc, argv);
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

static void *write_to_fifo_thread_fn(void *data)
{
  int rc;
  int packets_tx;
  ssize_t bytesFifo;
  char buf[MAX_BUF_SIZE_BYTES];
  uint32_t vacancy;

  /* shup up compiler */
  (void)data;

  packets_tx = 0;

  while (running)
  {
    do {
      rc = ioctl(writeFifoFd, AXIS_FIFO_GET_TX_VACANCY, &vacancy);
      if (rc) {
        perror("ioctl");
        return (void *)0;
      }
      if (vacancy < (uint32_t)MAX_BUF_SIZE_BYTES)
        usleep(100);
    } while (vacancy < (uint32_t)MAX_BUF_SIZE_BYTES);

    printf("Send a message to %s : ",_opt_dev_tx);
    scanf("%s",&buf[0]);
    printf("Sending %s\n\r",buf);

    bytesFifo = write(writeFifoFd, buf, strlen(buf));
    if (bytesFifo > 0) {
      printf("bytes to fifo %ld\n",bytesFifo);
      packets_tx++;
    } else {
      perror("write");
      quit();
    }
  }

  return (void *)0;
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

static void display_help(char * progName)
{
  printf("Usage : %s [OPTIONS]\n"
      "\n"
      "  -h, --help     Print this menu\n"
      "  -t, --devTx    Device to use ... /dev/axis_fifo_0x00000000a0000000\n"
      "  -r, --devRx    Device to use ... /dev/axis_fifo_0x00000000a0000000\n"
      ,
      progName
  );
}

static void print_opts()
{
  printf("Options : \n"
      "DevTX          : %s\n"
      "DevRx          : %s\n"
      ,
      _opt_dev_tx,
      _opt_dev_rx
  );
}

static int process_options(int argc, char * argv[])
{
  strcpy(_opt_dev_tx,DEF_DEV_TX);
  strcpy(_opt_dev_rx,DEF_DEV_RX);

  for (;;) {
    int option_index = 0;
    static const char *short_options = "hr:t:";
    static const struct option long_options[] = {
        {"help", no_argument, 0, 'h'},
        {"devRx", required_argument, 0, 'r'},
        {"devTx", required_argument, 0, 't'},
        {0,0,0,0},
    };

    int c = getopt_long(argc, argv, short_options,
                        long_options, &option_index);

    if (c == EOF) {
      break;
    }

    switch (c) {
      case 't':
        strcpy(_opt_dev_tx, optarg);
        break;

      case 'r':
        strcpy(_opt_dev_rx, optarg);
        break;

      default:
      case 'h':
        display_help(argv[0]);
        exit(0);
        break;
    }
  }

  print_opts();
  return 0;
}
