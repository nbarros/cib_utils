/*
 * cib_lbls_pin_ptr_streamer.cpp
 *
 *  Created on: May 5, 2024
 *      Author: Nuno Barros
 */




#include <cstdio>
#include <cstdlib>
#include <thread>
#include <chrono>
#include <csignal>
#include <atomic>
#include <spdlog/spdlog.h>
#include <cerror>
#include <zmq.hpp>
#include <cib.pb.h>
#include <string>

extern "C"
{
  #include <unistd.h>           // Close() system call
  #include <fcntl.h>              // Flags for open()
  #include <sys/ioctl.h>
  #include <inttypes.h>

}

/*----------------------------------------------------------------------------
 * Internal Definitions
 *----------------------------------------------------------------------------*/
// FIXME: Add real device block device
//#define DEF_DEV_TX "/dev/axis_fifo_0x43c10000"
#define DEF_DEV_RX "/dev/axis_fifo_0x43c10000"
#define MAX_BUF_SIZE_BYTES 1450

struct thread_data {
  int rc;
};

pthread_t read_from_fifo_thread;
static int readFifoFd;

static volatile std::atomic<bool> run = true;
static volatile std::atomic<bool> acquire = true;
static volatile std::atomic<std::string> lbls_server;
static volatile zmq::context_t  srv_context;
static volatile zmq::socket_t  *srv_socket;

// application is a simple service that receives a destination address and keeps a continuous loop
// reading out the axi fifo and shipping the data to the destination that was provided
static void signal_handler(int signal);
static void *read_from_fifo_thread_fn(void *data)
{
  ssize_t bytesFifo;
  int packets_rx;
  // buffer of received data
  uint8_t buf[MAX_BUF_SIZE_BYTES];
  uint64_t ts;
  /* shup up compiler */
  (void)data;

  packets_rx = 0;

  spdlog::debug("Setting up zmq client");
  zmq::context_t context;
  zmq::socket_t socket(context, ZMQ_REP);
  spdlog::debug("Setting up zmq client");

  while (running)
  {
    bytesFifo = read(readFifoFd, buf, MAX_BUF_SIZE_BYTES);
    if (bytesFifo > 0)
    {
      spdlog::trace("bytes from fifo {0}",bytesFifo);
//      printf("Read : %s\n\r",buf);
//      ts = *reinterpret_cast<uint64_t*>(&buf);
//      printf("Read : %" PRIu64 "\n",ts);
      // create a zmq message to be shipped
      zmq::message_t msg();
      socket.send(request,zmq::send_flags::none);

      // ship the received word to the destination
      packets_rx++;
    }
  }
  printf("Out of loop\n");


  return (void *)0;
}

static void quit(void)
{
  spdlog::warn("Found a call of quit(). Bailing out.");
  running.store(false);
  srv_context.terminate();
  srv_socket->close();
}

/*----------------------------------------------------------------------------
 * Main
 *----------------------------------------------------------------------------*/
int main(int argc, char** argv)
{
  // Listen to ctrl+c and assert
  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);
  std::signal(SIGQUIT, signal_handler);

  // init the logging mechanism
  spdlog::set_pattern("pin_ptr : [%^%L%$] %v");
  spdlog::set_level(spdlog::level::trace); // Set global log level to debug

  /*************/
  /* open FIFO */
  /*************/
  readFifoFd = open(DEF_DEV_RX, O_RDONLY | O_NONBLOCK);
  if (readFifoFd < 0)
  {
    spdlog::critical("Open read failed with error: {0}", std::strerror(errno));
    return 255;
  }

  // set up a server that waits for a configuration request

  //zmq::socket_t socket(srv_context, ZMQ_REP);
  srv_socket = new zmq::socket_t(srv_context,ZMQ_REP);

  char *addr;
  int len = asprintf(&addr,"tcp://%s:1234",ip);
  if (len < 0)
  {
    spdlog::critical("Failed to initialize the configuration socket");
    return 1;
  }
  socket->bind(addr);
  spdlog::info("LBLS PIN TRIGGER server ready to serve.");
  // enter a loop to count requests
  for (int ireq = 0; ; ireq++)
  {
    if (!running.load())
    {
      // get out of the loop
      break;
    }
    cib::Command command;
    zmq::message_t cmd;
    socket->recv(cmd,zmq::recv_flags::none);
    std::string reply_str;
    std::string cmd_str((char*)cmd.data(), cmd.size());
    if (!command.ParseFromString(cmd_str))
    {
      spdlog::error("Could not parse message {0} size {1}",ireq,cmd.size());
    }
    else if (command.cmd().Is<cib::ConfigLbls>())
    {
      // get the destination location to stream data
      cib::ConfigLbls conf;
      command.cmd().UnpackTo(&conf);
      lbls_server.store(conf.server());
      cib::Empty rep;
      rep.SerializeToString(&reply_str);
    }
    // do we actually need this?
    // we could start immediately after configuration is sent.
    // the server should already be available and data is only sent
    // when there is something in the fifo
    else if (command.cmd().Is<cib::StartRun>())
    {

    }
    else if (command.cmd().Is<cib::StopRun>())
    {

    }
    else
    {
      spdlog::error("Received an unknown message!");
    }
    if (running.load())
    {
      // chances are that the context has already been terminated
      // and the socket has been closed
      //so just drop out of the loop
      spdlog::warn("Leaving serving loop");
      // just in case, also terminate acquire
      acquire.store(false);
      break;
    }
    spdlog::debug("sending message {0} size {1} bytes",ireq,reply_str.size());
    zmq::message_t reply(reply_str.size());
    memcpy((void*)reply.data(), reply_str.c_str(), reply_str.size());
    socket->send(reply,zmq::send_flags::none);

  }

  // join the thread?
  spdlog::info("Joining acquisition thread (if it exists)");
  if ()
  }
  // there are no arguments to this service
  return 0;
}
