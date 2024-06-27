//
//
// THIS FILE IS NOT YET OPERATIONAL
//


#include <iostream>
#include <zmq.hpp>
#include <sstream>
//#include "cib.pb.h"
#include <cstdio>
#include <csignal>
#include <atomic>

volatile std::atomic<bool> g_run;

void signal_handler(int sig)
{
  g_run = 0;
}

void dump_config(cib::Config &c)
{
  printf("--------------------\n");
  printf("Config :            \n");
  printf("--------------------\n");
  printf("Packet size   : %u \n",c.packet_size());
  printf("stream host   : %s  \n",c.stream_host().c_str());
  printf("stream port   : %u \n",c.stream_port());
  printf("Monitor       : %i  \n",c.enable_monitor());
  printf("Monitor host  : %s  \n",c.monitor_host().c_str());
  printf("Monitor port  : %u \n",c.monitor_port());
  printf("--------------------\n");
}
int main(int argc, char **argv)
{

  std::signal(SIGINT, signal_handler);

  setvbuf(stdout, NULL, _IOLBF, 0);
  setvbuf(stderr, NULL, _IOLBF, 0);

  printf("cib_daq_server is initializing\n");
  printf("cib_daq_server will listen to poty 1234\n");

  zmq::context_t context;
  zmq::socket_t socket(context, ZMQ_REP);
  socket.bind("tcp://*:1234");
  printf("cib_daq_server ready to serve\n");

  g_run = 1;
  // this actuually counts served connections
  for (size_t i = 0; g_run != 0; i++)
  {
    zmq::message_t cmd;
    socket.recv(cmd,zmq::recv_flags::none);

    cib::Request request;
    std::string reply_str;
    std::string cmd_str((char*)cmd.data(), cmd.size());

    if (!request.ParseFromString(cmd_str))
    {
      printf("Could not parse message %li size %li\n",i,cmd.size());
    }
    else if (request.request().Is<cib::Config>())
    {
      printf("Received a config request with size %li.\n",cmd.size());
      cib::Config conf;
      request.request().UnpackTo(&conf);
      dump_config(conf);
      cib::Statistics s;
      s.SerializeToString(&reply_str);
    }
    else
    {
      printf("Received an unknown message!\n");
    }
    // now reply to the request
    zmq::message_t reply(reply_str.size());
    memcpy((void*)reply.data(), reply_str.c_str(), reply_str.size());
    socket.send(reply,zmq::send_flags::none);
  }


	std::cout << "Hello World" << std::endl;
	std::cout << "Version " << 1 << "." << 2 << std::endl;
	socket.close();

	return 0;
}
