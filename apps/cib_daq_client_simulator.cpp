/*
 * test_daq_client.cpp
 *
 *  Created on: Mar 24, 2024
 *      Author: Nuno Barros
 */


#include <iostream>
#include <sstream>
#include <cstdio>
#include <fstream>
#include <thread>
#include <chrono>
#include <sstream>

// necessary to have getopt
extern "C"
{
#include <readline/readline.h>
#include <readline/history.h>

#include <unistd.h>
};

#include <boost/asio.hpp>
#include <boost/array.hpp>

#include <json.hpp>

// logger stuff
// although we could just work with printf's
#include <spdlog/spdlog.h>
#include <spdlog/cfg/env.h>
#include <cib_data_fmt.h>
#include <mem_utils.h>
#include <cib_data_utils.h>

using json = nlohmann::json;

std::atomic<bool> g_receive;
std::thread *m_receiver_task;

json get_config()
{
  json cfg;
  cfg["command"] = "config";
  cfg["config"]["sockets"]["receiver"]["host"] = "localhost";
  cfg["config"]["sockets"]["receiver"]["port"] = 4243;
  cfg["config"]["sockets"]["receiver"]["timeout"] = 1000;
  spdlog::info("Raw configuration object: [{0}]",cfg.dump());

  return cfg;
}

/**
 * This method is a separate thread receiving data being sent
 * Some minimal consistency checks are done on the received data
 */
void receiver()
{
  unsigned int port = 4243;
  std::string host = "localhost";
  boost::asio::io_service       ios;
  boost::asio::ip::tcp::socket  socket(ios);
  std::chrono::microseconds     timeout(100000);
  boost::asio::ip::tcp::acceptor acceptor(ios,boost::asio::ip::tcp::endpoint( boost::asio::ip::tcp::v4(),port));

  char file_name[200] = "" ;
  time_t rawtime;
  time( & rawtime ) ;
  struct tm local_tm;
  struct tm * timeinfo = localtime_r( & rawtime , &local_tm) ;
  strftime( file_name, sizeof(file_name), "%F_%H.%M.%S.calib", timeinfo );
  std::string global_name = "cib_data_";
  global_name+= file_name ;

  std::ofstream out_file( global_name, std::ofstream::binary ) ;

  bool failed = false;

  while (g_receive.load())
  {
    std::future<void> accepting = async( std::launch::async, [&]{ acceptor.accept(socket) ; } ) ;
    while (g_receive.load())
    {
      if ( accepting.wait_for( timeout ) == std::future_status::ready )
      {
        SPDLOG_INFO("Connection request arrived.");
        break ;
      }
    }
    if (!g_receive.load())
    {
      // we want to stop receiving data
      SPDLOG_DEBUG("Creating temporary connection to consume lingering listener.");
      boost::asio::io_service tmp_ios;
      boost::asio::ip::tcp::resolver tmp_resolver( tmp_ios );
      boost::asio::ip::tcp::resolver::query tmp_query("localhost", std::to_string(port) ) ; //"np04-ctb-1", 8991
      boost::asio::ip::tcp::resolver::iterator tmp_iter = tmp_resolver.resolve(tmp_query) ;
      boost::asio::ip::tcp::socket tmp_sock(tmp_ios);
      tmp_sock.connect(tmp_iter->endpoint());
      boost::system::error_code tmp_ce;
      tmp_sock.shutdown(boost::asio::ip::tcp::socket::shutdown_send, tmp_ce);
      tmp_sock.close();
      tmp_resolver.cancel();
      tmp_ios.stop();
      // these do not cause trouble if they are called with a closed acceptor
      acceptor.cancel();
      acceptor.close();
      if (acceptor.is_open())
      {
        SPDLOG_WARN("Acceptor is still open. This should not happen");
      }
      SPDLOG_INFO("Exit request found.");
    }
    else
    {
      SPDLOG_INFO("Connection received. Start processing.\n");
    }
    boost::system::error_code error;
    cib::daq::iols_tcp_packet_t packet;
    //    boost::array<char, 1024> req_buff{" "} ;

    while (g_receive.load())
    {
      boost::system::error_code receiving_error;
      boost::asio::read( socket, boost::asio::buffer( &packet, sizeof(packet) ), receiving_error ) ;
      if ( ! receiving_error )
      {
        cib::daq::iols_trigger_t *word;
        word = &(packet.word); //reinterpret_cast<daq::iols_trigger_t*>(m_buffer);
        SPDLOG_DEBUG("RX Word : TS {0} M1 {1} M2 {2} M3 {3}",word->timestamp,cib::data::get_m1(*word),cib::data::get_m2(*word),cib::data::get_m3(*word));
        //continue ;
      }
      if ( receiving_error == boost::asio::error::eof)
      {
        SPDLOG_ERROR("Socket closed: {0}",receiving_error.message());
        failed = true;
        break;
      }

      if ( receiving_error )
      {
        SPDLOG_ERROR("Read falure: {0}",receiving_error.message());
        failed = true;
        break;
      }
      out_file.write(reinterpret_cast<const char*>(&(packet.word)),sizeof(cib::daq::iols_trigger_t));
      out_file.flush();
    }
    if (failed)
    {
      SPDLOG_ERROR("Detected readout failure. Leaving thread");
      break;
    }
  }
  // if it reached this point, one way or another the loop is terminated
  boost::system::error_code closing_error;
  if (failed)
  {
    socket.shutdown(boost::asio::ip::tcp::socket::shutdown_send,closing_error);
    if (closing_error)
    {
      SPDLOG_ERROR("Error in shutdown : {0}",closing_error.message());
    }
  }
  socket.close(closing_error);
  if (closing_error)
  {
    SPDLOG_ERROR("Error closing receiver socket : {0}",closing_error.message());
  }
  out_file.close();
  SPDLOG_INFO("Data taking stopped");
}


void print_usage(const char *prog) {
  spdlog::info( "Usage: {0} [-w ip] [cmd] ", prog);
}

void print_help() {
  printf("Available commands:\n");
  printf("  config\n");
  printf("    Reboot the WIB\n");
  printf("  start_run <run_number>\n");
  printf("    Start a data taking run\n");
  printf("  stop_run\n");
  printf("    Stop a data taking run\n");
  printf("  help\n");
  printf("    Show this help\n");
  printf("  exit\n");
  printf("    Closes the command interface\n");
}

int send_message(boost::asio::ip::tcp::socket  &socket,const std::string &msg)
{
  boost::system::error_code error;
  SPDLOG_DEBUG("Sending message : {0}",msg);
  boost::asio::write( socket, boost::asio::buffer( msg ), error ) ;
  boost::array<char, 1024> reply_buf{" "} ;
  socket.read_some( boost::asio::buffer(reply_buf ), error);
  std::stringstream raw_answer( std::string(reply_buf .begin(), reply_buf .end() ) ) ;
  SPDLOG_DEBUG("Unformatted answer : {0}",raw_answer.str());

  json answer;
  raw_answer >> answer ;
  json messages = answer.at("feedback");
  SPDLOG_DEBUG("Received messages : {0}",messages.size());
  bool ret = true ;
  for (nlohmann::json::size_type i = 0; i != messages.size(); ++i )
  {
    std::string type = messages[i]["type"].dump() ;
    if ( type.find("error") != std::string::npos || type.find("Error") != std::string::npos || type.find("ERROR") != std::string::npos )
    {
      SPDLOG_ERROR("{0}",messages[i]["message"].dump());
    }
    else if ( type.find("warn") != std::string::npos || type.find("WARN") != std::string::npos || type.find("WARNING") != std::string::npos )
    {
      SPDLOG_WARN("{0}", messages[i]["message"].dump());
    }
    else if ( type.find("info") != std::string::npos || type.find("Info") != std::string::npos || type.find("INFO") != std::string::npos)
    {
      SPDLOG_INFO("{0}",messages[i]["message"].dump());
    }
    else
    {
      SPDLOG_CRITICAL("Unknown severity : {0}",messages[i]["message"].dump());
    }
  }
  return 0;
}
int run_command(boost::asio::ip::tcp::socket  &socket, int argc, char **argv)
{
  if (argc < 1) return 1;
  int ret = 0;
  std::string cmd(argv[0]);
  if (cmd == "exit") {
    return 255;
  }
  else if (cmd == "config")
  {
    std::string cfg_frag = get_config().dump();
    ret = send_message(socket,cfg_frag);
    if (ret)
    {
      SPDLOG_ERROR("Failed to configure system. Check feedback messages");
    }
  }
  else if (cmd == "start_run")
  {

    SPDLOG_DEBUG("Starting the listening thread");
    m_receiver_task = new std::thread(&receiver);
    g_receive.store(true);
    json obj;
    static int rn = 0;
    rn++;
    obj["command"] = "start_run";
    obj["run_number"] = rn;
    ret = send_message(socket,obj.dump());
    if (ret)
    {
      SPDLOG_ERROR("Failed to start run. Check feedback messages");
    }
  }
  else if (cmd == "stop_run")
  {
    json obj;
    obj["command"] = "stop_run";
    ret = send_message(socket,obj.dump());
    if (ret)
    {
      SPDLOG_ERROR("Failed to stop run. Check feedback messages");
    }
    g_receive.store(false);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    m_receiver_task->join();
    delete m_receiver_task;
  }
  else if (cmd == "help")
  {
    print_help();
  }
  else
  {
    printf("Unrecognized Command: %s\n",argv[0]);
    return 0;
  }
  return 0;
}

int main(int argc, char **argv)
{
  char *ip = (char*)"127.0.0.1";
  spdlog::set_pattern("[%s:%!:%#][%^%L%$] [thread %t] %v");

  spdlog::set_level(spdlog::level::trace); // Set global log level to debug
  // if the SPDLOG_LEVEL variable is set, it overrides
  //SPDLOG_LEVEL=info,mylogger=trace

  spdlog::cfg::load_env_levels();
  // if the level was not

  SPDLOG_INFO( "spdlog active level {}",SPDLOG_ACTIVE_LEVEL);

  std::string server_host = "localhost";
  std::string server_port = "8992";

  signed char opt;
  //    while ((opt = getopt(argc, argv, "w:h")) != -1) {
  while ((opt = getopt(argc, argv, "hH:P:")) != -1) {
    switch (opt) {
      case 'h':
        print_usage(argv[0]);
        print_help();
        return 1;
      case 'H':
        server_host = optarg;
        break;
      case 'P':
        server_port = optarg;
        break;
        //           case 'w':
        //               ip = optarg;
        //               break;
      default: /* '?' */
        print_usage(argv[0]);
        return 1;
    }
  }
  try
  {
    // fixme: establish the connection here
    boost::system::error_code ec;
    boost::asio::io_service ios;
    boost::asio::ip::tcp::resolver resolver( ios );
    // deprecated code
    //boost::asio::ip::tcp::resolver::query query("localhost", "8992",boost::asio::ip::tcp::resolver::query::v4_mapped) ; //"np04-ctb-1", 8991
    //boost::asio::ip::tcp::resolver::iterator iter = resolver.resolve(query,ec) ;
    boost::asio::ip::tcp::resolver::iterator iter = resolver.resolve(boost::asio::ip::tcp::v4(),
                                                                     server_host, server_port,ec);

    boost::asio::ip::tcp::resolver::iterator end;
    boost::asio::ip::tcp::resolver::iterator tmpit = iter;

    if (ec)
    {
      SPDLOG_ERROR("Failed to resolve the server address : {0}",ec.message());
      return 0;
    }
    while(tmpit != end)
    {
      SPDLOG_TRACE("Resolver entry : name : {0} port: {1} v4: {2}",tmpit->host_name(),tmpit->service_name(),tmpit->endpoint().address().is_v4());
      tmpit++;
    }

    //SPDLOG_TRACE("Iterator size : {0}",tmp_iter);
    boost::asio::ip::tcp::socket socket(ios);
    SPDLOG_DEBUG("Opening connection to server");
    socket.connect(iter->endpoint(),ec);
    if (ec)
    {
      SPDLOG_CRITICAL("Failed to connect to the server. Message : {0}",ec.message());
      return 0;
    }
    // this is at the end...when exit is called

    if (optind < argc) {
      return run_command(socket,argc-optind,argv+optind);
    } else {
      char* buf;
      while ((buf = readline(">> ")) != nullptr) {
        if (strlen(buf) > 0) {
          add_history(buf);
        } else {
          free(buf);
          continue;
        }
        char *delim = (char*)" ";
        int count = 1;
        char *ptr = buf;
        while((ptr = strchr(ptr, delim[0])) != NULL) {
          count++;
          ptr++;
        }
        if (count > 0) {
          char **cmd = new char*[count];
          cmd[0] = strtok(buf, delim);
          int i;
          for (i = 1; cmd[i-1] != NULL && i < count; i++) {
            cmd[i] = strtok(NULL, delim);
          }
          if (cmd[i-1] == NULL) i--;
          int ret = run_command(socket,i,cmd);
          delete [] cmd;
          if (ret == 255)
          {
            // exit was called. Close the show
            //socket.shutdown(boost::asio::ip::tcp::socket::shutdown_send, tmp_ce);
            socket.close();
            //resolver.cancel();
            //ios.stop();

            return 0;
          }
          if (ret != 0) return ret;
        } else {
          return 0;
        }
        free(buf);
      }
    }
  }
  catch(std::exception &e)
  {
    SPDLOG_CRITICAL("Caught an exception: {0}",e.what());

  }
  catch(...)
  {
    SPDLOG_CRITICAL("Caught an unknown exception");
  }
  return 0;
}
