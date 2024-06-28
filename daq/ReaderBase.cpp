/*
 * CIBReader.cpp
 *
 *  Created on: Mar 25, 2024
 *      Author: Nuno Barros
 */

#include <ReaderBase.h>
#include <string>
#include <spdlog/spdlog.h>

extern "C"
{
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
}
using time_stamp = std::chrono::time_point<std::chrono::system_clock, std::chrono::microseconds>;

namespace cib
{

  ReaderBase::ReaderBase (const bool simulation)
                          :m_simulation(simulation)
                           , m_ready(false)
                           ,m_num_transfers(0)
                           ,m_sent_bytes(0)
                           ,m_sent_packets(0)
                           ,m_error_state(false)
                           ,m_readout_thread(nullptr)
                           ,m_receiver_port(0)
                           ,m_receiver_host("")
                           ,m_receiver_timeout(5000) // 5 ms
                           ,m_receiver_init(false)
                           , m_receiver_ios()
                           ,m_receiver_socket(m_receiver_ios)
                           {

    if (!m_simulation)
    {
    }
                           }

  ReaderBase::~ReaderBase ()
  {
    // there isn't anything to be destructed
    SPDLOG_TRACE("Reader leaving destructor.");
  }

  int ReaderBase::init()
  {
    SPDLOG_INFO("Processing init.");
    return 0;
  }
  // transmitter is just meant to establish the socket connection for data streaming
  void ReaderBase::init_transmitter()
  {

    if (m_state != kSet)
    {
      SPDLOG_WARN("System does not seem to be configured yet");
      return;
    }

    SPDLOG_INFO("Initiating ethernet connection : {}:{}",m_receiver_host.c_str(),m_receiver_port);
    // this is a client connection.
    try
    {
      boost::asio::ip::tcp::resolver resolver( m_receiver_ios );
      boost::asio::ip::tcp::resolver::query query(m_receiver_host, std::to_string(m_receiver_port),boost::asio::ip::tcp::resolver::query::v4_mapped ) ; //"np04-ctb-1", 8991
      boost::asio::ip::tcp::resolver::iterator iter = resolver.resolve(query) ;
      m_receiver_endpoint = iter->endpoint();
      m_receiver_socket.connect( m_receiver_endpoint );
      m_state = kReady;
      // now we're ready to ship the data
    }
    catch(std::exception &e)
    {
      SPDLOG_ERROR("Failed to init with exception {0}",e.what());
      m_state = kSet;
    }
    catch(...)
    {
      SPDLOG_ERROR("Failed to init with unknown exception");
      m_state = kSet;
    }
//    if (m_receiver_init)
//    {
//      SPDLOG_INFO("Transmitter socket initialized");
//    }
//    else
//    {
//      SPDLOG_WARN("Transmitter socket NOT initialized");
//    }
  }


  void ReaderBase::term_transmitter()
  {
    if (m_state == kRunning)
    {
      SPDLOG_WARN("Still running. Terminating the run first");
      m_run_enable.store(false);
      //stop_run();
    }

    m_receiver_socket.close();
    m_receiver_ios.stop();
    m_state = kSet;
  }


  void ReaderBase::get_feedback(std::vector<daq::iols_feedback_msg_t> &msgs)
  {
    // create a temp variable that will fetch everything
    daq::iols_feedback_msg_t tmp_msgs[1024];
    size_t n_items = m_message_queue.pop(tmp_msgs,1024);
    if (n_items > 0)
    {
      msgs.insert(msgs.end(),tmp_msgs,tmp_msgs+n_items);
    }
  }

  int ReaderBase::send_data(uint8_t *packet, size_t n_bytes)
  {
    // do a quick check that we actually have a socket to ship the data

      static uint32_t seq_num = 0;
      bool has_error = false;
      // we also accept to send data without running state, even though that may give trouble
      // but that is irrelevant for this method
      if ((m_state ==  kReady) || (m_state == kRunning))
      {
        SPDLOG_ERROR("Socket is not in a usable state");
        daq::iols_feedback_msg_t msg;
        msg.sev = "ERROR";
        msg.msg = "Socket is not in a usable state";
        m_message_queue.push(msg);
        return 1;
      }

      if (!m_receiver_init)
      {
        spdlog::error("Transmitter is not ready");
        return 1;
      }
      boost::system::error_code boost_error;

      try
      {
        boost::asio::write( m_receiver_socket, boost::asio::buffer( packet, n_bytes), boost_error ) ;
        if ( boost_error == boost::asio::error::eof)
        {
          std::string error_message = "Socket closed: " + boost_error.message();
          SPDLOG_ERROR("BOOST ASIO Connection lost: %s\n",error_message.c_str());
          has_error = true;
        }

        if ( boost_error )
        {
          std::string error_message = "Transmission failure: " + boost_error.message();
          SPDLOG_ERROR("BOOST non-descript error: %s\n",error_message.c_str());
          has_error = true;
        }
        if (!has_error)
        {
          m_tot_packets_sent++;
          m_tot_bytes_sent += n_bytes;
        }
      }
      catch(std::exception &e)
      {
        SPDLOG_ERROR("Caught an exception: {}",e.what());
        return 1;
      }
      catch(...)
      {
        SPDLOG_ERROR("Caught unknown exception\n");
        return 1;
      }
      if (has_error)
      {
        SPDLOG_ERROR("Failed to send data. Stopping execution.");
        return 1;
      }
      return 0;
  }


  int ReaderBase::set_eth_receiver(const std::string &host, const unsigned int port)
  {
    daq::iols_feedback_msg_t msg;
    if (m_state == kInit)
    {
      SPDLOG_INFO("Setting receiver to {0}:{1}",host,port);
    }
    else if (m_state == kSet)
    {
      SPDLOG_WARN("Overriding receiver data after initial configuration");
      add_feedback("WARN","Overriding receiver data after initial configuration");
    }
    else if (m_state == kReady)
    {
      SPDLOG_WARN("Socket is already initialized. Closing and reopening");
      add_feedback("WARN","Socket is already initialized. Closing and reopening");
      term_transmitter();
    }
    else
    {
      SPDLOG_ERROR("System is already running.");
      add_feedback("ERROR","System is already running");
      return 1;
    }

    SPDLOG_INFO("Setting receiver data to {0}:{1}",host.c_str(),port);
    m_receiver_host = host;
    m_receiver_port = port;
    m_state = kSet;
    add_feedback("INFO","Receiver configuration set");

    return 0;
  }

  int ReaderBase::start_run(const uint32_t run_number)
  {
    SPDLOG_INFO("Starting run.");
    init_transmitter();
    if (m_state != kReady)
    {
      SPDLOG_ERROR("Not ready to start a run");
      return 1;
    }
    m_run_enable.store(true);
    m_state = kRunning;
    return 0;
  }

  int ReaderBase::stop_run()
  {
    SPDLOG_INFO("Stopping run.");
    if (m_state != kRunning)
    {
      SPDLOG_ERROR("Trying to stop but run not running");
      return 1;
    }
    m_run_enable.store(false);
    m_state = kReady;
    term_transmitter();
    return 0;
  }

  void ReaderBase::add_feedback(const std::string severity, std::string msg)
  {
    daq::iols_feedback_msg_t entry;
    entry.sev = severity;
    entry.msg = msg;
    m_message_queue.push(entry);
  }


} /* namespace cib */
