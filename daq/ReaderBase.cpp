/*
 * CIBReader.cpp
 *
 *  Created on: Mar 25, 2024
 *      Author: Nuno Barros
 */

#include <ReaderBase.h>
#include <string>
#include <spdlog/spdlog.h>
#include <cib_data_utils.h>
#include <cib_mem.h>
#include <mem_utils.h>

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
                            ,m_ready(false)
                            ,m_num_transfers(0)
                            ,m_sent_bytes(0)
                            ,m_sent_packets(0)
                            ,m_error_state(false)
                            ,m_readout_thread(nullptr)
                            ,m_receiver_port(0)
                            ,m_receiver_host("")
                            ,m_receiver_init(false)
                            ,m_receiver_timeout(5000) // 5 ms
                            ,m_receiver_ios()
                            ,m_receiver_socket(m_receiver_ios)
                            ,m_take_data(false)
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

    SPDLOG_INFO("Initiating ethernet connection : {0}:{1}",m_receiver_host.c_str(),m_receiver_port);
    // this is a client connection.
    try
    {
      boost::asio::ip::tcp::resolver resolver( m_receiver_ios );
      //boost::asio::ip::tcp::resolver::query query(m_receiver_host, std::to_string(m_receiver_port),boost::asio::ip::tcp::resolver::query::v4_mapped ) ; //"np04-ctb-1", 8991
      //boost::asio::ip::tcp::resolver::iterator iter = resolver.resolve(query) ;
      boost::asio::ip::tcp::resolver::iterator iter = resolver.resolve(boost::asio::ip::tcp::v4(),
                                                                       m_receiver_host,
                                                                       std::to_string(m_receiver_port));
      m_receiver_endpoint = iter->endpoint();
      m_receiver_socket.connect( m_receiver_endpoint );
      m_state = kReady;
      // now we're ready to ship the data
    }
    catch(std::exception &e)
    {
      SPDLOG_ERROR("Failed to init_transmitter with exception {0}",e.what());
      std::ostringstream msg("");
      msg << "Failed to init_transmitter with exception : " << e.what();
      add_feedback("ERROR",msg.str());
      m_state = kSet;
    }
    catch(...)
    {
      SPDLOG_ERROR("Failed to init with unknown exception");
      std::ostringstream msg("");
      msg << "Failed to init_transmitter with unknown exception ";
      add_feedback("ERROR",msg.str());
      m_state = kSet;
    }
  }


  void ReaderBase::term_transmitter()
  {
    if (m_state == kRunning)
    {
      SPDLOG_WARN("Still running. Terminating the run first");
      add_feedback("WARN","Run still ongoing. Stopping it first.");
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

      //      static uint32_t seq_num = 0;
      bool has_error = false;
      // we also accept to send data without running state, even though that may give trouble
      // but that is irrelevant for this method
      if ((m_state !=  kReady) && (m_state != kRunning))
      {
        SPDLOG_ERROR("Socket is not in a usable state");
        add_feedback("ERROR","Socket is not in a usable state");
        return 1;
      }

      boost::system::error_code boost_error;

      try
      {
        boost::asio::write( m_receiver_socket, boost::asio::buffer( packet, n_bytes), boost_error ) ;
        if ( boost_error == boost::asio::error::eof)
        {
          std::string error_message = "Socket closed: " + boost_error.message();
          SPDLOG_ERROR("BOOST ASIO Connection lost: {0}",error_message.c_str());
          add_feedback("ERROR",error_message);
          has_error = true;
        }

        if ( boost_error )
        {
          std::string error_message = "Transmission failure: " + boost_error.message();
          SPDLOG_ERROR("BOOST non-descript error: {0}",error_message.c_str());
          add_feedback("ERROR",error_message);

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
        SPDLOG_ERROR("Caught an exception: {0}",e.what());
        std::ostringstream error_message("");
        error_message << "Caught STL exception : " << e.what();
        add_feedback("ERROR",error_message.str());

        return 1;
      }
      catch(...)
      {
        SPDLOG_ERROR("Caught unknown exception");
        add_feedback("ERROR","Caught unknown exception");

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


  void ReaderBase::fake_data_generator()
  {
    SPDLOG_INFO("Starting fake data generator");
    uint32_t run_packets_tx = 0;
    uint8_t seq_num = 0;
    uint32_t run_bytes_tx = 0;

    m_eth_packet.header.set_version(1);
    m_eth_packet.header.packet_size = 16; // payload/word size in bytes
    // daq::iols_trigger_t tword;
    int32_t i = -1000,j = 0,k = 100000000;
    int rc = 0;
    pdts_tstamp_t timestamp;
    // -- map the timestamp register
    int mem_dev_fd = 0;
    uintptr_t tstamp_addr = cib::util::map_phys_mem(mem_dev_fd,GPIO_TSTAMP_MEM_LOW,GPIO_TSTAMP_MEM_HIGH);
    if (tstamp_addr)
    {
      SPDLOG_DEBUG("Mapped address : 0x{0:X}", tstamp_addr);
    }
    else
    {
      SPDLOG_ERROR("Failed to map memory. This will fail somewhere.");
    }

    while (m_take_data.load())
    {
      // generate fake data
      if (tstamp_addr)
      {
        // read the timestamp from the mapped register
        timestamp.low = cib::util::reg_read(tstamp_addr + 0x0);
        timestamp.high = cib::util::reg_read(tstamp_addr + GPIO_CH_OFFSET);
        m_eth_packet.word.timestamp = timestamp.get_timestamp();
        SPDLOG_DEBUG("Read timestamp : {0}", m_eth_packet.word.timestamp);
      }
      else
      {
        m_eth_packet.word.timestamp = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::microseconds>(
              std::chrono::system_clock::now().time_since_epoch()
            ).count()
          );
      }      

      m_eth_packet.word.pos_m1 = 0;
      data::set_pos_m2(m_eth_packet.word, i);
      data::set_pos_m3(m_eth_packet.word, j);

      // m_eth_packet.word.timestamp = k;

      // update sequence id and send only the 20-byte packet (avoid struct padding)
      m_eth_packet.header.sequence_id = seq_num;
      SPDLOG_DEBUG("Sending packet: seq_id={0}, pos_m2={1}, pos_m3={2}, timestamp={3}",
                   static_cast<uint16_t>(m_eth_packet.header.sequence_id),
                   static_cast<int32_t>(data::get_pos_m2(m_eth_packet.word)),
                   static_cast<int32_t>(data::get_pos_m3(m_eth_packet.word)),
                   static_cast<uint64_t>(m_eth_packet.word.timestamp)
                   );
      rc = send_data(reinterpret_cast<uint8_t *>(&m_eth_packet), 20);
      if (rc != 0)
      {
        // failed transmission. Stop acquisition
        SPDLOG_ERROR("Failed to send data. Stopping acquisition");
        m_take_data.store(false);
      }
      else
      {
        i++;
        j++;
        k++;
        seq_num++; // let the variable rollover when we reach the end
        run_packets_tx++;
        run_bytes_tx += 20;
        m_tot_packets_sent++;
        m_tot_bytes_sent += 20;
      }
      // sleep a bit to simulate data rate
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    SPDLOG_INFO("Run stopped. Sent {0} bytes ({1} packets)", run_bytes_tx, run_packets_tx);

    if (tstamp_addr)
    {
      SPDLOG_DEBUG("Unmapping the MM register.");
      cib::util::unmap_mem(tstamp_addr, (GPIO_TSTAMP_MEM_HIGH-GPIO_TSTAMP_MEM_LOW));
      close(mem_dev_fd);
    }
    SPDLOG_INFO("Exiting fake data generator");
  }

  int ReaderBase::start_run(const uint32_t run_number)
  {
    SPDLOG_INFO("Starting run.");
    init_transmitter();
    if (m_state != kReady)
    {
      SPDLOG_ERROR("Not ready to start a run");
      // terminate the transmitter if it failed
      term_transmitter();
      return 1;
    }

    if (m_simulation)
    {
      SPDLOG_TRACE("Preparing to start taking data");
      m_take_data.store(true);

      SPDLOG_INFO("Simulation mode - starting fake data generator");
      m_readout_thread = std::unique_ptr<std::thread>(new std::thread(&ReaderBase::fake_data_generator, this));
      
      if (m_readout_thread.get() == nullptr)
      {
        SPDLOG_ERROR("Failed to launch readout thread.");
        add_feedback("ERROR", "Failed to launch readout thread");
        // at this point terminate the transmitter
        term_transmitter();
        return 1;
      }
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

    if (m_simulation)
    {
      SPDLOG_TRACE("Stopping fake data generator");
      m_take_data.store(false);
      // wait for a sec to allow the data taking to stop
      std::this_thread::sleep_for(std::chrono::milliseconds(50));

      if (m_readout_thread && m_readout_thread->joinable())
      {
        m_readout_thread->join();
      }
      m_readout_thread = nullptr;
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
