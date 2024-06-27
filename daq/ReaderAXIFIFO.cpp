/*
 * ReaderAXIFIFO.cpp
 *
 *  Created on: Jun 16, 2024
 *      Author: Nuno Barros
 */

#include <ReaderAXIFIFO.h>
#include <axis-fifo.h>
#include <spdlog/spdlog.h>
#include <cerrno>
#include <cib_data_fmt.h>
#include <thread>
#include <chrono>

extern "C"
{
#include <unistd.h>
#include <fcntl.h>
}

namespace cib
{

  ReaderAXIFIFO::ReaderAXIFIFO ()
      : m_is_ready(false)
        ,m_receiver_host("")
        ,m_receiver_port(0)
        ,m_axi_dev("/dev/axis_fifo_0x00000000a0000000")
        ,m_dev_fd(0)
        ,m_tot_packets_rx(0)
        ,m_tot_bytes_rx(0)
        ,m_tot_packets_sent(0)
        ,m_tot_bytes_sent(0)
  {

  }

  ReaderAXIFIFO::~ReaderAXIFIFO ()
  {

    // we may want to clean up the sockets here

  }

  void ReaderAXIFIFO::init()
  {
    int rc = 0;
    // init the AXI FIFO reading mechanism
    // -- open the FIFO
    m_dev_fd = open(m_axi_dev.c_str(),O_RDONLY | O_NONBLOCK);
    if (m_dev_fd < 0)
    {
      spdlog::error("Failed to open FIFO. Message : {0}",std::strerror(errno));
      m_is_ready = false;
    }
    // -- call a reset FIFO to clear out existing stale content
    // this should have nothing, since the stop procedure also clears out the remaining contents
    spdlog::trace("Resetting the FIFO");
    rc = ioctl(m_dev_fd,AXIS_FIFO_RESET_IP);
    if (rc)
    {
      spdlog::error("Failed to reset the FIFO");
      return;
    }

    // we are now ready to read
    m_is_ready = true;
  }

  void ReaderAXIFIFO::start_run(const uint32_t run_number)
  {
    /**
     * Method of operation:
     * 1. Establish the connection
     * 2. Initiate the FIFO reading process
     * 3. Activate the DAQ FIFO (this should not be controlled by the slow control)
     * 3. Ship when data is received
     */

    if (!m_is_ready)
    {
      spdlog::error("Failed to start acquisition because the FIFO is not initialized");
      return;
    }

      // step 1 - establish the connection
    // FIXME: Implement this

    // step 2 - start reading
    ssize_t bytes_rx = 0;
    uint32_t run_packets_rx = 0;
    int rc = 0;
    // this is for debug purposes

    while (m_take_data.load())
    {
      //bytes_rx = read(m_dev_fd,m_buffer,160);
      bytes_rx = read(m_dev_fd,&(m_eth_packet.word),16);
      if (bytes_rx > 0)
      {
        // we have data
        if (m_debug)
        {
          daq::iols_trigger_t *word;
          word = reinterpret_cast<daq::iols_trigger_t*>(m_buffer);
          spdlog::debug("RX Word : TS {0} M1 {1} M2 {2} M3 {3}",word->timestamp,word->get_pos_m1(),word->get_pos_m2(),word->get_pos_m3());
        }
        if (bytes_rx != sizeof(daq::iols_trigger_t))
        {
          spdlog::error("Mismatch in word size. Expected {0} but got {1}",sizeof(daq::iols_trigger_t));
        }

        send_data(reinterpret_cast<uint8_t*>(&m_eth_packet),sizeof(m_eth_packet));
        run_packets_rx++;
        m_tot_packets_rx++;
        m_tot_bytes_rx += bytes_rx;
      }
      else
      {
        // there is nothing. It likely timed out
        spdlog::trace("Returned {0}",bytes_rx);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    }
    spdlog::info("Run stopped. Received {0} bytes ({1} packets)",bytes_rx,run_packets_rx);
    spdlog::info("Crosschecking with FIFO reports");
    uint32_t pkts_read, bytes_read;
    rc = ioctl(m_dev_fd,AXIS_FIFO_GET_RX_PKTS_READ,&pkts_read);
    if (rc)
    {
      spdlog::error("IOCTL failure checking AXIS_FIFO_GET_RX_PKTS_READ");
    }
    rc = ioctl(m_dev_fd,AXIS_FIFO_GET_RX_BYTES_READ,&bytes_read);
    if (rc)
    {
      spdlog::error("IOCTL failure checking AXIS_FIFO_GET_RX_BYTES_READ\n");
    }
    spdlog::info("FIFO report : Received {0} bytes ({1} packets)",bytes_read,pkts_read);

  }

  void ReaderAXIFIFO::stop_run()
  {
    m_take_data.store(true);
  }

  /*
  int send_data(const ssize_t n_bytes)
  {
    // do a quick check that we actually have a socket to ship the data
    daq::iols_trigger_t word;

    static uint32_t seq_num = 0;

    if (!m_transmit_ready)
    {
      spdlog::error("Transmitter is not ready");
      // FIXME: Add a way to stop acquisition
      m_take_data.store(false);
      return;
    }
    boost::system::error_code boost_error;
    seq_num++;
    m_eth_packet.header.format_version = 0x1;
    m_eth_packet.header.sequence_id = seq_num;
    m_eth_packet.header.packet_size = sizeof(daq::iols_trigger_t) & 0xFFFF;

    try
    {
      boost::asio::write( m_receiver_socket, boost::asio::buffer( eth_buffer, n_bytes_sent), boost_error ) ;
      if ( boost_error == boost::asio::error::eof)
      {
        std::string error_message = "Socket closed: " + boost_error.message();
        SPDLOG_ERROR("BOOST ASIO Connection lost: %s\n",error_message.c_str());
        break;
      }

      if ( error )
      {
        std::string error_message = "Transmission failure: " + boost_error.message();
        SPDLOG_ERROR("BOOST non-descript error: %s\n",error_message.c_str());
        break;
      }

      m_tot_packets_sent++;
      m_tot_bytes_sent += sizeof(daq::iols_tcp_packet_t);

    }
    catch(std::exception &e)
    {
      SPDLOG_ERROR("Caught an exception: {}",e.what());
      stop_acquisition();
      return -1;
    }
    catch(...)
    {
      SPDLOG_ERROR("Caught unknown exception\n");
      stop_acquisition();
      return -1;
    }
    if (boost_error)
    {
      SPDLOG_ERROR("Failed to send data. Stopping execution.");
      stop_acquisition();
      return -1;
    }
    return 0;
  }
  */

} /* namespace cib */
