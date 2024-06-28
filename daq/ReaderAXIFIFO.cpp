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
#include <cib_mem.h>

extern "C"
{
#include <unistd.h>
#include <fcntl.h>
}

namespace cib
{

  ReaderAXIFIFO::ReaderAXIFIFO ()
      : m_debug(true)
        ,m_is_ready(false)
        ,m_receiver_host("")
        ,m_receiver_port(0)
        ,m_axi_dev("/dev/axis_fifo_0x00000000a0000000")
        ,m_dev_fd(0)
        ,m_ctl_reg_addr(0)
        ,m_dev_mem_fd(0)
        ,m_tot_packets_rx(0)
        ,m_tot_bytes_rx(0)
        ,m_tot_packets_sent(0)
        ,m_tot_bytes_sent(0)

  {

    // map the control register memmory address
    SPDLOG_INFO("Mapping DAQ control register at 0x{0:X}",GPIO_MISC_MEM_LOW);
    m_ctl_reg_addr = cib::util::map_phys_mem(m_dev_mem_fd,GPIO_MISC_MEM_LOW,GPIO_MISC_MEM_LOW+0x1000);
    if (m_ctl_reg_addr)
    {
      SPDLOG_DEBUG("Mapped address : 0x{0:X}",m_ctl_reg_addr);
    }
    else
    {
      SPDLOG_ERROR("Failed to map memory. Thi will fail somewhere.");
    }
  }

  ReaderAXIFIFO::~ReaderAXIFIFO ()
  {

    SPDLOG_TRACE("Passing destructor.");
    if (m_ctl_reg_addr)
    {
      SPDLOG_DEBUG("Unmapping the MM register.");
      cib::util::unmap_mem(m_ctl_reg_addr,0x1000);
      close(m_dev_mem_fd);
    }
  }

  void ReaderAXIFIFO::reset_daq_fifo()
  {
    if (!m_ctl_reg_addr)
    {
      // the memory mapped register is not mapped. Do nothing
      SPDLOG_ERROR("Memory not mapped. Doing nothing.");
      add_feedback("ERROR","Control register not mapped. Doing nothing.");
      return;
    }
    // this resets the FIFO
    const uint32_t bit = 25;
    uint32_t mask = cib::util::bitmask(bit,bit);
    cib::util::reg_write_mask_offset(m_ctl_reg_addr,0x1,mask,bit);
    std::this_thread::sleep_for(std::chrono::microseconds(10));
    cib::util::reg_write_mask_offset(m_ctl_reg_addr,0x0,mask,bit);
  }

  void ReaderAXIFIFO::start_daq_run()
  {
    if (!m_ctl_reg_addr)
    {
      // the memory mapped register is not mapped. Do nothing
      SPDLOG_ERROR("Memory not mapped. Doing nothing.");
      add_feedback("ERROR","Control register not mapped. Doing nothing.");
      return;
    }
    const uint32_t bit = 26;

    uint32_t mask = cib::util::bitmask(bit,bit);
    cib::util::reg_write_mask_offset(m_ctl_reg_addr,0x1,mask,bit);
  }

  void ReaderAXIFIFO::stop_daq_run()
  {
    if (!m_ctl_reg_addr)
    {
      // the memory mapped register is not mapped. Do nothing
      SPDLOG_ERROR("Memory not mapped. Doing nothing.");
      add_feedback("ERROR","Control register not mapped. Doing nothing.");
      return;
    }
    const uint32_t bit = 26;
    uint32_t mask = cib::util::bitmask(bit,bit);
    cib::util::reg_write_mask_offset(m_ctl_reg_addr,0x0,mask,bit);
  }



  int ReaderAXIFIFO::init()
  {
    int rc = 0;
    // init the AXI FIFO reading mechanism
    // -- open the FIFO
    m_dev_fd = open(m_axi_dev.c_str(),O_RDONLY | O_NONBLOCK);
    if (m_dev_fd < 0)
    {
      std::ostringstream msg;
      msg << "Failed to open FIFO. Message : " << std::strerror(errno);
      SPDLOG_ERROR("Failed to open FIFO. Message : {0}",std::strerror(errno));
      add_feedback("ERROR",msg.str());
      m_is_ready = false;
    }
    // -- call a reset FIFO to clear out existing stale content
    // this should have nothing, since the stop procedure also clears out the remaining contents
    SPDLOG_TRACE("Resetting the FIFO");
    rc = ioctl(m_dev_fd,AXIS_FIFO_RESET_IP);
    if (rc)
    {
      SPDLOG_ERROR("Failed to reset the FIFO");
      add_feedback("WARN","Failed to reset the DAQ FIFO");
      return 1;
    }

    // we are now ready to read
    m_is_ready = true;
    return 0;
  }

  void ReaderAXIFIFO::readout_task()
  {
    ssize_t bytes_rx = 0;
    uint32_t run_packets_rx = 0;
    int rc = 0;
    // this is for debug purposes
    SPDLOG_TRACE("Entering readout loop");

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
          word = &(m_eth_packet.word); //reinterpret_cast<daq::iols_trigger_t*>(m_buffer);
          SPDLOG_DEBUG("RX Word : TS {0} M1 {1} M2 {2} M3 {3}",word->timestamp,word->get_pos_m1(),word->get_pos_m2(),word->get_pos_m3());
        }
        if (bytes_rx != sizeof(daq::iols_trigger_t))
        {
          SPDLOG_ERROR("Mismatch in word size. Expected {0} but got {1}",sizeof(daq::iols_trigger_t));
        }

        send_data(reinterpret_cast<uint8_t*>(&m_eth_packet),sizeof(m_eth_packet));
        run_packets_rx++;
        m_tot_packets_rx++;
        m_tot_bytes_rx += bytes_rx;
      }
      else
      {
        // there is nothing. It likely timed out
        //SPDLOG_TRACE("Returned {0}",bytes_rx);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    }
    SPDLOG_INFO("Run stopped. Received {0} bytes ({1} packets)",bytes_rx,run_packets_rx);
    SPDLOG_INFO("Crosschecking with FIFO reports");
    uint32_t pkts_read, bytes_read;
    rc = ioctl(m_dev_fd,AXIS_FIFO_GET_RX_PKTS_READ,&pkts_read);
    if (rc)
    {
      SPDLOG_ERROR("IOCTL failure checking AXIS_FIFO_GET_RX_PKTS_READ");
    }
    rc = ioctl(m_dev_fd,AXIS_FIFO_GET_RX_BYTES_READ,&bytes_read);
    if (rc)
    {
      SPDLOG_ERROR("IOCTL failure checking AXIS_FIFO_GET_RX_BYTES_READ\n");
    }
    SPDLOG_INFO("FIFO report : Received {0} bytes ({1} packets)",bytes_read,pkts_read);

  }

  int ReaderAXIFIFO::start_run(const uint32_t run_number)
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
      SPDLOG_ERROR("Failed to start acquisition because the FIFO is not initialized");
      add_feedback("ERROR","FIFO is not initialized.");
      return 1;
    }

    // step 1 - establish the connection to the receiver
    init_transmitter();
    // reset the DAQ FIFO
    reset_daq_fifo();
    SPDLOG_TRACE("FIFO reset");

    // set the data taking flag
    SPDLOG_TRACE("Preparing to start taking data");
    m_take_data.store(true);

    // step 2 - start reading
    m_readout_thread = std::unique_ptr<std::thread>(new std::thread(&ReaderAXIFIFO::readout_task,this));
    if (m_readout_thread.get() == nullptr)
    {
      SPDLOG_ERROR("Failed to launch readout thread.");
      add_feedback("ERROR","Failed to launch readout thread");
      return 1;
    }
    // step 3 - now activate the DAQ fifo
    start_daq_run();

    m_state = kRunning;
    return 0;
  }

  int ReaderAXIFIFO::stop_run()
  {
    //
    if (m_state != kRunning)
    {
      SPDLOG_ERROR("Failed to stop acquisition because we are not running");
      add_feedback("ERROR","We are not in a running state");
      return 1;
    }
    // tell the DAQ to stop taking data
    stop_daq_run();

    //

    m_take_data.store(false);
    // wait for a sec to allow the data taking to stop
    std::this_thread::sleep_for(std::chrono::milliseconds(1));

    // join the readout thread
    if (m_readout_thread.get()->joinable())
    {
      SPDLOG_TRACE("Thread is joinable. Joining it.");
      m_readout_thread.get()->join();
      SPDLOG_TRACE("Thread joined.");
    }
    else
    {
      SPDLOG_WARN("Thread is not joinable. Forcing our way out of it.");
      // this should not happen.
      // It will certainly lead to dirty trail of remains
      //m_control_thread.get_deleter().default_delete();
      //SPDLOG_TRACE("Thread is now dead.");
      m_readout_thread = nullptr;
    }

    // terminate the transmission socket
    term_transmitter();

    return 0;
  }

  /*
  int send_data(const ssize_t n_bytes)
  {
    // do a quick check that we actually have a socket to ship the data
    daq::iols_trigger_t word;

    static uint32_t seq_num = 0;

    if (!m_transmit_ready)
    {
      SPDLOG_ERROR("Transmitter is not ready");
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
