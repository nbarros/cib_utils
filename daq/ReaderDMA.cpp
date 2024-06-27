/*
 * ReaderDMA.cpp
 *
 *  Created on: Mar 25, 2024
 *      Author: Nuno Barros
 *
 *      Description: DMA implementation of the CIB reader
 */

#include <ReaderDMA.h>
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

  ReaderDMA::ReaderDMA (const bool simulation)
                  :m_simulation(simulation)
                   , m_client_thread_collector(nullptr)
                   , m_client_thread_transmitter(nullptr)
                   , m_local_storage(false)
                   , m_dma_acquire(false)
                   , m_eth_transmit(false)
                   , m_ready(false)
                   ,m_is_running(false)
                   ,m_is_configured(false)
                   ,m_num_transfers(0)
                   ,m_sent_bytes(0)
                   ,m_sent_packets(0)
                   ,m_error_state(false)
                   ,m_receiver_port(0)
                   ,m_receiver_host("")
                   ,m_receiver_timeout(5000) // 5 ms
                   ,m_receiver_init(false)
                   , m_receiver_ios()
                   ,m_receiver_socket(m_receiver_ios)
                   ,m_sim_cur_ptr(0)
                   {

    if (!m_simulation)
    {
      // inititalize the DMA engine
      dma_setup();
    }
                   }

  ReaderDMA::~ReaderDMA ()
  {
    // stop acquisition, if anything happening
    // no harm in making sure that everything is down
    stop_acquisition();
    // there isn't anything to be destructed
    if (!m_simulation)
    {
      dma_terminate();
    }
    SPDLOG_TRACE("Reader leaving destructor.");
  }

  void ReaderDMA::transmitter_init()
  {
    if (!m_is_configured)
    {
      SPDLOG_WARN("Initiating connection with default settings");
    }
    SPDLOG_INFO("Initiating ethernet connection : {}:{}",m_receiver_host.c_str(),m_receiver_port);
    // this is a client connection.
    boost::asio::ip::tcp::resolver resolver( m_receiver_ios );
    boost::asio::ip::tcp::resolver::query query(m_receiver_host, std::to_string(m_receiver_port) ) ; //"np04-ctb-1", 8991
    boost::asio::ip::tcp::resolver::iterator iter = resolver.resolve(query) ;
    m_receiver_endpoint = iter->endpoint();
    m_receiver_socket.connect( m_receiver_endpoint );
    m_receiver_init = true;
  }

  // this is not really an init. This is more a setup
  void ReaderDMA::dma_setup()
  {
    std::string dev_name = "/dev/";
    dev_name += m_channel_name;
    dma_channel_.fd = open(dev_name.c_str(), O_RDWR);
    SPDLOG_DEBUG("Received handle [%d]\n",dma_channel_.fd);
    if (dma_channel_.fd < 1)
    {
      SPDLOG_CRITICAL("Unable to open DMA proxy device file: %s", m_channel_name);
    }
    // now memory map the channels
    dma_channel_.buf_ptr = (struct channel_buffer *)mmap(NULL, sizeof(struct channel_buffer) * num_dma_bufs_,
                                                         PROT_READ | PROT_WRITE,
                                                         MAP_SHARED,
                                                         dma_channel_.fd,
                                                         0);
    if (dma_channel_.buf_ptr == MAP_FAILED)
    {
      close(dma_channel_.fd);
      // critical terminates execution
      SPDLOG_CRITICAL("Failed to mmap rx channel\n");
    } else
    {
      SPDLOG_DEBUG("mmap of rx channel successful\n");
    }
    SPDLOG_INFO("DMA engine initialized");
  }

  void ReaderDMA::dma_terminate()
  {
    // this is just the cleaning of the memory
    // in principle we could do this at the destructor level
    munmap(dma_channel_.buf_ptr , sizeof(struct channel_buffer));
    close(dma_channel_.fd);
  }

  void ReaderDMA::start_acquisition()
  {
    // for the DMA to operate we just need it to be set up
    // however, that is done at constructor time
    // if we reached this point, setup is done
    if (m_is_configured)
    {
      if (m_is_running.load())
      {
        SPDLOG_ERROR("Acquisition is already running. Cancelling any action.");
        return;
      }
      SPDLOG_TRACE("Starting acquisition threads.");
      m_dma_acquire.store(true,std::memory_order_relaxed);

      if (m_simulation)
      {
        m_client_thread_collector = new std::thread(&ReaderDMA::sim_data_collector,this);
      }
      else
      {
        m_client_thread_collector = new std::thread(&ReaderDMA::dma_data_collector,this);
      }
      if (m_client_thread_collector == nullptr)
      {
        SPDLOG_ERROR("Unable to create collector thread . Failing..." );
      }
      SPDLOG_DEBUG("==> Creating transmitter\n" );
      m_eth_transmit.store(true,std::memory_order_relaxed);
      m_client_thread_transmitter = new std::thread(&ReaderDMA::eth_data_transmitter,this);
      if (m_client_thread_transmitter == nullptr)
      {
        SPDLOG_ERROR("Unable to create transmitter thread . Failing...\n" );
      }
    }
    else
    {
      SPDLOG_ERROR("Engine not ready for data collection.\n");
    }
    m_is_running.store(true);
  }

  void ReaderDMA::dma_init()
  {
    // this is responsible for setting up the buffers for the transfers
  }

  void ReaderDMA::dma_data_collector()
  {
    static size_t len;
    static dunedaq::cibmodules::content::buffer_t dma_buffer;
    size_t counter = 0;
    int buffer_id= 0;
    // initialize the engine
    dma_init();

    if (!m_dma_acquire.load(std::memory_order_acquire))
    {
      SPDLOG_ERROR("Acquisition controller variable not set.");
      return;
    }
    //
    // -- first set up the buffers and submit them for transfers
    //
    // on the first run start by initiating transfers for all buffers
    for (buffer_id = 0; buffer_id < num_dma_bufs_; buffer_id++)
    {
      SPDLOG_TRACE("Setting buffer %d",buffer_id);
      dma_channel_.buf_ptr[buffer_id].length = buf_size_;
      ioctl(dma_channel_.fd, START_XFER, &buffer_id);
    }

    buffer_id = 0;

    while(m_dma_acquire.load(std::memory_order_acquire))
    {
      if (!m_dma_acquire.load(std::memory_order_acquire))
      {
        SPDLOG_INFO("Received signal to stop acquiring data. Cleaning out...\n");
        break;
      }

      // attempt to complete this transfer
      ioctl(dma_channel_.fd, FINISH_XFER, &buffer_id);

      if (dma_channel_.buf_ptr[buffer_id].status != channel_buffer::PROXY_NO_ERROR)
      {
        // something failed...
        SPDLOG_ERROR("Proxy rx transfer error on buffer {}: status {}  # transfers completed {}\n",
                     buffer_id,(int)dma_channel_.buf_ptr[buffer_id].status, counter);
        // retry again
        continue;
      }

      // if it reached this point, it means that there is a successful transfer
      // set up the handle to point to the appropriate set of memory
      dma_buffer.handle = reinterpret_cast<uintptr_t>(&(dma_channel_.buf_ptr[buffer_id].buffer));
      // FIXME: Find a way to figure out how many packets were really operated
      dma_buffer.len = 0;

      buffer_queue_.push(dma_buffer);
      counter++;
    }
    SPDLOG_INFO("Stopping the data collection after {} acquisitions",counter);
  }

  void ReaderDMA::sim_data_collector()
  {
    // this just generates a buffer to memory that contains a singletrigger
    static dunedaq::cibmodules::content::buffer_t dma_buffer;
    dunedaq::cibmodules::content::word::trigger_t word;
    size_t counter = 0;

    while(m_dma_acquire.load(std::memory_order_acquire))
    {
      if (!m_dma_acquire.load(std::memory_order_acquire))
      {
        SPDLOG_INFO("Received signal to stop acquiring data. Cleaning out...\n");
        break;
      }

      time_stamp ts = std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::system_clock::now());
      //auto value = std::chrono::duration_cast<std::chrono::microseconds>(ts);
      auto value = ts.time_since_epoch();
      uint64_t duration = value.count();

      word.timestamp = duration;

      word.trigger_word = 0x0;
      word.word_type = dunedaq::cibmodules::content::word::t_trigger;
      std::stringstream msg;
      msg << "Word [" << std::hex
          << "0x" << word.word_type
          << "][0x"<< word.timestamp
          << "][0x"<< word.trigger_word
          << "]\n";
      SPDLOG_TRACE(msg.str().c_str());

      // copy this into the buffer
      SPDLOG_TRACE("Copying {} bytes to position {}",sizeof(word),m_sim_cur_ptr);
      std::memmove((void*)&(m_sim_buffer[m_sim_cur_ptr]),&word,sizeof(word));

      dma_buffer.handle = reinterpret_cast<uintptr_t>(&(m_sim_buffer[m_sim_cur_ptr]));
      SPDLOG_TRACE("Setting handle to {}",dma_buffer.handle);

      if ((m_sim_cur_ptr+sizeof(word) > (m_sim_buff_size-1)))
      {
        // move pointer to the beggining again
        SPDLOG_TRACE("Relocating word pointer to the beginning of the buffer");
        m_sim_cur_ptr = 0;
      }
      else
      {
        m_sim_cur_ptr+= sizeof(word);
      }
      dma_buffer.len = 0;

      buffer_queue_.push(dma_buffer);
      counter++;
      // sleep for 100 ms, since the trigger rate is **at most** 10 Hz
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    SPDLOG_INFO("Stopping the data collection after {} acquisitions",counter);
  }

  void ReaderDMA::reset_buffers()
  {
    SPDLOG_DEBUG("Resetting the software buffers.\n");
    uint32_t counter = 0;

    while(buffer_queue_.pop())
    {
      counter++;
    }
    SPDLOG_DEBUG("Popped {} entries from the buffer queue.\n",counter);
  }

  void ReaderDMA::eth_data_transmitter()
  {
    static uint32_t eth_buffer_size_u32 = 0x20000;
    static uint32_t global_eth_buffer[0x20000]; // 512 kB in 4 byte words
    static eth_packet debug_eth_buffer;
    bool shutdown_connection = false;
    // Local pointer that is effectively used to build the eth packet
    // It is just an auxiliary moving pointer
    uint32_t *eth_buffer = nullptr;
    // pointer that keeps track of the offset within the global buffer
    uint32_t global_eth_pos = 0;

    // just some counters
    static uint32_t n_u32_words = 0;
    static uint32_t n_bytes_sent = 0;
    static uint32_t seq_num_ = 1;
    // pointer to a new word
    dunedaq::cibmodules::content::word::word_t data_word;
    dunedaq::cibmodules::content::buffer_t data_buffer;
    try
    {
      transmitter_init();
    }
    catch(std::exception &e)
    {
      SPDLOG_ERROR("Caught an exception: {}",e.what());
      // we can't just stop the acquisition
      // disable the control variables
      m_eth_transmit.store(false);
      m_dma_acquire.store(false);
    }
    catch(...)
    {
      SPDLOG_ERROR("Caught unknown exception\n");
      m_eth_transmit.store(false);
      m_dma_acquire.store(false);
    }
    boost::system::error_code error;
    dunedaq::cibmodules::content::tcp_header eth_header;
    eth_header.word.format_version = (0x1U << 4) | ((~0x1U) & 0xF);
    while(m_eth_transmit.load(std::memory_order_acquire))
    {
      eth_buffer = &global_eth_buffer[global_eth_pos];
      eth_header.word.sequence_id = seq_num_;
      bool got_data = false;
      // -- Pop a buffer
      // if there is none, sleep and wait for one to be available
      // grab all the buffers until a timestamp word is generated
      if (!buffer_queue_.pop( data_buffer))
      {
        // -- should some sort of wait be put here?
        // Might hurt since it will require some sort of mutex
        std::this_thread::sleep_for(std::chrono::microseconds(1000));
        continue;
      }
      // -- at this point there is a buffer available
      // Assign the size to the header
      eth_header.word.packet_size = sizeof(data_word) & 0xFFFF;
      std::memmove((void*)&(eth_buffer[0]),&eth_header,sizeof(eth_header));
      // -- copy the whole buffer
      std::memmove(&(eth_buffer[1]),(void*)data_buffer.handle,sizeof(data_word));
      // send the data
      try
      {
        n_bytes_sent = sizeof(eth_header) + sizeof(data_word);
        n_u32_words = n_bytes_sent/sizeof(uint32_t);
        boost::asio::write( m_receiver_socket, boost::asio::buffer( eth_buffer, n_bytes_sent), error ) ;
        if ( error == boost::asio::error::eof)
        {
          std::string error_message = "Socket closed: " + error.message();
          SPDLOG_ERROR("BOOST ASIO Connection lost: %s\n",error_message.c_str());
          break;
        }

        if ( error )
        {
          std::string error_message = "Transmission failure: " + error.message();
          SPDLOG_ERROR("BOOST non-descript error: %s\n",error_message.c_str());
          break;
        }
        // move the global pointer forwards
        global_eth_pos += (n_u32_words+4);
        if ((global_eth_pos+0x400) > eth_buffer_size_u32)
        {
        // reset the pointer to the beginning
          global_eth_pos = 0;
        }
      }
      catch(std::exception &e)
      {
        SPDLOG_ERROR("Caught an exception: {}",e.what());
        stop_acquisition();
      }
      catch(...)
      {
        SPDLOG_ERROR("Caught unknown exception\n");
        stop_acquisition();
      }
    }

    // close the data connection
    boost::system::error_code closing_error;

    if ( shutdown_connection)
    {
      // we had a critical issue on our part.
      // forcefully terminate the connection from our end
      m_receiver_socket.shutdown(boost::asio::ip::tcp::socket::shutdown_send, closing_error);

      if ( closing_error )
      {
        std::stringstream msg;
        msg << "Error in shutdown " << closing_error.message();
        SPDLOG_ERROR("Error shutting down connection : {}",msg.str());
      }
    }
    // close the socket
    m_receiver_socket.close(closing_error) ;

    if ( closing_error )
    {
      std::stringstream msg;
      msg << "Socket closing failed:: " << closing_error.message();
      SPDLOG_ERROR("Error closing socket : {}",msg.str());
    }
    SPDLOG_INFO("Leaving the control listener");
  }

  void ReaderDMA::set_eth_receiver(const std::string &host, const unsigned int port)
  {
    if (m_is_configured)
    {
      SPDLOG_WARN("Overriding receiver data after initial configuration");
    }
    // if we are taking data, fail
    if (m_is_running.load())
    {
      SPDLOG_ERROR("Trying to change connection settings while running. Doing nothing.");
      return;
    }
    SPDLOG_DEBUG("Setting receiver data to {}:{}",host.c_str(),port);
    m_receiver_host = host;
    m_receiver_port = port;
    m_is_configured.store(true);
  }

  void ReaderDMA::clear_threads()
  {
    // First stop the loops
    m_dma_acquire.store(false,std::memory_order_relaxed);
    std::this_thread::sleep_for (std::chrono::milliseconds(100));
    // Kill the collector thread first
    // Ideally we would prefer to join the thread
    if (m_client_thread_collector != nullptr)
    {
      m_client_thread_collector->join();
      delete m_client_thread_collector;
      m_client_thread_collector = nullptr;
    }
    // Kill the transmitter thread.

    // Occasionally there seems to be memory corruption between these two steps
    m_eth_transmit.store(false,std::memory_order_relaxed);
    std::this_thread::sleep_for (std::chrono::milliseconds(200));
    // -- Apparently this is a bit of a problem since the transmitting thread never leaves cleanly
    if (m_client_thread_transmitter != nullptr)
    {
      m_client_thread_transmitter->join();
      delete m_client_thread_transmitter;
      m_client_thread_transmitter = nullptr;
    }
  }

  void ReaderDMA::stop_acquisition()
  {
    SPDLOG_INFO("Stopping acquisition.");
    SPDLOG_DEBUG("Clearning threads.");
    clear_threads();
    SPDLOG_DEBUG("Resetting buffers.");
    reset_buffers();
    m_is_running.store(false);
  }

} /* namespace cib */
