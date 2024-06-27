/*
 * ReaderDMA.h
 *
 *  Created on: Mar 25, 2024
 *      Author: Nuno Barros
 */

#ifndef READERBASE_H_
#define READERBASE_H_
#include <string>
#include <boost/lockfree/spsc_queue.hpp>
#include <boost/asio.hpp>
#include <atomic>
#include <thread>
#include <cstdint>
#include <json.hpp>
using json = nlohmann::json;
extern "C"
{
#include <dma-proxy.h>
};
#include <PacketContent.hpp>
#include <ReaderBase.h>

typedef struct message_t
{
  std::string sev;
  std::string msg;
} message_t;
typedef struct eth_packet
{
  size_t nbytes;
  size_t nentries;
  uint32_t data[0x20000]; // 512 kb
} eth_packet;

typedef struct dma_channel_t
{
  struct channel_buffer *buf_ptr;
  int fd;
  // do we really need a new thread for this?
  //pthread_t tid;
} dma_channel_t;

namespace cib
{

  /*
   *
   */
  class ReaderDMA final : public ReaderBase
  {
  public:
    ReaderDMA (const bool simulation = false);
    virtual ~ReaderDMA ();
    ReaderDMA (const ReaderDMA &other) = delete;
    ReaderDMA (ReaderDMA &&other) = delete;
    ReaderDMA& operator= (const ReaderDMA &other) = delete;
    ReaderDMA& operator= (ReaderDMA &&other) = delete;

    void set_eth_receiver(const std::string &host, const unsigned int port);
    void start_acquisition();
    void stop_acquisition();

    bool is_configured() {return m_is_configured.load();}
    bool is_running() {return m_is_running.load();}

    /**
     * DMA related methods
     */
  protected:
    /** Data collector function into the queue. Runs on it's own thread**/
    void dma_terminate();
    void dma_init();
    void dma_setup();

    /** Data transmitter through internet**/
    // this can actually call two different methods, depending if local_storage is set
    void transmitter_init();
    void eth_data_transmitter();

    void reset_buffers();


  private:
    // the real worker threads
    void clear_threads();
    void dma_data_collector();
    void sim_data_collector();

  private:
    bool m_simulation;

    std::thread     *m_client_thread_collector;
    std::thread     *m_client_thread_transmitter;

    bool m_local_storage;
    std::atomic<bool> m_dma_acquire;
    std::atomic<bool> m_eth_transmit;

    bool m_ready;
    std::atomic<bool> m_is_running;
    std::atomic<bool> m_is_configured;

    uint32_t m_num_transfers;
    std::atomic<unsigned int> m_sent_bytes;
    std::atomic<unsigned int> m_sent_packets;

    std::atomic<bool> m_error_state;
    //
    // -- eth stuff
    //

    boost::asio::ip::tcp::endpoint m_receiver_endpoint;
    boost::asio::io_service        m_receiver_ios;
    boost::asio::ip::tcp::socket  m_receiver_socket;
    uint16_t m_receiver_port;
    std::string m_receiver_host;
    std::chrono::microseconds m_receiver_timeout;
    bool m_receiver_init;

    //
    // -- DMA stuff
    //
    const char m_channel_name[32] = "dma_proxy_rx";
    static const size_t num_dma_bufs_ = 4096;       // number of allocated buffers for the DMA
    static const size_t buf_size_     = 4096;       // size of each individual buffer (1 memory page) in bytes
    uintptr_t buff_addr_[num_dma_bufs_];            // static array of


    static const size_t m_sim_buff_size = 4096;
    uint8_t m_sim_buffer[m_sim_buff_size];
    size_t m_sim_cur_ptr;

    // the queue is not for true memory buffers, but for memory addresses that are owned by the kernel space
    boost::lockfree::spsc_queue<message_t, boost::lockfree::capacity<1024> > m_message_queue;

    boost::lockfree::spsc_queue<dunedaq::cibmodules::content::buffer_t, boost::lockfree::capacity<num_dma_bufs_> > buffer_queue_;
    dma_channel_t dma_channel_;
  };

} /* namespace cib */

#endif /* READERBASE_H_ */
