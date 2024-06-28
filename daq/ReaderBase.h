/*
 * CIBReader.h
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

#include <PacketContent.hpp>
#include <cib_data_fmt.h>


namespace cib
{

  /*
   * This class only sets up the transmission part of the setup
   * And names high level functions for collection
   */
  class ReaderBase
  {
  public:
    enum State {kInit,kSet, kReady,kRunning};

    ReaderBase (const bool simulation = false);
    virtual ~ReaderBase ();
    ReaderBase (const ReaderBase &other) = delete;
    ReaderBase (ReaderBase &&other) = delete;
    ReaderBase& operator= (const ReaderBase &other) = delete;
    ReaderBase& operator= (ReaderBase &&other) = delete;

    int set_eth_receiver(const std::string &host, const unsigned int port);

    /**
     * Init is responsible for intiializing everything after the host is set
     */
    virtual int init();
    virtual int start_run(const uint32_t run_number);
    virtual int stop_run();


    bool is_configured() {return ((m_state == kSet) || (m_state == kReady));}
    bool is_running() {return (m_state == kRunning);}

    void get_feedback(std::vector<daq::iols_feedback_msg_t> &msgs);

    /**
     * DMA related methods
     */
  protected:

    /** Data transmitter through internet**/
    virtual void init_transmitter();
    virtual void term_transmitter();

    virtual void reset_buffers() {}
    int send_data(uint8_t *data, size_t n_bytes);


//  private:
//    // the real worker threads
//    virtual void clear_threads();
//    virtual void data_collector();


  protected:

    State m_state;

    bool m_simulation;

    bool m_ready;
    std::atomic<bool> m_run_enable;
//    std::atomic<bool> m_is_configured;

    uint32_t m_num_transfers;
    std::atomic<unsigned int> m_sent_bytes;
    std::atomic<unsigned int> m_sent_packets;

    unsigned int m_tot_packets_sent;
    unsigned int m_tot_bytes_sent;




    std::atomic<bool> m_error_state;
    boost::lockfree::spsc_queue<daq::iols_feedback_msg_t, boost::lockfree::capacity<1024> > m_message_queue;

    //
    // -- eth stuff
    //

    boost::asio::ip::tcp::endpoint  m_receiver_endpoint;
    boost::asio::io_service         m_receiver_ios;
    boost::asio::ip::tcp::socket    m_receiver_socket;
    uint16_t                        m_receiver_port;
    std::string                     m_receiver_host;
    std::chrono::microseconds       m_receiver_timeout;
    bool m_receiver_init;

  };

} /* namespace cib */

#endif /* READERBASE_H_ */
