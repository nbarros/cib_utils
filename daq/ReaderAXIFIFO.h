/*
 * ReaderAXIFIFO.h
 *
 *  Created on: Jun 16, 2024
 *      Author: Nuno Barros
 *
 * Description: This class implements the readout from an AXI FIFO and consecutive data shipment towards the DAQ
 *              Since the CIB produces a slow amount of triggers (no more than 10Hz), there is no need to split
 *              the process into two separate threads. Instead, there is a single worker thread that listens to the FIFO,
 *              wraps the received word in an ethernet header and ships it to the DAQ.
 *              In fact, we could even live without the ethernet wrapper, since the way the AXI FIFO works implies
 *              that a single word is fetched and shipped. Nonetheless, for consistency, we will use the wrap, to
 *              keep a sequence number
 */

#ifndef READERAXIFIFO_H_
#define READERAXIFIFO_H_

#include <ReaderBase.h>
#include <string>
#include <atomic>
#include <cib_data_fmt.h>

namespace cib
{

  /*
   *
   */
  class ReaderAXIFIFO final : public ReaderBase
  {
  public:
    ReaderAXIFIFO ();
    virtual ~ReaderAXIFIFO ();
    ReaderAXIFIFO (const ReaderAXIFIFO &other) = delete;
    ReaderAXIFIFO (ReaderAXIFIFO &&other) = delete;
    ReaderAXIFIFO& operator= (const ReaderAXIFIFO &other) = delete;
    ReaderAXIFIFO& operator= (ReaderAXIFIFO &&other) = delete;

    int init() override;
    // function overloads that implement the specific methods
    int start_run(const uint32_t run_number) override;
    int stop_run() override;
    void readout_task() override;

private:
    void start_daq_run();
    void stop_daq_run();
    void reset_daq_fifo();

    //void send_data(size_t n_bytes);
    bool m_debug;
    bool m_is_ready;
    std::string m_receiver_host;
    unsigned int m_receiver_port;
    std::atomic<bool> m_take_data;

    // AXI FIFO stuff
    std::string m_axi_dev;
    int m_dev_fd;
    // each word has 16 bytes
    //uint8_t m_buffer[32]; // room for 1000 words
    uintptr_t m_ctl_reg_addr;
    int m_dev_mem_fd;
    // statistics counting
    // this should perhaps go to the base class
    uint32_t m_tot_packets_rx;
    uint32_t m_tot_bytes_rx;
    uint32_t m_tot_packets_sent;
    uint32_t m_tot_bytes_sent; // note that this will include the header

    daq::iols_tcp_packet_t m_eth_packet;
  };

} /* namespace cib */

#endif /* READERAXIFIFO_H_ */
