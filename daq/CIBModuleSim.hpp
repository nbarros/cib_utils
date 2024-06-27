/*
 * CIBModuleSim.h
 *
 *  Created on: Mar 31, 2024
 *      Author: Nuno Barros
 */

#ifndef CIBMODULESIM_HPP_
#define CIBMODULESIM_HPP_

#include <memory>
#include <string>
#include <vector>
#include <fstream>
#include <shared_mutex>
#include <thread>

#include <boost/asio.hpp>
#include <boost/array.hpp>

#include <atomic>
#include <limits>
#include <string>
#include <deque>

#include "PacketContent.hpp"
#include <WorkerThread.hpp>
#include <json.hpp>

using json=nlohmann::json;

namespace dunedaq::cibmodules
{

  /*
   *
   */
  class CIBModuleSim final
  {
  public:
    explicit CIBModuleSim ();
    void init(const json& iniobj);
    virtual ~CIBModuleSim ();
    CIBModuleSim (const CIBModuleSim &other) = delete;
    CIBModuleSim (CIBModuleSim &&other) = delete;
    CIBModuleSim& operator= (const CIBModuleSim &other) = delete;
    CIBModuleSim& operator= (CIBModuleSim &&other) = delete;

    bool ErrorState() const { return m_error_state.load() ; }

    // Commands
    void do_configure(const nlohmann::json& obj);
    void do_start(const nlohmann::json& startobj);
    void do_stop(const nlohmann::json& obj);

  private:
    // control variables

    std::atomic<bool> m_is_running;
    std::atomic<bool> m_is_configured;

    /*const */unsigned int m_receiver_port;
    std::chrono::microseconds m_receiver_timeout;
    std::chrono::microseconds m_timeout;



    std::atomic<bool> m_error_state;

    boost::asio::io_service m_control_ios;
    boost::asio::io_service m_receiver_ios;
    boost::asio::ip::tcp::socket m_control_socket;
    boost::asio::ip::tcp::socket m_receiver_socket;
    boost::asio::ip::tcp::endpoint m_endpoint;


    // NFB: what's this for?
    void do_scrap(const nlohmann::json& /*obj*/) { }

    // the CIB does not need reset, since the DAQ operation is
    // decoupled from the instrumentation operation
    void send_reset() ;
    void send_config(const std::string & config);
    bool send_message(const std::string & msg);

    // Configuration
    json m_cfg;
    std::atomic<unsigned int> m_run_number;

    // Threading
    cib::utilities::WorkerThread  m_thread_;
    void receive(std::atomic<bool>&);

    template<typename T>
    bool read(T &obj);

    //
    //
    // members related to calibration stream
    //
    // this is a standalone output parallel to the DAQ
    void update_calibration_file();
    void init_calibration_file();
    bool set_calibration_stream( const std::string &prefix = "" );

    bool m_calibration_stream_enable = false;
    std::string m_calibration_dir = "";
    std::string m_calibration_prefix = "";
    std::chrono::minutes m_calibration_file_interval;
    std::ofstream m_calibration_file;
    std::chrono::steady_clock::time_point m_last_calibration_file_update;

    // counters per run
    std::atomic<unsigned long> m_run_gool_part_counter;
    std::atomic<unsigned long> m_run_packet_counter;
    std::atomic<unsigned long> m_run_trigger_counter;
    std::atomic<unsigned long> m_run_timestamp_counter;

    // overall counters
    std::atomic<unsigned int> m_num_TS_words;
    std::atomic<unsigned int> m_num_TR_words;


    //
    //
    // monitoring data/information
    //
    //
    std::deque<uint> m_buffer_counts; // NOLINT(build/unsigned)
    //std::shared_mutex m_buffer_counts_mutex;
    void update_buffer_counts(uint new_count); // NOLINT(build/unsigned)
    double read_average_buffer_counts();

    std::atomic<int>      m_num_control_messages_sent;
    std::atomic<int>      m_num_control_responses_received;
    std::atomic<uint64_t> m_last_readout_timestamp; // NOLINT(build/unsigned)


  };

} /* namespace cib */

#endif /* CIBMODULESIM_HPP_ */
