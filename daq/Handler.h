/*
 * Handler.h
 *
 *  Created on: Mar 30, 2024
 *      Author: Nuno Barros
 */

#ifndef HANDLER_H_
#define HANDLER_H_

#include <boost/asio.hpp>
#include <boost/lockfree/spsc_queue.hpp>

#include <json.hpp>

namespace cib
{

  class ReaderBase;
  /*
   *
   */
  class Handler final
  {
  public:
    Handler (const bool simulation = false);
    virtual ~Handler ();
    Handler (const Handler &other) = delete;
    Handler (Handler &&other) = delete;
    Handler& operator= (const Handler &other) = delete;
    Handler& operator= (Handler &&other) = delete;

    void init_listener();
    void stop_listener();

    ReaderBase *get_reader() {return m_reader;}
  protected:

    void listen_task();
    void process_request();

    int config(nlohmann::json &conf, nlohmann::json &resp);
    int start_run(nlohmann::json &resp, const uint32_t run_number = 0);
    int stop_run(nlohmann::json &resp);
    void get_status(nlohmann::json &resp) { }
    void add_feedback(nlohmann::json &resp, const std::string type, const std::string msg);

private:
    bool    m_simulation;
    ReaderBase *m_reader;

    // listener thread
    std::atomic<bool> m_stop_running;
    std::atomic<bool> m_is_running;
    std::atomic<bool> m_is_listening;
    std::unique_ptr<std::thread> m_control_thread;

    // communication stuff
    boost::asio::io_service       m_control_ios;
    boost::asio::ip::tcp::socket  m_control_socket;
    std::chrono::microseconds     m_control_timeout;
    unsigned int                  m_control_port;
    bool                          m_control_init;
    std::atomic<bool>             m_control_error;
  };

} /* namespace cib */

#endif /* HANDLER_H_ */
