/*
 * Handler.cpp
 *
 *  Created on: Mar 30, 2024
 *      Author: Nuno Barros
 */

#include <Handler.h>
#include <spdlog/spdlog.h>
#include <string>
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <json.hpp>
#include <ReaderAXIFIFO.h>
#include <memory>
#include <sstream>

namespace cib
{

  Handler::Handler (const bool simulation)
                              : m_simulation(simulation)
                                ,m_reader(nullptr)
                                ,m_control_port(8992)
                                ,m_control_timeout(10000) // microseconds = 10 ms
                                ,m_control_init(false)
                                ,m_control_error(false)
                                ,m_control_ios()
                                ,m_control_socket(m_control_ios)
                                ,m_is_running(false)
                                ,m_stop_running(false)
                                ,m_is_listening(false)
                                ,m_control_thread(nullptr)
                                {

    if (m_simulation)
    {
      SPDLOG_INFO("CIB DAQ Handler created in SIMULATION mode.");
      m_reader = new ReaderBase(simulation);
    }
    else
    {
      SPDLOG_INFO("CIB DAQ Handler created in REAL mode.");
      m_reader = new ReaderAXIFIFO();
    }
  }

  Handler::~Handler ()
  {
    SPDLOG_INFO("Cleaning  the reader.");
    if (m_reader) delete m_reader;

    SPDLOG_TRACE("Reader destroyed.");
    // seems to crash at this point

    SPDLOG_TRACE("Cleaning any lingering threads.");
    if (m_is_running.load())
    {
      stop_listener();
    }
    SPDLOG_TRACE("All destroyed in handler.");
  }


  void Handler::init_listener()
  {
    // there is no reader to process
    // At this stage there should be no reader available.
    // the reader is created at "config"
    if (m_is_running.load())
    {
      SPDLOG_ERROR("Listener already running. Doing nothing.");
      return;
    }
    //FIXME: Do we need a listener thread? We should be able to live with just the data taking one
    m_control_thread = std::unique_ptr<std::thread>(new std::thread(&Handler::listen_task,this));
    if (m_control_thread.get() == nullptr)
    {
      SPDLOG_ERROR("Failed to launch control thread.");
      m_is_running.store(false);
    }
    m_is_running.store(true);

  }

  void Handler::stop_listener()
  {
    // FIXME: If I call the listener to stop before the run is over. Assume  yes.
    if (m_is_running.load())
    {
      m_stop_running.store(true);
      // hold on for a short period to give the listener thread time to
      // terminate
      std::this_thread::sleep_for(2*m_control_timeout);
      // join won't work since the listener is waiting
      SPDLOG_DEBUG("Terminating the listener thread.");
      if (m_control_thread.get()->joinable())
      {
        SPDLOG_TRACE("Thread is joinable. Joining it.");
        m_control_thread.get()->join();
        SPDLOG_TRACE("Thread joined.");
      }
      else
      {
        SPDLOG_WARN("Thread is not joinable. Forcing our way out of it.");
        // this should not happen.
        // It will certainly lead to dirty trail of remains
        //m_control_thread.get_deleter().default_delete();
        //SPDLOG_TRACE("Thread is now dead.");
        m_control_thread = nullptr;
      }
    }
    m_is_running.store(false);
  }

  // this is the main worker
  // it runs on its own separate thread to allow for everything else to
  // operate asynchronously
  void Handler::listen_task()
  {
    // init the socket first
    // basically sets up the socket for accepting connections
    // but it does not intiitalize the listener
    boost::asio::ip::tcp::acceptor acceptor(m_control_ios,
                                            boost::asio::ip::tcp::endpoint( boost::asio::ip::tcp::v4(),
                                                                            m_control_port ) );

    // loop to keep reaccepting connections
    while (!m_stop_running.load())
    {
      SPDLOG_INFO("Initiating the control listener");
      int ret = 0;
      // open the socket for listening

      /**
       * This was actually causing an issue when joining the thread (it would hang)
       * See https://stackoverflow.com/questions/33161640/how-to-safely-cancel-a-boost-asio-asynchronous-accept-operation
       *
       * Since the socket is listening asynchronously, the acceptor would go into an undefined state
       * when terminating the connection. Basically the listener would detach itself and fail
       * to properly clean.
       *
       * The trick is to consume the acceptor by opening a connection that is then shut down
       */
      std::future<void> accepting = async( std::launch::async, [&]{ acceptor.accept(m_control_socket) ; } ) ;
      // the following loop is locked until either a connection is established
      // or run is stopped
      while (!m_stop_running.load())
      {
        // wait until the async tells that a connection arrived
        if ( accepting.wait_for( m_control_timeout ) == std::future_status::ready )
        {
          SPDLOG_INFO("Connection request arrived.");
          break ;
        }
      }
      // if we arrive at this stage we either received a connection or
      // received a termination request
      if (m_stop_running.load())
      {
        // it is a termination request.
        // in order to deal with the hanging async listening async process
        // The trick is to consume the current listener with a temporary connection
        // that will be immediately closed afterwards
        SPDLOG_DEBUG("Creating temporary connection to consume lingering listener.");
        boost::asio::io_service tmp_ios;
        boost::asio::ip::tcp::resolver tmp_resolver( tmp_ios );
        boost::asio::ip::tcp::socket tmp_sock(tmp_ios);
        boost::system::error_code tmp_ec;

        // deprecated code
        //        boost::asio::ip::tcp::resolver::query tmp_query("localhost", std::to_string(m_control_port),boost::asio::ip::tcp::resolver::query::v4_mapped ) ; //"np04-ctb-1", 8991
        //        boost::asio::ip::tcp::resolver::iterator tmp_iter = tmp_resolver.resolve(tmp_query) ;
        //tmp_sock.connect(tmp_iter->endpoint());
        tmp_sock.connect(tmp_resolver.resolve(boost::asio::ip::tcp::v4(),
                                              "localhost",std::to_string(m_control_port),tmp_ec)->endpoint());
        tmp_sock.shutdown(boost::asio::ip::tcp::socket::shutdown_send, tmp_ec);
        tmp_sock.close();
        tmp_resolver.cancel();
        tmp_ios.stop();


        // these do not cause trouble if they are called with a closed acceptor
        acceptor.cancel();
        acceptor.close();
        if (acceptor.is_open())
        {
          SPDLOG_WARN("Acceptor is still open. This should not happen");
        }
        m_control_error.store(true);
        SPDLOG_INFO("Exit request found.");
        // this is the issue. It seems to fail to exit from here.
        // exit this loop
        //break;
      }
      else
      {
        SPDLOG_INFO("Connection received. Start processing.\n");
      }
      boost::system::error_code error;
      // now enter the loop that keeps listening to this specific socket
      while (!m_stop_running.load())
      {
        nlohmann::json resp;
        
        // the tricky part of this kind of operation is that we do not know
        // exactly what /how many bytes are we receiving
        boost::array<char, 1024> req_buff{" "} ;
        // read_some is a blocking call, so it will hang here until it is ready to roll
        m_control_socket.read_some( boost::asio::buffer(req_buff ), error);
        if (error)
        {
          // if there was an error, exit the loop
          SPDLOG_INFO("Error reading from socket. Forcing a stop run for safety");
          // -- a caveats: if the connection is lost while running,
          // one needs to tell the reader to reset itself and stop taking data

          stop_run(resp);


          break;
          // the specific error will be parsed outside the loop
        }
        bool had_error = false;
        std::stringstream raw_request( std::string(req_buff.begin(), req_buff.end() ) ) ;
        SPDLOG_DEBUG("Unformatted message: [{}]",raw_request.str());
        try
        {
          nlohmann::json request ;
          raw_request >> request ;
          // process the command and redistribute as needed
          if (request.contains("command"))
          {
            std::string cmd = request.at("command").get<std::string>();
            if (cmd == std::string("config"))
            {
              nlohmann::json conf = request.at("config");
              SPDLOG_DEBUG("Received a config request\n");
              // call the configurator
              ret = config(conf,resp);
              if (ret)
              {
                had_error = true;
              }
            }
            else if (cmd == std::string("start_run"))
            {

              SPDLOG_DEBUG("Received a run start request");
              uint32_t run_number = request.at("run_number").get<uint32_t>();
              SPDLOG_DEBUG("Starting run {0}",run_number);
              ret = start_run(resp,run_number);
              if (ret)
              {
                had_error = true;
              }
              SPDLOG_TRACE("Feedback from Received a run start request [{0}]",resp.dump());
            }
            else if (cmd == std::string("stop_run"))
            {
              SPDLOG_DEBUG("Received a run stop request");
              ret = stop_run(resp);
              if (ret)
              {
                had_error = true;
              }
              SPDLOG_TRACE("Feedback from Received a run stop request [{}]",resp.dump());
            }
            else
            {
              SPDLOG_DEBUG("Unknown command {}",cmd.c_str());
              nlohmann::json entry;
              std::stringstream msg("");
              msg << "Unknown command : " << cmd;
              add_feedback(resp,"ERROR",msg.str());
              had_error = true;
            }
            if (m_control_error.load())
            {
              nlohmann::json entry;
              add_feedback(resp,"ERROR","Failed to execute request");
//              entry["type"] = "ERROR";
//              entry["message"] = "Failed to execute command";
//              resp["feedback"].push_back(entry);
            }
          }
          else
          {

            nlohmann::json entry;
            entry["type"] = "ERROR";
            entry["message"] = "Unknown request";
            resp["feedback"].push_back(entry);
          }
        }
        catch(nlohmann::json::exception &e)
        {
          had_error = true;
          SPDLOG_ERROR("Got exception");
          std::stringstream msg("");
          msg << "JSON exception : " << e.what();
          add_feedback(resp,"ERROR",msg.str());
          SPDLOG_ERROR("JSON exception :{}",e.what());
        }
        catch(std::exception &e)
        {
          had_error = true;
          std::stringstream msg("");
          msg << "STD exception : " << e.what();
          add_feedback(resp,"ERROR",msg.str());
          SPDLOG_ERROR("STD exception :{}",e.what());
        }
        catch(...)
        {
          had_error = true;
          std::stringstream msg("");
          msg << "Unknown exception";
          add_feedback(resp,"ERROR",msg.str());
          SPDLOG_ERROR("Unknown exception");
        }
        if (had_error)
        {
          add_feedback(resp,"ERROR","Requested had exceptions");
          SPDLOG_ERROR("Request had exceptions");
        }
        else
        {
          add_feedback(resp,"INFO"," Request was processed without exceptions");
          SPDLOG_INFO("Request was processed without exceptions");
        }        // write the response back
        // send feedback
        SPDLOG_TRACE("Sending a reply.");
        try {
          boost::asio::write( m_control_socket, boost::asio::buffer( resp.dump() ), error ) ;
          if (error)
          {
            std::stringstream msg;
            msg << "Error message: " << error.message();
            SPDLOG_ERROR("Error sending reply. {}",msg.str());
            throw std::runtime_error(error.message());
          }
          SPDLOG_DEBUG("Reply sent");
          // reset the response object
          resp.clear();
        }
        catch(nlohmann::json::exception &e)
        {
          m_control_error.store(true);
          std::stringstream msg("");
          msg << "JSON exception : " << e.what();
          SPDLOG_ERROR("JSON exception while sending response :{0}",e.what());
        }
        catch(std::exception &e)
        {
          m_control_error.store(true);
          std::stringstream msg("");
          msg << "STD exception : " << e.what();
          add_feedback(resp,"ERROR",msg.str());
          SPDLOG_ERROR("STD exception while sending response:{0}",e.what());
        }
        catch(...)
        {
          m_control_error.store(true);
          std::stringstream msg("");
          msg << "Unknown exception";
          add_feedback(resp,"ERROR",msg.str());
          SPDLOG_ERROR("Unknown exception while sending response.");
        }
        if (m_control_error.load())
        {
          SPDLOG_ERROR("Failed to reply.");
          break;
        }
        SPDLOG_TRACE("Reached the end of the loop.");
      }
      SPDLOG_TRACE("Closing control socket.");

      boost::system::error_code closing_error;

      if ( m_control_error.load() )
      {
        SPDLOG_TRACE("Closing due to internal error. Call for shutdown.");

        // we had a critical issue on our part.
        // forcefully terminate the connection from our end
        m_control_socket.shutdown(boost::asio::ip::tcp::socket::shutdown_send, closing_error);

        if ( closing_error )
        {
          std::stringstream msg;
          msg << "Error in shutdown " << closing_error.message();
          SPDLOG_ERROR("Error shutting down connection : {}",msg.str());
        }
      }
      // close the socket
      m_control_socket.close(closing_error) ;
      if ( closing_error )
      {
        std::stringstream msg;
        msg << "Socket closing failed:: " << closing_error.message();
        SPDLOG_ERROR("Error closing socket : {}",msg.str());
      }
      // stop the IO service
      m_control_ios.stop();

      // all ready to initiate a new listening socket
      SPDLOG_DEBUG("Reached end of this connection");
    }
    SPDLOG_INFO("Leaving the control listener");
  }

  int Handler::config(nlohmann::json &conf, nlohmann::json &resp)
  {
    bool had_error = false;
    int ret = 0;
    // refuse the configure if we are running
    if (m_reader->is_running())
    {
      add_feedback(resp,"ERROR","CIB is taking data");
      SPDLOG_ERROR("Already taking data");
      return 1;
    }
    SPDLOG_DEBUG("Configuration fragment : \n {0}",conf.dump());
    // there isn't much to be configured by the DAQ.
    // all we really care is to know where to send the data
    nlohmann::json receiver = conf.at("sockets").at("receiver");
    try
    {
      ret = m_reader->set_eth_receiver(receiver.at("host").get<std::string>(),
                                       receiver.at("port").get<unsigned int>());
      if (ret)
      {
        SPDLOG_ERROR("Failed to configure reader");
        had_error = true;
      }
      else
      {
        SPDLOG_INFO("CIB configured");
      }
      // grab all the feedback from the reader
      SPDLOG_DEBUG("Calling init");
      ret = m_reader->init();
      if (ret)
      {
        SPDLOG_ERROR("Failed to initialize reader");
        had_error = true;
      }
      else
      {
        SPDLOG_INFO("readout initialized");
      }
    }
    catch(nlohmann::json::exception &e)
    {
      had_error = true;
      std::stringstream msg("");
      msg << "JSON exception : " << e.what();
      add_feedback(resp,"ERROR",msg.str());
      SPDLOG_ERROR("JSON exception :{}",e.what());
    }
    catch(std::exception &e)
    {
      had_error = true;
      std::stringstream msg("");
      msg << "STD exception : " << e.what();
      add_feedback(resp,"ERROR",msg.str());
      SPDLOG_ERROR("STD exception :{}",e.what());
    }
    catch(...)
    {
      had_error = true;
      std::stringstream msg("");
      msg << "Unknown exception";
      add_feedback(resp,"ERROR",msg.str());
      SPDLOG_ERROR("Unknown exception");
    }
    // -- grab any feedback that may exist in the reader
    // grab all the feedback from the reader
    std::vector<daq::iols_feedback_msg_t> msgs;
    m_reader->get_feedback(msgs);
    for (auto entry: msgs)
    {
      add_feedback(resp,entry.sev, entry.msg);
    }
    // now return according to the situation
    if (had_error)
    {
      add_feedback(resp,"ERROR","CIB interface NOT configured");
      SPDLOG_ERROR("CIB interface NOT configured");
      return 1;
    }
    else
    {
      add_feedback(resp,"INFO","CIB interface configured");
      SPDLOG_INFO("CIB interface configured");
      return 0;
    }
  }

  int Handler::start_run(nlohmann::json &resp, const uint32_t run_number)
  {
    int ret = 0;
    if (!m_reader->is_configured())
    {
      add_feedback(resp,"ERROR","CIB is not yet configured.");
      SPDLOG_ERROR("CIB interface NOT configured yet");
      return 1;
    }
    if (m_reader->is_running())
    {
      add_feedback(resp,"ERROR","CIB is already taking data.");
      SPDLOG_ERROR("CIB is already running");
      return 1;
    }
    SPDLOG_DEBUG("Starting run");
    ret = m_reader->start_run(run_number);
    std::vector<daq::iols_feedback_msg_t> msgs;
    m_reader->get_feedback(msgs);
    for (auto entry: msgs)
    {
      add_feedback(resp,entry.sev, entry.msg);
    }

    if (ret)
    {
      SPDLOG_ERROR("Failed to start run");
      add_feedback(resp,"ERROR","Failed to start the run");
      return 1;
    }
    else
    {
      SPDLOG_INFO("Run started");
      add_feedback(resp,"INFO","Run started");
    }
    return 0;
  }
  int Handler::stop_run(nlohmann::json &resp)
  {
    int ret = 0;
    if (!m_reader->is_running())
    {
      add_feedback(resp,"ERROR","CIB is not running.");
      SPDLOG_ERROR("CIB is not running");
      return 1;
    }
    SPDLOG_DEBUG("Stopping run");
    ret = m_reader->stop_run();
    std::vector<daq::iols_feedback_msg_t> msgs;
    m_reader->get_feedback(msgs);
    for (auto entry: msgs)
    {
      add_feedback(resp,entry.sev, entry.msg);
    }

    if (ret)
    {
      SPDLOG_ERROR("Failed to stop run");
      add_feedback(resp,"ERROR","Failed to stop run. This will almost certainly cause trouble in the future.");
      return 1;
    }
    else
    {
      SPDLOG_INFO("Run Stopped");
      add_feedback(resp,"INFO","Run stopped.");
      return 0;
    }
  }

  void Handler::add_feedback(nlohmann::json &resp, const std::string type, const std::string msg)
  {
    nlohmann::json entry;
    entry["type"] = type;
    entry["message"] = msg;
    resp["feedback"].push_back(entry);
  }
} /* namespace cib */
