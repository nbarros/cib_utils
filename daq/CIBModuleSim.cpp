/*
 * CIBModuleSim.cpp
 *
 *  Created on: Mar 31, 2024
 *      Author: Nuno Barros
 */

#include <CIBModuleSim.hpp>
#include <chrono>
#include <string>
#include <thread>
#include <vector>
#include <spdlog/spdlog.h>

namespace dunedaq::cibmodules
{

  CIBModuleSim::CIBModuleSim ()
      :
          m_is_running(false)
          , m_is_configured(false)
          , m_num_TS_words(0)
          , m_num_TR_words(0)
          , m_error_state(false)
          , m_control_ios()
          , m_receiver_ios()
          , m_control_socket(m_control_ios)
          , m_receiver_socket(m_receiver_ios)
          // FIXME: replace this for a classic thread
          , m_thread_(std::bind(&CIBModuleSim::receive, this, std::placeholders::_1))
          , m_run_trigger_counter(0)
          , m_num_control_messages_sent(0)
          , m_num_control_responses_received(0)
          , m_last_readout_timestamp(0)

  {
    // TODO Auto-generated constructor stub

  }

  CIBModuleSim::~CIBModuleSim ()
  {
    if(m_is_running)
    {
      const nlohmann::json stopobj;
      // this should also take care of closing the streaming socket
      do_stop(stopobj);
    }
    m_control_socket.close() ;
  }

  void
  CIBModuleSim::init(const nlohmann::json& init_data)
  {
    SPDLOG_DEBUG("Entering init() method");
  }

  void
  CIBModuleSim::do_configure(const nlohmann::json& args)
  {
    // base config
    //{"command":"config","config":{"cib":{"sockets":{"receiver":{"host":"localhost","port":1234}}}}}
    SPDLOG_DEBUG("Configuring CIB");

    // this is automatically generated out of the jsonnet files in the (config) schema
    m_cfg = args;
    // set local caches for the variables that are needed to set up the receiving ends
    // this should be localhost
    //m_receiver_host = m_cfg.board_config.cib.sockets.receiver.host;
    m_receiver_port = m_cfg.at("cib").at("sockets").at("receiver").at("port").get<unsigned int>();
    unsigned int to = m_cfg.at("cib").at("sockets").at("receiver").at("timeout").get<unsigned int>();
    m_receiver_timeout = std::chrono::microseconds( to ) ;
    //m_receiver_host = m_cfg.at("cib").at("sockets").at("receiver").at("host").get<std::string>();

    SPDLOG_INFO("Board receiver network location {}:{}", "localhost",m_receiver_port);

    // Initialise monitoring variables
    m_num_control_messages_sent = 0;
    m_num_control_responses_received = 0;

    // network connection to cib hardware control
    std::string control_host =  m_cfg.at("cib_control_host").get<std::string>();
    unsigned int control_port = m_cfg.at("cib_control_port").get<unsigned int>();;
    unsigned int control_timeout = m_cfg.at("cib_control_timeout").get<unsigned int>();;

    boost::asio::ip::tcp::resolver resolver( m_control_ios );
    // once again, these are obtained from the configuration
    boost::asio::ip::tcp::resolver::query query(control_host,
                                                std::to_string(control_port) ) ; //"np04-ctb-1", 8991
    boost::asio::ip::tcp::resolver::iterator iter = resolver.resolve(query) ;

    m_endpoint = iter->endpoint();

    // attempt the connection.
    // FIXME: what to do if that fails?
    try
    {
      m_control_socket.connect( m_endpoint );
    }
    catch (std::exception& e)
    {
      SPDLOG_ERROR("Exeption caught while establishing connection to CIB : {}",e.what());
      // do nothing more. Just exist
      m_is_configured.store(false);
      return;
    }

    // if necessary, set the calibration stream
    // the CIB calibration stream is something a bit different
    if ( m_cfg.at("cib").at("misc").at("trigger_stream_enable").get<bool>())
    {
      SPDLOG_DEBUG("Trigger stream enabled");
      m_calibration_stream_enable = true ;
      m_calibration_dir = m_cfg.at("cib").at("misc").at("trigger_stream_output").get<std::string>() ;
      m_calibration_file_interval = std::chrono::minutes(m_cfg.at("cib").at("misc").at("trigger_stream_update").get<unsigned int>());
    }

    // create the json string out of the config fragment
    nlohmann::json config = m_cfg;

    //to_json(config, m_cfg.board_config);
    SPDLOG_DEBUG("CONF TEST: [{}]",config.dump());

    // FIXME: Actually would prefer to use protobufs, but this is also acceptable
    send_config(config.dump());
  }

  void
  CIBModuleSim::do_start(const nlohmann::json& startobj)
  {

    SPDLOG_DEBUG("Entering do_start() method");

    // actually, the first thing to check is whether the CIB has been configured
    // if not, this won't work
    if (!m_is_configured.load())
    {
      SPDLOG_ERROR("CIB has not been successfully configured.");
      //throw std::runtime_error("CIB has not been successfully configured.");
      return;
    }


    SPDLOG_DEBUG(": Sending start of run command");
    //FIXME: Check what worker thread actually does
    // and replace accordingly
    m_thread_.start_working_thread();

    if ( m_calibration_stream_enable )
    {
      std::stringstream run;
      run << "run_101";
      set_calibration_stream(run.str()) ;
    }
    nlohmann::json cmd;
    cmd["command"] = "start_run";
    cmd["run_number"] = 101;

    if ( send_message( cmd.dump() )  )
    {
      m_is_running.store(true);
      SPDLOG_DEBUG(" successfully started");
    }
    else
    {
      SPDLOG_ERROR("Unable to start CIB run");
      throw std::runtime_error("Unable to start CIB run");
    }

    SPDLOG_DEBUG("Exiting do_start() method");
  }

  void
  CIBModuleSim::do_stop(const nlohmann::json& /*stopobj*/)
  {

    SPDLOG_DEBUG("Entering do_stop() method");

    SPDLOG_INFO("Sending stop run command");
    if(send_message( "{\"command\":\"stop_run\"}" ) )
    {
      SPDLOG_DEBUG("successfully stopped");
      m_is_running.store( false ) ;
    }
    else
    {
      SPDLOG_ERROR("Unable to stop CIB run");
      throw std::runtime_error("Unable to stop CIB run");
    }
    //
    // FIXME: Update this
    m_thread_.stop_working_thread();
    // reset counters

    m_run_trigger_counter=0;
    m_run_packet_counter=0;

    SPDLOG_DEBUG("Exiting do_stop() method");
  }

  void
  CIBModuleSim::receive(std::atomic<bool>& running_flag)
  {
    SPDLOG_DEBUG("Entering receive() method");

    std::size_t n_bytes = 0 ;
    std::size_t n_words = 0 ;

    const size_t header_size = sizeof( dunedaq::cibmodules::content::tcp_header_t ) ;
    const size_t word_size = content::word::word_t::size_bytes ;

    SPDLOG_DEBUG(" Header size: {} \n Word size: {}",header_size,word_size);

    //connect to socket
    boost::asio::ip::tcp::acceptor acceptor(m_receiver_ios,
                                            boost::asio::ip::tcp::endpoint( boost::asio::ip::tcp::v4(),
                                                                            m_receiver_port ) );
    SPDLOG_INFO("Waiting for an incoming connection on port {} ",m_receiver_port);

    std::future<void> accepting = async( std::launch::async, [&]{ acceptor.accept(m_receiver_socket) ; } ) ;

    while ( running_flag.load() )
    {
      if ( accepting.wait_for( m_timeout ) == std::future_status::ready )
      {
        break ;
      }
    }

    SPDLOG_TRACE("Connection received: start reading");

    // -- A couple of variables to help in the data parsing
    /**
     * The structure is a bit different than in the CTB
     * The TCP packet is still wrapped in a header, and there is a timestamp word
     * marking the last packet
     * But other than that, everything are triggers
     */
    content::tcp_header_t head ;
    head.packet_size = 0;
    content::word::word_t temp_word ;
    boost::system::error_code receiving_error;
    bool connection_closed = false ;
    uint64_t ch_stat_beam, ch_stat_crt, ch_stat_pds;
    uint64_t llt_payload, channel_payload;
    uint64_t prev_timestamp = 0;
    std::pair<uint64_t,uint64_t> prev_channel, prev_prev_channel, prev_llt, prev_prev_llt; // pair<timestamp, trigger_payload>

    while (running_flag.load())
    {

      update_calibration_file();

      if ( ! read( head ) ) {
        connection_closed = true ;
        break;
      }

      n_bytes = head.packet_size ;
      // extract n_words

      n_words = n_bytes / word_size ;
      // read n words as requested from the header

      update_buffer_counts(n_words);

      for ( unsigned int i = 0 ; i < n_words ; ++i )
      {
        //read a word
        if ( ! read( temp_word ) )
        {
          connection_closed = true ;
          break ;
        }
        // put it in the calibration stream
        // the tricky thing is that there seems to be some backwards
        if ( m_calibration_stream_enable )
        {
          // note that this may ignore endianess
          m_calibration_file.write( reinterpret_cast<const char*>( & temp_word ), word_size ) ;
          m_calibration_file.flush() ;
        }
        // word printing in calibration stream

        //check if it is a TS word and increment the counter
        if ( temp_word.word_type == content::word::t_ts )
        {
          m_num_TS_words++ ;
          SPDLOG_TRACE("Received timestamp word! TS: {} ",temp_word.timestamp);
          prev_timestamp = temp_word.timestamp;
        }
        // FIXME: Should we reintroduce these
        // not for now. Use slow control to monitor the internal buffers
        else if (  temp_word.word_type == content::word::t_fback  )
        {
          m_error_state.store( true ) ;
          content::word::feedback_t * feedback = reinterpret_cast<content::word::feedback_t*>( & temp_word ) ;
          SPDLOG_WARN("Received feedback word!");
          std::stringstream msg;
          msg << ": Feedback word: " << std::endl
              << std::hex
              << " \t Type -> " << feedback -> word_type << std::endl
              << " \t TS -> " << feedback -> timestamp << std::endl
              << " \t Code -> " << feedback -> code << std::endl
              << " \t Source -> " << feedback -> source << std::endl
              << " \t Padding -> " << feedback -> padding << std::dec << std::endl ;
          SPDLOG_WARN(msg.str().c_str());
        }
        else if (temp_word.word_type == content::word::t_trigger)
        {
          SPDLOG_TRACE("Received IoLS trigger word!");

          ++m_num_TR_words;
          ++m_run_trigger_counter;
          content::word::trigger_t * trigger_word = reinterpret_cast<content::word::trigger_t*>( & temp_word ) ;
          std::stringstream msg;
          msg << "Word [" << std::hex
              << "0x" << trigger_word->word_type
              << "][0x"<< trigger_word->timestamp
              << "][0x"<< trigger_word->trigger_word
              << "]\n";
          SPDLOG_TRACE(msg.str().c_str());
          m_last_readout_timestamp = trigger_word->timestamp;
          // we do not need to know anything else
          // ideally, one could add other information such as the direction
          // this should be coming packed in the trigger word
          // note, however, that to reconstruct the trace direction we also would need the source position
          // and that we cannot afford to send, so we can just make it up out of the
          // IoLS system
          //
          // Send HSI data to a DLH
          std::array<uint32_t, 7> hsi_struct;
          hsi_struct[0] = (0x1 << 26) | (0x1 << 6) | 0x1; // DAQHeader, frame version: 1, det id: 1, link for low level 0, link for high level 1, leave slot and crate as 0
          hsi_struct[1] = trigger_word->timestamp & 0xFFFFFF;       // ts low
          hsi_struct[2] = trigger_word->timestamp >> 32; // ts high
          // we could use these 2 sets of 32 bits to identify the direction
          // TODO: Propose to change this to include additional information
          // these 64 bits could be used to define a direction
          hsi_struct[3] = 0x0;                      // lower 32b
          hsi_struct[4] = 0x0;                      // upper 32b
          hsi_struct[5] = trigger_word->trigger_word;    // trigger_map;
          hsi_struct[6] = m_run_trigger_counter;         // m_generated_counter;
          msg.clear(); msg.str("");
          msg << ": Formed HSI_FRAME_STRUCT for hlt "
              << std::hex
              << "0x"   << hsi_struct[0]
              << ", 0x" << hsi_struct[1]
              << ", 0x" << hsi_struct[2]
              << ", 0x" << hsi_struct[3]
              << ", 0x" << hsi_struct[4]
              << ", 0x" << hsi_struct[5]
              << ", 0x" << hsi_struct[6]
              << "\n";
          SPDLOG_TRACE(msg.str().c_str());
        }
      } // n_words loop

      if ( connection_closed )
      {
        break ;
      }
    }

    boost::system::error_code closing_error;

    if ( m_error_state.load() )
    {

      m_receiver_socket.shutdown(boost::asio::ip::tcp::socket::shutdown_send, closing_error);

      if ( closing_error )
      {
        std::stringstream msg;
        msg << "Error in shutdown " << closing_error.message();
        SPDLOG_ERROR(msg.str().c_str());
      }
    }

    m_receiver_socket.close(closing_error) ;

    if ( closing_error )
    {
      std::stringstream msg;
      msg << "Socket closing failed:: " << closing_error.message();
      SPDLOG_ERROR(msg.str().c_str());
    }


    SPDLOG_DEBUG("End of do_work loop: stop receiving data from the CIB");

    SPDLOG_DEBUG("Exiting receive() method");
  }

  //FIXME: Continue here

  template<typename T>
  bool CIBModuleSim::read( T &obj) {

    boost::system::error_code receiving_error;
    boost::asio::read( m_receiver_socket, boost::asio::buffer( &obj, sizeof(T) ), receiving_error ) ;

    if ( ! receiving_error )
    {
      return true ;
    }

    if ( receiving_error == boost::asio::error::eof)
    {
      std::string error_message = "Socket closed: " + receiving_error.message();
      SPDLOG_ERROR(error_message.c_str());
      return false ;
    }

    if ( receiving_error ) {
      std::string error_message = "Read failure: " + receiving_error.message();
      SPDLOG_ERROR(error_message.c_str());
      return false ;
    }

    return true ;
  }

  void CIBModuleSim::init_calibration_file()
  {

    if ( ! m_calibration_stream_enable )
    {
      return ;
    }
    char file_name[200] = "" ;
    time_t rawtime;
    time( & rawtime ) ;
    struct tm * timeinfo = localtime( & rawtime ) ;
    strftime( file_name, sizeof(file_name), "%F_%H.%M.%S.calib", timeinfo );
    std::string global_name = m_calibration_dir + m_calibration_prefix + file_name ;
    m_calibration_file.open( global_name, std::ofstream::binary ) ;
    m_last_calibration_file_update = std::chrono::steady_clock::now();
    // _calibration_file.setf ( std::ios::hex, std::ios::basefield );
    // _calibration_file.unsetf ( std::ios::showbase );
    SPDLOG_INFO("New Calibration Stream file: {}", global_name .c_str());

  }

  void CIBModuleSim::update_calibration_file()
  {

    if ( ! m_calibration_stream_enable )
    {
      return ;
    }

    std::chrono::steady_clock::time_point check_point = std::chrono::steady_clock::now();

    if ( check_point - m_last_calibration_file_update < m_calibration_file_interval ) {
      return ;
    }

    m_calibration_file.close() ;
    init_calibration_file() ;

  }

  bool CIBModuleSim::set_calibration_stream( const std::string & prefix )
  {

    if ( m_calibration_dir.back() != '/' )
    {
      m_calibration_dir += '/' ;
    }
    m_calibration_prefix = prefix ;
    if ( prefix.size() > 0 )
    {
      m_calibration_prefix += '_' ;
    }
    // possibly we could check here if the directory is valid and  writable before assuming the calibration stream is valid
    return true ;
  }


  void CIBModuleSim::send_config( const std::string & config ) {

    if ( m_is_configured.load() )
    {
      SPDLOG_DEBUG("Resetting before configuring");
      send_reset();
    }

    SPDLOG_INFO("Sending config");

    // structure the message to have a common management structure
    //json receiver = doc.at("ctb").at("sockets").at("receiver");

    nlohmann::json conf;
    conf["command"] = "config";
    conf["config"] = nlohmann::json::parse(config);

    if ( send_message( conf.dump() ) )
    {
      m_is_configured.store(true) ;
    }
    else
    {
      SPDLOG_CRITICAL("Unable to configure CIB");
    }
  }

  void CIBModuleSim::send_reset()
  {
    // actually, we do not want to do this to the CIB
    // the reset should go through the slow control

    SPDLOG_INFO("NOT Sending a reset");

    return;

    //    // actually, we do not want to do this to the CIB
    //    // the reset should go through the slow control
    //    if(send_message( "{\"command\":\"HardReset\"}" ))
    //    {
    //
    //      m_is_running.store(false);
    //      m_is_configured.store(false);
    //
    //    }
    //    else{
    //      ers::error(CTBCommunicationError(ERS_HERE, "Unable to reset CTB"));
    //    }

  }

  bool CIBModuleSim::send_message( const std::string & msg )
  {

    //add error options
    //FIXME: Migrate this to protobuf
    boost::system::error_code error;
    SPDLOG_INFO("Sending message: {}",msg);

    m_num_control_messages_sent++;

    boost::asio::write( m_control_socket, boost::asio::buffer( msg ), error ) ;
    boost::array<char, 1024> reply_buf{" "} ;
    m_control_socket.read_some( boost::asio::buffer(reply_buf ), error);
    std::stringstream raw_answer( std::string(reply_buf .begin(), reply_buf .end() ) ) ;
    SPDLOG_INFO("Unformatted answer: [{}]",raw_answer.str());

    nlohmann::json answer ;
    raw_answer >> answer ;
    nlohmann::json & messages = answer["feedback"] ;
    SPDLOG_INFO("Received messages: {}",messages.size());

    bool ret = true ;
    for (nlohmann::json::size_type i = 0; i != messages.size(); ++i )
    {

      m_num_control_responses_received++;

      std::string type = messages[i]["type"].dump() ;
      if ( type.find("error") != std::string::npos || type.find("Error") != std::string::npos || type.find("ERROR") != std::string::npos )
      {
        SPDLOG_ERROR(messages[i]["message"].dump());
        ret = false ;
      }
      else if ( type.find("warning") != std::string::npos || type.find("Warning") != std::string::npos || type.find("WARNING") != std::string::npos )
      {
        SPDLOG_WARN( messages[i]["message"].dump());
      }
      else if ( type.find("info") != std::string::npos || type.find("Info") != std::string::npos || type.find("INFO") != std::string::npos)
      {
        SPDLOG_INFO("Message from the board: {}",messages[i]["message"].dump());
      }
      else
      {
        std::stringstream blob;
        blob << messages[i] ;
        SPDLOG_DEBUG(": Unformatted from the board: {}",blob.str());
      }
    }

    return ret;

  }

  void
  CIBModuleSim::update_buffer_counts(uint new_count) // NOLINT(build/unsigned)
  {
    //std::unique_lock mon_data_lock(m_buffer_counts_mutex);
    if (m_buffer_counts.size() > 1000)
      m_buffer_counts.pop_front();
    m_buffer_counts.push_back(new_count);
  }


} /* namespace cib */
