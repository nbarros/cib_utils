/*
 * trigger_analyser.cpp
 *
 *  Created on: Oct 23, 2024
 *      Author: Nuno Barros
 */
#include <thread>
#include <chrono>
#include <fstream>

#include <spdlog/spdlog.h>
#include <time_utils.h>
#include <cib_data_fmt.h>
#include <cib_data_utils.h>

extern "C"
{
#include <readline/readline.h>
#include <readline/history.h>
#include <unistd.h>
};

// -- Method prototypes
void dump_word(cib::daq::iols_trigger_t *word);
void dump_triggers(const std::string infile, const std::string outfile);
int run_command(int argc, char **argv);
void print_help();

    //
    // Implementations
    //

    void dump_word(cib::daq::iols_trigger_t *word)
{
  assert(word != nullptr);
  spdlog::info("TS {0:016d} M1 {1:06d} M2 {2:06d} M3 {3:06d}",
               word->timestamp,
               cib::data::get_m1(*word),
               cib::data::get_m2(*word), 
               cib::data::get_m3(*word)
                );
}
void dump_triggers(const std::string infile, const std::string outfile)
{
  /**
   * procedure:
   * 1. open the file
   * 2. grab triggers one by one
   * 3. cast into the iols_trigger_t structure
   * 4. Extract the contents into separate columns
   */
  spdlog::info("Translating triggers from \n INPUT  : [{0}]\n OUTPUT : [{1}]", infile, outfile);
  std::ifstream fin(infile.c_str(), std::ios::binary);
  if (!fin.is_open())
  {
    spdlog::critical("Couldn't open file {0}", infile);
    return;
  }
  const size_t word_size = sizeof(cib::daq::iols_trigger_t);
  char *buffer = new char[2 * word_size];
  // if outfile is empty, just dump into the screen
  std::ofstream fout;
  if (outfile.length() != 0)
  {
    fout.open(outfile.c_str());
    if (!fout.is_open())
    {
      spdlog::critical("Failed to open output file [{0}] for writing. Aborting.", outfile);
      // close the input file that had just been opened earlier
      fin.close();
      return;
    }
    // write the header to the file
    fout << "TIMESTAMP;RNN800;RNN600;LSTAGE" << std::endl;
  }
  else
  {
    spdlog::info("{0:<16} {1:<6} {2:<6} {3:<6}","TIMESTAMP","RNN800","RNN600","LSTAGE");
  }
  
    // we're now good to go.
  while (fin.read(buffer, sizeof(cib::daq::iols_trigger_t)))
  {
    // verify that we did get the number of bytes we were expecting
    if (fin.gcount() != word_size)
    {
      spdlog::error("Error reading file. Got {0} bytes (expected {1})", fin.gcount(), word_size);
      break;
    }
    // all good. Cast and read
    cib::daq::iols_trigger_t *tword = reinterpret_cast<cib::daq::iols_trigger_t *>(buffer);
    // if (spdlog::get_level() == spdlog::level::trace)
    if (outfile.length()==0)
    {
      dump_word(tword);
    }
    else
    {
      if (spdlog::get_level() == spdlog::level::trace)
      {
        dump_word(tword);
      }
      // write the info to the dumping CSV
      fout << tword->timestamp << ";"
           << cib::data::get_m1(*tword) << ";"
           << cib::data::get_m2(*tword) << ";"
           << cib::data::get_m3(*tword) << std::endl;
    }
    
  }
  fout.close();
  fin.close();
  spdlog::info("Done converting data");
}

int run_command(int argc, char **argv)
{
  if (argc < 1)
  {
    return 1;
  }

  std::string cmd(argv[0]);
  spdlog::trace("Processing command : {0}",cmd);

  if (cmd == "exit")
  {
    return 255;
  }
  else if (cmd == "help")
  {
    print_help();
    return 0;
  }
  else if (cmd == "dump")
  {
    int ret = 0;
    if (argc == 2)
    {
      // just dump the input file
      std::string infile = argv[1];
      std::string outfile = "";
      dump_triggers(infile,outfile);
    }
    else if (argc == 3)
    {
      std::string infile = argv[1];
      std::string outfile = argv[2];
      dump_triggers(infile, outfile);
    }
    else
    {
      spdlog::warn("usage: dump_triggers <trigger_file> [out_csv_file] (output file is optional)");
    }
    if (ret != 0)
    {
      spdlog::error("Failed to execute command. Check previous errors.");
    }
    return 0;
  }
  else if (cmd == "conv_from_ts")
  {
    if (argc != 2)
    {
      spdlog::warn("Usage: conv_from_ts pdts_timestamp");
      return 0;
    }
    else
    {
      std::string tstamp = argv[1];
      uint64_t ts = std::stoull(tstamp);
      spdlog::trace("Converting timestamp {0}",ts);
      std::string res = cib::util::format_timestamp(ts,62500000);
      spdlog::info("DATE : {0}",res);
    }
    return 0;
  }
  else if (cmd == "conv_to_ts")
  {
    if (argc != 3)
    {
      spdlog::warn("Usage: conv_to_ts YYYY-MM-DD HH:MM:SS");
      return 0;
    }
    else
    {
      std::string date_time = argv[1];
      date_time += " ";
      date_time += argv[2];
      uint64_t ts = cib::util::calc_timestamp(date_time,62500000);
      spdlog::info("PDTS : {0}", ts);
    }
    return 0;
  }
  else
  {
    spdlog::error("Unknown command.");
    return 0;
  }
  return 0;
}


void print_help()
{
  spdlog::info("dump_triggers <input_file> [out_csv_file]");
  spdlog::info("        Produces a CSV file with the trigger info in human readable format");
  spdlog::info("conv_to_ts YYYY-MM-DD HH:MM:SS");
  spdlog::info("        Convert the date specified into a PDTS timestamp");
  spdlog::info("conv_from_ts <timestamp>");
  spdlog::info("        Convert a PDTS timestamp into a date time setting");
}

int main(int argc, char** argv)
{

  spdlog::set_pattern("cib_trigger : [%^%L%$] %v");
  spdlog::set_level(spdlog::level::info); // Set global log level to info

  std::string infile, outfile;
  int c;
  opterr = 0;
  int report_level = SPDLOG_LEVEL_INFO;
  while ((c = getopt(argc, argv, "vi:o:")) != -1)
  {
    switch (c)
    {
    case 'v':
      if (report_level > 0)
      {
        report_level--;
      }
      break;
    case 'i':
      infile = optarg;
      break;
    case 'o':
      outfile = optarg;
      break;
    default : /* ? */
      spdlog::warn("Usage: cib_manager [-v]  (repeated flags further increase verbosity)");
      return 1;
    }
  }
  if (report_level != SPDLOG_LEVEL_INFO)
  {
    spdlog::set_level(static_cast<spdlog::level::level_enum>(report_level)); // Set global log level to info
  }

  spdlog::info("Log level: {0} : {1}", static_cast<int>(spdlog::get_level()), spdlog::level::to_string_view(spdlog::get_level()).data());
  spdlog::trace("Just testing a trace");
  spdlog::debug("Just testing a debug");

  // by default set to the appropriate settings
  print_help();

  // -- now start the real work
  char *buf;
  while ((buf = readline(">> ")) != nullptr)
  {
    if (strlen(buf) > 0)
    {
      add_history(buf);
    }
    else
    {
      free(buf);
      continue;
    }
    char *delim = (char *)" ";
    int count = 1;
    char *ptr = buf;
    while ((ptr = strchr(ptr, delim[0])) != NULL)
    {
      count++;
      ptr++;
    }
    if (count > 0)
    {
      char **cmd = new char *[count];
      cmd[0] = strtok(buf, delim);
      int i;
      for (i = 1; cmd[i - 1] != NULL && i < count; i++)
      {
        cmd[i] = strtok(NULL, delim);
      }
      if (cmd[i - 1] == NULL)
        i--;
      int ret = run_command(i, cmd);
      delete[] cmd;
      if (ret == 255)
      {
        return 0;
      }
      if (ret != 0)
      {
        return ret;
      }
    }
    else
    {
      return 0;
    }
    free(buf);
  }
  return 0;
}
