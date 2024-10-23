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

void dump_word(cib::daq::iols_trigger_t *word)
{
  assert(word != nullptr);
  spdlog::info("TS {64:0} M1 {6:1} M2 {6:2} M3 {6:3}",
               word->timestamp,
               cib::data::get_m1(*word),
               cib::data::get_m2(*word), 
               cib::data::get_m3(*word)
                );
}

void dump_csv(const std::string infile, const std::string outfile)
{
  /**
   * procedure:
   * 1. open the file
   * 2. grab triggers one by one
   * 3. cast into the iols_trigger_t structure
   * 4. Extract the contents into separate columns
   */
  std::ifstream fin(infile.c_str(),std::ios::binary);
  if (!fin.is_open())
  {
    spdlog::critical("Couldn't open file {0}",infile);
    return;
  }
  const size_t word_size = sizeof(cib::daq::iols_trigger_t);
  char *buffer = new char[2 * word_size ];
  std::ofstream fout(outfile.c_str());
  if (!fout.is_open())
  {
    spdlog::critical("Failed to open output file [{0}] for writing. Aborting.",outfile);
    // close the input file that had just been opened earlier
    fin.close();
    return;
  }
  // write the header to the file
  fout << "TIMESTAMP;RNN800;RNN600;LSTAGE" << std::endl;
  // we're now good to go.
  while (fin.read(buffer, sizeof(cib::daq::iols_trigger_t)))
  {
    // verify that we did get the number of bytes we were expecting
    if (fin.gcount() != word_size)
    {
      spdlog::error("Error reading file. Got {0} bytes (expected {1})",fin.gcount(),word_size);
      break;
    }
    // all good. Cast and read
    cib::daq::iols_trigger_t *tword = reinterpret_cast<cib::daq::iols_trigger_t *>(buffer);
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
  fout.close();
  fin.close();
}

int main(int argc, char** argv)
{

  spdlog::set_pattern("cib : [%^%L%$] %v");
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

  if (infile.length() == 0)
  {
    spdlog::error("No input file specified.");
    return 0;
  }
  if (outfile.length() == 0)
  {
    spdlog::error("No output file specified.");
    return 0;
  }

  dump_csv(infile,outfile);
  return 0;
}
