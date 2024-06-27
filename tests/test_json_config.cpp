/*
 * test_json_config.cpp
 *
 *  Created on: Jun 5, 2024
 *      Author: Nuno Barros
 */


#include <iostream>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

int main(int argc, char** argv)
{
  json conf;
  conf["receiver"] = json();
  conf["receiver"]["host"] = "localhost";
  conf["receiver"]["port"] = 8992;
  conf["receiver"]["instance"] = 1;

  std::cout << "JSON structure: " << std::endl;
  std::cout << conf.dump() << std::endl;

  return 0;
}

