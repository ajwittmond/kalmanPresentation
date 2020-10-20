#include <array>
#include <filesystem>
#include <fstream>
#include <vector>
#include <regex>

#include "ini_parser.h"

const std::regex SECTION_START("^\\s*\\[(\\w+)\\]\\s*$");
const std::regex KEY_VALUE("^\\s*(\\S+)=(\\S+)\\s*$");


ParsedIniFile parse_ini_file(std::filesystem::path path, const int buffer_size){
  std::fstream input( path, std::fstream::in);

  ParsedIniFile output;

  std::string header_name;
  bool has_header = false;
  std::string line;
  do{
    std::getline(input,line,'\n');
    {
      std::smatch header_match;
      if(std::regex_match(line, header_match, SECTION_START)){
        header_name = header_match[1].str();
        output[header_name].push_back(std::unordered_map<std::string,std::string>());
        has_header = true;
        continue;
      }
    }
    if(has_header){
      std::smatch key_value_match;
      if (std::regex_match(line, key_value_match, KEY_VALUE)) {
        std::string key  = key_value_match[1].str();
        std::string value = key_value_match[2].str();
        output[header_name].back()[key] = value;
      }
    }
  }while(!input.eof());

  return output;
}
