#pragma once

#include <string>
#include <unordered_map>
#include <filesystem>
#include <vector>

/**
 *  The value returned after parsing the ini file.  The keys are the section headers and the values
 *  are a vector of maps containing the key value pairs in the given section for each section with
 *  that header.
 */
typedef std::unordered_map<std::string,
  std::vector<std::unordered_map<std::string, std::string>>>
    ParsedIniFile;

/**
 *  A very limited ini file parser.  Cannot handle spaces in lists or between key value pairs.
 */
ParsedIniFile parse_ini_file(std::filesystem::path  path, const int buffer_size = 500);
