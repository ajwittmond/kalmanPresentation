#pragma once

#include <string>
#include <unordered_map>
#include <filesystem>
#include <vector>

typedef std::unordered_map<std::string,
  std::vector<std::unordered_map<std::string, std::string>>>
    ParsedIniFile;

ParsedIniFile parse_ini_file(std::filesystem::path  path, const int buffer_size = 500);
