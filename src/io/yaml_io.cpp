/**
 * Developer: WANG_Guanhua on 20231014, Saturday
*/

#include "io/yaml_io.h"

#include <fstream>
#include <iostream>

namespace fastlio {

YAML_IO::YAML_IO(const std::string &path) {
    path_ = path;
    yaml_node_ = YAML::LoadFile(path_);
    if (yaml_node_.IsNull()) {
        std::cout << "**********************************" << std::endl;
        std::cout << "Failed to open yaml file: " << path << std::endl;
        std::cout << "**********************************" << std::endl;
    }
    is_opened_ = true;
}

YAML_IO::~YAML_IO() {}

bool YAML_IO::Save(const std::string &path) {
    if (path.empty()) {
        std::ofstream fout(path_);
        fout << yaml_node_;
    } else {
        std::ofstream fout(path);
        fout << yaml_node_;
    }
    return true;
}

}  // namespace fastlio
