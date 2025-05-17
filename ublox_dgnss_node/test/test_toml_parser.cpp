// test_toml_parser.cpp - Test toml11 integration
#include <iostream>
#include <string>
#include <vector>
#include <toml.hpp>

int main()
{
    try {
        // Parse TOML file
        const std::string file_path = "/home/geoff/ros2_ws/src/ublox_dgnss_fork/ublox_dgnss_node/test/toml_test.toml";
        std::cout << "Attempting to parse TOML file: " << file_path << std::endl;
        
        toml::value data = toml::parse(file_path);
        
        // Extract basic values
        std::string version = toml::get<std::string>(data.at("version"));
        std::vector<std::string> device_types = toml::get<std::vector<std::string>>(data.at("device_types"));
        
        std::cout << "Successfully parsed the TOML file!" << std::endl;
        std::cout << "Version: " << version << std::endl;
        
        std::cout << "Device Types:" << std::endl;
        for (const auto& device : device_types) {
            std::cout << "  - " << device << std::endl;
        }
        
        // Extract parameters array
        const auto& params = toml::get<toml::array>(data.at("parameters"));
        std::cout << "Parameters: " << params.size() << " found" << std::endl;
        
        for (const auto& param : params) {
            std::string name = toml::get<std::string>(param.at("name"));
            std::string key_id = toml::get<std::string>(param.at("key_id"));
            std::string description = toml::get<std::string>(param.at("description"));
            
            std::cout << "Parameter: " << name << std::endl;
            std::cout << "  Key ID: " << key_id << std::endl;
            std::cout << "  Description: " << description << std::endl;
            
            // Access possible values if they exist
            if (param.contains("possible_values")) {
                const auto& possible_values = toml::get<toml::table>(param.at("possible_values"));
                std::cout << "  Possible Values:" << std::endl;
                
                for (const auto& [value_name, value] : possible_values) {
                    std::cout << "    " << value_name << " = " << toml::get<std::string>(value) << std::endl;
                }
            }
            
            std::cout << std::endl;
        }
        
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
}
