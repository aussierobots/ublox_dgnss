// test_toml_config_parser.cpp - Test parsing of the UBX-CFG parameter TOML file
#include <iostream>
#include <string>
#include <vector>
#include <toml.hpp>

int main()
{
    try {
        // Parse the UBX-CFG parameters TOML file
        const std::string file_path = "/home/geoff/ros2_ws/src/ublox_dgnss_fork/config/ubx_cfg_parameters_full.toml";
        std::cout << "Attempting to parse UBX-CFG parameters TOML file: " << file_path << std::endl;
        
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
        
        // Check firmware versions
        const auto& firmware_versions = toml::find(data, "firmware_versions");
        std::cout << "\nFirmware Versions:" << std::endl;
        
        // ZED-F9P firmware versions
        const auto& f9p_versions = toml::find(firmware_versions, "ZED-F9P");
        std::cout << "  ZED-F9P:" << std::endl;
        for (const auto& version : f9p_versions.as_array()) {
            std::string version_number = toml::find<std::string>(version, "version");
            std::string description = toml::find<std::string>(version, "description");
            std::cout << "    - " << version_number << ": " << description << std::endl;
        }
        
        // Check parameters
        const auto& parameters = toml::find(data, "parameters").as_array();
        std::cout << "\nParameters: " << parameters.size() << " found" << std::endl;
        
        // Print a few parameter details as a sample
        for (size_t i = 0; i < std::min(size_t(2), parameters.size()); ++i) {
            const auto& param = parameters[i];
            std::string name = toml::find<std::string>(param, "name");
            std::string key_id = toml::find<std::string>(param, "key_id");
            std::string description = toml::find<std::string>(param, "description");
            
            std::cout << "\nParameter " << (i+1) << ": " << name << std::endl;
            std::cout << "  Key ID: " << key_id << std::endl;
            std::cout << "  Description: " << description << std::endl;
            
            // Check if possible_values exists for this parameter
            if (param.contains("possible_values")) {
                const auto& possible_values = toml::find(param, "possible_values");
                std::cout << "  Possible Values:" << std::endl;
                
                for (const auto& [value_name, value] : possible_values.as_table()) {
                    std::cout << "    " << value_name << " = " << toml::get<std::string>(value) << std::endl;
                }
            }
            
            // Check firmware support
            if (param.contains("firmware_support")) {
                const auto& firmware_support = toml::find(param, "firmware_support");
                std::cout << "  Firmware Support:" << std::endl;
                
                for (const auto& [device, support] : firmware_support.as_table()) {
                    std::string since = toml::find<std::string>(support, "since");
                    std::cout << "    " << device << ": since " << since << std::endl;
                    
                    // Check for behavior changes
                    if (support.contains("behavior_changes")) {
                        const auto& changes = toml::find(support, "behavior_changes").as_array();
                        for (const auto& change : changes) {
                            std::string change_version = toml::find<std::string>(change, "version");
                            std::string change_desc = toml::find<std::string>(change, "description");
                            std::cout << "      - Change in " << change_version << ": " << change_desc << std::endl;
                        }
                    }
                }
            }
        }
        
        std::cout << "\nTOML file validation successful!" << std::endl;
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
}
