#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <vector>
#include <chrono>
#include <iomanip>

//configuration profile
struct ConfigProfile {
    std::string profile_param;
    std::string enable_param;
    std::string name;
};

const ConfigProfile DEPTH = {"depth_module.depth_profile", "enable_depth", "Depth"};
const ConfigProfile COLOR = {"rgb_camera.color_profile", "enable_color", "Color"};


std::vector<std::string> get_configs(rclcpp::Node::SharedPtr node, const ConfigProfile& config) {
    
    //create parameter client
    auto param_client = std::make_shared<rclcpp::SyncParametersClient>(node, "/camera/camera");
    
    //wait for service
    if (!param_client->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_WARN(node->get_logger(), "Camera parameter service not available");
        return {};
    }

    //get parameter descriptor
    auto descriptors = param_client->describe_parameters({config.profile_param});
    
    //check if parameter descriptor is valid
    if (descriptors.empty()) {
        RCLCPP_WARN(node->get_logger(), "No descriptor found for parameter %s", 
                   config.profile_param.c_str());
        return {};
    }

    //initialize parsing
    std::vector<std::string> configs;
    std::istringstream iss(descriptors[0].description);
    std::string line;
    
    //discard description header line
    std::getline(iss, line);
    
    //read available configurations
    while (std::getline(iss, line)) {
        if (!line.empty()) {
            configs.push_back(line);
        }
    }

    return configs;
}


// Set configuration
void set_config(rclcpp::Node::SharedPtr node, const ConfigProfile& config, const std::string& value) {
    
    //create parameter client
    auto param_client = rclcpp::SyncParametersClient(node, "/camera/camera");

    //set parameter, then enable stream
    param_client.set_parameters({
        {config.profile_param, value},
        {config.enable_param, true}
    });
    
    std::cout << "\nSet " << config.name << " to: " << value << "\n";

}

//display menu in two columns with fixed spacing
void show_menu(const std::vector<std::string>& options, const std::string& title) {
    std::cout << "\n" << title << " Options:\n";
    
    size_t half_size = (options.size() + 1) / 2;
    
    for (size_t i = 0; i < half_size; i++) {
        // Left column (fixed width for 2-digit numbers)
        std::cout << std::setw(2) << i+1 << ") " << std::left << std::setw(15) << options[i];
        
        // Right column (if exists)
        if (i + half_size < options.size()) {
            std::cout << std::right << std::setw(5) << i+1+half_size << ") " << options[i+half_size];
        }
        std::cout << "\n";
    }
    
    std::cout << "0) Back\nChoose: ";
}

int main() {
    
    //initialize ros2
    rclcpp::init(0, nullptr);
    //initialize node
    auto node = std::make_shared<rclcpp::Node>("camera_config");

    while (rclcpp::ok()) {
        
        std::cout << "\nRealSense Configuration\n1) Color\n2) Depth\nPress any other key to exit\nChoose: ";
        int choice;
        std::cin >> choice;

        if (choice != 1 && choice != 2) break;  //exit on any non-1/2 input
        
        const ConfigProfile& config = (choice == 1) ? COLOR : DEPTH;
        auto configs = get_configs(node, config);
        
        if (configs.empty()) continue;

        //once entered color/depth, set resolution loop
        while (true) {
            
            show_menu(configs, config.name);
            std::cin >> choice;
            
            if (choice == 0) break;
            if (choice > 0 && static_cast<size_t>(choice) <= configs.size()) {
                set_config(node, config, configs[choice-1]);
            }

        }

    }

    rclcpp::shutdown();
    return 0;
}