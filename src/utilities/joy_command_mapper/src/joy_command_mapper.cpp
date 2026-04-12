#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include "joy_command_mapper/msg/robot_command.hpp"
#include <string>
#include <vector>
#include <map>
#include <cmath>

class JoyCommandMapper : public rclcpp::Node
{
public:
    JoyCommandMapper() : Node("joy_command_mapper")
    {
        // Declare parameters
        this->declare_parameter<std::string>("controller_type", "ps3");
        this->declare_parameter<double>("publish_rate", 50.0);
        this->declare_parameter<double>("deadzone", 0.12);
        this->declare_parameter<double>("max_linear_speed", 1.0);
        this->declare_parameter<double>("max_angular_speed", 1.396);
        
        // Velocity mappings
        this->declare_parameter<int>("velocity.vx.axis", 3);
        this->declare_parameter<double>("velocity.vx.scale", 1.0);
        this->declare_parameter<double>("velocity.vx.deadzone", 0.1);
        this->declare_parameter<bool>("velocity.vx.inverted", false);
        
        this->declare_parameter<int>("velocity.vy.axis", 2);
        this->declare_parameter<double>("velocity.vy.scale", 0.5);
        this->declare_parameter<double>("velocity.vy.deadzone", 0.1);
        this->declare_parameter<bool>("velocity.vy.inverted", false);
        
        this->declare_parameter<int>("velocity.wz.axis", 0);
        this->declare_parameter<double>("velocity.wz.scale", 1.0);
        this->declare_parameter<double>("velocity.wz.deadzone", 0.1);
        this->declare_parameter<bool>("velocity.wz.inverted", false);
        
        // Mode mappings
        this->declare_parameter<int>("modes.autonomy.axis", 4);
        this->declare_parameter<double>("modes.autonomy.threshold", -0.1);
        this->declare_parameter<bool>("modes.autonomy.inverted", true);
        
        this->declare_parameter<int>("modes.manual.axis", 5);
        this->declare_parameter<double>("modes.manual.threshold", -0.1);
        this->declare_parameter<bool>("modes.manual.inverted", false);
        
        // Action mappings
        this->declare_parameter<int>("actions.clear_cloud.button", 9);
        this->declare_parameter<int>("actions.emergency_stop.button", 10);
        
        // Get parameters
        controller_type_ = this->get_parameter("controller_type").as_string();
        publish_rate_ = this->get_parameter("publish_rate").as_double();
        max_linear_speed_ = this->get_parameter("max_linear_speed").as_double();
        max_angular_speed_ = this->get_parameter("max_angular_speed").as_double();
        
        // Load velocity mappings
        vx_config_.axis = this->get_parameter("velocity.vx.axis").as_int();
        vx_config_.scale = this->get_parameter("velocity.vx.scale").as_double();
        vx_config_.deadzone = this->get_parameter("velocity.vx.deadzone").as_double();
        vx_config_.inverted = this->get_parameter("velocity.vx.inverted").as_bool();
        
        vy_config_.axis = this->get_parameter("velocity.vy.axis").as_int();
        vy_config_.scale = this->get_parameter("velocity.vy.scale").as_double();
        vy_config_.deadzone = this->get_parameter("velocity.vy.deadzone").as_double();
        vy_config_.inverted = this->get_parameter("velocity.vy.inverted").as_bool();
        
        wz_config_.axis = this->get_parameter("velocity.wz.axis").as_int();
        wz_config_.scale = this->get_parameter("velocity.wz.scale").as_double();
        wz_config_.deadzone = this->get_parameter("velocity.wz.deadzone").as_double();
        wz_config_.inverted = this->get_parameter("velocity.wz.inverted").as_bool();
        
        // Load mode mappings
        autonomy_config_.axis = this->get_parameter("modes.autonomy.axis").as_int();
        autonomy_config_.threshold = this->get_parameter("modes.autonomy.threshold").as_double();
        autonomy_config_.inverted = this->get_parameter("modes.autonomy.inverted").as_bool();
        
        manual_config_.axis = this->get_parameter("modes.manual.axis").as_int();
        manual_config_.threshold = this->get_parameter("modes.manual.threshold").as_double();
        manual_config_.inverted = this->get_parameter("modes.manual.inverted").as_bool();
        
        // Load action mappings
        clear_cloud_button_ = this->get_parameter("actions.clear_cloud.button").as_int();
        emergency_stop_button_ = this->get_parameter("actions.emergency_stop.button").as_int();
        
        // Create publisher
        robot_cmd_pub_ = this->create_publisher<joy_command_mapper::msg::RobotCommand>("/robot_command", 10);
        
        // Subscribe to joy topic
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10, std::bind(&JoyCommandMapper::joyCallback, this, std::placeholders::_1));
        
        // Create timer for regular publishing
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate_)),
            std::bind(&JoyCommandMapper::publishCommand, this));
        
        // Initialize state
        last_joy_time_ = this->now();
        joy_timeout_ = 0.5; // seconds
        
        // Initialize button states for edge detection
        prev_clear_cloud_ = false;
        prev_emergency_stop_ = false;
        
        RCLCPP_INFO(this->get_logger(), 
            "Joy Command Mapper initialized with %s controller configuration", 
            controller_type_.c_str());
    }

private:
    struct AxisConfig {
        int axis;
        double scale;
        double deadzone;
        bool inverted;
    };
    
    struct ModeConfig {
        int axis;
        double threshold;
        bool inverted;
    };
    
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        joy_msg_ = msg;
        last_joy_time_ = this->now();
    }
    
    double applyDeadzone(double value, double deadzone)
    {
        if (std::abs(value) < deadzone) {
            return 0.0;
        }
        return value;
    }
    
    double getAxisValue(const sensor_msgs::msg::Joy::SharedPtr& joy, const AxisConfig& config)
    {
        if (!joy || config.axis >= static_cast<int>(joy->axes.size())) {
            return 0.0;
        }
        
        double value = joy->axes[config.axis];
        value = applyDeadzone(value, config.deadzone);
        
        if (config.inverted) {
            value = -value;
        }
        
        return value * config.scale;
    }
    
    bool getModeValue(const sensor_msgs::msg::Joy::SharedPtr& joy, const ModeConfig& config)
    {
        if (!joy || config.axis >= static_cast<int>(joy->axes.size())) {
            return false;
        }
        
        double value = joy->axes[config.axis];
        bool active = (value > config.threshold);
        
        if (config.inverted) {
            active = !active;
        }
        
        return active;
    }
    
    bool getButtonValue(const sensor_msgs::msg::Joy::SharedPtr& joy, int button)
    {
        if (!joy || button >= static_cast<int>(joy->buttons.size())) {
            return false;
        }
        return joy->buttons[button] > 0;
    }
    
    void publishCommand()
    {
        auto robot_cmd = joy_command_mapper::msg::RobotCommand();
        robot_cmd.header.stamp = this->now();
        robot_cmd.header.frame_id = "joy_command_mapper";
        robot_cmd.source = "joystick";
        
        // Check for joy timeout
        double time_since_joy = (this->now() - last_joy_time_).seconds();
        if (time_since_joy > joy_timeout_ || !joy_msg_) {
            return; // No message, no publishing
        } else {
            // Process velocity commands
            robot_cmd.vx = getAxisValue(joy_msg_, vx_config_) * max_linear_speed_;
            robot_cmd.vy = getAxisValue(joy_msg_, vy_config_) * max_linear_speed_;
            robot_cmd.wz = getAxisValue(joy_msg_, wz_config_) * max_angular_speed_;
            
            // Process mode switches
            robot_cmd.autonomy_mode = getModeValue(joy_msg_, autonomy_config_);
            robot_cmd.manual_mode = getModeValue(joy_msg_, manual_config_);
            
            // Obstacle check is enabled by default, disabled in manual mode
            robot_cmd.obstacle_check_enabled = !robot_cmd.manual_mode;
            
            // Process action triggers (edge detection for buttons)
            bool clear_cloud_pressed = getButtonValue(joy_msg_, clear_cloud_button_);
            bool emergency_stop_pressed = getButtonValue(joy_msg_, emergency_stop_button_);

            // Trigger actions on rising edge only
            robot_cmd.clear_cloud = clear_cloud_pressed && !prev_clear_cloud_;
            robot_cmd.emergency_stop = emergency_stop_pressed && !prev_emergency_stop_;

            // Update previous states
            prev_clear_cloud_ = clear_cloud_pressed;
            prev_emergency_stop_ = emergency_stop_pressed;
            
            // Calculate deadman switch (confidence based on trigger positions)
            // Using the maximum of the mode triggers as confidence
            double autonomy_value = joy_msg_->axes[autonomy_config_.axis];
            double manual_value = joy_msg_->axes[manual_config_.axis];
            robot_cmd.deadman_switch = std::max(
                std::abs(autonomy_value), 
                std::abs(manual_value)
            );        
            // Publish RobotCommand
            robot_cmd_pub_->publish(robot_cmd);
        }
    }
    
    // Configuration
    std::string controller_type_;
    double publish_rate_;
    double max_linear_speed_;
    double max_angular_speed_;
    double joy_timeout_;
    
    AxisConfig vx_config_, vy_config_, wz_config_;
    ModeConfig autonomy_config_, manual_config_;
    int clear_cloud_button_, emergency_stop_button_;

    // State
    sensor_msgs::msg::Joy::SharedPtr joy_msg_;
    rclcpp::Time last_joy_time_;
    bool prev_clear_cloud_, prev_emergency_stop_;
    
    // ROS interfaces
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<joy_command_mapper::msg::RobotCommand>::SharedPtr robot_cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoyCommandMapper>());
    rclcpp::shutdown();
    return 0;
}