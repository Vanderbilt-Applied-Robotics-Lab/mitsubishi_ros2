// Mitsubishi robot hardware interface
//
// Author: Joshua Holden Turner, 07/2024

#ifndef MITSUBISHI_ROBOT_HARDWARE_HPP
#define MITSUBISHI_ROBOT_HARDWARE_HPP

// ROS2 hardware_interface
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

// Networking
#include <netdb.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <boost/asio.hpp>

// Data packet
#include "mitsubishi_robot_hardware/packet.hpp"

using hardware_interface::return_type;

namespace MitsubishiRobot
{

    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    class HARDWARE_INTERFACE_PUBLIC RobotSystem : public hardware_interface::SystemInterface
    {
    public:
        /* -------------------------------------------------------------------------- */
        /*                               ROS2 Functions                               */
        /* -------------------------------------------------------------------------- */

        /**
         * @brief Communication between the Mitsubishi robot hardware need to be initialized during this step.
         *
         * @param hardware_interface::HardwareInfo& info, Hardware info from URDF
         * @return CallbackReturn, success of callback
         */
        CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;

        /**
         * @brief Exports the interfaces for various states. In regards to the Mitsubishi robot, this is
         *  the position, velocity, torque, etc. as defined in the URDF and as sent by the controller
         *
         * @return std::vector<hardware_interface::StateInterface>, Vector of read-only data handles
         */
        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

        /**
         * @brief Exports the command interfaces for each joint. In regards to the Mitsubishi robot, this
         * is the position, velocity, and torques, etc. as defined in the URDF and as allowed by the
         * controller
         *
         * @return std::vector<hardware_interface::CommandInterface>, Vector of command interface handles
         */
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        /**
         * @brief Reads data from the Mitsubishi robot and updates state interface values accordingly.
         *
         * @return return_type, Status of reading from UDP
         */
        return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;

        /**
         * @brief Write data to the Mitsubishi robot
         *
         * @return return_type, Status of writing to UDP
         */
        return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;

        /**
         * @brief Called on robot activation
         *
         * @param previous_state
         * @return CallbackReturn
         */
        CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;

        /**
         * @brief Called on robot deactivation
         *
         * @param previous_state
         * @return CallbackReturn
         */
        CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;

    private:
        /* -------------------------------------------------------------------------- */
        /*                                    Data                                    */
        /* -------------------------------------------------------------------------- */
        /* --------------------------- Static data members -------------------------- */
        static const constexpr uint8_t NUM_JOINTS{6};
        /* --------------------------- Non-static members --------------------------- */
        std::string robot_password_;
        std::string host_addr_;
        std::string ctrl_port_;
        std::string mxt_port_;
        std::vector<double> positions_;
        std::vector<double> joint_command_;
        std::vector<double> joint_command_prev_;
        std::unique_ptr<boost::asio::io_service> io_service_;
        std::unique_ptr<boost::asio::ip::tcp::socket> ctrl_socket_;
        std::unique_ptr<boost::asio::ip::udp::socket> mxt_socket_;
        std::unique_ptr<boost::asio::ip::udp::endpoint> host_endpoint_;
        std::unordered_map<std::string, std::vector<std::string>> joint_interfaces_{{"position", {}}};
        bool first_read_;
    };

}

#endif // MITSUBISHI_ROBOT_HARDWARE_HPP
