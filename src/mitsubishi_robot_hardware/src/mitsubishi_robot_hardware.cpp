#include "mitsubishi_robot_hardware/mitsubishi_robot_hardware.hpp"
#include "rclcpp/rclcpp.hpp"

namespace MitsubishiRobot
{
    CallbackReturn RobotSystem::on_init(const hardware_interface::HardwareInfo &info)
    {
        RCLCPP_INFO(rclcpp::get_logger("mitsubishi_robot_hardware"), "on_init");
        if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
        {
            return CallbackReturn::ERROR;
        }

        // State interfaces
        this->positions_.assign(NUM_JOINTS, 0);

        // Command interfaces
        this->joint_command_.assign(NUM_JOINTS, 0);
        this->joint_command_prev_.assign(NUM_JOINTS, 0);

        // Hardware parameters
        this->robot_password_ = info.hardware_parameters.at("robot_password");
        this->host_addr_ = info.hardware_parameters.at("robot_ip");
        this->ctrl_port_ = info.hardware_parameters.at("control_port");
        this->mxt_port_ = info.hardware_parameters.at("robot_port");
        RCLCPP_INFO(rclcpp::get_logger("mitsubishi_robot_hardware"), "Robot password: %s", this->robot_password_.c_str());
        RCLCPP_INFO(rclcpp::get_logger("mitsubishi_robot_hardware"), "Host (robot) address: %s", this->host_addr_.c_str());
        RCLCPP_INFO(rclcpp::get_logger("mitsubishi_robot_hardware"), "Control port: %s", this->ctrl_port_.c_str());
        RCLCPP_INFO(rclcpp::get_logger("mitsubishi_robot_hardware"), "MXT port: %s", this->mxt_port_.c_str());
        RCLCPP_INFO(rclcpp::get_logger("mitsubishi_robot_hardware"), "Packet size: %lu", sizeof(Packet));

        this->first_read_ = true;

        // Make boost UDP objects
        try
        {
            // Sockets
            this->io_service_ = std::make_unique<boost::asio::io_service>();

            // Make sockets
            this->ctrl_socket_ = std::make_unique<boost::asio::ip::tcp::socket>(*this->io_service_);
            this->mxt_socket_ = std::make_unique<boost::asio::ip::udp::socket>(*this->io_service_);

            // TCP for control port
            boost::asio::ip::tcp::resolver tcp_resolver(*this->io_service_);
            boost::asio::ip::tcp::resolver::query tcp_query(boost::asio::ip::tcp::v4(), this->host_addr_, this->ctrl_port_);
            boost::asio::connect(*this->ctrl_socket_, tcp_resolver.resolve(tcp_query));

            // UDP for real-time control port
            // boost::asio::ip::udp::resolver udp_resolver(*this->io_service_);
            // boost::asio::ip::udp::resolver::query udp_query(boost::asio::ip::udp::v4(), this->host_addr_, this->mxt_port_);
            // connect(*this->mxt_socket_, udp_resolver.resolve(udp_query));
            boost::asio::ip::udp::resolver udp_resolver(*this->io_service_);
            boost::asio::ip::udp::resolver::query udp_query(boost::asio::ip::udp::v4(), this->host_addr_, this->mxt_port_);
            auto endpoints = udp_resolver.resolve(udp_query);
            for (auto const& ep : endpoints)
            {
                RCLCPP_INFO(rclcpp::get_logger("mitsubishi_robot_hardware"),
                            "Resolved UDP endpoint: %s:%d",
                            ep.endpoint().address().to_string().c_str(),
                            ep.endpoint().port());
            }

            // Explicitly open and bind the UDP socket to an ephemeral local port.
            this->mxt_socket_->open(boost::asio::ip::udp::v4());
            boost::asio::ip::udp::endpoint local_endpoint(boost::asio::ip::udp::v4(), 0);
            this->mxt_socket_->bind(local_endpoint);

            // Connect the UDP socket to the first resolved remote endpoint.
            boost::asio::ip::udp::endpoint remote_endpoint = *endpoints.begin();
            this->mxt_socket_->connect(remote_endpoint);
        }
        catch (std::exception &e)
        {
            RCLCPP_FATAL(rclcpp::get_logger("mitsubishi_robot_hardware"), "Failure creating communication sockets: %s", e.what());
            return CallbackReturn::FAILURE;
        }

        // Add interfaces
        for (const auto &joint : info.joints)
        {
            for (const auto &interface : joint.state_interfaces)
            {
                this->joint_interfaces_[interface.name].push_back(joint.name);
            }
        }

        rclcpp::on_shutdown(std::bind(&MitsubishiRobot::RobotSystem::on_deactivate, this, rclcpp_lifecycle::State()));

        return CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> RobotSystem::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;

        uint8_t i(0);
        for (const std::string &joint_name : this->joint_interfaces_["position"])
        {
            RCLCPP_INFO(rclcpp::get_logger("mitsubishi_robot_hardware"), "Exporting position state interface for joint '%s'", joint_name.c_str());
            state_interfaces.emplace_back(joint_name, "position", &this->positions_[i++]);
        }
        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> RobotSystem::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;

        uint8_t i(0);
        for (const std::string &joint_name : this->joint_interfaces_["position"])
        {
            RCLCPP_INFO(rclcpp::get_logger("mitsubishi_robot_hardware"), "Exporting position command interface for joint '%s'", joint_name.c_str());
            command_interfaces.emplace_back(joint_name, "position", &this->joint_command_[i++]);
        }
        return command_interfaces;
    }

    CallbackReturn RobotSystem::on_activate(const rclcpp_lifecycle::State &)
    {

        RCLCPP_INFO(rclcpp::get_logger("mitsubishi_robot_hardware"), "Activating robot");

        // // Send password
        // const char *buf = this->robot_password_.c_str();
        // boost::asio::write(*(this->ctrl_socket_), boost::asio::buffer(buf, strlen(buf)));

        // // Wait for response
        // boost::asio::streambuf reply;
        // boost::asio::read_until(*this->ctrl_socket_, reply, '\r');

        // // Check if valid
        // std::istream convert(&reply);
        // int rc(0);
        // convert >> rc;

        // if (rc != 1)
        // {
        //     RCLCPP_FATAL(rclcpp::get_logger("mitsubishi_robot_hardware"), "Failure starting robot");
        //     return CallbackReturn::FAILURE;
        // }

        this->first_read_ = false;
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn RobotSystem::on_deactivate(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(rclcpp::get_logger("mitsubishi_robot_hardware"), "Deactivating robot");

        // // Send password
        // Packet end_command;
        // memset((uint8_t *)(uint8_t *)&end_command, 0, sizeof(end_command));
        // end_command.command = Packet::Command::END;

        // Transmit
        // this->mxt_socket_->send(boost::asio::buffer((char *)&end_command, sizeof(end_command)));
        
        return CallbackReturn::SUCCESS;
    }

    return_type RobotSystem::read(const rclcpp::Time &, const rclcpp::Duration &)
    {
        if (this->first_read_)
        {
            usleep(7e7);
        }

        RCLCPP_DEBUG(rclcpp::get_logger("mitsubishi_robot_hardware"), "RobotSystem::read(...): Crafting read joints request");
        Packet joint_request, joint_response;
        memset((uint8_t *)&joint_request, 0, sizeof(joint_request));
        joint_request.reply_0.type = Packet::Type::JOINT;
        joint_request.reply_1.type = Packet::Type::XYZ;
        joint_request.reply_2.type = Packet::Type::PERCENT_CURRENT_FEEDBACK;
        joint_request.reply_3.type = Packet::Type::ENCODER_JOINT;

        RCLCPP_DEBUG(rclcpp::get_logger("mitsubishi_robot_hardware"), "RobotSystem::read(...): Sending read joints request");
        this->mxt_socket_->send(boost::asio::buffer((char *)&joint_request, sizeof(joint_request)));
        RCLCPP_DEBUG(rclcpp::get_logger("mitsubishi_robot_hardware"), "RobotSystem::read(...): Sent read joints request");
        this->mxt_socket_->receive(boost::asio::buffer((char *)&joint_response, sizeof(joint_response)));
        RCLCPP_DEBUG(rclcpp::get_logger("mitsubishi_robot_hardware"), "RobotSystem::read(...): Received joint values");


        // Joint pos
        Packet::Joint &joint(joint_response.reply_0.data.joint);
        for (uint8_t i(0); i < NUM_JOINTS; ++i)
        {
            this->positions_[i] = joint.j[i];
        }

        if (this->first_read_)
        {
            this->joint_command_ = this->positions_;
            this->joint_command_prev_ = this->positions_;
            this->first_read_ = false;
        }

        return return_type::OK;
    }

    return_type RobotSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
    {
        // RCLCPP_DEBUG(rclcpp::get_logger("mitsubishi_robot_hardware"), "Command: %f %f %f %f %f %f", this->joint_command_[0], this->joint_command_[1], this->joint_command_[2], this->joint_command_[3], this->joint_command_[4], this->joint_command_[5]);
        const float epsilon(0.00001);
        uint8_t num_same(0);
        for (uint8_t i(0); i < NUM_JOINTS; ++i)
        {
            bool same(std::fabs(this->joint_command_prev_[i] - this->joint_command_[i]) < epsilon);
            if (same)
            {
                ++num_same;
            }
        }

        if (num_same == NUM_JOINTS)
        {
            // RCLCPP_DEBUG(rclcpp::get_logger("mitsubishi_robot_hardware"), "Issued command same as previous, skipping");
            return return_type::OK;
        }

        // Make packet
        Packet joint_request, joint_response;
        memset((uint8_t *)&joint_request, 0, sizeof(joint_request));
        joint_request.command = Packet::Command::VALID;
        joint_request.send_type = Packet::Type::JOINT;
        joint_request.reply_0.type = Packet::Type::JOINT;
        std::copy(this->joint_command_.begin(), this->joint_command_.end(), joint_request.reply_0.data.joint.j);

        this->mxt_socket_->send(boost::asio::buffer((char *)&joint_request, sizeof(joint_request)));
        this->mxt_socket_->receive(boost::asio::buffer((char *)&joint_response, sizeof(joint_response)));

        this->joint_command_prev_ = this->joint_command_;
        return return_type::OK;
    }
}

// "Finally, all ros2_control plugins should have the following two lines of code at the end of the file"
// Source: https://control.ros.org/iron/doc/ros2_control_demos/example_7/doc/userdoc.html
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(MitsubishiRobot::RobotSystem, hardware_interface::SystemInterface)
