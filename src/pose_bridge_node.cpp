#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <atomic>
#include <cerrno>
#include <cmath>
#include <cctype>
#include <cstring>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace {

    constexpr double kPi = 3.14159265358979323846;
    constexpr std::size_t kMaxDatagramSize = 512;

    enum HandIndex { LEFT = 0, RIGHT = 1, HAND_COUNT = 2 };
    enum TrackerIndex { WAIST = 0, LEFT_FOOT = 1, RIGHT_FOOT = 2, HEAD = 3, TRACKER_COUNT = 4 };

    struct TrackerSample {
        TrackerIndex tracker = WAIST;
        double position[3]{0.0, 0.0, 0.0};
        double orientation[4]{0.0, 0.0, 0.0, 1.0};  // x, y, z, w
        int64_t timestamp_ns = 0;
    };

    struct PoseSample {
        HandIndex hand = LEFT;
        double position[3]{0.0, 0.0, 0.0};
        double orientation[4]{0.0, 0.0, 0.0, 1.0};  // x, y, z, w
        double trigger_value = 0.0;  // 食指扳机值 (0-1)
        double grip_value = 0.0;     // 握把扳机值 (0-1)
        bool button_a = false;       // A键状态（右手柄）
        bool button_b = false;       // B键状态（右手柄）
        bool button_x = false;       // X键状态（左手柄）
        bool button_y = false;       // Y键状态（左手柄）
        bool joystick_click = false; // 摇杆按下状态
        double joystick_x = 0.0;     // 摇杆X轴值 (-1到1)
        double joystick_y = 0.0;     // 摇杆Y轴值 (-1到1)
        double rate = 0.1;           // 倍率（0.1或1.0）
        int64_t timestamp_ns = 0;
    };

    enum class ButtonId { A = 0, B = 1, X = 2, Y = 3, JOYSTICK = 4 };

    bool stringEqualsIgnoreCase(const std::string &a, const std::string &b) {
        if (a.size() != b.size()) {
            return false;
        }
        for (std::size_t i = 0; i < a.size(); ++i) {
            if (std::tolower(a[i]) != std::tolower(b[i])) {
                return false;
            }
        }
        return true;
    }

}  // namespace

class PoseBridgeNode : public rclcpp::Node {
public:
    PoseBridgeNode() : Node("pico_pose_bridge"), running_(true) {
        listen_address_ = declare_parameter<std::string>("listen_address", "0.0.0.0");
        listen_port_ = declare_parameter<int>("listen_port", 5100);
        frame_id_ = declare_parameter<std::string>("frame_id", "pico_hmd");
        child_frame_ids_[LEFT] =
                declare_parameter<std::string>("left_child_frame_id", "pico_left_controller");
        child_frame_ids_[RIGHT] =
                declare_parameter<std::string>("right_child_frame_id", "pico_right_controller");
        pose_topics_[LEFT] =
                declare_parameter<std::string>("left_pose_topic", "/pico_left_controller/pose");
        pose_topics_[RIGHT] =
                declare_parameter<std::string>("right_pose_topic", "/pico_right_controller/pose");
        trigger_topics_[LEFT] =
                declare_parameter<std::string>("left_trigger_topic", "/pico_left_controller/trigger");
        trigger_topics_[RIGHT] =
                declare_parameter<std::string>("right_trigger_topic", "/pico_right_controller/trigger");
        grip_topics_[LEFT] =  declare_parameter<std::string>("left_grip_topic", "/pico_left_controller/grip");
        grip_topics_[RIGHT] = declare_parameter<std::string>("right_grip_topic", "/pico_right_controller/grip");
        joystick_x_topics_[LEFT] = declare_parameter<std::string>("left_joystick_x_topic", "/pico_left_controller/joystick_x");
        joystick_x_topics_[RIGHT] = declare_parameter<std::string>("right_joystick_x_topic", "/pico_right_controller/joystick_x");
        joystick_y_topics_[LEFT] = declare_parameter<std::string>("left_joystick_y_topic", "/pico_left_controller/joystick_y");
        joystick_y_topics_[RIGHT] = declare_parameter<std::string>("right_joystick_y_topic", "/pico_right_controller/joystick_y");
        button_a_topic_ = declare_parameter<std::string>("button_a_topic", "pico_right_controller/button_a");
        button_b_topic_ = declare_parameter<std::string>("button_b_topic", "pico_right_controller/button_b");
        button_x_topic_ = declare_parameter<std::string>("button_x_topic", "pico_left_controller/button_x");
        button_y_topic_ = declare_parameter<std::string>("button_y_topic", "pico_left_controller/button_y");
        joystick_click_topics_[LEFT] =
                declare_parameter<std::string>("left_joystick_click_topic", "pico_left_controller/joystick_click");
        joystick_click_topics_[RIGHT] =
                declare_parameter<std::string>("right_joystick_click_topic", "pico_right_controller/joystick_click");

        // 体感追踪器话题参数
        tracker_topics_[WAIST] =
                declare_parameter<std::string>("waist_pose_topic", "/pico_tracker/waist/pose");
        tracker_topics_[LEFT_FOOT] =
                declare_parameter<std::string>("left_foot_pose_topic", "/pico_tracker/left_foot/pose");
        tracker_topics_[RIGHT_FOOT] =
                declare_parameter<std::string>("right_foot_pose_topic", "/pico_tracker/right_foot/pose");
        tracker_topics_[HEAD] =
                declare_parameter<std::string>("head_pose_topic", "/pico_head/pose");
        // Use absolute topic names by default so they appear as /pico_* instead of being prefixed by the node name
        rate_topics_[LEFT] = declare_parameter<std::string>("left_rate_topic", "/pico_left_controller/rate");
        rate_topics_[RIGHT] = declare_parameter<std::string>("right_rate_topic", "/pico_right_controller/rate");
        chassis_linear_speed_topic_ =
                declare_parameter<std::string>("chassis_linear_speed_topic", "/pico_chassis/max_linear_speed");
        chassis_angular_speed_topic_ =
                declare_parameter<std::string>("chassis_angular_speed_topic", "/pico_chassis/max_angular_speed");
        RCLCPP_INFO(get_logger(), "DEBUG: Rate topics - LEFT: '%s', RIGHT: '%s'",
                    rate_topics_[LEFT].c_str(), rate_topics_[RIGHT].c_str());
        // 默认只发送左右手姿态四元数、扳机和按键，不再发布 TF（如需 TF，可通过参数开启）
        publish_tf_ = declare_parameter<bool>("publish_tf", false);

        for (int i = 0; i < HAND_COUNT; ++i) {
            latest_samples_[i].hand = static_cast<HandIndex>(i);
            // 仅发布位姿（位置 + 四元数）、扳机，不再发布欧拉角 rpy
            pose_publishers_[i] = create_publisher<geometry_msgs::msg::PoseStamped>(pose_topics_[i], 10);
            trigger_publishers_[i] =
                    create_publisher<std_msgs::msg::Float32>(trigger_topics_[i], 10);
            grip_publishers_[i] =
                    create_publisher<std_msgs::msg::Float32>(grip_topics_[i], 10);
            joystick_x_publishers_[i] =
                    create_publisher<std_msgs::msg::Float32>(joystick_x_topics_[i], 10);
            joystick_y_publishers_[i] =
                    create_publisher<std_msgs::msg::Float32>(joystick_y_topics_[i], 10);
            RCLCPP_INFO(get_logger(), "DEBUG: Creating rate publisher for '%s'", rate_topics_[i].c_str());
            rate_publishers_[i] = create_publisher<std_msgs::msg::Float32>(rate_topics_[i], 10);
            // 检查是否创建成功
            if (rate_publishers_[i]) {
                RCLCPP_INFO(get_logger(), "DEBUG: ✓ Rate publisher created successfully");
            } else {
                RCLCPP_ERROR(get_logger(), "DEBUG: ✗ Rate publisher creation FAILED!");
            }
        }
        button_a_publisher_ = create_publisher<std_msgs::msg::Bool>(button_a_topic_, 10);
        button_b_publisher_ = create_publisher<std_msgs::msg::Bool>(button_b_topic_, 10);
        button_x_publisher_ = create_publisher<std_msgs::msg::Bool>(button_x_topic_, 10);
        button_y_publisher_ = create_publisher<std_msgs::msg::Bool>(button_y_topic_, 10);
        chassis_linear_speed_publisher_ =
                create_publisher<std_msgs::msg::Float32>(chassis_linear_speed_topic_, 10);
        chassis_angular_speed_publisher_ =
                create_publisher<std_msgs::msg::Float32>(chassis_angular_speed_topic_, 10);
        joystick_click_publishers_[LEFT] =
                create_publisher<std_msgs::msg::Bool>(joystick_click_topics_[LEFT], 10);
        joystick_click_publishers_[RIGHT] =
                create_publisher<std_msgs::msg::Bool>(joystick_click_topics_[RIGHT], 10);

        // 体感追踪器 publisher
        for (int i = 0; i < TRACKER_COUNT; ++i) {
            tracker_pose_publishers_[i] =
                    create_publisher<geometry_msgs::msg::PoseStamped>(tracker_topics_[i], 10);
        }
        RCLCPP_INFO(get_logger(), "Tracker publishers: WAIST='%s', LEFT_FOOT='%s', RIGHT_FOOT='%s', HEAD='%s'",
                    tracker_topics_[WAIST].c_str(), tracker_topics_[LEFT_FOOT].c_str(),
                    tracker_topics_[RIGHT_FOOT].c_str(), tracker_topics_[HEAD].c_str());

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        receiver_thread_ = std::thread(&PoseBridgeNode::receiverLoop, this);

        // 添加调试输出
        RCLCPP_INFO(get_logger(), "========== Publisher Creation Summary ==========");
        for (int i = 0; i < HAND_COUNT; ++i) {
            std::string hand_name = (i == LEFT) ? "LEFT" : "RIGHT";
            RCLCPP_INFO(get_logger(), "%s Hand:", hand_name.c_str());
            RCLCPP_INFO(get_logger(), "  Pose: %s - %s",
                        pose_topics_[i].c_str(),
                        pose_publishers_[i] ? "CREATED" : "FAILED");
            RCLCPP_INFO(get_logger(), "  Rate: %s - %s",
                        rate_topics_[i].c_str(),
                        rate_publishers_[i] ? "CREATED" : "FAILED");
        }
        RCLCPP_INFO(get_logger(), "================================================");
    }

    ~PoseBridgeNode() override {
        running_.store(false);
        if (socket_fd_ >= 0) {
            ::shutdown(socket_fd_, SHUT_RDWR);
            ::close(socket_fd_);
            socket_fd_ = -1;
        }
        if (receiver_thread_.joinable()) {
            receiver_thread_.join();
        }
    }

private:
    void receiverLoop() {
        if (!openSocket()) {
            RCLCPP_ERROR(get_logger(), "Failed to initialize UDP socket. Receiver thread exiting.");
            return;
        }

        while (rclcpp::ok() && running_.load()) {
            sockaddr_in remote_addr{};
            socklen_t addr_len = sizeof(remote_addr);
            char buffer[kMaxDatagramSize];
            const ssize_t received = ::recvfrom(socket_fd_, buffer, sizeof(buffer) - 1, 0,
                                                reinterpret_cast<sockaddr *>(&remote_addr), &addr_len);

            if (received < 0) {
                if (!running_.load()) {
                    break;
                }
                if (errno == EAGAIN || errno == EWOULDBLOCK) {
                    continue;
                }
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                     "recvfrom failed: %s", std::strerror(errno));
                continue;
            }

            buffer[received] = '\0';
            std::string datagram(buffer, received);
            char remote_ip[INET_ADDRSTRLEN] = {0};
            const char *remote_ip_ptr =
                    ::inet_ntop(AF_INET, &remote_addr.sin_addr, remote_ip, sizeof(remote_ip));
            const uint16_t remote_port = ntohs(remote_addr.sin_port);
            if (remote_ip_ptr != nullptr) {
                const std::string remote_ip_str(remote_ip_ptr);
                if (remote_ip_str != last_sender_ip_ || remote_port != last_sender_port_) {
                    last_sender_ip_ = remote_ip_str;
                    last_sender_port_ = remote_port;
                    RCLCPP_INFO(get_logger(), "收到来自 %s:%u 的 PICO UDP 数据",
                                last_sender_ip_.c_str(), last_sender_port_);
                }
            }

            // 先尝试解析为体感追踪器/头显报文（WAIST/LEFT_FOOT/RIGHT_FOOT/HEAD）
            TrackerSample tracker_sample;
            if (parseTrackerDatagram(datagram, tracker_sample)) {
                publishTrackerSample(tracker_sample);
                continue;
            }

            if (handleModernControllerDatagram(datagram)) {
                continue;
            }

            // 否则按手柄报文解析
            PoseSample sample;
            if (!parseDatagram(datagram, sample)) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                     "Failed to parse datagram: '%s'", buffer);
                continue;
            }

            RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000,
                                  "RAW UDP data - Hand: %s, pos=(%.4f,%.4f,%.4f) (no transform applied)",
                                  sample.hand == LEFT ? "LEFT" : "RIGHT",
                                  sample.position[0], sample.position[1], sample.position[2]);

            publishSample(sample);
        }
    }

    bool openSocket() {
        socket_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
        if (socket_fd_ < 0) {
            RCLCPP_ERROR(get_logger(), "Unable to create UDP socket: %s", std::strerror(errno));
            return false;
        }

        const int reuse = 1;
        if (::setsockopt(socket_fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) < 0) {
            RCLCPP_WARN(get_logger(), "Failed to set SO_REUSEADDR: %s", std::strerror(errno));
        }

        timeval timeout{};
        timeout.tv_sec = 1;
        timeout.tv_usec = 0;
        if (::setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) < 0) {
            RCLCPP_WARN(get_logger(), "Failed to set SO_RCVTIMEO: %s", std::strerror(errno));
        }

        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(static_cast<uint16_t>(listen_port_));
        if (listen_address_ == "0.0.0.0") {
            addr.sin_addr.s_addr = INADDR_ANY;
        } else if (::inet_pton(AF_INET, listen_address_.c_str(), &addr.sin_addr) != 1) {
            RCLCPP_ERROR(get_logger(), "Invalid listen_address '%s'", listen_address_.c_str());
            return false;
        }

        if (::bind(socket_fd_, reinterpret_cast<const sockaddr *>(&addr), sizeof(addr)) < 0) {
            RCLCPP_ERROR(get_logger(), "Failed to bind UDP socket to %s:%d -> %s",
                         listen_address_.c_str(), listen_port_, std::strerror(errno));
            return false;
        }

        RCLCPP_INFO(get_logger(), "Listening for controller poses on %s:%d",
                    listen_address_.c_str(), listen_port_);
        return true;
    }

    bool parseHandToken(const std::string &token, HandIndex &hand) const {
        if (stringEqualsIgnoreCase(token, "L") || stringEqualsIgnoreCase(token, "LEFT")) {
            hand = LEFT;
            return true;
        }
        if (stringEqualsIgnoreCase(token, "R") || stringEqualsIgnoreCase(token, "RIGHT")) {
            hand = RIGHT;
            return true;
        }
        return false;
    }

    bool parseButtonId(const std::string &token, ButtonId &button_id) const {
        if (stringEqualsIgnoreCase(token, "A")) {
            button_id = ButtonId::A;
            return true;
        }
        if (stringEqualsIgnoreCase(token, "B")) {
            button_id = ButtonId::B;
            return true;
        }
        if (stringEqualsIgnoreCase(token, "X")) {
            button_id = ButtonId::X;
            return true;
        }
        if (stringEqualsIgnoreCase(token, "Y")) {
            button_id = ButtonId::Y;
            return true;
        }
        if (stringEqualsIgnoreCase(token, "J") ||
            stringEqualsIgnoreCase(token, "JOY") ||
            stringEqualsIgnoreCase(token, "STICK") ||
            stringEqualsIgnoreCase(token, "JOYSTICK")) {
            button_id = ButtonId::JOYSTICK;
            return true;
        }
        return false;
    }

    rclcpp::Time resolveStamp(int64_t timestamp_ns) const {
        return timestamp_ns > 0 ? rclcpp::Time(timestamp_ns) : now();
    }

    void publishPoseMessage(const PoseSample &sample, const rclcpp::Time &stamp) {
        auto pose_msg = geometry_msgs::msg::PoseStamped();
        pose_msg.header.stamp = stamp;
        pose_msg.header.frame_id = frame_id_;
        pose_msg.pose.position.x = sample.position[0];
        pose_msg.pose.position.y = sample.position[1];
        pose_msg.pose.position.z = sample.position[2];
        pose_msg.pose.orientation.x = sample.orientation[0];
        pose_msg.pose.orientation.y = sample.orientation[1];
        pose_msg.pose.orientation.z = sample.orientation[2];
        pose_msg.pose.orientation.w = sample.orientation[3];

        pose_publishers_[sample.hand]->publish(pose_msg);

        if (publish_tf_) {
            geometry_msgs::msg::TransformStamped tf_msg;
            tf_msg.header = pose_msg.header;
            tf_msg.child_frame_id = child_frame_ids_[sample.hand];
            tf_msg.transform.translation.x = pose_msg.pose.position.x;
            tf_msg.transform.translation.y = pose_msg.pose.position.y;
            tf_msg.transform.translation.z = pose_msg.pose.position.z;
            tf_msg.transform.rotation = pose_msg.pose.orientation;
            tf_broadcaster_->sendTransform(tf_msg);

            geometry_msgs::msg::TransformStamped world_tf_msg;
            world_tf_msg.header.stamp = stamp;
            world_tf_msg.header.frame_id = "world";
            world_tf_msg.child_frame_id =
                    (sample.hand == LEFT) ? "left_ee_from_world" : "right_ee_from_world";
            world_tf_msg.transform.translation.x = sample.position[0];
            world_tf_msg.transform.translation.y = sample.position[1];
            world_tf_msg.transform.translation.z = sample.position[2];
            world_tf_msg.transform.rotation.x = sample.orientation[0];
            world_tf_msg.transform.rotation.y = sample.orientation[1];
            world_tf_msg.transform.rotation.z = sample.orientation[2];
            world_tf_msg.transform.rotation.w = sample.orientation[3];
            tf_broadcaster_->sendTransform(world_tf_msg);
        }
    }

    void publishTriggerMessage(HandIndex hand, double trigger_value) {
        auto trigger_msg = std_msgs::msg::Float32();
        trigger_msg.data = static_cast<float>(trigger_value);
        trigger_publishers_[hand]->publish(trigger_msg);
    }

    void publishGripMessage(HandIndex hand, double grip_value) {
        auto grip_msg = std_msgs::msg::Float32();
        grip_msg.data = static_cast<float>(grip_value);
        grip_publishers_[hand]->publish(grip_msg);
    }

    void publishJoystickMessage(HandIndex hand, double joystick_x, double joystick_y) {
        auto joystick_x_msg = std_msgs::msg::Float32();
        joystick_x_msg.data = static_cast<float>(joystick_x);
        joystick_x_publishers_[hand]->publish(joystick_x_msg);

        auto joystick_y_msg = std_msgs::msg::Float32();
        joystick_y_msg.data = static_cast<float>(joystick_y);
        joystick_y_publishers_[hand]->publish(joystick_y_msg);
    }

    void publishRateMessage(double rate_value) {
        auto rate_msg = std_msgs::msg::Float32();
        rate_msg.data = static_cast<float>(rate_value);
        rate_publishers_[LEFT]->publish(rate_msg);
        rate_publishers_[RIGHT]->publish(rate_msg);
    }

    void publishChassisLinearSpeedMessage(double speed_value) {
        auto speed_msg = std_msgs::msg::Float32();
        speed_msg.data = static_cast<float>(speed_value);
        chassis_linear_speed_publisher_->publish(speed_msg);
    }

    void publishChassisAngularSpeedMessage(double speed_value) {
        auto speed_msg = std_msgs::msg::Float32();
        speed_msg.data = static_cast<float>(speed_value);
        chassis_angular_speed_publisher_->publish(speed_msg);
    }

    void publishButtonMessage(ButtonId button_id, HandIndex hand, bool pressed) {
        auto button_msg = std_msgs::msg::Bool();
        button_msg.data = pressed;

        switch (button_id) {
            case ButtonId::A:
                button_a_publisher_->publish(button_msg);
                break;
            case ButtonId::B:
                button_b_publisher_->publish(button_msg);
                break;
            case ButtonId::X:
                button_x_publisher_->publish(button_msg);
                break;
            case ButtonId::Y:
                button_y_publisher_->publish(button_msg);
                break;
            case ButtonId::JOYSTICK:
                joystick_click_publishers_[hand]->publish(button_msg);
                break;
        }
    }

    void setButtonState(PoseSample &sample, ButtonId button_id, bool pressed) {
        switch (button_id) {
            case ButtonId::A:
                sample.button_a = pressed;
                break;
            case ButtonId::B:
                sample.button_b = pressed;
                break;
            case ButtonId::X:
                sample.button_x = pressed;
                break;
            case ButtonId::Y:
                sample.button_y = pressed;
                break;
            case ButtonId::JOYSTICK:
                sample.joystick_click = pressed;
                break;
        }
    }

    bool handleModernControllerDatagram(const std::string &payload) {
        std::istringstream iss(payload);
        std::string token;
        if (!(iss >> token)) {
            return false;
        }

        if (stringEqualsIgnoreCase(token, "HAND")) {
            std::string hand_token;
            HandIndex hand;
            if (!(iss >> hand_token) || !parseHandToken(hand_token, hand)) {
                return false;
            }

            auto &sample = latest_samples_[hand];
            sample.hand = hand;
            for (double &component : sample.position) {
                if (!(iss >> component)) {
                    return false;
                }
            }
            for (double &component : sample.orientation) {
                if (!(iss >> component)) {
                    return false;
                }
            }
            if (!(iss >> sample.trigger_value >> sample.grip_value)) {
                return false;
            }

            // Remaining fields vary by Pico app version:
            // - short: rate [timestamp_ns]
            // - extended: button_a button_b button_x button_y joystick_click joystick_x joystick_y rate timestamp_ns
            // - or: joystick_x joystick_y rate [timestamp_ns]
            std::vector<double> tail_values;
            double tail_value = 0.0;
            while (iss >> tail_value) {
                tail_values.push_back(tail_value);
            }

            const auto is_valid_rate = [](double value) {
                return std::abs(value - 0.1) < 1e-6 || std::abs(value - 1.0) < 1e-6;
            };

            sample.button_a = false;
            sample.button_b = false;
            sample.button_x = false;
            sample.button_y = false;
            sample.joystick_click = false;
            sample.joystick_x = 0.0;
            sample.joystick_y = 0.0;
            sample.rate = 0.1;
            sample.timestamp_ns = 0;

            if (tail_values.size() >= 9) {
                sample.button_a = tail_values[0] != 0.0;
                sample.button_b = tail_values[1] != 0.0;
                sample.button_x = tail_values[2] != 0.0;
                sample.button_y = tail_values[3] != 0.0;
                sample.joystick_click = tail_values[4] != 0.0;
                sample.joystick_x = tail_values[5];
                sample.joystick_y = tail_values[6];
                sample.rate = is_valid_rate(tail_values[7]) ? tail_values[7] : 0.1;
                sample.timestamp_ns = static_cast<int64_t>(tail_values[8]);
            } else if (tail_values.size() >= 4) {
                sample.joystick_x = tail_values[0];
                sample.joystick_y = tail_values[1];
                sample.rate = is_valid_rate(tail_values[2]) ? tail_values[2] : 0.1;
                sample.timestamp_ns = static_cast<int64_t>(tail_values[3]);
            } else if (tail_values.size() == 3) {
                sample.joystick_x = tail_values[0];
                sample.joystick_y = tail_values[1];
                sample.rate = is_valid_rate(tail_values[2]) ? tail_values[2] : 0.1;
            } else if (tail_values.size() == 2) {
                if (is_valid_rate(tail_values[0])) {
                    sample.rate = tail_values[0];
                    sample.timestamp_ns = static_cast<int64_t>(tail_values[1]);
                } else {
                    sample.joystick_x = tail_values[0];
                    sample.joystick_y = tail_values[1];
                }
            } else if (tail_values.size() == 1) {
                if (is_valid_rate(tail_values[0])) {
                    sample.rate = tail_values[0];
                } else {
                    sample.timestamp_ns = static_cast<int64_t>(tail_values[0]);
                }
            }

            publishSample(sample);
            return true;
        }

        if (stringEqualsIgnoreCase(token, "RELEASE")) {
            std::string hand_token;
            HandIndex hand;
            if (!(iss >> hand_token) || !parseHandToken(hand_token, hand)) {
                return false;
            }

            auto &sample = latest_samples_[hand];
            sample.hand = hand;
            if (!(iss >> sample.trigger_value >> sample.grip_value >> sample.rate)) {
                return false;
            }
            if (!(iss >> sample.timestamp_ns)) {
                sample.timestamp_ns = 0;
            }

            publishTriggerMessage(hand, sample.trigger_value);
            publishGripMessage(hand, sample.grip_value);
            publishRateMessage(sample.rate);
            return true;
        }

        if (stringEqualsIgnoreCase(token, "BTN")) {
            std::string hand_token;
            std::string button_token;
            int pressed_int = 0;
            HandIndex hand;
            ButtonId button_id;
            if (!(iss >> hand_token >> button_token >> pressed_int) ||
                !parseHandToken(hand_token, hand) ||
                !parseButtonId(button_token, button_id)) {
                return false;
            }

            int64_t timestamp_ns = 0;
            if (!(iss >> timestamp_ns)) {
                timestamp_ns = 0;
            }

            auto &sample = latest_samples_[hand];
            sample.hand = hand;
            sample.timestamp_ns = timestamp_ns;
            setButtonState(sample, button_id, pressed_int != 0);
            publishButtonMessage(button_id, hand, pressed_int != 0);
            return true;
        }

        if (stringEqualsIgnoreCase(token, "JOY")) {
            std::string hand_token;
            HandIndex hand;
            if (!(iss >> hand_token) || !parseHandToken(hand_token, hand)) {
                return false;
            }

            auto &sample = latest_samples_[hand];
            sample.hand = hand;
            if (!(iss >> sample.joystick_x >> sample.joystick_y)) {
                return false;
            }
            if (!(iss >> sample.timestamp_ns)) {
                sample.timestamp_ns = 0;
            }

            publishJoystickMessage(hand, sample.joystick_x, sample.joystick_y);
            return true;
        }

        if (stringEqualsIgnoreCase(token, "TRIG")) {
            std::string hand_token;
            HandIndex hand;
            if (!(iss >> hand_token) || !parseHandToken(hand_token, hand)) {
                return false;
            }

            auto &sample = latest_samples_[hand];
            sample.hand = hand;
            if (!(iss >> sample.trigger_value)) {
                return false;
            }
            if (!(iss >> sample.timestamp_ns)) {
                sample.timestamp_ns = 0;
            }

            publishTriggerMessage(hand, sample.trigger_value);
            return true;
        }

        if (stringEqualsIgnoreCase(token, "RATE")) {
            double rate_value = 0.1;
            int64_t timestamp_ns = 0;
            if (!(iss >> rate_value)) {
                return false;
            }
            if (!(iss >> timestamp_ns)) {
                timestamp_ns = 0;
            }

            for (auto &sample : latest_samples_) {
                sample.rate = rate_value;
                sample.timestamp_ns = timestamp_ns;
            }
            publishRateMessage(rate_value);
            return true;
        }

        if (stringEqualsIgnoreCase(token, "CFG")) {
            std::string config_token;
            double config_value = 0.0;
            if (!(iss >> config_token >> config_value)) {
                return false;
            }
            int64_t ignored_timestamp_ns = 0;
            (void)(iss >> ignored_timestamp_ns);

            if (stringEqualsIgnoreCase(config_token, "LIN") ||
                stringEqualsIgnoreCase(config_token, "LINEAR")) {
                publishChassisLinearSpeedMessage(config_value);
                return true;
            }
            if (stringEqualsIgnoreCase(config_token, "ANG") ||
                stringEqualsIgnoreCase(config_token, "ANGULAR")) {
                publishChassisAngularSpeedMessage(config_value);
                return true;
            }
            return false;
        }

        return false;
    }

    bool parseDatagram(const std::string &payload, PoseSample &out_sample) const {
        std::istringstream iss(payload);
        std::string hand_token;

        if (!(iss >> hand_token)) {
            return false;
        }

        if (!parseHandToken(hand_token, out_sample.hand)) {
            return false;
        }

        for (double &component : out_sample.position) {
            if (!(iss >> component)) {
                return false;
            }
        }
        for (double &component : out_sample.orientation) {
            if (!(iss >> component)) {
                return false;
            }
        }

        // 解析扳机值（如果存在）
        if (!(iss >> out_sample.trigger_value)) {
            out_sample.trigger_value = 0.0;
        }
        if (!(iss >> out_sample.grip_value)) {
            out_sample.grip_value = 0.0;
        }

        // 解析按键状态（如果存在）
        int button_a_int = 0, button_b_int = 0, button_x_int = 0, button_y_int = 0;
        if (!(iss >> button_a_int)) {
            button_a_int = 0;
        }
        if (!(iss >> button_b_int)) {
            button_b_int = 0;
        }
        if (!(iss >> button_x_int)) {
            button_x_int = 0;
        }
        if (!(iss >> button_y_int)) {
            button_y_int = 0;
        }
        out_sample.button_a = (button_a_int != 0);
        out_sample.button_b = (button_b_int != 0);
        out_sample.button_x = (button_x_int != 0);
        out_sample.button_y = (button_y_int != 0);

        out_sample.joystick_x = 0.0;
        out_sample.joystick_y = 0.0;
        out_sample.rate = 0.1;
        out_sample.timestamp_ns = 0;

        std::vector<double> tail_values;
        double tail_value = 0.0;
        while (iss >> tail_value) {
            tail_values.push_back(tail_value);
        }

        const auto is_valid_rate = [](double value) {
            return std::abs(value - 0.1) < 1e-6 || std::abs(value - 1.0) < 1e-6;
        };
        const auto parse_rate_or_timestamp = [&](double value) {
            if (is_valid_rate(value)) {
                out_sample.rate = value;
            } else {
                out_sample.timestamp_ns = static_cast<int64_t>(value);
                RCLCPP_WARN(
                    get_logger(),
                    "Invalid rate value detected: %.2f, treating it as timestamp and resetting rate to 0.1",
                    value);
            }
        };

        if (tail_values.size() >= 4) {
            // Extended OpenFlex packet: joystick_x joystick_y rate timestamp.
            out_sample.joystick_x = tail_values[0];
            out_sample.joystick_y = tail_values[1];
            parse_rate_or_timestamp(tail_values[2]);
            out_sample.timestamp_ns = static_cast<int64_t>(tail_values[3]);
        } else if (tail_values.size() == 3) {
            // Extended packet without timestamp: joystick_x joystick_y rate.
            out_sample.joystick_x = tail_values[0];
            out_sample.joystick_y = tail_values[1];
            parse_rate_or_timestamp(tail_values[2]);
        } else if (tail_values.size() == 2) {
            if (is_valid_rate(tail_values[0])) {
                // Legacy packet: rate timestamp.
                out_sample.rate = tail_values[0];
                out_sample.timestamp_ns = static_cast<int64_t>(tail_values[1]);
            } else {
                // Extended packet without rate/timestamp: joystick_x joystick_y.
                out_sample.joystick_x = tail_values[0];
                out_sample.joystick_y = tail_values[1];
            }
        } else if (tail_values.size() == 1) {
            // Legacy packet: rate, or timestamp when rate is omitted.
            parse_rate_or_timestamp(tail_values[0]);
        }

        // 调试：显示解析结果
        RCLCPP_DEBUG(get_logger(),
                     "Parsed hand=%s rate=%.2f ts=%ld trig=%.3f grip=%.3f joy=(%.3f,%.3f)",
                     out_sample.hand == LEFT ? "LEFT" : "RIGHT",
                     out_sample.rate,
                     out_sample.timestamp_ns,
                     out_sample.trigger_value,
                     out_sample.grip_value,
                     out_sample.joystick_x,
                     out_sample.joystick_y);

        return true;
    }

    bool parseTrackerDatagram(const std::string &payload, TrackerSample &out) const {
        std::istringstream iss(payload);
        std::string token;
        if (!(iss >> token)) {
            return false;
        }

        if (stringEqualsIgnoreCase(token, "WAIST")) {
            out.tracker = WAIST;
        } else if (stringEqualsIgnoreCase(token, "LEFT_FOOT")) {
            out.tracker = LEFT_FOOT;
        } else if (stringEqualsIgnoreCase(token, "RIGHT_FOOT")) {
            out.tracker = RIGHT_FOOT;
        } else if (stringEqualsIgnoreCase(token, "HEAD")) {
            out.tracker = HEAD;
        } else {
            return false;  // 不是追踪器/头显报文
        }

        for (double &c : out.position) {
            if (!(iss >> c)) return false;
        }
        for (double &c : out.orientation) {
            if (!(iss >> c)) return false;
        }
        if (!(iss >> out.timestamp_ns)) {
            out.timestamp_ns = 0;
        }
        return true;
    }

    void publishTrackerSample(const TrackerSample &sample) {
        const rclcpp::Time stamp =
                sample.timestamp_ns > 0 ? rclcpp::Time(sample.timestamp_ns) : now();

        auto msg = geometry_msgs::msg::PoseStamped();
        msg.header.stamp = stamp;
        msg.header.frame_id = frame_id_;
        msg.pose.position.x = sample.position[0];
        msg.pose.position.y = sample.position[1];
        msg.pose.position.z = sample.position[2];
        msg.pose.orientation.x = sample.orientation[0];
        msg.pose.orientation.y = sample.orientation[1];
        msg.pose.orientation.z = sample.orientation[2];
        msg.pose.orientation.w = sample.orientation[3];

        tracker_pose_publishers_[sample.tracker]->publish(msg);

        static int tracker_log_count = 0;
        tracker_log_count++;
        if (tracker_log_count % 60 == 0) {
            const char* names[] = {"WAIST", "LEFT_FOOT", "RIGHT_FOOT", "HEAD"};
            RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000,
                                  "Tracker %s: pos=(%.4f,%.4f,%.4f) quat=(%.4f,%.4f,%.4f,%.4f)",
                                  names[sample.tracker],
                                  msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
                                  msg.pose.orientation.x, msg.pose.orientation.y,
                                  msg.pose.orientation.z, msg.pose.orientation.w);
        }
    }

    void publishSample(const PoseSample &sample) {
        latest_samples_[sample.hand] = sample;

        // 调试：显示发布信息
        RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000,
                              "Publishing hand=%s rate=%.2f rate_pub[L=%d R=%d]",
                              sample.hand == LEFT ? "LEFT" : "RIGHT",
                              sample.rate,
                              rate_publishers_[LEFT] ? 1 : 0,
                              rate_publishers_[RIGHT] ? 1 : 0);
        const rclcpp::Time stamp = resolveStamp(sample.timestamp_ns);

        publishPoseMessage(sample, stamp);

        // 调试输出：每 20 帧输出一次位置和姿态信息（更频繁，便于观察）
        static int frame_count = 0;
        frame_count++;
        if (frame_count % 20 == 0) {
            RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 500,
                                  "Controller %s (raw): pos=(%.4f,%.4f,%.4f) quat=(%.4f,%.4f,%.4f,%.4f)",
                                  sample.hand == LEFT ? "LEFT" : "RIGHT",
                                  sample.position[0], sample.position[1], sample.position[2],
                                  sample.orientation[0], sample.orientation[1],
                                  sample.orientation[2], sample.orientation[3]);
        }

        publishTriggerMessage(sample.hand, sample.trigger_value);
        publishGripMessage(sample.hand, sample.grip_value);
        publishJoystickMessage(sample.hand, sample.joystick_x, sample.joystick_y);
        publishButtonMessage(ButtonId::JOYSTICK, sample.hand, sample.joystick_click);

        RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000,
                              "Publishing rate=%.2f to both hands", sample.rate);
        publishRateMessage(sample.rate);

        // 发布按键状态（根据手柄类型只发布对应的按键）
        if (sample.hand == RIGHT) {
            // 右手柄：只发布 A 和 B 按键
            static bool last_a = false, last_b = false;
            if (sample.button_a != last_a || sample.button_b != last_b) {
                RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000,
                                      "Right hand button state changed: A=%d B=%d",
                                      sample.button_a ? 1 : 0, sample.button_b ? 1 : 0);
                last_a = sample.button_a;
                last_b = sample.button_b;
            }

            publishButtonMessage(ButtonId::A, sample.hand, sample.button_a);
            publishButtonMessage(ButtonId::B, sample.hand, sample.button_b);
        } else {
            // 左手柄：只发布 X 和 Y 按键
            static bool last_x = false, last_y = false;
            if (sample.button_x != last_x || sample.button_y != last_y) {
                RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000,
                                      "Left hand button state changed: X=%d Y=%d",
                                      sample.button_x ? 1 : 0, sample.button_y ? 1 : 0);
                last_x = sample.button_x;
                last_y = sample.button_y;
            }

            publishButtonMessage(ButtonId::X, sample.hand, sample.button_x);
            publishButtonMessage(ButtonId::Y, sample.hand, sample.button_y);
        }
    }

    std::string listen_address_;
    int listen_port_{5100};
    std::string frame_id_;
    std::array<std::string, HAND_COUNT> child_frame_ids_;
    std::array<std::string, HAND_COUNT> pose_topics_;
    std::array<std::string, HAND_COUNT> trigger_topics_;
    std::array<std::string, HAND_COUNT> grip_topics_;
    std::array<std::string, HAND_COUNT> joystick_x_topics_;
    std::array<std::string, HAND_COUNT> joystick_y_topics_;
    std::array<std::string, HAND_COUNT> joystick_click_topics_;
    std::array<PoseSample, HAND_COUNT> latest_samples_;
    std::array<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr, HAND_COUNT>
            pose_publishers_;
    std::array<rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr, HAND_COUNT>
            trigger_publishers_;
    std::array<rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr, HAND_COUNT>
            grip_publishers_;
    std::array<rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr, HAND_COUNT>
            joystick_x_publishers_;
    std::array<rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr, HAND_COUNT>
            joystick_y_publishers_;
    std::array<rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr, HAND_COUNT>
            joystick_click_publishers_;
    std::array<rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr, HAND_COUNT>
            rate_publishers_;  // 添加这行
    std::string button_a_topic_;
    std::string button_b_topic_;
    std::string button_x_topic_;
    std::string button_y_topic_;
    std::array<std::string, HAND_COUNT> rate_topics_;  // 添加这行
    std::string chassis_linear_speed_topic_;
    std::string chassis_angular_speed_topic_;

    // 体感追踪器
    std::array<std::string, TRACKER_COUNT> tracker_topics_;
    std::array<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr, TRACKER_COUNT>
            tracker_pose_publishers_;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr button_a_publisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr button_b_publisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr button_x_publisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr button_y_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr chassis_linear_speed_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr chassis_angular_speed_publisher_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    bool publish_tf_{true};

    std::thread receiver_thread_;
    std::atomic<bool> running_;
    int socket_fd_{-1};
    std::string last_sender_ip_;
    uint16_t last_sender_port_{0};
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PoseBridgeNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
