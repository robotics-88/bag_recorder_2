#include <rclcpp/rclcpp.hpp>
#include "bag_recorder_2/srv/record.hpp"

#include "rosbag2_transport/recorder.hpp"
#include "rosbag2_transport/reader_writer_factory.hpp"

class BagRecorder : public rclcpp::Node
{
public:
    BagRecorder(rclcpp::executors::MultiThreadedExecutor &executor);
    ~BagRecorder() = default;

private:
    rclcpp::Service<bag_recorder_2::srv::Record>::SharedPtr record_service_;
    void trigger_recording(const std::shared_ptr<bag_recorder_2::srv::Record::Request> request,
                                 std::shared_ptr<bag_recorder_2::srv::Record::Response> response);
    void start_recording(std::string config_file, std::string data_directory);
    void stop_recording();

    bool load_config(std::string config_name, std::vector<std::string>& topics, std::set<std::string> loaded = std::set<std::string>());
    std::string sanitize_topic(std::string topic);

    bool is_recording_;
    std::shared_ptr<rosbag2_transport::Recorder> recorder_;
    std::thread recording_thread_;

    rclcpp::executors::MultiThreadedExecutor &executor_;

};