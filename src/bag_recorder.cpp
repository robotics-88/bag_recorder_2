#include "bag_recorder_2/bag_recorder.hpp"

using namespace std::placeholders;

BagRecorder::BagRecorder(rclcpp::executors::MultiThreadedExecutor &executor)
  : Node("bag_recorder"),
  executor_(executor)
{
  record_service_ = this->create_service<bag_recorder_2::srv::Record>("~/record", std::bind(&BagRecorder::trigger_recording, this, _1, _2));
}

void BagRecorder::trigger_recording(const std::shared_ptr<bag_recorder_2::srv::Record::Request> request,
                                          std::shared_ptr<bag_recorder_2::srv::Record::Response> response) {

  if (request->start && !is_recording_) {
    start_recording(request->topics, request->data_directory);
    response->success = true;
  } else if (!request->start && is_recording_) {
    stop_recording();
    response->success = true;
  } else {
    response->success = false;
  }
}

void BagRecorder::start_recording(std::vector<std::string> topics, std::string data_directory)
{
  RCLCPP_INFO(this->get_logger(), "Requesting start recording");
  if (is_recording_) {
    RCLCPP_WARN(this->get_logger(), "Recording already in progress");
    return;
  }

  rosbag2_transport::RecordOptions record_options;
  record_options.rmw_serialization_format = std::string(rmw_get_serialization_format());
  record_options.all = false;  // Set to false to record specific topics
  record_options.topics = topics;  // List the topics to record

  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri = data_directory + "/bag_" + get_time_str();
  storage_options.max_bagfile_duration = 60;

  // Recorder will be started
  auto writer = rosbag2_transport::ReaderWriterFactory::make_writer(record_options);
  recorder_ = std::make_shared<rosbag2_transport::Recorder>(std::move(writer), storage_options, record_options);

  recorder_->record();
  executor_.add_node(recorder_);

  is_recording_ = true;
}

void BagRecorder::stop_recording()
{
  RCLCPP_INFO(this->get_logger(), "Requesting stop recording");
  if (!is_recording_) {
    RCLCPP_WARN(this->get_logger(), "No recording in progress to stop");
    return;
  }

  recorder_->stop();  // This stops the recording
  if (recording_thread_.joinable()) {
    recording_thread_.join();  // Ensure the thread finishes cleanly
  }

  RCLCPP_INFO(this->get_logger(), "Stopped recording.");
  is_recording_ = false;
}

std::string BagRecorder::get_time_str()
{
  auto now = std::chrono::system_clock::now();
  std::time_t now_time = std::chrono::system_clock::to_time_t(now);
  std::tm now_tm = *std::localtime(&now_time);
  std::stringstream ss;
  ss << std::put_time(&now_tm, "%Y-%m-%d_%H-%M-%S");
  return ss.str();
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // Use a MultiThreadedExecutor to allow for parallel service calls and recording
  rclcpp::executors::MultiThreadedExecutor executor;
  auto bag_recorder_node = std::make_shared<BagRecorder>(executor);
  executor.add_node(bag_recorder_node);

  executor.spin();  // Spin the executor, which also spins the recorder
  return 0;
}