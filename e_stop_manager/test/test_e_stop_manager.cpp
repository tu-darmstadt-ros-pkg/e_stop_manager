#include <gtest/gtest.h>
#include <e_stop_manager/e_stop_manager.h>
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <mutex>
#include <condition_variable>

using namespace std::chrono_literals;

// Redefinition of topics and e-stops
// redefined here to test if e-stop manager is able to read the config params correctly and setup the correct publishers
std::vector<std::string> E_STOP_TOPICS = {"/emergency_stop_hardware", "/emergency_stop_software"};
std::map<std::string, std::string> E_STOP_NAMES_TO_TOPIC =
        {{"hard_remote_e_stop",    "/emergency_stop_hardware"},
         {"soft_remote_e_stop",    "/emergency_stop_hardware"},
         {"big_red_button_e_stop", "/emergency_stop_software"},
         {"ui_e_stop",             "/emergency_stop_software"}};

template<typename MsgType>
struct MsgContainer {
    using MessageSharedPtr = std::shared_ptr<MsgType>;

    MsgContainer(const rclcpp::Node::SharedPtr &node, const std::string &topic_name)
            : msg_received_(false), topic_name_(topic_name) {
        RCLCPP_INFO(rclcpp::get_logger("test_client"), "Initializing MsgContainer!");

        auto callback = [this](const typename MsgType::SharedPtr msg) {
            this->storeMessage(msg);
        };
        subscriber_ = node->create_subscription<MsgType>(topic_name_, 10, callback);
    }

    void storeMessage(const MessageSharedPtr &msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        msg_ = msg;
        msg_received_ = true;
    }

    MessageSharedPtr getCurrentMessage() {
        std::lock_guard<std::mutex> lock(mutex_);
        return msg_;
    }

    void reset() {
        std::lock_guard<std::mutex> lock(mutex_);
        msg_received_ = false;
    }

    // Wait for a message to be received with a specific timeout duration
    bool waitForMessage(std::chrono::milliseconds timeout = std::chrono::milliseconds(200), double frequency=10) {
        rclcpp::Rate rate(frequency);
        auto start_time = std::chrono::steady_clock::now();
        while (!msg_received_) {
            if (std::chrono::steady_clock::now() - start_time > timeout) {
                return false;  // Timeout reached
            }
            rate.sleep();
        }
        return true;  // Message received within the timeout
    }

    // Method to check if at least one publisher is subscribed to the subscriber
    bool isSubscriberConnected() const {
        if (subscriber_) {
            auto num_publishers = subscriber_->get_publisher_count();
            return (num_publishers > 0);
        }
        return false;
    }

    // Wait for a publisher to be connected with a specific frequency and timeout
    bool waitForPublisher(std::chrono::milliseconds timeout = std::chrono::milliseconds(200),
                          double frequency = 10.0) {
        rclcpp::Rate rate(frequency);
        auto start_time = std::chrono::steady_clock::now();
        while (rclcpp::ok()) {
            if (subscriber_ && subscriber_->get_publisher_count() > 0) {
                return true;  // Publisher connected
            }
            if (std::chrono::steady_clock::now() - start_time > timeout) {
                return false;  // Timeout reached
            }
            rate.sleep();
        }
        return false;  // Unexpected failure or interruption
    }

    bool receivedMsgSinceLastReset() const{
        std::lock_guard<std::mutex> lock(mutex_);
        return msg_received_;
    }
private:
    mutable std::mutex mutex_;
    MessageSharedPtr msg_;
    bool msg_received_;
    std::string topic_name_;
    typename rclcpp::Subscription<MsgType>::SharedPtr subscriber_;
};


// Mock client class for the service
class TestClient {
public:
    explicit TestClient(const rclcpp::NodeOptions &options = rclcpp::NodeOptions()) :
            node_(std::make_shared<rclcpp::Node>("test_client", options)),
            e_stop_list_msgs(MsgContainer<e_stop_manager_msgs::msg::EStopList>{node_, "e_stop_list"}) {
        RCLCPP_INFO(rclcpp::get_logger("test_client"), "Initializing TestClient!");

        client_ = node_->create_client<e_stop_manager_msgs::srv::SetEStop>("set_e_stop");
        for (auto const &topic: E_STOP_TOPICS) {
            auto sub = std::make_shared<MsgContainer<std_msgs::msg::Bool>>(node_, topic);
            e_stop_msgs.emplace(topic, sub);
        }
        RCLCPP_INFO(rclcpp::get_logger("test_client"), "Initializing TestClient Done!");

    };

    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface() const {
        return this->node_->get_node_base_interface();
    }


    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<e_stop_manager_msgs::srv::SetEStop>::SharedPtr client_;
    std::map<std::string, std::shared_ptr<MsgContainer<std_msgs::msg::Bool>>> e_stop_msgs;
    MsgContainer<e_stop_manager_msgs::msg::EStopList> e_stop_list_msgs;
};

// Fixture class for testing EStopManager
class EStopManagerTest : public ::testing::Test {
protected:
    EStopManagerTest() : executor_(std::make_shared<rclcpp::executors::SingleThreadedExecutor>()) {}

    void SetUp() override {
        RCLCPP_INFO(rclcpp::get_logger("test_client"), "Setup called!");
        test_client_ = std::make_shared<TestClient>();
        executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        executor_->add_node(test_client_->get_node_base_interface());
        executor_thread_ = std::thread([this]() { this->executor_->spin(); });
        RCLCPP_INFO(rclcpp::get_logger("test_client"), "Setup Done!");

    }

    void TearDown() override {
        // Clean up resources if needed
        test_client_.reset();
        executor_->cancel();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        if (executor_thread_.joinable()){
            executor_thread_.join();
            RCLCPP_INFO(rclcpp::get_logger("test_client"), "Stopping executor thread!");

        }
        executor_.reset();
    }

    std::shared_ptr<TestClient> test_client_;
    rclcpp::Executor::SharedPtr executor_;
    std::thread executor_thread_;
};

/**
 * Waits for the publishers and service clients of the test node to connect to the e-stop manager
 * -> also asserts the existence of the publishers in the e-stop-manager
 * @param test_client
 */
void waitForConnection(const std::shared_ptr<TestClient> &test_client) {
    for (const auto &pair: test_client->e_stop_msgs) {
        ASSERT_TRUE(pair.second->waitForPublisher(300ms));
    }
    ASSERT_TRUE(test_client->client_->wait_for_service(300ms));
}

void resetAll(const std::shared_ptr<TestClient> &test_client) {
    for (const auto &pair: test_client->e_stop_msgs) {
        pair.second->reset();
    }
}
/**
 * Tests if the TestClient is correctly connected to the e_stop_manager
 */
TEST_F(EStopManagerTest, TestConnection) {
    waitForConnection(test_client_);
}

/**
 * Client request via the service client that an e_stop is off
 * -> the corresponding e_stop topic must receive a msg with the correct value
 * -> e_stop_list must be published
 */
TEST_F(EStopManagerTest, ServiceRequestChangesTopics) {
    waitForConnection(test_client_);

    // TODO: only true for first iteration -> keep state of active/inactive e-stops
    // iterate all available e-stops
    for (const auto &pair: E_STOP_NAMES_TO_TOPIC) {
        resetAll(test_client_);
        // request 1 e-stop to be active
        auto request = std::make_shared<e_stop_manager_msgs::srv::SetEStop::Request>();
        request->name = pair.first;
        request->value = true;
        auto result = test_client_->client_->async_send_request(request);
        // Wait for the result.
        auto status = result.wait_for(100ms);
        EXPECT_TRUE(status == std::future_status::ready);
        EXPECT_TRUE(result.get()->result == e_stop_manager_msgs::srv::SetEStop::Response::SUCCESS);
        RCLCPP_INFO_STREAM(rclcpp::get_logger("test_client"), "Called "<<pair.first<<" with value true");

        // iterate available topics
        for(const auto& topic: E_STOP_TOPICS){
            EXPECT_TRUE(test_client_->e_stop_msgs[topic]->waitForMessage());
            auto msg = test_client_->e_stop_msgs[topic]->getCurrentMessage();
            auto received_msg = test_client_->e_stop_msgs[topic]->receivedMsgSinceLastReset();
            RCLCPP_INFO_STREAM(rclcpp::get_logger("test_client"), "\t topic "<<topic<<" received "<<
                               (received_msg?"a":"no")<<" msg"<<(received_msg? "with value "+std::to_string(msg->data):""));
            //TODO: BUG: comparing topic vs. name
            if(topic == pair.first){
                EXPECT_TRUE(msg->data);
            }else{
                RCLCPP_INFO_STREAM(rclcpp::get_logger("test_client"),"topic "<<topic<<" pair.first "<<pair.first);
                EXPECT_FALSE(msg->data);
            }

        }

    }

}


int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    rclcpp::init(argc, argv);

    int result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return result;
}