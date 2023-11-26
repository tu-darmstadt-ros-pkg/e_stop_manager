#include <gtest/gtest.h>
#include <e_stop_manager/e_stop_manager.h>
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <mutex>
#include <condition_variable>

using namespace std::chrono_literals;

/**
 * Start the tests with
 * cd <ros2_workspace>
 * ros2 launch_test src/e_stop_manager/e_stop_manager/test/test_e_stop_manager.launch.py test_binary_dir:="build/e_stop_manager"
 *
 */

// Redefinition of topics and e-stops
// redefined here to test if e-stop manager is able to read the config params correctly and set up the correct publishers
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
            : msg_received_(false), topic_name_(topic_name), msg_{nullptr} {

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
    bool waitForMessage(std::chrono::milliseconds timeout = std::chrono::milliseconds(200), double frequency = 10) {
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

    bool receivedMsgSinceLastReset() const {
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

        client_ = node_->create_client<e_stop_manager_msgs::srv::SetEStop>("set_e_stop");
        for (auto const &topic: E_STOP_TOPICS) {
            auto sub = std::make_shared<MsgContainer<std_msgs::msg::Bool>>(node_, topic);
            e_stop_msgs.emplace(topic, sub);
        }

        // initialize all e_stops as off (must be the same in e_stop_manager config)
        for (const auto &pair: E_STOP_NAMES_TO_TOPIC) {
            e_stop_state_[pair.first] = false;
        }
    };

    /*
     * reset Start State -> all of them should be off
     */
    void resetStartStateOfEStopManager(){
        waitForConnection();
        //turn all off
        for (const auto &pair: E_STOP_NAMES_TO_TOPIC) {
            callEStop(pair.first, false);
        }
    }

    bool getExpectedEStopTopicState(const std::string &topic) {
        // RCLCPP_INFO_STREAM(rclcpp::get_logger("test_client"),"getExpectedEStopTopicState "<<topic);
        assert(std::find(E_STOP_TOPICS.begin(), E_STOP_TOPICS.end(), topic) != E_STOP_TOPICS.end());
        bool state = false;
        for (const auto &pair: E_STOP_NAMES_TO_TOPIC) {
            if (topic == pair.second && e_stop_state_[pair.first]) {
                state = true;
                break;
            }
        }
        return state;
    }

    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface() const {
        return this->node_->get_node_base_interface();
    }



    /**
     * Waits for the publishers and service clients of the test node to connect to the e-stop manager
     * -> also asserts the existence of the publishers in the e-stop-manager
     * @param test_client
     */
    void waitForConnection() {
        for (const auto &pair: e_stop_msgs) {
            ASSERT_TRUE(pair.second->waitForPublisher(300ms));
        }
        ASSERT_TRUE(client_->wait_for_service(300ms));
    }

    /**
     * Reset all msg containers
     */
    void resetAllMsgContainers() {
        for (const auto &pair: e_stop_msgs) {
            pair.second->reset();
        }
        e_stop_list_msgs.reset();
    }

    /**
     * Calls the e_stop service with the given values.
     * @param e_stop_name
     * @param requested_state
     */
    void callEStop(const std::string &e_stop_name, bool requested_state, bool invalid_e_stop_name = false) {
        auto request = std::make_shared<e_stop_manager_msgs::srv::SetEStop::Request>();
        request->name = e_stop_name;
        request->value = requested_state;
        auto result = client_->async_send_request(request);
        // Wait for the result.
        auto status = result.wait_for(100ms);
        EXPECT_TRUE(status == std::future_status::ready);
        if (!invalid_e_stop_name){
            EXPECT_TRUE(result.get()->result == e_stop_manager_msgs::srv::SetEStop::Response::SUCCESS);
        }else{
            EXPECT_TRUE(result.get()->result == e_stop_manager_msgs::srv::SetEStop::Response::INVALID_ESTOP_NAME);

        }

        e_stop_state_[e_stop_name] = requested_state;
    }

    /**
     * Waits for msg on every e_stop + e_stop_list topic and compares the received state
     * with the expected e_stop_topic state.
     * Assumes: that resetAllMsgContainers has been called before the e-stop request to the e-stop-manager
     */
    void waitForMsgAndVerifyState(){
        // Check e_stop_topics
        for (const auto &topic: E_STOP_TOPICS) {
            EXPECT_TRUE(e_stop_msgs[topic]->waitForMessage());
            auto msg = e_stop_msgs[topic]->getCurrentMessage();
            if(msg){
                auto received_msg = e_stop_msgs[topic]->receivedMsgSinceLastReset();
                RCLCPP_INFO_STREAM(rclcpp::get_logger("test_client"),
                                   "\t topic " << topic << " received " << (received_msg ? "a" : "no") << " msg"
                                               << (received_msg ? "with value " + std::to_string(msg->data) : "")
                                               << " expected topic state is: "
                                               << getExpectedEStopTopicState(topic));
                // compare expected state with real state
                EXPECT_EQ(getExpectedEStopTopicState(topic), msg->data);
            }

        }

        // Check e_stop_list topic
        EXPECT_TRUE(e_stop_list_msgs.waitForMessage());
        auto msg = e_stop_list_msgs.getCurrentMessage();
        if (msg){
            ASSERT_EQ(msg->values.size(), msg->names.size());
            ASSERT_EQ(msg->values.size(), E_STOP_NAMES_TO_TOPIC.size());
            for(size_t i=0; i<msg->values.size(); i++){
                RCLCPP_INFO_STREAM(rclcpp::get_logger("test_client"),
                                   "\t topic " << msg->names[i]
                                               <<" value " <<msg->values[i]
                                               << " expected topic state is: "
                                               << e_stop_state_[msg->names[i]]);
                EXPECT_EQ(msg->values[i], e_stop_state_[msg->names[i]]);
            }
        }
    }


    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<e_stop_manager_msgs::srv::SetEStop>::SharedPtr client_;
    std::map<std::string, std::shared_ptr<MsgContainer<std_msgs::msg::Bool>>> e_stop_msgs;
    MsgContainer<e_stop_manager_msgs::msg::EStopList> e_stop_list_msgs;
    std::map<std::string, bool> e_stop_state_;
};

// Fixture class for testing EStopManager
class EStopManagerTest : public ::testing::Test {
protected:
    EStopManagerTest() : executor_(std::make_shared<rclcpp::executors::SingleThreadedExecutor>()) {}

    void SetUp() override {
        test_client_ = std::make_shared<TestClient>();
        executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        executor_->add_node(test_client_->get_node_base_interface());
        executor_thread_ = std::thread([this]() { this->executor_->spin(); });
        RCLCPP_INFO(rclcpp::get_logger("test_client"), "\n ************************ NEW TEST ****************** \n");

    }

    void TearDown() override {
        // Clean up resources if needed
        test_client_.reset();
        executor_->cancel();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        if (executor_thread_.joinable()) {
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
 * Tests if the TestClient is correctly connected to the e_stop_manager
 */
TEST_F(EStopManagerTest, TestConnection) {
    test_client_->waitForConnection();
}

/**
 * Client request via the service client that an e_stop is on
 * -> the corresponding e_stop topic must receive a msg with the correct value
 * -> e_stop_list must be published
 * -> test starts with all e_stops off and iterativley turns every e_stop on then off again
 */
TEST_F(EStopManagerTest, ServiceRequestChangesTopics) {
    test_client_->resetStartStateOfEStopManager();
    // iterate all available e-stops
    for (const auto &pair: E_STOP_NAMES_TO_TOPIC) {
        test_client_->resetAllMsgContainers();
        // request 1 e-stop to be active
        test_client_->callEStop(pair.first, true);
        test_client_->waitForMsgAndVerifyState();

        // turn Off
        test_client_->resetAllMsgContainers();
        // request 1 e-stop to be active
        test_client_->callEStop(pair.first, false);
        test_client_->waitForMsgAndVerifyState();
    }
}

/**
 *  Client request via the service client that an e_stop is off, but is already off
 *  -> should stay that way
 */
TEST_F(EStopManagerTest, ServiceRequesteStopAlreadyOff) {
    test_client_->resetStartStateOfEStopManager();
    // iterate all available e-stops
    for (const auto &pair: E_STOP_NAMES_TO_TOPIC) {
        test_client_->resetAllMsgContainers();
        test_client_->callEStop(pair.first, false);
        EXPECT_FALSE(test_client_->e_stop_list_msgs.waitForMessage(300ms));
        for(const auto& topic: E_STOP_TOPICS){
            EXPECT_FALSE(test_client_->e_stop_msgs[topic]->waitForMessage(1ms));//already waited long enough above
        }
    }
}

/**
 *  Client request via the service client that an e_stop is on, but is already on
 *  -> should stay that way
 */
TEST_F(EStopManagerTest, ServiceRequesteStopAlreadyOn) {
    test_client_->resetStartStateOfEStopManager();
    // Turn all on
    for (const auto &pair: E_STOP_NAMES_TO_TOPIC) {
        test_client_->resetAllMsgContainers();
        // request 1 e-stop to be active
        test_client_->callEStop(pair.first, true);
        test_client_->waitForMsgAndVerifyState();
    }
    // turn on again -> should remain on
    for (const auto &pair: E_STOP_NAMES_TO_TOPIC) {
        test_client_->resetAllMsgContainers();
        test_client_->callEStop(pair.first, true    );

        EXPECT_FALSE(test_client_->e_stop_list_msgs.waitForMessage(300ms));
        for(const auto& topic: E_STOP_TOPICS){
            EXPECT_FALSE(test_client_->e_stop_msgs[topic]->waitForMessage(10ms));//already waited long enough above
        }
    }
}

/**
 * Call service with non_exising e_stop name
 */
TEST_F(EStopManagerTest, NonExistingEStopName) {
    test_client_->resetStartStateOfEStopManager();

    // turn 1 e_stop on
    test_client_->resetAllMsgContainers();
    test_client_->callEStop(E_STOP_NAMES_TO_TOPIC.begin()->first, true);
    test_client_->waitForMsgAndVerifyState();

    test_client_->resetAllMsgContainers();
    // turn on non-exising e_stop
    test_client_->callEStop("DOES_NOT_EXIST", true, true);
    // test that no msg arrives -> e_stop remains correct
    EXPECT_FALSE(test_client_->e_stop_list_msgs.waitForMessage(300ms));
    for(const auto& topic: E_STOP_TOPICS){
        EXPECT_FALSE(test_client_->e_stop_msgs[topic]->waitForMessage(10ms));//already waited long enough above
    }

    // turn 1 e_stop off
    test_client_->callEStop(E_STOP_NAMES_TO_TOPIC.begin()->first, false);
    test_client_->waitForMsgAndVerifyState();
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    rclcpp::init(argc, argv);

    int result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return result;
}