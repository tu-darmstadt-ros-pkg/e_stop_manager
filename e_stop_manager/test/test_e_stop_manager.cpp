#include <algorithm>
#include <chrono>
#include <gtest/gtest.h>
#include <map>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <string>
#include <thread>
#include <vector>

#include <e_stop_manager_msgs/msg/e_stop_list.hpp>
#include <e_stop_manager_msgs/srv/set_e_stop.hpp>

using namespace std::chrono_literals;

struct EStopConfigEntry {
  std::string aggregated_topic;
  bool tracked;
};

const std::vector<std::string> AGGREGATED_NAMES = { "emergency_stop_hardware", "emergency_stop_software" };

const std::map<std::string, EStopConfigEntry> E_STOP_CONFIG = {
    { "hard_remote_e_stop", { "/emergency_stop_hardware", true } },
    { "soft_remote_e_stop", { "/emergency_stop_hardware", true } },
    { "big_red_button_e_stop", { "/emergency_stop_software", true } },
    { "ui_e_stop", { "/emergency_stop_software", false } },
};

const std::string MANAGED_ESTOP = "ui_e_stop";
const std::string NODE_NAME = "e_stop_manager";

std::string aggregatedTopicName( const std::string &aggregated_name )
{
  auto normalized = aggregated_name;
  if ( !normalized.empty() && normalized.front() == '/' ) {
    normalized.erase( normalized.begin() );
  }
  return "/" + NODE_NAME + "/aggregated_state/" + normalized;
}

template<typename MsgType>
struct MsgContainer {
  using MessageSharedPtr = std::shared_ptr<MsgType>;

  MsgContainer( const rclcpp::Node::SharedPtr &node, const std::string &topic_name )
      : msg_received_( false ), topic_name_( topic_name ), msg_{ nullptr }
  {
    auto callback = [this]( const typename MsgType::SharedPtr msg ) { this->storeMessage( msg ); };
    subscriber_ =
        node->create_subscription<MsgType>( topic_name_, rclcpp::QoS( rclcpp::KeepLast( 10 ) ).reliable().transient_local(), callback );
  }

  void storeMessage( const MessageSharedPtr &msg )
  {
    std::lock_guard<std::mutex> lock( mutex_ );
    msg_ = msg;
    msg_received_ = true;
  }

  MessageSharedPtr getCurrentMessage()
  {
    std::lock_guard<std::mutex> lock( mutex_ );
    return msg_;
  }

  void reset()
  {
    std::lock_guard<std::mutex> lock( mutex_ );
    msg_received_ = false;
  }

  bool waitForMessage( std::chrono::milliseconds timeout = std::chrono::milliseconds( 300 ), double frequency = 10 )
  {
    rclcpp::Rate rate( frequency );
    auto start_time = std::chrono::steady_clock::now();
    while ( !msg_received_ ) {
      if ( std::chrono::steady_clock::now() - start_time > timeout ) {
        return false;
      }
      rate.sleep();
    }
    return true;
  }

  bool waitForPublisher( std::chrono::milliseconds timeout = std::chrono::milliseconds( 300 ), double frequency = 10.0 )
  {
    rclcpp::Rate rate( frequency );
    auto start_time = std::chrono::steady_clock::now();
    while ( rclcpp::ok() ) {
      if ( subscriber_ && subscriber_->get_publisher_count() > 0 ) {
        return true;
      }
      if ( std::chrono::steady_clock::now() - start_time > timeout ) {
        return false;
      }
      rate.sleep();
    }
    return false;
  }

private:
  mutable std::mutex mutex_;
  MessageSharedPtr msg_;
  bool msg_received_;
  std::string topic_name_;
  typename rclcpp::Subscription<MsgType>::SharedPtr subscriber_;
};

class TestClient
{
public:
  explicit TestClient( const rclcpp::NodeOptions &options = rclcpp::NodeOptions() )
      : node_( std::make_shared<rclcpp::Node>( "test_client", options ) ), e_stop_list_msgs_( node_, "/" + NODE_NAME + "/e_stop_list" ),
        managed_topic_msgs_( node_, "/" + NODE_NAME + "/" + MANAGED_ESTOP )
  {
    client_ = node_->create_client<e_stop_manager_msgs::srv::SetEStop>( "/"+ NODE_NAME + "/set_e_stop" );

    for ( const auto &aggregated_name : AGGREGATED_NAMES ) {
      aggregated_msgs_.emplace( aggregated_name,
                                std::make_shared<MsgContainer<std_msgs::msg::Bool>>( node_, aggregatedTopicName( aggregated_name ) ) );
    }

    for ( const auto &entry : E_STOP_CONFIG ) {
      e_stop_state_[entry.first] = false;
      if ( entry.second.tracked ) {
        tracked_publishers_[entry.first] = node_->create_publisher<std_msgs::msg::Bool>(
            "/" + NODE_NAME + "/" + entry.first, rclcpp::QoS( rclcpp::KeepLast( 10 ) ).reliable().transient_local() );
      }
    }
  }

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface() const { return this->node_->get_node_base_interface(); }

  void waitForConnection()
  {
    for ( const auto &pair : aggregated_msgs_ ) { ASSERT_TRUE( pair.second->waitForPublisher( 1s ) ); }
    ASSERT_TRUE( e_stop_list_msgs_.waitForPublisher( 1s ) );
    ASSERT_TRUE( managed_topic_msgs_.waitForPublisher( 1s ) );
    ASSERT_TRUE( client_->wait_for_service( 1s ) );

    for ( const auto &pub : tracked_publishers_ ) {
      auto start_time = std::chrono::steady_clock::now();
      while ( pub.second->get_subscription_count() == 0 ) {
        ASSERT_LT( std::chrono::steady_clock::now() - start_time, 1s );
        std::this_thread::sleep_for( 10ms );
      }
    }
  }

  void resetStartState()
  {
    for ( auto &state : e_stop_state_ ) { state.second = false; }
    for ( const auto &pub : tracked_publishers_ ) { publishTracked( pub.first, false ); }
    callManagedEStop( false );
  }

  void resetAllMsgContainers()
  {
    for ( const auto &pair : aggregated_msgs_ ) { pair.second->reset(); }
    managed_topic_msgs_.reset();
    e_stop_list_msgs_.reset();
  }

  void publishTracked( const std::string &name, bool value )
  {
    ASSERT_TRUE( tracked_publishers_.count( name ) > 0 );
    std_msgs::msg::Bool msg;
    msg.data = value;
    tracked_publishers_[name]->publish( msg );
    e_stop_state_[name] = value;
  }

  void callManagedEStop( bool requested_state )
  {
    auto request = std::make_shared<e_stop_manager_msgs::srv::SetEStop::Request>();
    request->name = MANAGED_ESTOP;
    request->value = requested_state;
    auto result = client_->async_send_request( request );
    ASSERT_EQ( result.wait_for( 200ms ), std::future_status::ready );
    auto response = result.get();
    EXPECT_EQ( response->result, e_stop_manager_msgs::srv::SetEStop::Response::SUCCESS );
    e_stop_state_[MANAGED_ESTOP] = requested_state;
  }

  void callTrackedExpectFailure( const std::string &name, bool requested_state )
  {
    auto request = std::make_shared<e_stop_manager_msgs::srv::SetEStop::Request>();
    request->name = name;
    request->value = requested_state;
    auto result = client_->async_send_request( request );
    ASSERT_EQ( result.wait_for( 200ms ), std::future_status::ready );
    auto response = result.get();
    EXPECT_EQ( response->result, e_stop_manager_msgs::srv::SetEStop::Response::FAILURE );
  }

  void callInvalidName()
  {
    auto request = std::make_shared<e_stop_manager_msgs::srv::SetEStop::Request>();
    request->name = "does_not_exist";
    request->value = true;
    auto result = client_->async_send_request( request );
    ASSERT_EQ( result.wait_for( 200ms ), std::future_status::ready );
    auto response = result.get();
    EXPECT_EQ( response->result, e_stop_manager_msgs::srv::SetEStop::Response::INVALID_ESTOP_NAME );
  }

  bool expectedAggregatedState( const std::string &topic ) const
  {
    for ( const auto &entry : E_STOP_CONFIG ) {
      auto normalized = entry.second.aggregated_topic;
      if ( !normalized.empty() && normalized.front() == '/' ) {
        normalized.erase( normalized.begin() );
      }
      std::replace( normalized.begin(), normalized.end(), '-', '_' );
      if ( normalized == topic && e_stop_state_.at( entry.first ) ) {
        return true;
      }
    }
    return false;
  }

  void waitForMsgAndVerifyState( bool expect_managed_topic )
  {
    for ( const auto &topic : AGGREGATED_NAMES ) {
      EXPECT_TRUE( aggregated_msgs_.at( topic )->waitForMessage() );
      auto msg = aggregated_msgs_.at( topic )->getCurrentMessage();
      ASSERT_TRUE( msg );
      EXPECT_EQ( expectedAggregatedState( topic ), msg->data );
    }

    if ( expect_managed_topic ) {
      EXPECT_TRUE( managed_topic_msgs_.waitForMessage() );
      auto msg = managed_topic_msgs_.getCurrentMessage();
      ASSERT_TRUE( msg );
      EXPECT_EQ( e_stop_state_[MANAGED_ESTOP], msg->data );
    }

    EXPECT_TRUE( e_stop_list_msgs_.waitForMessage() );
    auto list_msg = e_stop_list_msgs_.getCurrentMessage();
    ASSERT_TRUE( list_msg );

    ASSERT_EQ( list_msg->names.size(), list_msg->values.size() );
    ASSERT_EQ( list_msg->names.size(), E_STOP_CONFIG.size() );
    for ( size_t i = 0; i < list_msg->names.size(); ++i ) {
      const auto &name = list_msg->names[i];
      ASSERT_TRUE( E_STOP_CONFIG.count( name ) > 0 );
      EXPECT_EQ( list_msg->values[i], e_stop_state_[name] );
    }

    ASSERT_EQ( list_msg->aggregated_names.size(), list_msg->aggregated_values.size() );
    for ( size_t i = 0; i < list_msg->aggregated_names.size(); ++i ) {
      const auto &aggregated_name = list_msg->aggregated_names[i];
      if ( std::find( AGGREGATED_NAMES.begin(), AGGREGATED_NAMES.end(), aggregated_name ) == AGGREGATED_NAMES.end() ) {
        continue;
      }
      EXPECT_EQ( list_msg->aggregated_values[i], expectedAggregatedState( aggregated_name ) );
    }
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<e_stop_manager_msgs::srv::SetEStop>::SharedPtr client_;
  std::map<std::string, std::shared_ptr<MsgContainer<std_msgs::msg::Bool>>> aggregated_msgs_;
  MsgContainer<std_msgs::msg::Bool> managed_topic_msgs_;
  MsgContainer<e_stop_manager_msgs::msg::EStopList> e_stop_list_msgs_;
  std::map<std::string, rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr> tracked_publishers_;
  std::map<std::string, bool> e_stop_state_;
};

class EStopManagerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    test_client_ = std::make_shared<TestClient>();
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node( test_client_->get_node_base_interface() );
    executor_thread_ = std::thread( [this]() { this->executor_->spin(); } );
  }

  void TearDown() override
  {
    test_client_.reset();
    executor_->cancel();
    std::this_thread::sleep_for( std::chrono::milliseconds( 100 ) );
    if ( executor_thread_.joinable() ) {
      executor_thread_.join();
    }
    executor_.reset();
  }

  std::shared_ptr<TestClient> test_client_;
  rclcpp::Executor::SharedPtr executor_;
  std::thread executor_thread_;
};

TEST_F( EStopManagerTest, TestConnection ) { test_client_->waitForConnection(); }

TEST_F( EStopManagerTest, TrackedSourcesUpdateAggregates )
{
  test_client_->waitForConnection();
  test_client_->resetStartState();

  for ( const auto &entry : E_STOP_CONFIG ) {
    if ( !entry.second.tracked ) {
      continue;
    }
    test_client_->resetAllMsgContainers();
    test_client_->publishTracked( entry.first, true );
    test_client_->waitForMsgAndVerifyState( false );

    test_client_->resetAllMsgContainers();
    test_client_->publishTracked( entry.first, false );
    test_client_->waitForMsgAndVerifyState( false );
  }
}

TEST_F( EStopManagerTest, ManagedServiceUpdatesState )
{
  test_client_->waitForConnection();
  test_client_->resetStartState();

  test_client_->resetAllMsgContainers();
  test_client_->callManagedEStop( true );
  test_client_->waitForMsgAndVerifyState( true );

  test_client_->resetAllMsgContainers();
  test_client_->callManagedEStop( false );
  test_client_->waitForMsgAndVerifyState( true );
}

TEST_F( EStopManagerTest, ServiceRejectsTrackedEStop )
{
  test_client_->waitForConnection();
  test_client_->resetStartState();
  test_client_->resetAllMsgContainers();

  test_client_->callTrackedExpectFailure( "hard_remote_e_stop", true );

  for ( const auto &topic : AGGREGATED_NAMES ) { EXPECT_FALSE( test_client_->aggregated_msgs_.at( topic )->waitForMessage( 200ms ) ); }
  EXPECT_FALSE( test_client_->e_stop_list_msgs_.waitForMessage( 200ms ) );
}

TEST_F( EStopManagerTest, InvalidEStopName )
{
  test_client_->waitForConnection();
  test_client_->resetStartState();
  test_client_->resetAllMsgContainers();

  test_client_->callInvalidName();

  for ( const auto &topic : AGGREGATED_NAMES ) { EXPECT_FALSE( test_client_->aggregated_msgs_.at( topic )->waitForMessage( 200ms ) ); }
  EXPECT_FALSE( test_client_->e_stop_list_msgs_.waitForMessage( 200ms ) );
}

int main( int argc, char **argv )
{
  ::testing::InitGoogleTest( &argc, argv );
  rclcpp::init( argc, argv );

  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
