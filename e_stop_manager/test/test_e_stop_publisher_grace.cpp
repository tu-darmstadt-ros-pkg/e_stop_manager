#include <chrono>
#include <gtest/gtest.h>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <e_stop_manager/e_stop_manager.h>
#include <e_stop_manager/publisher_grace.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

using namespace std::chrono_literals;
using std_msgs::msg::Bool;

namespace
{
const auto LATCHED_QOS = rclcpp::QoS( rclcpp::KeepLast( 1 ) ).reliable().transient_local();
const std::string REMOTE_TOPIC = "/e_stop_manager/remote";

rclcpp::NodeOptions options( const std::vector<rclcpp::Parameter> &overrides )
{
  std::vector<rclcpp::Parameter> parameters = {
      rclcpp::Parameter( "e_stop_names", std::vector<std::string>{ "remote", "ui" } ),
      rclcpp::Parameter( "e_stop_config.remote.aggregated_topic", "emergency_stop_hardware" ),
      rclcpp::Parameter( "e_stop_config.remote.tracked_topic", true ),
      rclcpp::Parameter( "e_stop_config.ui.aggregated_topic", "emergency_stop_software" ),
      rclcpp::Parameter( "e_stop_config.ui.tracked_topic", false ),
  };
  parameters.insert( parameters.end(), overrides.begin(), overrides.end() );
  return rclcpp::NodeOptions().parameter_overrides( parameters );
}
} // namespace

class EStopPublisherGraceTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() { rclcpp::init( 0, nullptr ); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void start( const std::vector<rclcpp::Parameter> &overrides )
  {
    manager_ = std::make_unique<e_stop_manager::EStopManager>( options( overrides ) );
    client_ = std::make_shared<rclcpp::Node>( "publisher_grace_test_client" );
    hardware_sub_ = client_->create_subscription<Bool>( "/e_stop_manager/aggregated_state/emergency_stop_hardware", LATCHED_QOS,
                                                        [this]( const Bool &msg ) {
                                                          std::lock_guard<std::mutex> lock( mutex_ );
                                                          ++hardware_count_;
                                                          hardware_engaged_ = msg.data;
                                                          hardware_engaged_count_ += msg.data ? 1 : 0;
                                                        } );
    executor_.add_node( manager_->get_node_base_interface() );
    executor_.add_node( client_ );
    spin_thread_ = std::thread( [this] { executor_.spin(); } );
  }

  void TearDown() override
  {
    if ( !spin_thread_.joinable() ) {
      return;
    }
    // spin() sets the spinning flag again when it starts, so a cancel() before that is lost.
    const auto deadline = std::chrono::steady_clock::now() + 2s;
    while ( !executor_.is_spinning() && std::chrono::steady_clock::now() < deadline ) { std::this_thread::sleep_for( 1ms ); }
    executor_.cancel();
    spin_thread_.join();
  }

  rclcpp::Publisher<Bool>::SharedPtr createRemotePublisher()
  {
    auto publisher = client_->create_publisher<Bool>( REMOTE_TOPIC, LATCHED_QOS );
    EXPECT_TRUE( waitFor( [&publisher] { return publisher->get_subscription_count() > 0; }, 2s ) );
    return publisher;
  }

  template<typename Predicate>
  bool waitFor( Predicate predicate, std::chrono::milliseconds timeout )
  {
    const auto end = std::chrono::steady_clock::now() + timeout;
    while ( std::chrono::steady_clock::now() < end ) {
      {
        std::lock_guard<std::mutex> lock( mutex_ );
        if ( predicate() ) {
          return true;
        }
      }
      std::this_thread::sleep_for( 10ms );
    }
    return false;
  }

  bool hardwareEngaged()
  {
    std::lock_guard<std::mutex> lock( mutex_ );
    return hardware_engaged_;
  }

  int hardwareEngagedCount()
  {
    std::lock_guard<std::mutex> lock( mutex_ );
    return hardware_engaged_count_;
  }

  //! Releases the remote source with a publisher that then leaves, and waits for the grace to engage it.
  void releaseAndLosePublisher()
  {
    auto publisher = createRemotePublisher();
    publisher->publish( Bool().set__data( false ) );
    ASSERT_TRUE( waitFor( [this] { return !hardware_engaged_; }, 2s ) );
    // Several publisher checks count the publisher, so the grace runs without relying on the message.
    std::this_thread::sleep_for( 300ms );
    publisher.reset();
    ASSERT_TRUE( waitFor( [this] { return hardware_engaged_; }, 3s ) );
  }

  rclcpp::executors::SingleThreadedExecutor executor_;
  std::unique_ptr<e_stop_manager::EStopManager> manager_;
  rclcpp::Node::SharedPtr client_;
  rclcpp::Subscription<Bool>::SharedPtr hardware_sub_;
  std::thread spin_thread_;
  std::mutex mutex_;
  int hardware_count_ = 0;
  bool hardware_engaged_ = false;
  int hardware_engaged_count_ = 0;
};

TEST_F( EStopPublisherGraceTest, RejectsPublisherGraceBelowMinimum )
{
  EXPECT_THROW( std::make_unique<e_stop_manager::EStopManager>( options( { rclcpp::Parameter( "publisher_grace_ms", 99 ) } ) ),
                std::invalid_argument );
  EXPECT_THROW( std::make_unique<e_stop_manager::EStopManager>( options( { rclcpp::Parameter( "publisher_grace_ms", -1 ) } ) ),
                std::invalid_argument );
}

TEST_F( EStopPublisherGraceTest, UnsetGraceKeepsReleaseAfterPublisherLeaves )
{
  start( {} );
  auto publisher = createRemotePublisher();
  publisher->publish( Bool().set__data( false ) );
  ASSERT_TRUE( waitFor( [this] { return !hardware_engaged_; }, 2s ) );
  publisher.reset();

  std::this_thread::sleep_for( 1500ms );
  EXPECT_EQ( hardwareEngagedCount(), 0 );
}

TEST_F( EStopPublisherGraceTest, SourceWithoutPublisherKeepsInitialValue )
{
  start( { rclcpp::Parameter( "publisher_grace_ms", 200 ) } );
  ASSERT_TRUE( waitFor( [this] { return hardware_count_ >= 1; }, 2s ) );

  std::this_thread::sleep_for( 1s );
  EXPECT_EQ( hardwareEngagedCount(), 0 );
}

TEST_F( EStopPublisherGraceTest, LostPublisherTriggersTrackedSourceUntilReleased )
{
  start( { rclcpp::Parameter( "publisher_grace_ms", 200 ) } );
  releaseAndLosePublisher();

  auto publisher = createRemotePublisher();
  publisher->publish( Bool().set__data( false ) );
  EXPECT_TRUE( waitFor( [this] { return !hardware_engaged_; }, 2s ) );
}

TEST_F( EStopPublisherGraceTest, OneShotReleaseLastsOnlyTheGrace )
{
  start( { rclcpp::Parameter( "publisher_grace_ms", 200 ) } );
  auto publisher = createRemotePublisher();
  publisher->publish( Bool().set__data( false ) );
  publisher.reset();

  std::this_thread::sleep_for( 500ms );
  EXPECT_TRUE( waitFor( [this] { return hardware_engaged_; }, 2s ) );
}

TEST_F( EStopPublisherGraceTest, ReleaseHoldsWhilePublisherStays )
{
  start( { rclcpp::Parameter( "publisher_grace_ms", 200 ) } );
  auto publisher = createRemotePublisher();
  publisher->publish( Bool().set__data( false ) );
  ASSERT_TRUE( waitFor( [this] { return !hardware_engaged_; }, 2s ) );

  std::this_thread::sleep_for( 1s );
  EXPECT_EQ( hardwareEngagedCount(), 0 );
}

TEST_F( EStopPublisherGraceTest, ReleaseRefreshesTheGrace )
{
  start( { rclcpp::Parameter( "publisher_grace_ms", 1000 ) } );
  // An expired grace, so only the release itself can hold the source.
  releaseAndLosePublisher();

  auto one_shot = createRemotePublisher();
  one_shot->publish( Bool().set__data( false ) );
  one_shot.reset();
  ASSERT_TRUE( waitFor( [this] { return !hardware_engaged_; }, 2s ) );

  const int engaged_before = hardwareEngagedCount();
  std::this_thread::sleep_for( 500ms );
  EXPECT_EQ( hardwareEngagedCount(), engaged_before );
}

using GraceClock = e_stop_manager::PublisherGrace::Clock;

TEST( PublisherGraceTest, NeverExpiresWithoutPublisherOrMessage )
{
  const GraceClock::time_point t0;
  e_stop_manager::PublisherGrace grace( 1000ms );
  EXPECT_FALSE( grace.expired( 0, t0 ) );
  EXPECT_FALSE( grace.expired( 0, t0 + 60s ) );
}

TEST( PublisherGraceTest, MessageBeforePublisherIsCountedKeepsSource )
{
  const GraceClock::time_point t0;
  e_stop_manager::PublisherGrace grace( 1000ms );
  grace.messageReceived( t0 + 5s );
  EXPECT_FALSE( grace.expired( 0, t0 + 5s + 100ms ) );
  EXPECT_FALSE( grace.expired( 1, t0 + 5s + 200ms ) );
  EXPECT_FALSE( grace.expired( 1, t0 + 60s ) );
}

TEST( PublisherGraceTest, OneShotMessageExpiresAfterGrace )
{
  const GraceClock::time_point t0;
  e_stop_manager::PublisherGrace grace( 1000ms );
  grace.messageReceived( t0 + 5s );
  EXPECT_FALSE( grace.expired( 0, t0 + 6s ) );
  EXPECT_TRUE( grace.expired( 0, t0 + 6s + 1ms ) );
}

TEST( PublisherGraceTest, LostPublisherExpiresAfterGrace )
{
  const GraceClock::time_point t0;
  e_stop_manager::PublisherGrace grace( 1000ms );
  EXPECT_FALSE( grace.expired( 1, t0 + 5s ) );
  EXPECT_FALSE( grace.expired( 0, t0 + 6s ) );
  EXPECT_TRUE( grace.expired( 0, t0 + 6s + 1ms ) );
}

TEST( PublisherGraceTest, MessageAfterExpiryRefreshesGrace )
{
  const GraceClock::time_point t0;
  e_stop_manager::PublisherGrace grace( 1000ms );
  EXPECT_FALSE( grace.expired( 1, t0 ) );
  EXPECT_TRUE( grace.expired( 0, t0 + 2s ) );
  grace.messageReceived( t0 + 3s );
  EXPECT_FALSE( grace.expired( 0, t0 + 3s + 500ms ) );
}
