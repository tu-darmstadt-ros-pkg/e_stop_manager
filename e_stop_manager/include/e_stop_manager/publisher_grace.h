#ifndef E_STOP_MANAGER_PUBLISHER_GRACE_H
#define E_STOP_MANAGER_PUBLISHER_GRACE_H

#include <chrono>
#include <cstddef>
#include <optional>

namespace e_stop_manager
{

/// A tracked source counts as gone once it had a publisher or a message and then neither a publisher was counted
/// nor a message arrived for longer than the grace. A source that never had either never expires.
/// The grace keeps a message that arrives before its publisher is counted, and catches a publisher that
/// releases and leaves before a check counted it.
class PublisherGrace
{
public:
  using Clock = std::chrono::steady_clock;

  explicit PublisherGrace( std::chrono::milliseconds grace ) : grace_( grace ) { }

  void messageReceived( Clock::time_point now ) { last_alive_ = now; }

  bool expired( std::size_t publisher_count, Clock::time_point now )
  {
    if ( publisher_count > 0 ) {
      last_alive_ = now;
    }
    return last_alive_ && now - *last_alive_ > grace_;
  }

private:
  std::chrono::milliseconds grace_;
  std::optional<Clock::time_point> last_alive_;
};
} // namespace e_stop_manager

#endif // E_STOP_MANAGER_PUBLISHER_GRACE_H
