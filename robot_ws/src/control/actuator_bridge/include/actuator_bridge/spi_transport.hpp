#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace actuator_bridge
{

class SpiTransport
{
public:
  SpiTransport() = default;
  ~SpiTransport();

  bool open_device(
    const std::string & device,
    uint32_t speed_hz,
    uint8_t mode,
    uint8_t bits_per_word);

  bool is_open() const;

  bool transfer(
    const std::vector<uint8_t> & tx,
    std::vector<uint8_t> & rx);

  void close_device();

  std::string last_error() const;

private:
  int fd_{-1};
  uint32_t speed_hz_{1000000};
  uint8_t mode_{0};
  uint8_t bits_per_word_{8};
  std::string last_error_;
};

}  // namespace actuator_bridge