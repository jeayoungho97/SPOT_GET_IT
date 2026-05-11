#include "actuator_bridge/spi_transport.hpp"

#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>

namespace actuator_bridge
{

SpiTransport::~SpiTransport()
{
  close_device();
}

bool SpiTransport::open_device(
  const std::string & device,
  uint32_t speed_hz,
  uint8_t mode,
  uint8_t bits_per_word)
{
  close_device();

  fd_ = ::open(device.c_str(), O_RDWR);
  if (fd_ < 0) {
    last_error_ = "failed to open " + device + ": " + std::strerror(errno);
    return false;
  }

  speed_hz_ = speed_hz;
  mode_ = mode;
  bits_per_word_ = bits_per_word;

  if (ioctl(fd_, SPI_IOC_WR_MODE, &mode_) < 0) {
    last_error_ = "SPI_IOC_WR_MODE failed: " + std::string(std::strerror(errno));
    close_device();
    return false;
  }

  if (ioctl(fd_, SPI_IOC_RD_MODE, &mode_) < 0) {
    last_error_ = "SPI_IOC_RD_MODE failed: " + std::string(std::strerror(errno));
    close_device();
    return false;
  }

  if (ioctl(fd_, SPI_IOC_WR_BITS_PER_WORD, &bits_per_word_) < 0) {
    last_error_ = "SPI_IOC_WR_BITS_PER_WORD failed: " + std::string(std::strerror(errno));
    close_device();
    return false;
  }

  if (ioctl(fd_, SPI_IOC_RD_BITS_PER_WORD, &bits_per_word_) < 0) {
    last_error_ = "SPI_IOC_RD_BITS_PER_WORD failed: " + std::string(std::strerror(errno));
    close_device();
    return false;
  }

  if (ioctl(fd_, SPI_IOC_WR_MAX_SPEED_HZ, &speed_hz_) < 0) {
    last_error_ = "SPI_IOC_WR_MAX_SPEED_HZ failed: " + std::string(std::strerror(errno));
    close_device();
    return false;
  }

  if (ioctl(fd_, SPI_IOC_RD_MAX_SPEED_HZ, &speed_hz_) < 0) {
    last_error_ = "SPI_IOC_RD_MAX_SPEED_HZ failed: " + std::string(std::strerror(errno));
    close_device();
    return false;
  }

  return true;
}

bool SpiTransport::is_open() const
{
  return fd_ >= 0;
}

bool SpiTransport::transfer(
  const std::vector<uint8_t> & tx,
  std::vector<uint8_t> & rx)
{
  if (fd_ < 0) {
    last_error_ = "SPI device is not open";
    return false;
  }

  if (tx.empty()) {
    last_error_ = "tx buffer is empty";
    return false;
  }

  rx.assign(tx.size(), 0x00);

  spi_ioc_transfer tr{};
  tr.tx_buf = reinterpret_cast<unsigned long>(tx.data());
  tr.rx_buf = reinterpret_cast<unsigned long>(rx.data());
  tr.len = static_cast<uint32_t>(tx.size());
  tr.speed_hz = speed_hz_;
  tr.bits_per_word = bits_per_word_;
  tr.delay_usecs = 0;
  tr.cs_change = 0;

  const int ret = ioctl(fd_, SPI_IOC_MESSAGE(1), &tr);
  if (ret < 1) {
    last_error_ = "SPI_IOC_MESSAGE failed: " + std::string(std::strerror(errno));
    return false;
  }

  return true;
}

void SpiTransport::close_device()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

std::string SpiTransport::last_error() const
{
  return last_error_;
}

}  // namespace actuator_bridge