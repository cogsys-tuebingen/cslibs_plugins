#ifndef CSLIBS_PLUGINS_DATA_HPP
#define CSLIBS_PLUGINS_DATA_HPP

#include <assert.h>

#include <cslibs_time/time_frame.hpp>
#include <memory>

namespace cslibs_plugins_data {
class Data {
 public:
  using Ptr = std::shared_ptr<Data>;
  using ConstPtr = std::shared_ptr<const Data>;

  Data(const std::string &_frame) : frame_{_frame} {}

  Data(const std::string &frame, const cslibs_time::TimeFrame &time_frame,
       const cslibs_time::Time &time_received)
      : frame_{frame}, time_frame_{time_frame}, time_received_{time_received} {}

  virtual ~Data() = default;

  inline std::string const &frame() const { return frame_; }

  inline cslibs_time::TimeFrame const &timeFrame() const { return time_frame_; }

  inline cslibs_time::Time const &stampReceived() const {
    return time_received_;
  }

  template <typename T>
  inline bool isType() const {
    const T *t = dynamic_cast<const T *>(this);
    return t != nullptr;
  }

  template <typename T>
  inline T const &as() const {
    return dynamic_cast<const T &>(*this);
  }

 protected:
  Data() = delete;
  inline Data(const Data &other) = default;
  inline Data(Data &&other) = default;

  std::string frame_;
  cslibs_time::TimeFrame time_frame_;
  cslibs_time::Time time_received_;
};
}  // namespace cslibs_plugins_data

#endif  // CSLIBS_PLUGINS_DATA_HPP
