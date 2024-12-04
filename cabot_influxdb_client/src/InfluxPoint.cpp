#include "InfluxPoint.hpp"

InfluxPoint::InfluxPoint(const std::string& measurement)
  : measurement_(measurement) {}

InfluxPoint& InfluxPoint::addField(const std::string& field, const fieldValue& value) {
  if (!field.empty()) fields_[field] = value;
  return *this;
}

InfluxPoint& InfluxPoint::addTag(const std::string& tag, const std::string& value) {
  if (!tag.empty() && !value.empty()) tags_[tag] = value;
  return *this;
}

InfluxPoint& InfluxPoint::setTimestamp(std::chrono::time_point<std::chrono::system_clock> timestamp) {
  timestamp_ = timestamp;
  return *this;
}

std::string InfluxPoint::toLineProtocol() const {
  std::string line = escape(measurement_);
  for (const auto& tag : tags_) {
    line += "," + escape(tag.first) + "=" + escape(tag.second);
  }
  if (!fields_.empty()) {
    bool firstField = true;
    line += " ";
    for (const auto& field : fields_) {
      line += (firstField ? "" : ",") + escape(field.first) + "=" + formatValue(field.second);
      firstField = false;
    }
  }
  if (timestamp_) {
    long long timestamp_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      *timestamp_ - std::chrono::system_clock::from_time_t(0)).count();
    line += " " + std::to_string(timestamp_ns);
  }
  return line;
}

std::string InfluxPoint::escape(const std::string& input) const {
  std::string output;
  output.reserve(input.size() * 2);
  for (char c : input) {
    if (c == ',' || c == '=' || c == ' ' || c == '"' || c == '\\') output += '\\';
    output += c;
  }
  return output;
}

std::string InfluxPoint::formatValue(const fieldValue& value) const {
  std::string  line;
  std::visit([this, &line](const auto& v) {
    using T = std::decay_t<decltype(v)>;
    if constexpr (std::is_same_v<T, std::string>) {
      line = "\"" + v + "\"";
    } else if constexpr (std::is_same_v<T, bool>) {
      line = v ? "true" : "false";
    } else if constexpr (std::is_integral_v<T>) {
      line = std::to_string(v) + "i";
    } else if constexpr (std::is_floating_point_v<T>) {
      line = this -> formatFloatingPoint(v);
    } else {
      line = std::to_string(v);
    }
  }, value);
  return line;
}

std::string InfluxPoint::formatFloatingPoint(double value, int precision) const {
  char buffer[64];
  char* end = buffer + sizeof(buffer);
  auto result = std::to_chars(buffer, end, value, std::chars_format::fixed, precision);
  if (result.ec == std::errc()) {
    return std::string(buffer, result.ptr);
  } else {
    throw std::runtime_error("Failed to format floating point number");
  }
}
