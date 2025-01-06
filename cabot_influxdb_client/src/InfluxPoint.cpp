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
  std::ostringstream oss;
  oss << escape(measurement_);
  for (const auto& tag : tags_) {
    oss << "," << escape(tag.first) << "=" << escape(tag.second);
  }
  if (!fields_.empty()) {
    bool firstField = true;
    oss << " ";
    for (const auto& field : fields_) {
      if (!firstField) {
        oss << ",";
      }
      oss << escape(field.first) << "=" << formatValue(field.second);
      firstField = false;
    }
  }
  if (timestamp_) {
    long long timestamp_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      *timestamp_ - std::chrono::system_clock::from_time_t(0)).count();
    oss << " " << timestamp_ns;
  }
  return oss.str();
}

std::string InfluxPoint::escape(const std::string& input) const {
  std::string output;
  output.reserve(input.size());
  for (char c :input) {
    switch (c) {
      case ',':
      case '=':
      case ' ':
      case '"':
      case '\\':
        output += '\\';
        [[fallthrough]];
      default:
        output += c;
    }
  }
  return output;
}

std::string InfluxPoint::formatValue(const fieldValue& value) const {
  return std::visit([this](const auto& v) -> std::string {
    using T = std::decay_t<decltype(v)>;
    if constexpr (std::is_same_v<T, std::string>) {
      return "\"" + v + "\"";
    } else if constexpr (std::is_same_v<T, bool>) {
      return v ? "true" : "false";
    } else if constexpr (std::is_integral_v<T>) {
      return std::to_string(v) + "i";
    } else if constexpr (std::is_floating_point_v<T>) {
      return this->formatFloatingPoint(v);
    } else {
      return std::to_string(v);
    }
  }, value);
}

std::string InfluxPoint::formatFloatingPoint(double value, int precision) const {
  char buffer[64];
  auto result = std::to_chars(buffer, buffer + sizeof(buffer), value, std::chars_format::fixed, precision);
  if (result.ec == std::errc()) {
    return std::string(buffer, result.ptr);
  }
  return "0";
}
