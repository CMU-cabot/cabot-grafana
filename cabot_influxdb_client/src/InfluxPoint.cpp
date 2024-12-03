#include "InfluxPoint.hpp"

InfluxPoint::InfluxPoint(const std::string& measurement)
  : measurement_(measurement) {}

InfluxPoint& InfluxPoint::addField(const std::string& field, const fieldValue& value) {
  if (!field.empty()) {
    fields_[field] = value;
  }
  return *this;
}

InfluxPoint& InfluxPoint::addTag(const std::string& tag, const std::string& value) {
  if (!tag.empty() && !value.empty()) {
    tags_[tag] = value;
  }
  return *this;
}

InfluxPoint& InfluxPoint::setTimestamp(std::chrono::time_point<std::chrono::system_clock> timestamp) {
  timestamp_ = timestamp;
  return *this;
}

std::string InfluxPoint::toLineProtocol() const {
  std::ostringstream line;
  line << escape(measurement_);
  for (const auto& tag : tags_) {
    line << "," << escape(tag.first) << "=" << escape(tag.second);
  }
  if (!fields_.empty()) {
    bool firstField = true;
    line << " ";
    for (const auto& field : fields_) {
      if (!firstField) line << ",";
      line << escape(field.first) << "=" << formatValue(field.second);
      firstField = false;
    }
  }
  if (timestamp_) {
    long long timestamp_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      *timestamp_ - std::chrono::system_clock::from_time_t(0)).count();
    line << " " << timestamp_ns;
  }
  return line.str();
}

std::string InfluxPoint::escape(const std::string& input) {
  std::ostringstream output;
  for (char c : input) {
    if (c == ',' || c == '=' || c == ' ' || c == '"' || c == '\\') {
      output << '\\';
    }
    output << c;
  }
  return output.str();
}

std::string InfluxPoint::formatValue(const fieldValue& value) {
  std::ostringstream line;
  std::visit([&line](const auto& v) {
    using T = std::decay_t<decltype(v)>;
    if constexpr (std::is_same_v<T, std::string>) {
      line << "\"" << v << "\"";
    } else if constexpr (std::is_same_v<T, bool>) {
      line << (v ? "true" : "false");
    } else if constexpr (std::is_integral_v<T>) {
      line << v << "i";
    } else if constexpr (std::is_floating_point_v<T>) {
      line << std::fixed << std::setprecision(10) << v;
    } else {
      line << v;
    }
  }, value);
  return line.str();
}

