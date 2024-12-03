#ifndef INFLUXPOINT_H
#define INFLUXPOINT_H

#include <string>
#include <unordered_map>
#include <variant>
#include <sstream>
#include <chrono>
#include <optional>
#include <iomanip>

class InfluxPoint {
public:
  using fieldValue = std::variant<int, long long, std::string, double, bool, unsigned int, unsigned long long>;

  explicit InfluxPoint(const std::string& measurement);
  
  InfluxPoint& addField(const std::string& field, const fieldValue& value);
  InfluxPoint& addTag(const std::string& tag, const std::string& value);
  InfluxPoint& setTimestamp(std::chrono::time_point<std::chrono::system_clock> timestamp);
  
  std::string toLineProtocol() const;

private:
  std::string measurement_;
  std::unordered_map<std::string, std::string> tags_;
  std::unordered_map<std::string, fieldValue> fields_;
  std::optional<std::chrono::time_point<std::chrono::system_clock>> timestamp_;

  static std::string escape(const std::string& input);
  static std::string formatValue(const fieldValue& value);
};

#endif // INFLUXPOINT_H

