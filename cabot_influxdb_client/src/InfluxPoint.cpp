//
// Copyright (c) 2024  Carnegie Mellon University and Miraikan
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#include "InfluxPoint.hpp"

InfluxPoint::InfluxPoint(const std::string & measurement)
: measurement_(measurement) {}

InfluxPoint & InfluxPoint::addField(const std::string & field, const fieldValue & value)
{
  if (!field.empty()) {fields_[field] = value;}
  return *this;
}

InfluxPoint & InfluxPoint::addTag(const std::string & tag, const std::string & value)
{
  if (!tag.empty() && !value.empty()) {tags_[tag] = value;}
  return *this;
}

InfluxPoint & InfluxPoint::setTimestamp(std::chrono::time_point<std::chrono::system_clock> timestamp)
{
  timestamp_ = timestamp;
  return *this;
}

std::string InfluxPoint::toLineProtocol() const
{
  std::ostringstream oss;
  oss << escape(measurement_);
  for (const auto & tag : tags_) {
    oss << "," << escape(tag.first) << "=" << escape(tag.second);
  }
  if (!fields_.empty()) {
    bool firstField = true;
    oss << " ";
    for (const auto & field : fields_) {
      if (!firstField) {
        oss << ",";
      }
      oss << escape(field.first) << "=" << formatValue(field.second);
      firstField = false;
    }
  }
  if (timestamp_) {
    int64_t timestamp_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      *timestamp_ - std::chrono::system_clock::from_time_t(0)).count();
    oss << " " << timestamp_ns;
  }
  return oss.str();
}

std::string InfluxPoint::escape(const std::string & input) const
{
  std::string output;
  output.reserve(input.size());
  for (char c : input) {
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

std::string InfluxPoint::formatValue(const fieldValue & value) const
{
  return std::visit(
    [this](const auto & v) -> std::string {
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

std::string InfluxPoint::formatFloatingPoint(double value, int precision) const
{
  char buffer[64];
  auto result = std::to_chars(buffer, buffer + sizeof(buffer), value, std::chars_format::fixed, precision);
  if (result.ec == std::errc()) {
    return std::string(buffer, result.ptr);
  }
  return "0";
}
