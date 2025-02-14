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

#ifndef INFLUXPOINT_HPP_
#define INFLUXPOINT_HPP_

#include <string>
#include <unordered_map>
#include <variant>
#include <sstream>
#include <chrono>
#include <optional>
#include <iomanip>
#include <charconv>

class InfluxPoint
{
public:
  using fieldValue = std::variant<int, int64_t, std::string, double, bool, unsigned int, uint64_t>;
  explicit InfluxPoint(const std::string & measurement);
  InfluxPoint & addField(const std::string & field, const fieldValue & value);
  InfluxPoint & addTag(const std::string & tag, const std::string & value);
  InfluxPoint & setTimestamp(std::chrono::time_point<std::chrono::system_clock> timestamp);
  std::string toLineProtocol() const;

private:
  std::string measurement_;
  std::unordered_map<std::string, std::string> tags_;
  std::unordered_map<std::string, fieldValue> fields_;
  std::optional<std::chrono::time_point<std::chrono::system_clock>> timestamp_;
  std::string escape(const std::string & input) const;
  std::string formatValue(const fieldValue & value) const;
  std::string formatFloatingPoint(double value, int precision = 6) const;
};

#endif  // INFLUXPOINT_HPP_
