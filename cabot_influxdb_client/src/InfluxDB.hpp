/*******************************************************************************
 * Copyright (c) 2024  Carnegie Mellon University and Miraikan
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *******************************************************************************/

#ifndef INFLUXDB_HPP
#define INFLUXDB_HPP

#include <curl/curl.h>
#include <iostream>
#include <string>
#include <cstring>
#include <thread>
#include <atomic>

class InfluxDB {
public:
  InfluxDB(const std::string& host, const std::string& token, const std::string& org, const std::string& bucket);
  ~InfluxDB();
  bool setupCurl();
  void resetConnection();
  bool sendData(const std::string& lineProtocolData, int max_retries = 5);
  bool pingServer();
  void healthCheck(int interval_seconds);

private:
  static size_t write_callback(void* contents, size_t size, size_t nmemb, void* userp);
  std::string escapeUrl(const std::string& str) const;
  void log_error(const std::string& message) const;
  void log_warning(const std::string& message) const;
  void log_info(const std::string& message) const;

  std::string host_, token_, org_, bucket_;
  CURL* curl_;
  struct curl_slist* headers_ = nullptr;
  std::atomic<int> backoff_ms{1000};
  std::thread healthCheckThread_;
  std::atomic<bool> keep_running_{true};
};

#endif // INFLUXDB_HPP
