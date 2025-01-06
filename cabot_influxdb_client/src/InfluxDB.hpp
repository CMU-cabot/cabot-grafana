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
