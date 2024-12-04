#ifndef INFLUXDB_H
#define INFLUXDB_H

#include <curl/curl.h>
#include <iostream>
#include <string>
#include <memory>
#include <functional>
#include <map>
#include <vector>

class InfluxDB {
public:
  InfluxDB(const std::string& host, const std::string& token, const std::string& org, const std::string& bucket);
  ~InfluxDB();

  bool sendData(const std::string& lineProtocolData);
  //void sendDataInLoop(const std::vector<std::string>& lineProtocolDataList);

private:
  static size_t write_callback(void* contents, size_t size, size_t nmemb, void* userp);

  std::string escapeUrl(const std::string& str) const;
  void log_error(const std::string& message) const;
  void log_warning(const std::string& message) const;
  void log_info(const std::string& message) const;

  std::string host_;
  std::string token_;
  std::string org_;
  std::string bucket_;
  CURL* curl_;
  struct curl_slist* headers_;
  bool is_connected_;

};

#endif // INFLUXDB_H
