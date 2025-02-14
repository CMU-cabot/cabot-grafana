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

#include <algorithm>
#include "InfluxDB.hpp"

InfluxDB::InfluxDB(const std::string & host, const std::string & token, const std::string & org, const std::string & bucket)
: host_(host), token_(token), org_(org), bucket_(bucket), curl_(curl_easy_init())
{
  if (!setupCurl()) {
    log_error("Initialize CURL setup failed");
  }
}

InfluxDB::~InfluxDB()
{
  keep_running_ = false;
  if (healthCheckThread_.joinable()) {
    healthCheckThread_.join();
  }
  if (curl_) {
    curl_easy_cleanup(curl_);
  }
  if (headers_) {
    curl_slist_free_all(headers_);
  }
}

bool InfluxDB::setupCurl()
{
  if (!curl_) {
    curl_ = curl_easy_init();
    if (!curl_) {
      log_error("Failed to initialize CURL");
      return false;
    }
    // Initial setup only
    curl_easy_setopt(curl_, CURLOPT_TIMEOUT, 5L);
    curl_easy_setopt(curl_, CURLOPT_CONNECTTIMEOUT, 5L);
    curl_easy_setopt(curl_, CURLOPT_MAXREDIRS, 3L);
    curl_easy_setopt(curl_, CURLOPT_FOLLOWLOCATION, 1L);
    curl_easy_setopt(curl_, CURLOPT_TCP_KEEPALIVE, 1L);
  }

  std::string url = host_ + "/api/v2/write?org=" + escapeUrl(org_) + "&bucket=" + escapeUrl(bucket_) + "&precision=ns";
  curl_easy_setopt(curl_, CURLOPT_URL, url.c_str());

  if (headers_) {
    curl_slist_free_all(headers_);
    headers_ = nullptr;
  }
  headers_ = curl_slist_append(headers_, ("Authorization: Token " + token_).c_str());
  headers_ = curl_slist_append(headers_, "Content-Type: text/plain");
  headers_ = curl_slist_append(headers_, "Connection: keep-alive");
  curl_easy_setopt(curl_, CURLOPT_HTTPHEADER, headers_);
  return true;
}

void InfluxDB::resetConnection()
{
  if (curl_) {
    curl_easy_cleanup(curl_);
    curl_ = nullptr;
  }
  headers_ = nullptr;
  if (!setupCurl()) {
    log_error("Failed to reinitialize CURL during reset.");
  }
}

bool InfluxDB::sendData(const std::string & lineProtocolData, int max_retries)
{
  int backoff_ms = 1000;
  for (int attempt = 1; attempt <= max_retries; ++attempt) {
    if (!setupCurl()) {
      log_error("Failed to set up CURL connection");
      return false;
    }
    curl_easy_setopt(curl_, CURLOPT_POSTFIELDS, lineProtocolData.c_str());
    curl_easy_setopt(curl_, CURLOPT_POSTFIELDSIZE, lineProtocolData.size());
    std::string response_body;
    curl_easy_setopt(curl_, CURLOPT_WRITEFUNCTION, write_callback);
    curl_easy_setopt(curl_, CURLOPT_WRITEDATA, &response_body);
    CURLcode res = curl_easy_perform(curl_);
    int64_t http_code = 0;
    curl_easy_getinfo(curl_, CURLINFO_RESPONSE_CODE, &http_code);
    if (res == CURLE_OK && http_code == 204) {
      // log_info("Data sent successfully");
      return true;
    }

    log_error("Attempt (" + std::to_string(attempt) + "/" + std::to_string(max_retries) + "failed: " + std::string(curl_easy_strerror(res)) + ", HTTP code: " + std::to_string(http_code));

    if (attempt < max_retries) {
      std::this_thread::sleep_for(std::chrono::milliseconds(backoff_ms));
      backoff_ms = std::min(backoff_ms * 2, 10000);
      resetConnection();
    }
  }
  log_error("Failed to send data after " + std::to_string(max_retries) + " attempts.");
  return false;
}

size_t InfluxDB::write_callback(void * contents, size_t size, size_t nmemb, void * userp)
{
  if (userp) {
    ((std::string *)userp)->append(reinterpret_cast<char *>(contents), size * nmemb);
  }
  return size * nmemb;
}

std::string InfluxDB::escapeUrl(const std::string & str) const
{
  char * escaped = curl_easy_escape(curl_, str.c_str(), static_cast<int>(str.size()));
  if (escaped) {
    std::string escapedStr(escaped);
    curl_free(escaped);
    return escapedStr;
  }
  return "";
}

bool InfluxDB::pingServer()
{
  if (!curl_) {
    curl_ = curl_easy_init();
  }
  CURL * ping_curl = curl_easy_init();
  if (!ping_curl) {
    log_error("Failed to initialize CURL for ping.");
    return false;
  }
  std::string ping_url = host_ + "/ping";
  curl_easy_setopt(ping_curl, CURLOPT_URL, ping_url.c_str());
  curl_easy_setopt(ping_curl, CURLOPT_NOBODY, 1L);
  curl_easy_setopt(ping_curl, CURLOPT_TIMEOUT, 5L);
  CURLcode res = curl_easy_perform(ping_curl);
  curl_easy_cleanup(ping_curl);
  if (res == CURLE_OK) {
    return true;
  } else {
    log_warning("Ping to server failed: " + std::string(curl_easy_strerror(res)));
    return false;
  }
}

void InfluxDB::healthCheck(int interval_seconds)
{
  healthCheckThread_ = std::thread(
    [this, interval_seconds]() {
      while (keep_running_) {
        if (!pingServer()) {
          log_warning("Server is unreachable. Resetting connction.");
          resetConnection();
        }
        std::this_thread::sleep_for(std::chrono::seconds(interval_seconds));
      }
    });
  healthCheckThread_.detach();
}

void InfluxDB::log_error(const std::string & message) const
{
  std::cerr << "ERROR: " << message << std::endl;
}

void InfluxDB::log_warning(const std::string & message) const
{
  std::cerr << "WARNING: " << message << std::endl;
}

void InfluxDB::log_info(const std::string & message) const
{
  std::cout << "INFO: " << message << std::endl;
}
