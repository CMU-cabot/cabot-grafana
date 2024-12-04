#include "InfluxDB.hpp"

InfluxDB::InfluxDB(const std::string& host, const std::string& token, const std::string& org, const std::string& bucket)
  : host_(host), token_(token), org_(org), bucket_(bucket), curl_(nullptr), headers_(nullptr), is_connected_(false) {

  curl_global_init(CURL_GLOBAL_DEFAULT);
  curl_ = curl_easy_init();

  if (curl_) {
    std::string url = host_ + "/api/v2/write?org=" + escapeUrl(org_) + "&bucket=" + escapeUrl(bucket_) + "&precision=ns";
    headers_ = curl_slist_append(headers_, ("Authorization: Token " + token_).c_str());
    headers_ = curl_slist_append(headers_, "Content-Type: text/plain");
    headers_ = curl_slist_append(headers_, "Connection: keep-alive");

    curl_easy_setopt(curl_, CURLOPT_URL, url.c_str());
    curl_easy_setopt(curl_, CURLOPT_HTTPHEADER, headers_);
    curl_easy_setopt(curl_, CURLOPT_TIMEOUT, 5L);
    curl_easy_setopt(curl_, CURLOPT_CONNECTTIMEOUT, 5L);
    curl_easy_setopt(curl_, CURLOPT_MAXREDIRS, 3L);
    curl_easy_setopt(curl_, CURLOPT_FOLLOWLOCATION, 1L);
    curl_easy_setopt(curl_, CURLOPT_TCP_KEEPALIVE, 1L);

    is_connected_ = true;
  } else {
    log_error("Failed to initialize CURL");
  }
}

InfluxDB::~InfluxDB() {
  if (curl_) {
    curl_easy_cleanup(curl_);
  }
  if (headers_) {
    curl_slist_free_all(headers_);
  }
  curl_global_cleanup();
}

bool InfluxDB::sendData(const std::string& lineProtocolData) {
  if (!curl_ || !is_connected_) return false;
  curl_easy_setopt(curl_, CURLOPT_POSTFIELDS, lineProtocolData.c_str());
  std::string response_body;
  curl_easy_setopt(curl_, CURLOPT_WRITEFUNCTION, write_callback);
  curl_easy_setopt(curl_, CURLOPT_WRITEDATA, &response_body);
  CURLcode res = curl_easy_perform(curl_);
  long http_code = 0;
  curl_easy_getinfo(curl_, CURLINFO_RESPONSE_CODE, &http_code);
  if (res == CURLE_OK && http_code == 204) {
    return true;
  } else {
    log_error("CURL error: " + std::string(curl_easy_strerror(res)) + ", HTTP code: " + std::to_string(http_code) + ", response: " + response_body);
    return false;
  }
}

/*
void InfluxDB::sendDataInLoop(const std::vector<std::string>& lineProtocolDataList) {
  for (const auto& data : lineProtocolDataList) {
    if (!sendData(data)) {
      log_error("Failed to send data");
      break;
    }
  }
}
*/

size_t InfluxDB::write_callback(void* contents, size_t size, size_t nmemb, void* userp) {
  ((std::string*)userp)->append((char*)contents, size * nmemb);
  return size * nmemb;
}

std::string InfluxDB::escapeUrl(const std::string& str) const {
  char* escaped = curl_easy_escape(nullptr, str.c_str(), str.length());
  if (escaped) {
    std::string escapedStr(escaped);
    curl_free(escaped);
    return escapedStr;
  }
  return "";
}

void InfluxDB::log_error(const std::string& message) const {
  std::cerr << "ERROR: " << message << std::endl;
}

void InfluxDB::log_warning(const std::string& message) const {
  std::cerr << "WARNING: " << message << std::endl;
}

void InfluxDB::log_info(const std::string& message) const {
  std::cout << "INFO: " << message << std::endl;
}
