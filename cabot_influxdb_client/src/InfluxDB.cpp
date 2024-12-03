#include "InfluxDB.hpp"

InfluxDB::InfluxDB(const std::string& host, const std::string& token, const std::string& org, const std::string& bucket)
  : host_(host), token_(token), org_(org), bucket_(bucket) {}

bool InfluxDB::sendData(const std::string& lineProtocolData) {
  CURL* curl = curl_easy_init();
  if (!curl) {
    log_error("Failed to initialize CURL");
    return false;
  }

  std::function<void(CURL*)> cleanup = [](CURL* curl) { curl_easy_cleanup(curl); };
  std::unique_ptr<CURL, std::function<void(CURL*)>> curlGuard(curl, cleanup);

  std::string url = host_ + "/api/v2/write?org=" + escapeUrl(org_) + "&bucket=" + escapeUrl(bucket_) + "&precision=ns";
  struct curl_slist* headers = nullptr;
  headers = curl_slist_append(headers, ("Authorization: Token " + token_).c_str());
  headers = curl_slist_append(headers, "Content-Type: text/plain");

  curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
  curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
  curl_easy_setopt(curl, CURLOPT_POSTFIELDS, lineProtocolData.c_str());
  curl_easy_setopt(curl, CURLOPT_TIMEOUT, 5L);
  curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, 5L);
  curl_easy_setopt(curl, CURLOPT_MAXREDIRS, 3L);
  curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);

  std::string response_body;
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, write_callback);
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response_body);

  CURLcode res = curl_easy_perform(curl);
  long http_code = 0;
  curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &http_code);

  bool success = false;
  if (res == CURLE_OK) {
    if (http_code == 204) {
      success = true;
    } else {
      log_error("HTTP error, code: " + std::to_string(http_code) + ", response: " + response_body);
    }
  } else {
    log_error("CURL error: " + std::string(curl_easy_strerror(res)) + ", HTTP code: " + std::to_string(http_code) + ", response: " + response_body);
  }

  curl_slist_free_all(headers);
  return success;
}

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

