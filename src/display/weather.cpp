#include "weather.h"

#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <float.h>

namespace {
bool getJson(const String &url, String &body, String &error) {
  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient http;
  http.setTimeout(12000);
  if (!http.begin(client, url)) {
    error = "HTTPS setup failed";
    return false;
  }
  const int code = http.GET();
  if (code != HTTP_CODE_OK) {
    error = "OpenWeather HTTP " + String(code);
    http.end();
    return false;
  }
  body = http.getString();
  http.end();
  return body.length() > 0;
}
}  // namespace

bool WeatherClient::update(const String &apiKey, const String &latitude, const String &longitude, bool imperial,
                           WeatherData &data, String &error) {
  const String query = "?lat=" + latitude + "&lon=" + longitude + "&appid=" + apiKey +
                       "&units=" + (imperial ? "imperial" : "metric");
  const String base = "https://api.openweathermap.org/data/2.5/";
  String body;
  if (!getJson(base + "weather" + query, body, error)) return false;
  JsonDocument currentDoc;
  if (deserializeJson(currentDoc, body) || currentDoc["main"]["temp"].isNull()) {
    error = "Invalid current-weather response";
    return false;
  }
  WeatherData next;
  next.current.temperature = currentDoc["main"]["temp"].as<float>();
  next.current.feelsLike = currentDoc["main"]["feels_like"] | NAN;
  next.current.humidity = currentDoc["main"]["humidity"] | NAN;
  next.current.windSpeed = currentDoc["wind"]["speed"] | 0.0f;
  next.current.conditionId = currentDoc["weather"][0]["id"] | 800;
  next.current.description = currentDoc["weather"][0]["description"].as<String>();
  next.current.valid = true;

  if (!getJson(base + "forecast" + query, body, error)) return false;
  JsonDocument forecastDoc;
  if (deserializeJson(forecastDoc, body) || !forecastDoc["list"].is<JsonArray>()) {
    error = "Invalid forecast response";
    return false;
  }
  struct Bin {
    int key = 0;
    time_t date = 0;
    float high = -FLT_MAX;
    float low = FLT_MAX;
    float precipitation = 0;
    int noonDistance = 25;
    int condition = 800;
    String description;
  } bins[6];
  int binCount = 0;
  time_t now = time(nullptr);
  tm today{};
  localtime_r(&now, &today);
  const int todayKey = (today.tm_year + 1900) * 10000 + (today.tm_mon + 1) * 100 + today.tm_mday;
  for (JsonObject item : forecastDoc["list"].as<JsonArray>()) {
    const time_t stamp = item["dt"] | 0;
    tm local{};
    localtime_r(&stamp, &local);
    const int key = (local.tm_year + 1900) * 10000 + (local.tm_mon + 1) * 100 + local.tm_mday;
    int b = 0;
    while (b < binCount && bins[b].key != key) ++b;
    if (b == binCount) {
      if (binCount == 6) continue;
      bins[b].key = key;
      bins[b].date = stamp;
      ++binCount;
    }
    bins[b].high = max(bins[b].high, item["main"]["temp"].as<float>());
    bins[b].low = min(bins[b].low, item["main"]["temp"].as<float>());
    bins[b].precipitation = max(bins[b].precipitation, item["pop"] | 0.0f);
    const int distance = abs(local.tm_hour - 12);
    if (distance < bins[b].noonDistance) {
      bins[b].noonDistance = distance;
      bins[b].condition = item["weather"][0]["id"] | 800;
      bins[b].description = item["weather"][0]["description"].as<String>();
    }
  }
  int output = 0;
  for (int b = 0; b < binCount; ++b) {
    if (bins[b].key == todayKey) {
      next.current.high = bins[b].high;
      next.current.low = bins[b].low;
    } else if (output < 4) {
      ForecastDay &day = next.forecast[output++];
      day.date = bins[b].date;
      day.high = bins[b].high;
      day.low = bins[b].low;
      day.precipitation = bins[b].precipitation;
      day.conditionId = bins[b].condition;
      day.description = bins[b].description;
      day.valid = true;
    }
  }
  if (output < 4) {
    error = "Forecast has fewer than four days";
    return false;
  }
  next.updatedAt = time(nullptr);
  data = next;
  error = "";
  return true;
}

