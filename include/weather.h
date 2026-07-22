#pragma once

#include <Arduino.h>
#include <time.h>

struct ForecastDay {
  time_t date = 0;
  float high = NAN;
  float low = NAN;
  float precipitation = 0;
  int conditionId = 800;
  String description;
  bool valid = false;
};

struct CurrentWeather {
  float temperature = NAN;
  float feelsLike = NAN;
  float high = NAN;
  float low = NAN;
  float humidity = NAN;
  float windSpeed = NAN;
  int conditionId = 800;
  String description;
  bool valid = false;
};

struct WeatherData {
  CurrentWeather current;
  ForecastDay forecast[4];
  time_t updatedAt = 0;
};

class WeatherClient {
 public:
  bool update(const String &apiKey, const String &latitude, const String &longitude, bool imperial,
              WeatherData &data, String &error);
};

