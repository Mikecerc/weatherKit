#include "weather_calc.h"
#include <math.h>

float weather_calc_dew_point(float temp_c, float humidity)
{
    // Magnus-Tetens approximation
    const float a = 17.27f;
    const float b = 237.7f;
    
    float alpha = ((a * temp_c) / (b + temp_c)) + logf(humidity / 100.0f);
    return (b * alpha) / (a - alpha);
}

float weather_calc_heat_index(float temp_c, float humidity)
{
    // Convert to Fahrenheit for calculation
    float temp_f = weather_calc_c_to_f(temp_c);
    
    // Heat index only applies above 80°F
    if (temp_f < 80.0f) {
        return temp_c;
    }
    
    // Rothfusz regression
    float hi = -42.379f
        + 2.04901523f * temp_f
        + 10.14333127f * humidity
        - 0.22475541f * temp_f * humidity
        - 0.00683783f * temp_f * temp_f
        - 0.05481717f * humidity * humidity
        + 0.00122874f * temp_f * temp_f * humidity
        + 0.00085282f * temp_f * humidity * humidity
        - 0.00000199f * temp_f * temp_f * humidity * humidity;
    
    // Adjustments for low/high humidity
    if (humidity < 13.0f && temp_f >= 80.0f && temp_f <= 112.0f) {
        hi -= ((13.0f - humidity) / 4.0f) * sqrtf((17.0f - fabsf(temp_f - 95.0f)) / 17.0f);
    } else if (humidity > 85.0f && temp_f >= 80.0f && temp_f <= 87.0f) {
        hi += ((humidity - 85.0f) / 10.0f) * ((87.0f - temp_f) / 5.0f);
    }
    
    // Convert back to Celsius
    return weather_calc_f_to_c(hi);
}

float weather_calc_absolute_humidity(float temp_c, float humidity)
{
    // Calculate saturation vapor pressure using Magnus formula
    float es = 6.112f * expf((17.67f * temp_c) / (temp_c + 243.5f));
    
    // Calculate actual vapor pressure
    float e = (humidity / 100.0f) * es;
    
    // Calculate absolute humidity (g/m³)
    return (216.7f * e) / (temp_c + 273.15f);
}

pressure_trend_t weather_calc_pressure_trend(float change_3h)
{
    // Classify trend based on 3-hour change (thresholds in hPa)
    if (change_3h < -4.0f) return PRESSURE_TREND_FALLING_FAST;
    if (change_3h < -1.5f) return PRESSURE_TREND_FALLING;
    if (change_3h > 4.0f) return PRESSURE_TREND_RISING_FAST;
    if (change_3h > 1.5f) return PRESSURE_TREND_RISING;
    return PRESSURE_TREND_STABLE;
}

float weather_calc_c_to_f(float celsius)
{
    return (celsius * 9.0f / 5.0f) + 32.0f;
}

float weather_calc_f_to_c(float fahrenheit)
{
    return (fahrenheit - 32.0f) * 5.0f / 9.0f;
}

float weather_calc_hpa_to_inhg(float hpa)
{
    return hpa * 0.02953f;
}

float weather_calc_km_to_miles(float km)
{
    return km * 0.621371f;
}
