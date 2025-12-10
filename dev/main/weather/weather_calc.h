#ifndef WEATHER_CALC_H
#define WEATHER_CALC_H

/**
 * @brief Calculate dew point from temperature and humidity
 * @param temp_c Temperature in Celsius
 * @param humidity Relative humidity in %
 * @return Dew point in Celsius
 */
float weather_calc_dew_point(float temp_c, float humidity);

/**
 * @brief Calculate heat index from temperature and humidity
 * @param temp_c Temperature in Celsius
 * @param humidity Relative humidity in %
 * @return Heat index in Celsius
 */
float weather_calc_heat_index(float temp_c, float humidity);

/**
 * @brief Calculate absolute humidity
 * @param temp_c Temperature in Celsius
 * @param humidity Relative humidity in %
 * @return Absolute humidity in g/m³
 */
float weather_calc_absolute_humidity(float temp_c, float humidity);

/**
 * @brief Pressure trend enumeration
 */
typedef enum {
    PRESSURE_TREND_FALLING_FAST = -2,
    PRESSURE_TREND_FALLING = -1,
    PRESSURE_TREND_STABLE = 0,
    PRESSURE_TREND_RISING = 1,
    PRESSURE_TREND_RISING_FAST = 2
} pressure_trend_t;

/**
 * @brief Calculate pressure trend from 3-hour change
 * @param change_3h Pressure change over 3 hours in hPa
 * @return Pressure trend
 */
pressure_trend_t weather_calc_pressure_trend(float change_3h);

/**
 * @brief Convert Celsius to Fahrenheit
 */
float weather_calc_c_to_f(float celsius);

/**
 * @brief Convert Fahrenheit to Celsius
 */
float weather_calc_f_to_c(float fahrenheit);

/**
 * @brief Convert hPa to inHg
 */
float weather_calc_hpa_to_inhg(float hpa);

/**
 * @brief Convert km to miles
 */
float weather_calc_km_to_miles(float km);

#endif // WEATHER_CALC_H
