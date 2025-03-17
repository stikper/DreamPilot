//
// Created by stikper on 07.03.25.
//

#include "NMEAParser.h"

#include <esp_log.h>
#include <esp_timer.h>

std::vector<std::string> NMEAParser::split(const std::string& s, const std::string& delimiter)
{
    std::vector<std::string> result;

    if (delimiter.empty())
    {
        if (!s.empty())
        {
            result.push_back(s);
        }
        return result;
    }

    size_t start = 0;
    size_t end = s.find(delimiter);

    while (end != std::string::npos)
    {
        result.push_back(s.substr(start, end - start));
        start = end + delimiter.length();
        end = s.find(delimiter, start);
    }

    if (start <= s.length())
    {
        result.push_back(s.substr(start));
    }

    return result;
}

std::string NMEAParser::replace(const std::string& str, const std::string& old_str, const std::string& new_str)
{
    if (str.empty() || old_str.empty())
    {
        return str;
    }

    std::string result = str;
    size_t pos = 0;

    while ((pos = result.find(old_str, pos)) != std::string::npos)
    {
        result.replace(pos, old_str.length(), new_str);
        pos += new_str.length();
    }

    return result;
}

std::string NMEAParser::trim(const std::string& str, const std::string& chars = " \t\n\r\f\v") {
    if (str.empty()) {
        return str;
    }

    size_t start = str.find_first_not_of(chars);
    if (start == std::string::npos) {
        return "";
    }

    size_t end = str.find_last_not_of(chars);

    return str.substr(start, end - start + 1);
}

bool NMEAParser::checkIntegrity(const std::string& nmea)
{
    size_t checksumPos = nmea.find('*');
    if (checksumPos == std::string::npos || checksumPos + 1 >= nmea.length()) {
        return false; // '*' не найден или нет символов после него
    }

    std::string checksumStr = nmea.substr(checksumPos + 1);

    if (checksumStr.empty() || checksumStr.length() > 2) {
        return false;
    }

    for (char c : checksumStr) {
        if (!isxdigit(c)) {
            return false;
        }
    }

    unsigned long receivedChecksum;
    try {
        receivedChecksum = std::stoul(checksumStr, nullptr, 16);
    } catch (const std::exception&) {
        return false;
    }

    unsigned int calculatedChecksum = 0;
    for (size_t i = 1; i < checksumPos; ++i) {
        calculatedChecksum ^= nmea[i];
    }

    return calculatedChecksum == static_cast<unsigned char>(receivedChecksum);
}

NMEAParser::GPSData NMEAParser::parse(std::string nmea)
{
    NMEAParser::GPSData result = {};
    result.timestamp = esp_timer_get_time();

    nmea = trim(nmea);

    // Checking integrity
    if (!checkIntegrity(nmea))
    {
        result.checksum = false;
        return result;
    }
    result.checksum = true;

    nmea = nmea.substr(1, nmea.length() - 4);

    // Get split data
    const auto tokens = split(nmea, ",");

    if (tokens[0].length() != 5)
    {
        result.parse_error = true;
        return result;
    }

    result.source = tokens[0].substr(0, 2);
    result.type = tokens[0].substr(2, 3);

    if (result.type == "GGA")
    {
        parseGGA(tokens, &result);
    } else if (result.type == "RMC")
    {
        parseRMC(tokens, &result);
    } else if (result.type == "TXT")
    {
        parseTXT(tokens, &result);
    } else
    {
        result.ignore = true;
    }

    return result;
}

void NMEAParser::parseGGA(const std::vector<std::string>& tokens, GPSData* result)
{
    if (tokens.size() == 15)
    {
        if (tokens[6] == "1") result->valid = true;
        else return;
        result->time = std::stof(tokens[1]);
        result->lat = parseLatitude(tokens[2], tokens[3][0]);
        result->lon = parseLongitude(tokens[4], tokens[5][0]);
        result->satellites = std::stoi(tokens[7]);
        result->hdop = std::stof(tokens[8]);
        //TODO!
        if (tokens[10] == "M") result->alt = std::stof(tokens[9]);
        else result->valid = false;
    } else {
        result->parse_error = true;
    }
}

void NMEAParser::parseRMC(const std::vector<std::string>& tokens, GPSData* result)
{
    if (tokens.size() == 13)
    {
        if (tokens[2] == "A") result->valid = true;
        else return;
        result->time = std::stof(tokens[1]);
        result->lat = parseLatitude(tokens[3], tokens[4][0]);
        result->lon = parseLongitude(tokens[5], tokens[6][0]);
        result->spd = std::stof(tokens[7]) / 1.944f;
        if (!tokens[8].empty())
            result->hdg = std::stof(tokens[8]);
        else
            result->hdg = 0;
        result->date = std::stof(tokens[9]);
    } else {
        result->parse_error = true;
    }
}



void NMEAParser::parseTXT(const std::vector<std::string>& tokens, GPSData* result)
{
    if (tokens.size() == 5)
    {
        result->text = tokens[4];
    } else {
        result->parse_error = true;
    }
}

double NMEAParser::parseLatitude(const std::string& lat, const char& direction) {
    // Проверяем длину строки
    if (lat.length() < 9) return 0.0;

    // Извлекаем градусы (первые два символа)
    double degrees = std::stod(lat.substr(0, 2));

    // Извлекаем минуты (остальная часть)
    double minutes = std::stod(lat.substr(2));

    // Преобразуем в десятичные градусы
    double decimal_degrees = degrees + (minutes / 60.0);

    // Если направление южное, меняем знак
    if (direction == 'S') {
        decimal_degrees = -decimal_degrees;
    }

    return decimal_degrees;
}

double NMEAParser::parseLongitude(const std::string& lon, const char& direction) {
    // Проверяем длину строки
    if (lon.length() < 10) return 0.0;

    // Извлекаем градусы (первые три символа)
    double degrees = std::stod(lon.substr(0, 3));

    // Извлекаем минуты (остальная часть)
    double minutes = std::stod(lon.substr(3));

    // Преобразуем в десятичные градусы
    double decimal_degrees = degrees + (minutes / 60.0);

    // Если направление западное, меняем знак
    if (direction == 'W') {
        decimal_degrees = -decimal_degrees;
    }

    return decimal_degrees;
}

