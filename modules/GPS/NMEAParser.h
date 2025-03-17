//
// Created by stikper on 07.03.25.
//

#ifndef NMEAPARSER_H
#define NMEAPARSER_H

#include <string>
#include <vector>

#include "IGPSModule.h"


class NMEAParser {
    using GPSData = IGPSModule::GPSData;

    // String functions
    static std::vector<std::string> split(const std::string& s, const std::string& delimiter);
    static std::string replace(const std::string& str, const std::string& old_str, const std::string& new_str);
    static std::string trim(const std::string& str, const std::string& chars);

    static bool checkIntegrity(const std::string& nmea);

    // TODO: NOT COMPLETED
    static void parseGGA(const std::vector<std::string>& tokens, GPSData* result);
    static void parseRMC(const std::vector<std::string>& tokens, GPSData* result);
    static void parseTXT(const std::vector<std::string>& tokens, GPSData* result);

    static double parseLongitude(const std::string& lon, const char& direction);
    static double parseLatitude(const std::string& lat, const char& direction);
public:
    static GPSData parse(std::string nmea);
};



#endif //NMEAPARSER_H
