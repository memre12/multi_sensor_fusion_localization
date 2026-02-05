//
// Created by mehce on 16.06.2022.
//


#ifndef NTRIPLIB_NTRIP_UTIL_H_
#define NTRIPLIB_NTRIP_UTIL_H_

#include <string>


namespace libntrip {

int BccCheckSumCompareForGGA(char const* src);
int Base64Encode(std::string const& raw, std::string* out);
int Base64Decode(std::string const& raw, std::string* out);
int GGAFrameGenerate(double latitude, double longitude,
    double altitude, std::string* gga_out);

}  // namespace libntrip

#endif  // NTRIPLIB_NTRIP_UTIL_H_
