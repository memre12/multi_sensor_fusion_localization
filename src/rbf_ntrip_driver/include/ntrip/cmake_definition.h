//
// Created by mehce on 16.06.2022.
//

#ifndef RBF_GNSS_INS_DRIVER_CMAKE_DEFINITION_H
#define RBF_GNSS_INS_DRIVER_CMAKE_DEFINITION_H

namespace libntrip {

    constexpr char kCasterAgent[] = "NTRIP NTRIPCaster/@NTRIP_VERSION_MAJOR@.@NTRIP_VERSION_MINOR@.@NTRIP_VERSION_PATCH@.@GIT_HASH@";
    constexpr char kClientAgent[] = "NTRIP NTRIPClient/@NTRIP_VERSION_MAJOR@.@NTRIP_VERSION_MINOR@.@NTRIP_VERSION_PATCH@.@GIT_HASH@";
    constexpr char kServerAgent[] = "NTRIP NTRIPServer/@NTRIP_VERSION_MAJOR@.@NTRIP_VERSION_MINOR@.@NTRIP_VERSION_PATCH@.@GIT_HASH@";

}  // namespace libntrip

#endif //RBF_GNSS_INS_DRIVER_CMAKE_DEFINITION_H
