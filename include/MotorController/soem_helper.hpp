// Copyright (c) 2025 Raion Robotics Inc.
//
// Any unauthorized copying, alteration, distribution, transmission,
// performance, display or use of this material is prohibited.
//
// All rights reserved.

#include <iostream>
#include <string>
#include "soem/soem.h"

// Helper function: Get a vector of unique network interface names
static inline std::vector<std::string> GetNetworkInterfaces() {
    std::vector<std::string> interfaceNames;
    struct ifaddrs* ifaddr = nullptr;
    
    if (getifaddrs(&ifaddr) == -1) {
        perror("getifaddrs");
        return interfaceNames;
    }
    
    // Use a set to avoid duplicate names (an interface may have multiple addresses)
    std::set<std::string> uniqueNames;
    for (struct ifaddrs* ifa = ifaddr; ifa != nullptr; ifa = ifa->ifa_next) {
        if (ifa->ifa_addr == nullptr)
            continue;
        uniqueNames.insert(ifa->ifa_name);
    }
    freeifaddrs(ifaddr);
    
    for (const auto& name : uniqueNames) {
        interfaceNames.push_back(name);
    }
    return interfaceNames;
}

std::string getEcErrorMessage(uint32_t ec_err) {
    std::string errorMsg;

    if (ec_err & EC_ERR_TYPE_SDO_ERROR)
        errorMsg += "SDO Error | ";
    if (ec_err & EC_ERR_TYPE_EMERGENCY)
        errorMsg += "Emergency Error | ";
    if (ec_err & EC_ERR_TYPE_PACKET_ERROR)
        errorMsg += "Packet Error | ";
    if (ec_err & EC_ERR_TYPE_SDOINFO_ERROR)
        errorMsg += "SDO Info Error | ";
    if (ec_err & EC_ERR_TYPE_FOE_ERROR)
        errorMsg += "FoE Error | ";
    if (ec_err & EC_ERR_TYPE_FOE_BUF2SMALL)
        errorMsg += "FoE Buffer Too Small | ";
    if (ec_err & EC_ERR_TYPE_FOE_PACKETNUMBER)
        errorMsg += "FoE Packet Number Error | ";
    if (ec_err & EC_ERR_TYPE_SOE_ERROR)
        errorMsg += "SoE Error | ";
    if (ec_err & EC_ERR_TYPE_MBX_ERROR)
        errorMsg += "Mailbox Error | ";
    if (ec_err & EC_ERR_TYPE_FOE_FILE_NOTFOUND)
        errorMsg += "FoE File Not Found | ";
    if (ec_err & EC_ERR_TYPE_EOE_INVALID_RX_DATA)
        errorMsg += "EOE Invalid RX Data | ";

    if (errorMsg.empty()) {
        errorMsg = "No error detected";
    } else {
        // Remove the last " | " separator
        errorMsg = errorMsg.substr(0, errorMsg.size() - 3);
    }

    return errorMsg;
}