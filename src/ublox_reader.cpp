#include "ublox_reader.h"
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <vector>

using namespace std;

int decodeUBX(uint8_t *buffer, classId *gps) {
  if (!buffer || !gps)
    return 1;

  gps->iTOW = *((uint32_t *)(buffer));
  gps->lon = *((int32_t *)(buffer + 4));
  gps->lat = *((int32_t *)(buffer + 8));
  gps->height = *((int32_t *)(buffer + 12));
  gps->hMSL = *((int32_t *)(buffer + 16));
  gps->hAcc = *((uint32_t *)(buffer + 20));
  gps->vAcc = *((uint32_t *)(buffer + 24));

  return 0;
}

GPS gpsFromData(const classId &gps) {
  GPS out;
  out.lat = gps.lat * 1e-7;
  out.lon = gps.lon * 1e-7;
  out.height = gps.height * 1e-3;
  return out;
}

static vector<uint8_t> hexStringToBytes(const string &hex) {
  vector<uint8_t> bytes;
  for (size_t i = 0; i < hex.size(); i += 2) {
    string byteString = hex.substr(i, 2);
    uint8_t byte = (uint8_t)strtol(byteString.c_str(), nullptr, 16);
    bytes.push_back(byte);
  }
  return bytes;
}

pair<GPS, GPS> readUbloxFile(const string &filename) {
  ifstream file(filename);
  if (!file.is_open()) {
    cerr << "Error opening UBX file: " << filename << endl;
    return {};
  }

  string line1, line2;
  getline(file, line1);
  getline(file, line2);

  auto bytes1 = hexStringToBytes(line1);
  auto bytes2 = hexStringToBytes(line2);

  classId data1{}, data2{};
  decodeUBX(bytes1.data(), &data1);
  decodeUBX(bytes2.data(), &data2);

  return {gpsFromData(data1), gpsFromData(data2)};
}
