#include <iostream>
#include <GeographicLib/Geodesic.hpp>
#include <iomanip>

int main() {

    GeographicLib::Geodesic geod = GeographicLib::Geodesic::WGS84();

    // 定义起始点A的经纬度
    double latA = /* A点的纬度 */31.176355711282966;
    double lonA = /* A点的经度 */121.59351153481504;

    double latB, lonB, temp;
    // 向东的距离和向北的距离
    double distanceEast = 9.10;
    double distanceNorth = 3.207932;

    // 计算目标点B的经纬度
    geod.Direct(latA, lonA, 90, distanceEast, temp, lonB);
    geod.Direct(latA, lonA, 0, distanceNorth, latB, temp);

    std::cout << std::fixed << std::setprecision(16) << "lonB :" << lonB << std::endl;
    std::cout << std::fixed << std::setprecision(16) << "latB :" << latB << std::endl;

    return 0;
}
