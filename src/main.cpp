#include "ublox_reader.h"
#include "gridmap.h"
#include "planning.h"
#include "odometry.h"
#include <iostream>

using namespace std;

int main() {
    // 1. Read UBX file (two hex lines: start and goal)
    string filename = "gps_data.txt"; // replace with your file
    auto [startGPS, goalGPS] = readUbloxFile(filename);

    cout << "Start GPS: " << startGPS.lat << ", " << startGPS.lon << endl;
    cout << "Goal GPS: " << goalGPS.lat << ", " << goalGPS.lon << endl;

    // 2. Create grid map
    double cellsize = 1.0;  // 1 meter per grid cell
    int rows = 15;
    int cols = 15;
    Gridmapper gridMap(startGPS, cellsize, rows, cols);
    auto grid = gridMap.getGrid();

    // 3. Convert GPS to grid coordinates
    auto startGrid = gridMap.gpstogrid(startGPS);
    auto goalGrid = gridMap.gpstogrid(goalGPS);

    cout << "Start grid: (" << startGrid.first << ", " << startGrid.second << ")\n";
    cout << "Goal grid: (" << goalGrid.first << ", " << goalGrid.second << ")\n";

    // 4. Path planning
    Planner planner(grid);
    auto path = planner.pathplanning(startGrid, goalGrid);

    if (path.empty()) {
        cout << "No path found!" << endl;
        return 1;
    }

    cout << "Planned path:" << endl;
    for (auto &p : path) {
        cout << "(" << p.first << ", " << p.second << ") ";
    }
    cout << endl;

    // 5. Compute odometry commands
    double wheel_radius = 0.05; // meters
    double rpm = 60.0;
    Odometry odom(wheel_radius, rpm);
    MotionCommand cmd = odom.computeCommands(path);

    cout << "Total distance traversal time: " << cmd.time_sec << " sec\n";
    cout << "Total angle rotated: " << cmd.angle_deg << " degrees\n";

    return 0;
}
