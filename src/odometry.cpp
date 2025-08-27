#include "odometry.h"
#include <cmath>

Odometry::Odometry(double wheel_radius, double rpm)
    : radius(wheel_radius), rpm(rpm) {
  double rps = rpm / 60.0;
  linear_vel = 2 * M_PI * radius * rps;
}

double Odometry::distance(int x1, int y1, int x2, int y2) {
  return hypot(x2 - x1, y2 - y1);
}

double Odometry::angle(int x1, int y1, int x2, int y2) {
  return atan2(y2 - y1, x2 - x1) * 180.0 / M_PI;
}

MotionCommand Odometry::computeCommands(vector<pair<int, int>> &path) {
  MotionCommand cmd{0.0, 0.0};
  if (path.size() < 2)
    return cmd;

  double total_angle = 0.0;
  double total_distance = 0.0;

  double prev_angle =
      angle(path[0].first, path[0].second, path[1].first, path[1].second);

  for (size_t i = 1; i < path.size(); i++) {
    auto p1 = path[i - 1];
    auto p2 = path[i];

    total_distance += distance(p1.first, p1.second, p2.first, p2.second);

    if (i + 1 < path.size()) {
      double next_angle =
          angle(p1.first, p1.second, path[i + 1].first, path[i + 1].second);

      double dtheta = next_angle - prev_angle;
      while (dtheta > 180)
        dtheta -= 360;
      while (dtheta < -180)
        dtheta += 360;

      total_angle += fabs(dtheta);
      prev_angle = next_angle;
    }
  }

  cmd.time_sec = (linear_vel > 1e-6) ? (total_distance / linear_vel) : 0.0;
  cmd.angle_deg = total_angle;

  return cmd;
}
