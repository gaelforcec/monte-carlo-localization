#pragma once

#include "api.h"
#include <vector>
#include <random>
#include <memory>
#include <cmath>
#include "kalmanFilter.hpp"
class MCLLocalizer {
public:
    struct Particle {
        double x;
        double y;
        double theta;
        double weight;
    };

    struct FieldWall {
        double x1, y1;  // Start point
        double x2, y2;  // End point
    };

    MCLLocalizer(
        const std::shared_ptr<pros::Imu>& imu1,
        const std::shared_ptr<pros::Imu>& imu2,
        const std::shared_ptr<pros::Rotation>& forwardTracker,
        const std::shared_ptr<pros::Rotation>& horizontalTracker,
        const std::shared_ptr<pros::Distance>& distanceSensor,
        int numParticles = 1000
    );

    void update();
    void resample();
    void getPose(double& x, double& y, double& theta);
    void initializeParticles(double x, double y, double theta, double spread = 1.0);

private:
    // Sensors
    std::shared_ptr<pros::Imu> imu1, imu2;
    std::shared_ptr<pros::Rotation> forwardTracker, horizontalTracker;
    std::shared_ptr<pros::Distance> distanceSensor;
    std::vector<Particle> particles;
    std::vector<FieldWall> fieldWalls;
    // Initialize Kalman Filter
    std::shared_ptr<KalmanFilter> kf_;
    // Previous sensor readings
    double prevForwardPos = 0;
    double prevHorizontalPos = 0;
    double prevImu1Heading = 0;
    double prevImu2Heading = 0;

    // Random number generation
    std::random_device rd;
    std::mt19937 gen;
    std::normal_distribution<> noise_pos;
    std::normal_distribution<> noise_heading;

    // Constants
    const double FIELD_SIZE_X = 144.0;  // Field size in inches
    const double FIELD_SIZE_Y = 144.0;
    const double DISTANCE_NOISE_SD = 1.0;
    const double MAX_VALID_DISTANCE = 7.87402;

    void motionUpdate(double deltaForward, double deltaHorizontal, double deltaHeading);
    double calculateWeight(const Particle& p);
    double normalizeAngle(double angle);
    double predictDistanceReading(const Particle& p);
    double raycast(double x, double y, double angle);
    bool lineIntersection(double x1, double y1, double x2, double y2,
                         double x3, double y3, double x4, double y4,
                         double& intersectX, double& intersectY);
    void setupFieldWalls();
};