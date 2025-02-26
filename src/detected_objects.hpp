#ifndef DETECTED_OBJECTS_HPP
#define DETECTED_OBJECTS_HPP

#include <vector>
#include <Arduino.h>

struct DetectedObject {
    String label;
    int x, y, width, height;
    float conf;
};

extern std::vector<DetectedObject> detectedObjects;

#endif // DETECTED_OBJECTS_HPP
