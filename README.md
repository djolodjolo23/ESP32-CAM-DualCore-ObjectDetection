# ESP32-CAM Object Detection & Video Streaming

This is an ESP32-CAM project that lets you run an Edge Impulse object detection model alongside an MJPEG video streaming server.

## How It Works

The project runs two main tasks on separate cores:
- **Core 1:** Handles the MJPEG server for serving clients and streaming video.
- **Core 2:** Runs the object detection inference.

Inference is **not** performed on every single frame since the processing time depends on the model size. Instead, the video stream runs continuously, and a tiny JavaScript script in `index.html` dynamically updates centroids. The actual drawing happens in the browser while streaming and live inference run on the ESP32.

Currently, this setup works with **FOMO** (Fast Object Detection for Mobile Devices) as it's much faster compared to other models. The JavaScript script calculates centroids by default. If you want to modify the code to use bounding boxes instead, you’ll need to update the JavaScript file accordingly.

## How to Run

1. Train your **object detection model** and export the generated files into the `src` folder.
2. Open `inference.hpp` and include your model's header file:

   ```cpp
   #include "your-own-model_inferencing.h"
   ```
3. Create a `secrets.h` file inside the `src` folder and add your Wi-Fi credentials:

   ```cpp
   #define SSID "your_wifi_ssid"
   #define PASSWORD "your_wifi_password"
   ```
4. Compile and upload the firmware to your ESP32-CAM.
5. Connect to the ESP32-CAM’s IP address in a browser to see the video stream with live inference.