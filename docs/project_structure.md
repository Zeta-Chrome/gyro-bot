.
├── app
│   ├── bin
│   │   └── gyrocontroller-1.0-arm64-v8a-debug.apk
│   ├── buildozer.spec
│   ├── images
│   │   ├── app_icon.png
│   │   └── slider_knob.png
│   ├── main.py
│   ├── network
│   │   ├── __init__.py
│   │   └── server.py
│   ├── pyrightconfig.json
│   ├── README.md
│   └── widgets
│       ├── dropdown.py
│       ├── __init__.py
│       ├── joystick.py
│       ├── servo_control.py
│       ├── settings.py
│       └── triangle_button.py
├── docs
│   ├── project_structure
│   └── project_structure.md
├── firmware
│   ├── common
│   │   └── common.hpp
│   ├── components
│   │   └── networking
│   │       ├── CMakeLists.txt
│   │       ├── include
│   │       │   ├── tcp_client.hpp
│   │       │   ├── udp_client.hpp
│   │       │   └── wifi_station.hpp
│   │       └── src
│   │           ├── tcp_client.cpp
│   │           ├── udp_client.cpp
│   │           └── wifi_station.cpp
│   ├── control
│   │   ├── CMakeLists.txt
│   │   ├── components
│   │   │   ├── imu
│   │   │   │   ├── CMakeLists.txt
│   │   │   │   ├── include
│   │   │   │   │   ├── config.hpp
│   │   │   │   │   └── imu.hpp
│   │   │   │   └── src
│   │   │   │       └── imu.cpp
│   │   │   ├── motor_driver
│   │   │   │   ├── CMakeLists.txt
│   │   │   │   ├── include
│   │   │   │   │   └── motor_driver.hpp
│   │   │   │   └── src
│   │   │   │       └── motor_driver.cpp
│   │   │   ├── servo
│   │   │   │   ├── CMakeLists.txt
│   │   │   │   ├── include
│   │   │   │   │   └── servo.hpp
│   │   │   │   └── src
│   │   │   │       └── servo.cpp
│   │   │   └── ultrasound_sensor
│   │   │       ├── CMakeLists.txt
│   │   │       ├── include
│   │   │       │   └── ultrasound_sensor.hpp
│   │   │       └── src
│   │   │           └── ultrasound_sensor.cpp
│   │   ├── dependencies.lock
│   │   ├── main
│   │   │   ├── CMakeLists.txt
│   │   │   ├── include
│   │   │   │   └── gyro_context.hpp
│   │   │   └── src
│   │   │       ├── gyro_context.cpp
│   │   │       └── main.cpp
│   │   ├── sdkconfig
│   │   └── sdkconfig.old
│   ├── dependencies.lock
│   ├── env.sh
│   ├── README.md
│   └── vision
│       ├── CMakeLists.txt
│       ├── components
│       │   └── camera
│       │       ├── CMakeLists.txt
│       │       ├── include
│       │       │   └── camera.hpp
│       │       └── src
│       │           └── camera.cpp
│       ├── dependencies.lock
│       ├── main
│       │   ├── CMakeLists.txt
│       │   ├── idf_component.yml
│       │   ├── include
│       │   │   └── vision_context.hpp
│       │   └── src
│       │       ├── main.cpp
│       │       └── vision_context.cpp
│       ├── sdkconfig
│       └── sdkconfig.old
├── fpga
│   └── README.md
├── hardware
│   ├── model_design
│   │   ├── GyroBot.20251129-222356.FCBak
│   │   ├── GyroBot-Base.stl
│   │   ├── GyroBot.FCStd
│   │   ├── GyroBot-MotorMount.stl
│   │   └── GyroBot-Parts.stl
│   └── README.md
├── README.md
└── server
    ├── config.py
    ├── display.py
    ├── main.py
    ├── object_detector.py
    ├── path_planner.py
    ├── README.md
    ├── sensor_fusion.py
    ├── sensor_receiver.py
    ├── test
    │   ├── packets.py
    │   └── tcp_image_receiver.py
    └── yolov8n.pt

43 directories, 80 files
