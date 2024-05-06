
# Robocup Soccer Open 2024

Code for Robocup Soccer Open 2024 participating with Roborregos

## Authors

- [@JocelynVelarde](https://github.com/JocelynVelarde)
- [@Yair-Reyes](https://github.com/Yair-Reyes)

## Features

📁 File Breakdown:
- ESP32: Organizes data exchange between the camera and Raspberry Pi Pico, prioritizing speed and data integrity.
- Main Logic: Handles core decision-making processes for the robot.
- Open MV: Contains algorithms for ball, goalpost, and line detection using computer vision.

⚽ Ball Detection: Leveraging a regression model and pixel mapping, we determine real-time ball distance and angle. Advanced image processing techniques like LAB values and multi-blob tracking enhance accuracy.

🥅 Goalpost Detection: Utilizing coordinate translation and cosine calculations, we precisely locate goalposts relative to the robot's position using color.

🛤️ Line Detection: Employing grayscale filtering and pixel counting, our algorithm identifies potential obstacles (lines) and navigates around them.

🎮 Robot Control: Integration of 2 PID controllers ensures stable performance:
- Omega PID: Maintains central orientation.
- Translational PID: Guides the robot towards the ball and goalpost.
- Kinematic Equations: Enable omnidirectional movement using 4 wheels.

## Roadmap

- 


## Structure
```bash
ROBOCUP-SOCCER-OPEN-2024 
├─ Goalkeeper
│  └─ ESP32
│  └─ Pico
├─ Libs
│  └─ Bno
│  └─ Imu
│  └─ Motor
│  └─ Motors
│  └─ Photo
│  └─ PID
│  └─ Serial
│  └─ Transmission
├─ Striker
│  └─ ESP32
│  └─ Pico
├─ Test
├─ Vision
├─ .gitignore
├─ constants.h
└─ requirements.txt
```
## Tools

Make sure to install the OpenMV IDE to upload the Vision files to the camera and to add the corresponding Arduino packages/libraries in your folder environment
- [OpenMV IDE](https://openmv.io/pages/download)
- [Simple MPU6050](https://github.com/ZHomeSlice/Simple_MPU6050)
- [Wire](https://github.com/codebendercc/arduino-library-files/blob/master/libraries/Wire/Wire.h )
- [Adafruit BNO055](https://github.com/adafruit/Adafruit_BNO055)
- [Adafruit ADS1X15](https://github.com/adafruit/Adafruit_ADS1X15)

## Documentation

[Poster for competition](https://drive.google.com/file/d/1UDHHcAP6nueFe7EwNo9wCtxwj0QSR5_y/view?usp=sharing)

## Install dependencies

```Python
pip install -r requirements.txt
```

## Optimizations

optimizations


## License

[MIT](https://choosealicense.com/licenses/mit/)





