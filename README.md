# YOLO-Based Wildlife Detection and Autonomous Observation Robot Using A\* Path Planning

A software-based wildlife observation prototype that combines live
camera input, YOLO-based wildlife detection, JSON communication, and
MATLAB rover-navigation simulation.

## Overview

Wildlife monitoring can be difficult, time-consuming, and risky when
people need to approach animals. This project demonstrates a workflow in
which a camera feed is used to detect selected wildlife classes, and a
virtual rover plans an observation route around mapped obstacles.

The prototype integrates three main components:

1.  **Perception:** Python, OpenCV, and a trained YOLO model process
    live video and detect wildlife.
2.  **Communication:** Detection metadata is exchanged with MATLAB
    through a JSON file.
3.  **Navigation:** MATLAB estimates a virtual observation goal, plans a
    route using A\*, and simulates rover movement and obstacle checks.

> **Important:** This is a software simulation, not a physical
> robot-control system. Camera-based target range is approximate, and
> the LiDAR is simulated.

## Wildlife Classes

The detector is configured for these four classes:

-   Buffalo
-   Elephant
-   Rhino
-   Zebra

## Features

-   Captures live video from a smartphone IP Webcam stream.
-   Uses Python and OpenCV to process video frames.
-   Detects and tracks wildlife with Ultralytics YOLO.
-   Exports the target ID, class, bounding box, and frame dimensions to
    `vision_data.json`.
-   Passes detection information from Python to MATLAB.
-   Estimates a virtual observation goal from fresh detections.
-   Uses A\* path planning to navigate around predefined rock obstacles.
-   Simulates rover movement and virtual LiDAR checks.
-   Includes target-loss scanning and reacquisition logic.

## System Workflow

``` text
Smartphone IP Webcam
        |
        v
Python + OpenCV (frame capture)
        |
        v
YOLO wildlife detection and tracking
        |
        v
vision_data.json
        |
        v
MATLAB reads fresh detections
        |
        v
Estimate virtual target / goal
        |
        v
A* path planning around mapped rocks
        |
        v
Virtual rover + simulated LiDAR
        |
        v
Scan and reacquire if target is lost
```

## Technologies Used

-   **Python 3**
-   **OpenCV** --- video capture and frame processing
-   **Ultralytics YOLO** --- wildlife detection and tracking
-   **JSON** --- communication between Python and MATLAB
-   **MATLAB** --- virtual rover simulation and navigation
-   **A\* path planning** --- route planning around mapped obstacles
-   **Smartphone IP Webcam** --- live video source

## Methodology

1.  Capture live frames from the phone camera in Python.
2.  Run YOLO detection and tracking on the frames.
3.  Write the target ID, class, bounding box, and frame size to
    `vision_data.json`.
4.  Have MATLAB read fresh detection data and estimate a virtual
    observation goal.
5.  Use A\* to find a route through an occupancy grid while avoiding
    mapped rock obstacles.
6.  Move the virtual rover along the planned waypoints while simulated
    LiDAR checks nearby mapped rocks.
7.  If the target disappears, run scan/search behavior and reacquire it
    when detections return.

## Reported Results

The project presentation reports the following detector validation
metrics:

  Metric            Reported value
  --------------- ----------------
  Precision                  0.948
  Recall                     0.912
  mAP@0.50                   0.958
  mAP@0.50:0.95              0.789

These values describe **object-detection validation**, not navigation
success rates. Add actual Python and MATLAB screenshots to the
repository to document the implementation and simulation results.

## Requirements

Install or configure the following components before running the
project:

-   Python 3
-   OpenCV
-   Ultralytics
-   MATLAB
-   A trained YOLO model compatible with the wildlife classes
-   A smartphone IP Webcam stream accessible from the computer

Python packages can typically be installed with:

``` bash
pip install opencv-python ultralytics
```

The exact setup may depend on the model file, MATLAB scripts,
camera-stream URL, and local environment used by your implementation.

## Repository Structure

A suggested structure for publishing the project is:

``` text
wildlife-observation-robot/
├── README.md
├── python/
│   ├── wildlife_detection.py
│   └── vision_data.json
├── matlab/
│   └── rover_simulation.m
├── models/
│   └── best.pt
├── results/
│   ├── python_detection.png
│   └── matlab_simulation.png
└── docs/
    └── project_presentation.pptx
```

This is a **suggested layout**, not a claim about the files currently
included in the repository. Adjust filenames and folders to match your
actual code. Avoid committing large model files or generated JSON files
if they are not needed; Git LFS or release assets may be more suitable
for large models.

## Running the Prototype

The precise commands depend on the filenames and configuration in your
implementation. At a high level:

1.  Clone or download the repository.
2.  Install the Python dependencies.
3.  Configure the smartphone IP Webcam stream URL.
4.  Configure the path to the trained YOLO model.
5.  Start the Python detection script so it writes fresh detection
    metadata to `vision_data.json`.
6.  Open the MATLAB simulation and configure it to read that JSON file.
7.  Run the MATLAB simulation and observe route planning, rover
    movement, obstacle visualization, and target reacquisition.

Ensure Python and MATLAB point to the same JSON file location. If
detections are not updating, check the camera URL, model path, file
path, and whether the JSON file is being refreshed.

## Limitations

-   Target distance is approximate and is not calibrated from the
    camera.
-   The rover and LiDAR are simulated rather than physical hardware.
-   Obstacle avoidance is based on mapped rock obstacles in the
    simulation.
-   Detector metrics do not measure the success rate of the navigation
    system.
-   Performance may vary with lighting, camera quality, wildlife
    visibility, and model training data.

## Future Improvements

-   Calibrate camera-based distance estimation.
-   Add dynamic obstacle detection and avoidance.
-   Validate the system on real robot hardware in a controlled, safe
    environment.
-   Evaluate navigation success rate, route length, and
    target-reacquisition time.
-   Add reproducible setup instructions and example configuration files.
-   Include screenshots or recordings of the Python detector and MATLAB
    simulation.

## References

1.  J. Redmon et al., "You Only Look Once: Unified, Real-Time Object
    Detection," CVPR, 2016.
2.  [Ultralytics YOLO Documentation](https://docs.ultralytics.com/)
3.  P. E. Hart, N. J. Nilsson, and B. Raphael, "A Formal Basis for the
    Heuristic Determination of Minimum Cost Paths," *IEEE Transactions
    on Systems Science and Cybernetics*, 1968.
4.  M. Zhang et al., "ByteTrack: Multi-Object Tracking by Associating
    Every Detection Box," ECCV, 2022.
5.  [MathWorks MATLAB Documentation](https://www.mathworks.com/help/)

## Project Team

-   Akkala Tanvindhar Reddy
-   Kommalapati Nanda Gopal
-   Panitapu Eswarkanth

**Course:** Dynamics and Control of Robotics (22AIE441)\
**Institution:** Amrita Vishwa Vidyapeetham, Amaravati Campus\
**Guide:** Dr Ravishankar Prakash Desai

------------------------------------------------------------------------

If you use or extend this project, update this README to match the
actual source files, setup steps, and results in your repository.
