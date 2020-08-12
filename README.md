# scuba_steve
Scuba Steve ROS packages.

The following is the directory structure of the STEVE packages:

```
scuba_steve/
├── .gitignore
├── README.md
├── steve_arduino/
    ├── CMakeLists.txt
    ├── package.xml
    ├── src/
    └── test/
├── steve_auv/
    ├── actions/
    ├── CMakeLists.txt
    ├── launch/
    ├── nodes/
    ├── package.xml
    ├── setup.py
    ├── src/
        └── steve_auv/
            ├── comms/
                ├── server/
                └── utils/
            ├── gnc/
                ├── server/
                └── utils/
            ├── mission_manager/
                ├── state_machine/
                ├── states/
                └── utils/
            ├── sim/
            └── transducers/
    └── test/
        └── steve_auv/
            ├── comms/
            ├── gnc/
            └── mission_manager/
├── steve_ground_station/
    ├── CMakeLists.txt
    ├── package.xml
    ├── setup.py
    ├── src/
    └── test/
└── steve_launch_structure/
    ├── CMakeLists.txt
    ├── package.xml
    ├── setup.py
    ├── src/
    └── test/
```
