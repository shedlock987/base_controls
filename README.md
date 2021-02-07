# base_controls
Simple PID Controller with full test suite. Controller Incorporates a D-Term LPF and Anit-Windup Protection.

# Getting Started
These instructions will help you generate the necessary documentation for using this package, and list the required dependencies.

# Documentation
The documentation for this project is Doxygen based. To generate, execute the following commands:

cd <path>/base_controls

doxygen Doxyfile
  
# Dependencies
The follwing dependencies are required, and can be installed accordingly.

sudo apt install doxygen

sudo apt install libgtest-dev

sudo apt install build-essential

sudo apt install python-catkin-tools

sudo apt install ros-noetic-desktop-full (Includes required Eigen3 library)

# Running the tests
To compile unit and pipeline tests, use the following command:

catkin build pid_controller --no-deps --catkin-make-args run_tests

Break down into end to end tests

The PID Controller test verifies basic functionality 

pid_controller_test.cpp 

## Built With

* [catkin_tools](https://catkin-tools.readthedocs.io/en/latest/index.html) - Build tool used for compiling this project
* [Google Test](https://github.com/google/googletest) - Unit testing framework
* [ros_noetic](http://wiki.ros.org/noetic) - Open source meta-operating system


## Authors

* **Ryan Shedlock**

## License

This project is licensed under the MIT License - see the LICENSE file for details


