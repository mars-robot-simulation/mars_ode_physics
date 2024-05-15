# mars_ode_physics.
## Description

A physics library based on ODE-16, which is an industrial quality library for simulating articulated rigid body dynamics

## Dependencies
The following packages are required in order to build mars_ode_physics. 

Required:
  - [lib_manager](https://github.com/rock-simulation/lib_manager.git)
  - [mars_utils](https://github.com/rock-simulation/mars/tree/mars2/common/utils)
  - [mars_interfaces](https://github.com/mars/mars2/mars_interfaces.git)
  - [configmaps](https://github.com/rock-simulation/configmaps.git)
  - [ode-16](https://bitbucket.org/odedevs/ode.git)
  - [data_broker](https://github.com/rock-simulation/mars/tree/mars2/common/data_broker)



## Compiling
The library can be compiled on Linux with CMake or [Rock](http://rock-robotics.org/stable/).

- Run `doxygen Doxyfile` to build the API documentation.
- View the documentation by running `firefox docs/html/index.html`

## Content
This package contains the following subdirectories:

- [configuration] Contains all configuration files
- [src/joints] Contains all header and source files for joints
- [src/objects] Contains all header and source files for objects

## Contributing
Commits into the master-branch should only be done via merge requests. For project-specific development this repository should be forked into the project's gitlab-group.
