# AutoModality Supervisor ROS Node

Watches the health of all nodes to determine if the flight shall continue..

[![Release](https://github.com/AutoModality/am_super/workflows/Release/badge.svg)](https://github.com/AutoModality/am_super/workflows/Release/badge.svg)
[![Latest Version @ Cloudsmith](https://api-prd.cloudsmith.io/badges/version/automodality/release/deb/ros-melodic-am-super/latest/d=ubuntu%252Fbionic;t=1/?render=true&badge_token=gAAAAABetY4e0kXP_ZlIdblJEZG8GiEIYJRkjvt9-nVmp3U4QiqyH-2mOfwi_B7meqOAh3rgt-lbVvFTiAmsysp4iMNx79oZfuVCEac-Lqz-dXxW4W7AbYU%3D)](https://cloudsmith.io/~automodality/repos/release/packages/detail/deb/ros-melodic-am-super/latest/d=ubuntu%252Fbionic;t=1/) 


# ROS Node

[AMSuper](include/am_super/am_super.h) is the ROS Node watching all other nodes for system health and reliability.

# Library

Other nodes must communicate with the Supervisor and should do so using the [library interface](include/super_lib/).

# Run Tests

See [test](test) for more.