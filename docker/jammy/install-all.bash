#!/bin/bash

install-glog.bash   # CMAKE 3.22
install-fmt.bash  # CMAKE 3.8...3.26

install-eigen.bash
install-octomap.bash
install-pcl.bash

install-sophus.bash    # REQUIRE eigen, fmt
install-ceres.bash    # CMAKE 3.16...3.27, REQUIRE eigen, glog
install-opencv.bash   # CMAKE 3.5.1
install-g2o.bash    # REQUIRE eigen
