#!/bin/bash

install-glog.bash
install-fmt.bash
install-eigen.bash
install-octomap.bash
install-pcl.bash

install-ceres.bash    # CMAKE 3.16...3.27, REQUIRE eigen, glog
install-sophus.bash    # REQUIRE eigen, fmt
install-g2o.bash    # REQUIRE eigen
install-opencv.bash   # CMAKE 3.5.1
