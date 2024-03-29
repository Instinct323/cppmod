#!/bin/bash

install-eigen.bash
install-pcl.bash

install-glog.bash
install-fmt.bash  # CMAKE 3.8...3.26

install-sophus.bash    # REQUIRE eigen, fmt
install-ceres.bash    # CMAKE 3.16...3.27, REQUIRE eigen, glog
install-opencv.bash   # CMAKE 3.5.1
install-g2o.bash    # REQUIRE eigen
