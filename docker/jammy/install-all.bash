#!/bin/bash

install-glog.bash   # cmake 3.22
install-fmt.bash  # cmake 3.8...3.26

install-eigen.bash
install-octomap.bash
install-pcl.bash

install-sophus.bash    # after eigen, fmt
install-ceres.bash    # cmake 3.16...3.27, after eigen, glog
install-opencv.bash   # cmake 3.5.1
install-g2o.bash    # after eigen
install-pangolin.bash   # cmake 3.10
