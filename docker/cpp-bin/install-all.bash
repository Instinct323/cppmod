#!/bin/bash

install-glog.bash
install-eigen.bash
install-fmt.bash
install-octomap.bash
install-pcl.bash

install-ceres.bash    # after eigen, glog
install-opencv.bash
install-dbow3.bash    # after opencv
install-g2o.bash    # after eigen
install-pangolin.bash
install-ros.bash
install-sophus.bash    # after eigen, fmt
