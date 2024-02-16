#!/bin/bash

install-glog.bash
install-eigen.bash
install-ceres.bash    # after eigen, glog
install-opencv.bash
install-dbow3.bash    # after opencv
install-fmt.bash
install-g2o.bash    # after eigen
install-octomap.bash
install-pangolin.bash
install-pcl.bash
install-ros.bash
install-sophus.bash    # after eigen, fmt
