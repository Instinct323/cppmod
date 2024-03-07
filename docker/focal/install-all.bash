#!/bin/bash

install-eigen.bash
install-pcl.bash

install-glog.bash   # cmake 3.22
install-fmt.bash  # cmake 3.8...3.26

install-sophus.bash    # after eigen, fmt
install-ceres.bash    # cmake 3.16...3.27, after eigen, glog
install-opencv.bash   # cmake 3.5.1
install-g2o.bash    # after eigen
install-pangolin.bash   # cmake 3.10
