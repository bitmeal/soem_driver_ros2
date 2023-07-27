#!/usr/bin/env bash

TARGET_PKG=soem_ethercat_grant
TARGET_EXECUTABLE=$(readlink -f `ros2 pkg prefix ${TARGET_PKG}`/lib/${TARGET_PKG}/${TARGET_PKG})

TARGET_CAPS="cap_net_raw,cap_net_admin,cap_sys_nice=ep"

echo "will set ${TARGET_CAPS} on ${TARGET_EXECUTABLE}"
sudo setcap ${TARGET_CAPS} ${TARGET_EXECUTABLE}

getcap ${TARGET_EXECUTABLE}
