rosrun rqt_joint_trajectory_controller rqt_joint_trajectory_controller  --force-discover
utra_api
1.
// #ifdef linux
#include <sys/prctl.h>
#include <sys/timerfd.h>
// #endif

2.
    int nanoseconds = (int)(1e9 * fmod(period_s_, 1.f));

3. src/common
#include "common/linuxcvl.h"

4. src/base/arm_report_status.cpp src/base/arm_report_config.cpp
int ArmReportStatus::get_data(arm_report_status_t* rx_data) {
  // if (is_error_) return -1;