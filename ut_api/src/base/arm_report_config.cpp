
#include "base/arm_report_config.h"
#include "common/hex_data.h"
#include "common/print.h"

ArmReportConfig::ArmReportConfig(void) {}
ArmReportConfig::ArmReportConfig(Socket* socket_fp) { arminit(socket_fp); }
ArmReportConfig::~ArmReportConfig(void) {
  is_error_ = true;
  delete socket_fp_;
}
void ArmReportConfig::arminit(Socket* socket_fp) {
  frame_len = 80;
  socket_fp_ = socket_fp;
  recv_task_ = new RtPeriodicMemberFun<ArmReportConfig>(0.0005, "recv_task", 1024 * 512, 30, &ArmReportConfig::recv_proc, this);
  recv_task_->start();
}

void ArmReportConfig::close(void) { is_error_ = true; }

bool ArmReportConfig::is_error(void) { return is_error_; }

bool ArmReportConfig::is_update(void) {
  int temp = is_update_;
  if (is_update_) is_update_ = false;
  return temp;
}

void ArmReportConfig::recv_proc(void) {
  int ret = socket_fp_->read_frame(&rxdata_, 0);
  if (ret == 0) flush_data(rxdata_.data, rxdata_.len);
}

void ArmReportConfig::flush_data(uint8_t* rx_data, int len) {
  if (len % frame_len != 0) {
    printf("[ArmReSta] Error: flush_data len = %d\n", len);
    is_error_ = true;
    return;
  }

  int k = (int)(len / frame_len) - 1;
  k *= frame_len;

  if (++report_flag_ >= 3) report_flag_ = 0;
  memcpy(&report_config_[report_flag_], &rx_data[k * frame_len], sizeof(arm_report_config_t));
  rxcnt_++;
  is_update_ = true;
}

int ArmReportConfig::get_data(arm_report_config_t* rx_data) {
  // if (is_error_) return -1;
  int flag = report_flag_;
  if (--flag < 0) flag = 2;
  memcpy(rx_data, &report_config_[flag], sizeof(arm_report_config_t));
  return 0;
}

void ArmReportConfig::print_data(arm_report_config_t* rx_data) {
  printf("rxcnt      = %d\n", rxcnt_);
  printf("trs_maxacc = %f\n", rx_data->trs_maxacc);
  printf("trs_jerk   = %f\n", rx_data->trs_jerk);
  printf("rot_maxacc = %f\n", rx_data->rot_maxacc);
  printf("rot_jerk   = %f\n", rx_data->rot_jerk);
  printf("p2p_maxacc = %f\n", rx_data->p2p_maxacc);
  printf("p2p_jerk   = %f\n", rx_data->p2p_jerk);
  printf("colli_sens = %d\n", rx_data->colli_sens);
  printf("teach_sens = %d\n", rx_data->teach_sens);
  Print::nvect("tcp_offset  = ", rx_data->tcp_offset, 6);
  Print::nvect("tcp_load    = ", rx_data->tcp_load, 4);
  Print::nvect("gravity_dir = ", rx_data->gravity_dir, 3);
  printf("\n");
}

void ArmReportConfig::print_data(void) { print_data(&report_config_[report_flag_]); }