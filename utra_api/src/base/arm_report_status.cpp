
#include "base/arm_report_status.h"
#include "common/hex_data.h"
#include "common/print.h"

ArmReportStatus::ArmReportStatus(void) {}
ArmReportStatus::ArmReportStatus(Socket* socket_fp, int axis) { arminit(socket_fp, axis); }
ArmReportStatus::~ArmReportStatus(void) {
  is_error_ = true;
  delete socket_fp_;
}
void ArmReportStatus::arminit(Socket* socket_fp, int axis) {
  axis_ = axis;
  frame_len = 17 + axis_ * 4 + 6 * 4 + axis_ * 4;
  socket_fp_ = socket_fp;
  recv_task_ = new RtPeriodicMemberFun<ArmReportStatus>(0.0005, "recv_task", 1024 * 512, 30, &ArmReportStatus::recv_proc, this);
  recv_task_->start();
}

void ArmReportStatus::close(void) { is_error_ = true; }

bool ArmReportStatus::is_error(void) { return is_error_; }

bool ArmReportStatus::is_update(void) {
  int temp = is_update_;
  if (is_update_) is_update_ = false;
  return temp;
}

void ArmReportStatus::recv_proc(void) {
  int ret = socket_fp_->read_frame(&rxdata_, 0);
  if (ret == 0) flush_data(rxdata_.data, rxdata_.len);
}

void ArmReportStatus::flush_data(uint8_t* rx_data, int len) {
  if (len % frame_len != 0) {
    printf("[ArmReSta] Error: flush_data len = %d\n", len);
    is_error_ = true;
    return;
  }

  int k = (int)(len / frame_len) - 1;
  k *= frame_len;

  if (++report_flag_ >= 3) report_flag_ = 0;
  memcpy(&report_status_[report_flag_], &rx_data[k * frame_len], 17);
  int axis = report_status_[report_flag_].axis;
  HexData::hex_to_fp32(&rx_data[k * frame_len + 17], report_status_[report_flag_].joint, axis);
  HexData::hex_to_fp32(&rx_data[k * frame_len + 17 + axis * 4], report_status_[report_flag_].pose, 6);
  HexData::hex_to_fp32(&rx_data[k * frame_len + 17 + axis * 4 + 6 * 4], report_status_[report_flag_].tau, axis);
  rxcnt_++;
  is_update_ = true;
}

int ArmReportStatus::get_data(arm_report_status_t* rx_data) {
  if (is_error_) return -1;
  int flag = report_flag_;
  if (--flag < 0) flag = 2;
  memcpy(rx_data, &report_status_[flag], sizeof(arm_report_status_t));
  return 0;
}

void ArmReportStatus::print_data(arm_report_status_t* rx_data) {
  printf("rxcnt    = %d\n", rxcnt_);
  printf("axis     = %d\n", rx_data->axis);
  printf("status   = %d\n", rx_data->motion_status);
  printf("mode     = %d\n", rx_data->motion_mode);
  printf("mt_brake = 0x%x\n", rx_data->mt_brake);
  printf("mt_able  = 0x%x\n", rx_data->mt_able);
  printf("err_code = %d\n", rx_data->err_code);
  printf("war_code = %d\n", rx_data->war_code);
  printf("cmd_num  = %d\n", rx_data->cmd_num);
  Print::nvect("joint   = ", rx_data->joint, rx_data->axis);
  Print::nvect("pose    = ", rx_data->pose, 6);
  Print::nvect("tau     = ", rx_data->tau, rx_data->axis);
  printf("\n");
}

void ArmReportStatus::print_data(void) { print_data(&report_status_[report_flag_]); }