#pragma once

#include <iostream>
#include <functional>
#include <mutex>

#include <inesctec_mrdt_serial_port/AsyncSerial.h>

#include "inesctec_mrdt_hangfa_discovery_q2_driver/SerialChannelsConfig.h"

namespace inesctec_mrdt_hangfa_discovery_q2_driver
{

  const int kMotCtrlFreq = 100;

  struct Motor
  {
  public:
    double encoder_res = 1;
    double gear_reduction = 1;
    int32_t enc_ticks = 0;
    int32_t enc_ticks_prev = 0;
    int32_t enc_ticks_delta = 0;
    int32_t enc_ticks_delta_pub = 0;
    double w_r = 0;
    double w = 0;
    double sample_time = 0;
    double sample_time_prev = 0;
    double sample_period = 0;

  public:
    void setEncoderRes(const double &enc_res);
    void setGearReduction(const double &gear_ratio);

    void setEncTicksDelta(const int32_t &delta_enc_ticks);
    void setEncTicks(const int32_t &total_enc_ticks);
    double getEncTicksDeltaPub();

    void setWr(const double &w_ref);

    void setSampleTime(const int32_t &time_sample);

    void reset();

  private:
    void setW();
  };

  class InesctecMrdtHangfaDiscoveryQ2
  {
  public:
    Motor mot[4];
    std::mutex mtx_;

    std::function<void()> run;

  private:
    std::string serial_port_name_;
    SerialChannelsConfig *serial_cfg_;
    CallbackAsyncSerial *serial_async_;

  public:
    InesctecMrdtHangfaDiscoveryQ2();
    InesctecMrdtHangfaDiscoveryQ2(std::string serial_port_name);
    ~InesctecMrdtHangfaDiscoveryQ2();

    void init();

    bool openSerial(const bool dbg = false);
    void closeSerial(const bool dbg = false);
    bool isSerialOpen();

    void setSerialPortName(const std::string &serial_port_name);

    void reset();

    void stopMotors();

  private:
    void rcvSerialData(const char *data, unsigned int len);
    void sendSerialData();
  };

} // namespace inesctec_mrdt_hangfa_discovery_q2_driver
