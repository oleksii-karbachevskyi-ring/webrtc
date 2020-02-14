#include "modules/remote_bitrate_estimator/kalman_detector.h"

#include "modules/congestion_controller/goog_cc/trendline_estimator.h"

namespace webrtc {

KalmanDetector::KalmanDetector(const OverUseDetectorOptions& options,
                               const WebRtcKeyValueConfig* key_value_config)
  : estimator_(options)
  , detector_(key_value_config)
{}

void KalmanDetector::Update(double recv_delta_ms,
                            double send_delta_ms,
                            int64_t /*send_time_ms*/,
                            int64_t arrival_time_ms,
                            size_t /*packet_size*/,
                            int size_delta,
                            bool calculated_deltas) {
  if (calculated_deltas == false)
    return;
  estimator_.Update(recv_delta_ms, send_delta_ms, size_delta, detector_.State(),
                    arrival_time_ms);
  detector_.Detect(estimator_.offset(), send_delta_ms,
                   estimator_.num_of_deltas(), arrival_time_ms);
}

BandwidthUsage KalmanDetector::State() const {
  return detector_.State();
}

class KalmanOverTrendlineDetector : public DelayIncreaseDetectorInterface {
public:
  KalmanOverTrendlineDetector(const WebRtcKeyValueConfig* key_value_config, NetworkStatePredictor* network_state_predictor)
    : trendline_detector_(key_value_config, network_state_predictor)
    , kalman_detector_(OverUseDetectorOptions(), key_value_config)
  { }

  ~KalmanOverTrendlineDetector() override {}

  void Update(double recv_delta_ms,
              double send_delta_ms,
              int64_t send_time_ms,
              int64_t arrival_time_ms,
              size_t packet_size,
              int size_delta,
              bool calculated_deltas) override {
    trendline_detector_.Update(recv_delta_ms, send_delta_ms, send_time_ms, arrival_time_ms,
                               packet_size, size_delta, calculated_deltas);
    kalman_detector_.Update(recv_delta_ms, send_delta_ms, send_time_ms, arrival_time_ms,
                            packet_size, size_delta, calculated_deltas);
  }

  BandwidthUsage State() const override {
    return kalman_detector_.State();
  }

private:
  TrendlineEstimator trendline_detector_;
  KalmanDetector kalman_detector_;
  RTC_DISALLOW_COPY_AND_ASSIGN(KalmanOverTrendlineDetector);
};

KalmanDetectorFactory::KalmanDetectorFactory(const WebRtcKeyValueConfig* key_value_config)
  : key_value_config_(key_value_config)
{}

DelayIncreaseDetectorInterface* KalmanDetectorFactory::Create() {
  return static_cast<DelayIncreaseDetectorInterface*>(
    new KalmanDetector(OverUseDetectorOptions(), key_value_config_));
}

KalmanOverTrendlineDetectorFactory::KalmanOverTrendlineDetectorFactory(const WebRtcKeyValueConfig* key_value_config,
                                                                       NetworkStatePredictor* network_state_predictor)
  : key_value_config_(key_value_config)
  , network_state_predictor_(network_state_predictor)
{}

DelayIncreaseDetectorInterface* KalmanOverTrendlineDetectorFactory::Create() {
  return static_cast<DelayIncreaseDetectorInterface*>(
    new KalmanOverTrendlineDetector(key_value_config_, network_state_predictor_));
}

} // namespace webrtc
