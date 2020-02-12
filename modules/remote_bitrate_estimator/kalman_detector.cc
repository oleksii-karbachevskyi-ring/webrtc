#include "modules/remote_bitrate_estimator/kalman_detector.h"

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

KalmanDetectorFactory::KalmanDetectorFactory(const WebRtcKeyValueConfig* key_value_config)
  : key_value_config_(key_value_config)
{}

DelayIncreaseDetectorInterface* KalmanDetectorFactory::Create() {
  return static_cast<DelayIncreaseDetectorInterface*>(
    new KalmanDetector(OverUseDetectorOptions(), key_value_config_));
}

} // namespace webrtc
