#ifndef MODULES_REMOTE_BITRATE_ESTIMATOR_KALMAN_DETECTOR_H
#define MODULES_REMOTE_BITRATE_ESTIMATOR_KALMAN_DETECTOR_H

#include "modules/congestion_controller/goog_cc/delay_increase_detector_interface.h"
#include "modules/remote_bitrate_estimator/overuse_detector.h"
#include "modules/remote_bitrate_estimator/overuse_estimator.h"


namespace webrtc {

class KalmanDetector : public DelayIncreaseDetectorInterface {
public:
  KalmanDetector(const OverUseDetectorOptions& options, const WebRtcKeyValueConfig* key_value_config);

  // Update the detector with a new sample. The deltas should represent deltas
  // between timestamp groups as defined by the InterArrival class.
  void Update(double recv_delta_ms,
              double send_delta_ms,
              int64_t send_time_ms,
              int64_t arrival_time_ms,
              size_t packet_size,
              int size_delta,
              bool calculated_deltas) override;

  BandwidthUsage State() const override;

private:
  OveruseEstimator estimator_;
  OveruseDetector detector_;
};

class KalmanDetectorFactory : public DetectorFactoryInterface {
public:
  KalmanDetectorFactory(const WebRtcKeyValueConfig* key_value_config);

  DelayIncreaseDetectorInterface* Create() override;

private:
  const WebRtcKeyValueConfig* key_value_config_;
};

}  // namespace webrtc

#endif  // MODULES_REMOTE_BITRATE_ESTIMATOR_KALMAN_DETECTOR_H
