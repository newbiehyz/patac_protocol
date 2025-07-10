//
#include "interface/parking/serializer/odometry_serializer.h"

#include <utils/exception.h>

#include "interface/parking/odometry_data.h"
#include "protocol/odometry_data.pb.h"
#include "serializer/common_serializer.h"

namespace autodrive {

int OdometrySerializer::Serialize(const message::SerializeMsg* data_ptr,
                                  message::DeserializeMsg* msg_ptr) const {
  const auto proto_msg = dynamic_cast<DMsgType*>(msg_ptr)->MutableMsg();
  const auto data_msg = dynamic_cast<const SMsgType*>(data_ptr)->GetData();
  CommonSerializer::Serialize(data_msg, proto_msg->mutable_header());
  auto& odo_data{data_msg->GetOdometry()};

  proto_msg->mutable_position()->set_x(odo_data.position.x);
  proto_msg->mutable_position()->set_y(odo_data.position.y);
  proto_msg->mutable_position()->set_z(odo_data.position.z);

  proto_msg->mutable_orientation()->set_x(odo_data.orientation.x);
  proto_msg->mutable_orientation()->set_y(odo_data.orientation.y);
  proto_msg->mutable_orientation()->set_z(odo_data.orientation.z);

  proto_msg->set_speed(odo_data.speed);
  proto_msg->set_speed_scale(odo_data.speed_scale);

  proto_msg->mutable_acceleration()->set_x(odo_data.acceleration.x);
  proto_msg->mutable_acceleration()->set_y(odo_data.acceleration.y);

  proto_msg->set_yaw_rate(odo_data.yaw_rate);
  proto_msg->set_yaw_rate_bias(odo_data.yaw_rate_bias);
  proto_msg->set_wheel_angle(odo_data.wheel_angle);
  proto_msg->set_slope(odo_data.slope);

  {
    auto& pitch_data{odo_data.stable_pitch};
    workflow::proto::StableDataPitch* pitch_proto =
        proto_msg->mutable_stable_pitch();
    pitch_proto->set_pitch(pitch_data.pitch);
    pitch_proto->set_is_valid(pitch_data.is_valid);
    pitch_proto->set_is_jitter(pitch_data.is_jitter);
  }  // end stable_pitch

  proto_msg->set_slope_confidence(odo_data.slope_confidence);
  proto_msg->set_diagnostic_code(odo_data.diagnostic_code);

  proto_msg->mutable_cov_translation()->set_x(odo_data.cov_translation.x);
  proto_msg->mutable_cov_translation()->set_y(odo_data.cov_translation.y);
  proto_msg->mutable_cov_translation()->set_z(odo_data.cov_translation.z);

  proto_msg->mutable_cov_eular_angle()->set_x(odo_data.cov_eular_angle.x);
  proto_msg->mutable_cov_eular_angle()->set_y(odo_data.cov_eular_angle.y);
  proto_msg->mutable_cov_eular_angle()->set_z(odo_data.cov_eular_angle.z);

  proto_msg->mutable_position_hr()->set_x(odo_data.position_hr.x);
  proto_msg->mutable_position_hr()->set_y(odo_data.position_hr.y);
  proto_msg->mutable_position_hr()->set_z(odo_data.position_hr.z);

  proto_msg->set_source(odo_data.source);
  proto_msg->set_is_slope_available(odo_data.is_slope_available);
  proto_msg->set_type(
      static_cast<workflow::proto::OdometryDataType>(odo_data.type));
  proto_msg->set_state(
      static_cast<workflow::proto::OdometryDataState>(odo_data.state));
  proto_msg->set_error_code(
      static_cast<workflow::proto::OdometryDataErrorCode>(odo_data.error_code));
  proto_msg->set_yaw_rate_bias_estimate_state(
      static_cast<workflow::proto::OdometryDataYawRateBiasEstimateState>(
          odo_data.yaw_rate_bias_estimate_state));
  {
    auto& veh_data{odo_data.vehicle_info};
    workflow::proto::Vehicle* veh_proto = proto_msg->mutable_vehicle_info();
    veh_proto->set_light_brightness_raw(veh_data.light_brightness_raw);
    veh_proto->set_lignt_validity(
        static_cast<workflow::proto::LightIntensityValidity>(
            veh_data.lignt_validity));
    veh_proto->set_left_turning_light_status(
        static_cast<workflow::proto::TurningLightStatus>(
            veh_data.left_turning_light_status));
    veh_proto->set_right_turning_light_status(
        static_cast<workflow::proto::TurningLightStatus>(
            veh_data.right_turning_light_status));
    veh_proto->set_rain_fall_level(
        static_cast<workflow::proto::RainFallLevel>(veh_data.rain_fall_level));
    veh_proto->set_gear(static_cast<workflow::proto::Gear>(veh_data.gear));
 
    for (uint32_t i {0}; i < veh_data.whl_pulse.size(); ++i){
        veh_proto->add_whl_pulse(veh_data.whl_pulse.at(i));
        veh_proto->add_whl_pulse_valid(veh_data.whl_pulse_valid.at(i));
        veh_proto->add_whl_speed(veh_data.whl_speed.at(i));
        veh_proto->add_wl_pluse_direction(veh_data.wl_pluse_direction.at(i));
        veh_proto->add_tire_pressure_valid(veh_data.tire_pressure_valid.at(i));
        veh_proto->add_tire_pressure(veh_data.tire_pressure.at(i));
    }
    veh_proto->set_yaw_rate_valid(veh_data.yaw_rate_valid);
    veh_proto->set_speed_valid(veh_data.speed_valid);
    veh_proto->set_gear_valid(veh_data.gear_valid);
    veh_proto->set_wheel_size(veh_data.wheel_size);
    }
  return 0;
}

int OdometrySerializer::Deserialize(const message::DeserializeMsg* msg_ptr,
                                    message::SerializeMsg* data_ptr) const {
  const auto& proto_msg = dynamic_cast<const DMsgType*>(msg_ptr)->GetMsg();
  const auto& data_msg = dynamic_cast<SMsgType*>(data_ptr)->MutableData();
  CommonSerializer::Deserialize(proto_msg->mutable_header(), data_msg);
  auto& odo_data{data_msg->MutableOdometry()};

  if (proto_msg->has_position()) {
    auto& pos_data{odo_data.position};
    auto& pos_proto{proto_msg->position()};
    pos_data.x = pos_proto.x();
    pos_data.y = pos_proto.y();
    pos_data.z = pos_proto.z();
  }  // end position

  if (proto_msg->has_orientation()) {
    auto& orienta_data{odo_data.orientation};
    auto& orienta_proto{proto_msg->orientation()};
    orienta_data.x = orienta_proto.x();
    orienta_data.y = orienta_proto.y();
    orienta_data.z = orienta_proto.z();
  }  // end orientation

  odo_data.speed = proto_msg->speed();
  odo_data.speed_scale = proto_msg->speed_scale();

  if (proto_msg->has_acceleration()) {
    auto& acc_data{odo_data.acceleration};
    auto& acc_proto{proto_msg->acceleration()};
    acc_data.x = acc_proto.x();
    acc_data.y = acc_proto.y();
  }  // end acceleration

  odo_data.yaw_rate = proto_msg->yaw_rate();
  odo_data.yaw_rate_bias = proto_msg->yaw_rate_bias();
  odo_data.wheel_angle = proto_msg->wheel_angle();
  odo_data.slope = proto_msg->slope();

  if (proto_msg->has_stable_pitch()) {
    auto& pitch_data{odo_data.stable_pitch};
    auto& pitch_proto{proto_msg->stable_pitch()};
    pitch_data.pitch = pitch_proto.pitch();
    pitch_data.is_valid = pitch_proto.is_valid();
    pitch_data.is_jitter = pitch_proto.is_jitter();
  }  // end stable_pitch

  odo_data.slope_confidence = proto_msg->slope_confidence();
  odo_data.diagnostic_code = proto_msg->diagnostic_code();

  odo_data.cov_translation.x = proto_msg->cov_translation().x();
  odo_data.cov_translation.y = proto_msg->cov_translation().y();
  odo_data.cov_translation.z = proto_msg->cov_translation().z();

  odo_data.cov_eular_angle.x = proto_msg->cov_eular_angle().x();
  odo_data.cov_eular_angle.y = proto_msg->cov_eular_angle().y();
  odo_data.cov_eular_angle.z = proto_msg->cov_eular_angle().z();

  odo_data.position_hr.x = proto_msg->position_hr().x();
  odo_data.position_hr.y = proto_msg->position_hr().y();
  odo_data.position_hr.z = proto_msg->position_hr().z();

  odo_data.source = proto_msg->source();
  odo_data.is_slope_available = proto_msg->is_slope_available();

  odo_data.type =
      static_cast<OdometryData::OdometryDataType>(proto_msg->type());
  odo_data.state =
      static_cast<OdometryData::OdometryDataState>(proto_msg->state());
  odo_data.error_code =
      static_cast<OdometryData::OdometryDataErrorCode>(proto_msg->error_code());
  odo_data.yaw_rate_bias_estimate_state =
      static_cast<OdometryData::OdometryDataYawRateBiasEstimateState>(
          proto_msg->yaw_rate_bias_estimate_state());

  if (proto_msg->has_vehicle_info()) {
    auto& veh_data{odo_data.vehicle_info};
    auto& veh_proto{proto_msg->vehicle_info()};
    veh_data.light_brightness_raw = veh_proto.light_brightness_raw();
    veh_data.lignt_validity = static_cast<OdometryData::LightIntensityValidity>(
        veh_proto.lignt_validity());
    veh_data.left_turning_light_status =
        static_cast<OdometryData::TurningLightStatus>(
            veh_proto.left_turning_light_status());
    veh_data.right_turning_light_status =
        static_cast<OdometryData::TurningLightStatus>(
            veh_proto.right_turning_light_status());
    veh_data.rain_fall_level =
        static_cast<OdometryData::RainFallLevel>(veh_proto.rain_fall_level());
    veh_data.gear = static_cast<OdometryData::Gear>(veh_proto.gear());
  
    for (uint32_t i{0}; i < static_cast<uint32_t>(veh_proto.whl_pulse_size());
         ++i) {
        veh_data.whl_pulse.at(i) = veh_proto.whl_pulse(i);
        veh_data.whl_pulse_valid.at(i) = veh_proto.whl_pulse_valid(i);
        veh_data.whl_speed.at(i) = veh_proto.whl_speed(i);
        if(veh_proto.wl_pluse_direction_size()){
            veh_data.wl_pluse_direction.at(i) = veh_proto.wl_pluse_direction(i);
        }
        if(veh_proto.tire_pressure_valid_size()){
            veh_data.tire_pressure_valid.at(i) = veh_proto.tire_pressure_valid(i);
        }
        if(veh_proto.tire_pressure_size()){
            veh_data.tire_pressure.at(i) = veh_proto.tire_pressure(i);
        }
        
    }
    veh_data.yaw_rate_valid=veh_proto.yaw_rate_valid();
    veh_data.speed_valid=veh_proto.speed_valid();
    veh_data.gear_valid=veh_proto.gear_valid();
    veh_data.wheel_size = veh_proto.wheel_size();
  }
  return 0;
}
}  // namespace autodrive