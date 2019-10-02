
#ifndef DOUBLE_POSE_SENSORMANAGER_H
#define DOUBLE_POSE_SENSORMANAGER_H

#include <ros/ros.h>

#include <msf_core/msf_core.h>
#include <msf_core/msf_sensormanagerROS.h>
#include <msf_core/msf_IMUHandler_ROS.h>
#include "msf_statedef.hpp"
#include <msf_updates/pose_sensor_handler/pose_sensorhandler.h>
#include <msf_updates/pose_sensor_handler/pose_measurement.h>
#include <msf_updates/DoublePoseSensorConfig.h>

#include "sensor_fusion_comm/InitScale.h"
#include "sensor_fusion_comm/InitHeight.h"

namespace msf_double_pose_sensor {

typedef msf_updates::DoublePoseSensorConfig Config_T;
typedef dynamic_reconfigure::Server<Config_T> ReconfigureServer;
typedef shared_ptr<ReconfigureServer> ReconfigureServerPtr;

class DoublePoseSensorManager : 
    public msf_core::MSF_SensorManagerROS<msf_updates::EKFState>{

    /* GPS Pose sensor Handler type definition */
    typedef msf_pose_sensor::PoseSensorHandler<
        msf_updates::pose_measurement::PoseMeasurement<
            msf_updates::EKFState::StateDefinition_T::L,
            msf_updates::EKFState::StateDefinition_T::q_ic,
            msf_updates::EKFState::StateDefinition_T::p_ic,
            msf_updates::EKFState::StateDefinition_T::q_wv,
            msf_updates::EKFState::StateDefinition_T::p_wv>, 
        DoublePoseSensorManager> GPSPoseSensorHandler_T;
    
    /* Camera Pose sensor Handler type definition */
    typedef msf_pose_sensor::PoseSensorHandler<
        msf_updates::pose_measurement::PoseMeasurement<
            msf_updates::EKFState::StateDefinition_T::L,
            msf_updates::EKFState::StateDefinition_T::q1_ic,
            msf_updates::EKFState::StateDefinition_T::p1_ic,
            msf_updates::EKFState::StateDefinition_T::q1_wv,
            msf_updates::EKFState::StateDefinition_T::p1_wv>, 
        DoublePoseSensorManager> CameraPoseSensorHandler_T;
    
public:
    typedef msf_updates::EKFState EKFState_T;
    typedef EKFState_T::StateSequence_T StateSequence_T;
    typedef EKFState_T::StateDefinition_T StateDefinition_T;

    DoublePoseSensorManager(ros::NodeHandle pnh = ros::NodeHandle("~/double_pose_sensor")) {
    bool distortmeas = false;  ///< Distort the pose measurements.

    imu_handler_.reset(
        new msf_core::IMUHandler_ROS<msf_updates::EKFState>(*this, "msf_core",
                                                            "imu_handler"));
    gps_pose_handler_.reset(
        new GPSPoseSensorHandler_T(*this, "", "gps_pose_sensor", distortmeas));

    camera_pose_handler_.reset(
        new CameraPoseSensorHandler_T(*this, "", "camera_pose_sensor", distortmeas));

    AddHandler(gps_pose_handler_);
    AddHandler(camera_pose_handler_);

    reconf_server_.reset(new ReconfigureServer(pnh));
    ReconfigureServer::CallbackType f = boost::bind(&DoublePoseSensorManager::Config,
                                                    this, _1, _2);
    reconf_server_->setCallback(f);

    init_scale_srv_ = pnh.advertiseService("initialize_msf_scale",
                                            &DoublePoseSensorManager::InitScale, this);
    init_height_srv_ = pnh.advertiseService("initialize_msf_height",
                                            &DoublePoseSensorManager::InitHeight,
                                            this);
    }
    virtual ~DoublePoseSensorManager() { }

    virtual const Config_T& Getcfg() {
        return config_;
    }

private:
    shared_ptr<msf_core::IMUHandler_ROS<msf_updates::EKFState> > imu_handler_;
    shared_ptr<GPSPoseSensorHandler_T> gps_pose_handler_;
    shared_ptr<CameraPoseSensorHandler_T> camera_pose_handler_;

    Config_T config_;
    ReconfigureServerPtr reconf_server_;
    ros::ServiceServer init_scale_srv_;
    ros::ServiceServer init_height_srv_;

    /// Minimum initialization height. If a abs(height) is smaller than this value, 
    /// no initialization is performed.
    static constexpr double MIN_INITIALIZATION_HEIGHT = 0.01;

    /**
    * \brief Dynamic reconfigure callback.
    */
    virtual void Config(Config_T &config, uint32_t level) {
        config_ = config;
        gps_pose_handler_->SetNoises(config.gps_pose_noise_meas_p,
                                    config.gps_pose_noise_meas_q);
        gps_pose_handler_->SetDelay(config.gps_pose_delay);

        camera_pose_handler_->SetNoises(config.camera_pose_noise_meas_p,
                                        config.camera_pose_noise_meas_q);
        camera_pose_handler_->SetDelay(config.camera_pose_delay);
        
        if ((level & msf_updates::DoublePoseSensor_INIT_FILTER)
            && config.core_init_filter == true) {
            Init(config.pose_initial_scale);
            config.core_init_filter = false;
        }
        // Init call with "set height" checkbox.
        if ((level & msf_updates::DoublePoseSensor_SET_HEIGHT)
            && config.core_set_height == true) {
            Eigen::Matrix<double, 3, 1> p = gps_pose_handler_->GetPositionMeasurement();
            if (p.norm() == 0) {
                MSF_WARN_STREAM(
                    "No measurements received yet to initialize position. Height init "
                    "not allowed.");
                return;
            }
            double scale = p[2] / config.core_height;
            Init(scale);
            config.core_set_height = false;
        }
    }

    bool InitScale(sensor_fusion_comm::InitScale::Request &req,
                    sensor_fusion_comm::InitScale::Response &res) {
        ROS_INFO("Initialize filter with scale %f", req.scale);
        Init(req.scale);
        res.result = "Initialized scale";
        return true;
    }

    bool InitHeight(sensor_fusion_comm::InitHeight::Request &req,
                    sensor_fusion_comm::InitHeight::Response &res) {
        ROS_INFO("Initialize filter with height %f", req.height);
        Eigen::Matrix<double, 3, 1> p = gps_pose_handler_->GetPositionMeasurement();
        if (p.norm() == 0) {
            MSF_WARN_STREAM(
                "No measurements received yet to initialize position. Height init "
                "not allowed.");
            return false;
        }
        std::stringstream ss;
        if (std::abs(req.height) > MIN_INITIALIZATION_HEIGHT) {
            double scale = p[2] / req.height;
            Init(scale);
            ss << scale;
            res.result = "Initialized by known height. Initial scale = " + ss.str();
        } else {
            ss << "Height to small for initialization, the minimum is "
                << MIN_INITIALIZATION_HEIGHT << "and " << req.height << "was set.";
            MSF_WARN_STREAM(ss.str());
            res.result = ss.str();
            return false;
        }
        return true;
    }

    void Init(double scale) const {
        // TODO: 
    }
};

} // end namespace

#endif // DOUBLE_POSE_SENSORMANAGER_H
