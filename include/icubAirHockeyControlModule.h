#include <yarp/os/RFModule.h>

#include <iDynTree/ModelIO/ModelLoader.h>
#include <iDynTree/KinDynComputations.h>
#include <BipedalLocomotion/IK/QPInverseKinematics.h>
#include <BipedalLocomotion/IK/SE3Task.h>
#include <BipedalLocomotion/IK/JointTrackingTask.h>
#include <BipedalLocomotion/ContinuousDynamicalSystem/ForwardEuler.h>
#include <BipedalLocomotion/ContinuousDynamicalSystem/FloatingBaseSystemKinematics.h>
#include <BipedalLocomotion/ParametersHandler/YarpImplementation.h>
#include <BipedalLocomotion/System/VariablesHandler.h>
#include <BipedalLocomotion/Conversions/ManifConversions.h>
#include <Eigen/StdVector>
#include <Eigen/Geometry>
#include <vector>
#include "controlBoardWrapper.h"

class IcubAirHockeyControlModule : public yarp::os::RFModule
{
private:
    iDynTree::ModelLoader modelLoader;
    std::vector<std::string> joint_names;

    BipedalLocomotion::IK::QPInverseKinematics QPIK;

    Eigen::Vector3d base_position;
    manif::SO3d base_orientation;
    std::vector<int> right_arm_controlled_joint_ids;

    std::vector<double> torso_encoder_values;
    std::vector<double> right_arm_encoder_values;

    Eigen::VectorXd joint_measured_pos;
    Eigen::VectorXd joint_command_pos;
    Eigen::VectorXd torso_command;
    Eigen::VectorXd right_arm_command;

    ControlBoard* torso_cb;
    ControlBoard* right_arm_cb;

    double integration_step;
    BipedalLocomotion::ContinuousDynamicalSystem::ForwardEuler<BipedalLocomotion::ContinuousDynamicalSystem::FloatingBaseSystemKinematics> integrator;
    std::shared_ptr<BipedalLocomotion::ContinuousDynamicalSystem::FloatingBaseSystemKinematics> system;
    std::shared_ptr<BipedalLocomotion::ParametersHandler::YarpImplementation> params_handler;
    std::shared_ptr<iDynTree::KinDynComputations> kinDyn;
    BipedalLocomotion::System::VariablesHandler variables_handler;

    std::shared_ptr<BipedalLocomotion::IK::SE3Task> ee_task;
    std::shared_ptr<BipedalLocomotion::IK::SE3Task> base_task;
    std::shared_ptr<BipedalLocomotion::IK::JointTrackingTask> joint_regularization_task;

    manif::SE3d manif_initial_pose;
    manif::SE3d manif_pose;
    manif::SE3Tangentd velocity;

    int i{0};
    double now{0};

public:
    IcubAirHockeyControlModule();
    bool configure(yarp::os::ResourceFinder &rf) override;
    bool updateModule() override;
    bool close() override;
    double getPeriod() override;
};