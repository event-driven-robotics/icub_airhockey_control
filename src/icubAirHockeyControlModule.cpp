#include "icubAirHockeyControlModule.h"
#include <iomanip>

using namespace std;
using namespace BipedalLocomotion::GenericContainer::literals;
    
IcubAirHockeyControlModule::IcubAirHockeyControlModule()
{
    joint_names = vector<string>{"torso_yaw", "torso_roll", "torso_pitch", "r_shoulder_pitch", "r_shoulder_roll", "r_shoulder_yaw",
                                 "r_elbow", "r_wrist_prosup", "r_wrist_pitch", "r_wrist_yaw"};

    base_position = Eigen::Vector3d::Zero();
    base_orientation = manif::SO3d::Identity();
    right_arm_controlled_joint_ids = vector<int>{0, 1, 2, 3, 4, 5, 6};

    joint_measured_pos = Eigen::VectorXd::Zero(joint_names.size());

    torso_cb = new ControlBoard("torso", "icubSim");
    right_arm_cb = new ControlBoard("right_arm", "icubSim");

    ee_task = make_shared<BipedalLocomotion::IK::SE3Task>();
    base_task = make_shared<BipedalLocomotion::IK::SE3Task>();
    joint_regularization_task = make_shared<BipedalLocomotion::IK::JointTrackingTask>();

    kinDyn = make_shared<iDynTree::KinDynComputations>();
    system = make_shared<BipedalLocomotion::ContinuousDynamicalSystem::FloatingBaseSystemKinematics>();
    params_handler = make_shared<BipedalLocomotion::ParametersHandler::YarpImplementation>();
    integration_step = 0.01;
}

bool IcubAirHockeyControlModule::configure(yarp::os::ResourceFinder &rf)
{
    if (!modelLoader.loadReducedModelFromFile("/usr/local/share/iCub/robots/iCubGenova02/model.urdf", joint_names))
    {
        throw runtime_error("Something went wrong in model loading/parsing");
    }

    if (!kinDyn->loadRobotModel(modelLoader.model()))
    {
        throw runtime_error("Something went wrong with model loading for kinematics/dynamics computation");
    }


    torso_cb->setPartControlMode(VOCAB_CM_POSITION);
    right_arm_cb->setPartControlMode(VOCAB_CM_POSITION);

    std::vector<double> starting_torso_pos{0, 0, 0};
    std::vector<double> starting_right_arm_pos{-36, 30, 0, 50, 0, 3, 7, 27, 27, 0, 82, 3, 0, 0, 0, 0};

    torso_cb->positionMove(starting_torso_pos.data());
    right_arm_cb->positionMove(starting_right_arm_pos.data());


    std::vector<double> torso_encoder_values = torso_cb->getEncoderValues();
    std::vector<double> right_arm_encoder_values = right_arm_cb->getEncoderValues();

    torso_cb->setPartControlMode(VOCAB_CM_POSITION_DIRECT);
    right_arm_cb->setPartControlMode(VOCAB_CM_POSITION_DIRECT);

    joint_measured_pos << torso_encoder_values[0], torso_encoder_values[1], torso_encoder_values[2], // Torso roll, pitch, yaw
        right_arm_encoder_values[0], right_arm_encoder_values[1], right_arm_encoder_values[2],       // Shoulder roll, pitch, yaw
        right_arm_encoder_values[3],                                                                 // elbow
        right_arm_encoder_values[4], right_arm_encoder_values[5], right_arm_encoder_values[6];

    joint_measured_pos *= (M_PI / 180);

    kinDyn->setJointPos(joint_measured_pos);

    const tuple<Eigen::Vector3d, manif::SO3d, Eigen::VectorXd> state{base_position, base_orientation, joint_measured_pos};
    system->setState(state);

    integrator.setDynamicalSystem(system);
    integrator.setIntegrationStep(std::chrono::nanoseconds((int)(integration_step * 1e6)));
    params_handler->setFromFile("../ik.ini");

    variables_handler.initialize(params_handler->getGroup("VARIABLES"));

    ee_task->setKinDyn(kinDyn);
    ee_task->initialize(params_handler->getGroup("EE_TASK"));

    QPIK.initialize(params_handler->getGroup("IK"));
    base_task->setKinDyn(kinDyn);
    base_task->initialize(params_handler->getGroup("BASE_TASK"));

    joint_regularization_task->setKinDyn(kinDyn);
    joint_regularization_task->initialize(params_handler->getGroup("JOINT_REGULARIZATION_TASK"));

    QPIK.addTask(ee_task, "ee_task", 0);
    QPIK.addTask(base_task, "base_task", 0);
    QPIK.addTask(joint_regularization_task, "joint_regularization_task", 1, Eigen::VectorXd::Ones(10) * 0.1);

    QPIK.finalize(variables_handler);

    joint_regularization_task->setSetPoint(joint_measured_pos);

    manif_initial_pose = BipedalLocomotion::Conversions::toManifPose(kinDyn->getWorldTransform("r_hand_dh_frame"));
    manif_pose = manif_initial_pose;

    velocity = manif::SE3Tangentd::Zero();

    base_task->setSetPoint(manif::SE3d::Identity());
    
    return true;
}

bool IcubAirHockeyControlModule::updateModule()
{
    double amplitude = 50;
    double f = 1;
    manif_pose.translation(manif_initial_pose.translation() + Eigen::Vector3d{0, amplitude * sin(f * M_PI * i * integration_step), 0});

    velocity.lin()(1) = amplitude * f * M_PI * cos(M_PI * i * integration_step);
    ee_task->setSetPoint(manif_pose, velocity);

    torso_encoder_values = torso_cb->getEncoderValues();
    right_arm_encoder_values = right_arm_cb->getEncoderValues();

    joint_measured_pos << torso_encoder_values[0], torso_encoder_values[1], torso_encoder_values[2], // Torso roll, pitch, yaw
        right_arm_encoder_values[0], right_arm_encoder_values[1], right_arm_encoder_values[2],       // Shoulder roll, pitch, yaw
        right_arm_encoder_values[3],                                                                 // elbow
        right_arm_encoder_values[4], right_arm_encoder_values[5], right_arm_encoder_values[6];

    joint_measured_pos *= (M_PI / 180);

    kinDyn->setJointPos(joint_measured_pos);

    QPIK.advance();
    
    controlInput.get_from_hash<"twist"_h>() = Eigen::Matrix<double, 6, 1>({0, 0, 0, 0, 0, 0});
    controlInput.get_from_hash<"ds"_h>() = QPIK.getOutput().jointVelocity;

    system->setControlInput(controlInput);

    integrator.integrate(std::chrono::nanoseconds(0), std::chrono::nanoseconds((int)(integration_step*1e6)));
    const auto &[base_position, base_orientation, joint_command_pos] = integrator.getSolution();

    torso_command = joint_command_pos.head(3);
    torso_command *= (180 / M_PI);
    right_arm_command = joint_command_pos.tail(7);
    right_arm_command *= (180 / M_PI);

    torso_cb->positionDirectMove(torso_command.data());
    right_arm_cb->positionDirectMove(right_arm_command.data(), right_arm_controlled_joint_ids);
    
    i++;
    
    std::cout << std::fixed << std::setprecision(5) <<yarp::os::Time::now() - now << std::endl;
    now = yarp::os::Time::now();
    return true;
}

bool IcubAirHockeyControlModule::close()
{
    this->torso_cb->close();
    this->right_arm_cb->close();
    return true;
}

double IcubAirHockeyControlModule::getPeriod()
{
    return integration_step;
}