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
#include <cmath>

#include <yarp/sig/Matrix.h>
#include <yarp/os/RFModule.h>
#include <yarp/math/Math.h>
#include <chrono>
#include <thread>

#include <controlBoardWrapper.h>

using namespace std;
using namespace std::chrono_literals;
namespace CDS = BipedalLocomotion::ContinuousDynamicalSystem;



int main(int argc, char const *argv[])
{

    iDynTree::ModelLoader modelLoader;
    vector<string> joint_names{"torso_yaw", "torso_roll", "torso_pitch", "r_shoulder_pitch", "r_shoulder_roll", "r_shoulder_yaw",
                               "r_elbow", "r_wrist_prosup", "r_wrist_pitch", "r_wrist_yaw"};

    if (!modelLoader.loadReducedModelFromFile("/usr/local/share/iCub/robots/iCubGenova02/model.urdf", joint_names))
    {
        throw runtime_error("Something went wrong in model loading/parsing");
    }

    auto kinDyn = make_shared<iDynTree::KinDynComputations>();

    if (!kinDyn->loadRobotModel(modelLoader.model()))
    {
        throw runtime_error("Something went wrong with model loading for kinematics/dynamics computation");
    }

    // Create IK related objects
    BipedalLocomotion::IK::QPInverseKinematics QPIK;

    // Base (robot root reference frame) is always static in this case
    Eigen::Vector3d base_position{0, 0, 0};
    manif::SO3d base_orientation = manif::SO3d::Identity();
    double integration_step = 0.01;
    std::vector<int> right_arm_controlled_joint_ids({0, 1, 2, 3, 4, 5, 6});
    Eigen::VectorXd joint_measured_pos(10);
    Eigen::VectorXd joint_command_pos(10);

    ControlBoard torso_cb("torso", "icubSim");
    ControlBoard right_arm_cb("right_arm", "icubSim");
    std::this_thread::sleep_for(500ms);

    std::vector<double> torso_encoder_values = torso_cb.getEncoderValues();
    std::vector<double> right_arm_encoder_values = right_arm_cb.getEncoderValues();

    torso_cb.setPartControlMode(VOCAB_CM_POSITION_DIRECT);
    right_arm_cb.setPartControlMode(VOCAB_CM_POSITION_DIRECT);

    joint_measured_pos << torso_encoder_values[0], torso_encoder_values[1], torso_encoder_values[2], // Torso roll, pitch, yaw
        right_arm_encoder_values[0], right_arm_encoder_values[1], right_arm_encoder_values[2],       // Shoulder roll, pitch, yaw
        right_arm_encoder_values[3],                                                                 // elbow
        right_arm_encoder_values[4], right_arm_encoder_values[5], right_arm_encoder_values[6];

    joint_measured_pos *= (M_PI / 180);
    kinDyn->setJointPos(joint_measured_pos);
    auto system = make_shared<CDS::FloatingBaseSystemKinematics>();

    const tuple<Eigen::Vector3d, manif::SO3d, Eigen::VectorXd> state{base_position, base_orientation, joint_measured_pos};
    system->setState(state);

    CDS::ForwardEuler<CDS::FloatingBaseSystemKinematics> integrator;

    integrator.setDynamicalSystem(system);
    integrator.setIntegrationStep(integration_step);

    auto params_handler = make_shared<BipedalLocomotion::ParametersHandler::YarpImplementation>();
    params_handler->setFromFile("../ik.ini");

    cout << params_handler->toString() << endl;

    auto variables_handler = BipedalLocomotion::System::VariablesHandler();

    variables_handler.initialize(params_handler->getGroup("VARIABLES"));

    auto ee_task = make_shared<BipedalLocomotion::IK::SE3Task>();

    ee_task->setKinDyn(kinDyn);
    ee_task->initialize(params_handler->getGroup("EE_TASK"));
    QPIK.initialize(params_handler->getGroup("IK"));

    auto base_task = make_shared<BipedalLocomotion::IK::SE3Task>();

    base_task->setKinDyn(kinDyn);
    base_task->initialize(params_handler->getGroup("BASE_TASK"));

    auto joint_regularization_task = make_shared<BipedalLocomotion::IK::JointTrackingTask>();

    joint_regularization_task->setKinDyn(kinDyn);
    joint_regularization_task->initialize(params_handler->getGroup("JOINT_REGULARIZATION_TASK"));

    QPIK.addTask(ee_task, "ee_task", 0);
    QPIK.addTask(base_task, "base_task", 0);
    QPIK.addTask(joint_regularization_task, "joint_regularization_task", 1, Eigen::VectorXd::Ones(10) * 0.1);

    QPIK.finalize(variables_handler);

    joint_regularization_task->setSetPoint(joint_measured_pos);
    // cout << kinDyn->getWorldTransform("r_hand_dh_frame").asHomogeneousTransform().toString() << endl;

    manif::SE3d manif_initial_pose = BipedalLocomotion::Conversions::toManifPose(kinDyn->getWorldTransform("r_hand_dh_frame"));
    manif::SE3d manif_pose(manif_initial_pose);

    manif::SE3Tangentd velocity = manif::SE3Tangentd::Zero();
    int i = 0;

    base_task->setSetPoint(manif::SE3d::Identity());

    while (true)
    {

        manif_pose.translation(manif_initial_pose.translation() + Eigen::Vector3d{0, 0.1 * sin(M_PI * i * integration_step), 0});
        velocity.lin()(1) = 0.2 * M_PI * cos(M_PI * i * integration_step);

        i++;
        // cout << velocity.coeffs().transpose() << endl;
        ee_task->setSetPoint(manif_pose, velocity);

        torso_encoder_values = torso_cb.getEncoderValues();
        right_arm_encoder_values = right_arm_cb.getEncoderValues();

        joint_measured_pos << torso_encoder_values[0], torso_encoder_values[1], torso_encoder_values[2], // Torso roll, pitch, yaw
            right_arm_encoder_values[0], right_arm_encoder_values[1], right_arm_encoder_values[2],       // Shoulder roll, pitch, yaw
            right_arm_encoder_values[3],                                                                 // elbow
            right_arm_encoder_values[4], right_arm_encoder_values[5], right_arm_encoder_values[6];
        joint_measured_pos *= (M_PI / 180);

        kinDyn->setJointPos(joint_measured_pos);

        QPIK.advance();

        // cout << QPIK.getOutput().jointVelocity.transpose() << endl;
        system->setControlInput({{0, 0, 0, 0, 0, 0}, QPIK.getOutput().jointVelocity});

        // cout << QPIK.getOutput().jointVelocity.transpose() << endl;

        integrator.integrate(0, integration_step);
        const auto &[base_position, base_orientation, joint_command_pos] = integrator.getSolution();

        Eigen::VectorXd torso_command = joint_command_pos.head(3);
        torso_command *= (180 / M_PI);
        torso_cb.positionDirectMove(torso_command.data());
        Eigen::VectorXd right_arm_command = joint_command_pos.tail(7);
        right_arm_command *= (180 / M_PI);

        cout << right_arm_command.transpose() << "\n\n";
        right_arm_cb.positionDirectMove(right_arm_command.data(), right_arm_controlled_joint_ids);
        std::this_thread::sleep_for(100ms);
    }

    return 0;
}
