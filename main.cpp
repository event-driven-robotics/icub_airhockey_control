#include <iDynTree/ModelIO/ModelLoader.h>
#include <iDynTree/KinDynComputations.h>
#include <BipedalLocomotion/IK/QPInverseKinematics.h>
#include <BipedalLocomotion/IK/SE3Task.h>
#include <BipedalLocomotion/IK/JointTrackingTask.h>
#include <BipedalLocomotion/ContinuousDynamicalSystem/ForwardEuler.h>
#include <BipedalLocomotion/ContinuousDynamicalSystem/FloatingBaseSystemKinematics.h>
#include <BipedalLocomotion/ParametersHandler/YarpImplementation.h>
#include <BipedalLocomotion/System/VariablesHandler.h>
#include <Eigen/StdVector>
#include <Eigen/Geometry>
#include <cmath>
#include <yarp/dev/IPositionDirect.h>
#include <yarp/dev/IPositionControl.h>
#include <yarp/dev/CartesianControl.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/IControlMode.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/sig/Matrix.h>
#include <yarp/math/Math.h>


// -0.30 0.20 0.02 -0.14 -0.78 0.59 3.07


using namespace std;
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

    // Open iCub devices for torso and right arm
    yarp::dev::ICartesianControl* cart_control;

    yarp::dev::IPositionControl* torso_ipos;
    yarp::dev::IPositionDirect* torso_iposd;
    yarp::dev::IControlMode* torso_imod;
    yarp::dev::IEncoders* torso_enc;
    yarp::dev::PolyDriver torso_driver;
    yarp::os::Property torso_options;
    torso_options.put("device", "remote_controlboard");
    torso_options.put("remote", "/icubSim/torso");
    torso_options.put("local", "/icub_airhockey_control/torso");
    torso_driver.open(torso_options);
    torso_driver.view(torso_ipos);
    torso_driver.view(torso_iposd);
    torso_driver.view(torso_imod);
    torso_driver.view(torso_enc);
    
    int torso_naxes;
    torso_ipos->getAxes(&torso_naxes);
    std::vector<int> torso_modes(torso_naxes, VOCAB_CM_POSITION_DIRECT);
    torso_imod->setControlModes(torso_modes.data()); 
    std::vector<double>torso_encoder_values(torso_naxes);
    while (!(torso_enc->getEncoders(torso_encoder_values.data()))) continue;

    yarp::dev::IPositionControl* right_arm_ipos;
    yarp::dev::IPositionDirect* right_arm_iposd;
    yarp::dev::IControlMode* right_arm_imod;
    yarp::dev::PolyDriver right_arm_driver;
    yarp::dev::IEncoders* right_arm_enc;
    yarp::os::Property right_arm_options;
    right_arm_options.put("device", "remote_controlboard");
    right_arm_options.put("remote", "/icubSim/right_arm");
    right_arm_options.put("local", "/icub_airhockey_control/right_arm");
    right_arm_driver.open(right_arm_options);
    right_arm_driver.view(right_arm_ipos);
    right_arm_driver.view(right_arm_iposd);
    right_arm_driver.view(right_arm_imod);
    right_arm_driver.view(right_arm_enc);

    int right_arm_naxes;
    right_arm_ipos->getAxes(&right_arm_naxes);
    std::vector<double>right_arm_encoder_values(right_arm_naxes);

    while (!(right_arm_enc->getEncoders(right_arm_encoder_values.data()))) continue;

    std::vector<int> right_arm_modes(right_arm_naxes, VOCAB_CM_POSITION_DIRECT);
    right_arm_imod->setControlModes(right_arm_modes.data()); 

    // Create IK related objects
    BipedalLocomotion::IK::QPInverseKinematics QPIK;

    // Base (robot root reference frame) is always static in this case
    Eigen::Vector3d base_position{0, 0, 0};
    manif::SO3d base_orientation = manif::SO3d::Identity();
    double integration_step = 0.001;
    std::vector<int> right_arm_controlled_joint_ids({0,1,2,3,4,5,6});
    Eigen::VectorXd joint_measured_pos(10);
    Eigen::VectorXd joint_command_pos(10);

    auto system = make_shared<CDS::FloatingBaseSystemKinematics>();

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

    auto joint_regularization_task = make_shared<BipedalLocomotion::IK::JointTrackingTask>();
    
    joint_regularization_task->setKinDyn(kinDyn);
    joint_regularization_task->initialize(params_handler->getGroup("JOINT_REGULARIZATION_TASK"));

    QPIK.addTask(ee_task, "ee_task", 0);
    QPIK.addTask(joint_regularization_task, "joint_regularization_task", 1, Eigen::VectorXd::Ones(10) * 0.1);

    QPIK.finalize(variables_handler);

    std::vector<Eigen::Vector3d> trajectory_positions;
    
    for (int i = 0; i < 1000; ++i)
    {
        trajectory_positions.push_back(Eigen::Vector3d{-0.3, 0.2 * cos(M_2_PI * i / 1000), 0.0});
    }

    bool wp_reached = true;
    auto traj_iter = trajectory_positions.begin();
    


    while (true)    
    {
        if (wp_reached){
            wp_reached = false;
            traj_iter ++;
            Eigen::Vector3d T = *traj_iter;
            Eigen::Vector4d q( 0, -0.7071068, 0.7071068, 0);
            q.normalize();
            manif::SO3d O = manif::SO3d(q);
            ee_task->setSetPoint(manif::SE3d(T, O));
        }


        while (!(right_arm_enc->getEncoders(right_arm_encoder_values.data()))) continue;
        while (!(torso_enc->getEncoders(torso_encoder_values.data()))) continue;
        joint_measured_pos << torso_encoder_values[0], torso_encoder_values[1], torso_encoder_values[2], // Torso roll, pitch, yaw
                            right_arm_encoder_values[0], right_arm_encoder_values[1], right_arm_encoder_values[2], // Shoulder roll, pitch, yaw
                            right_arm_encoder_values[3], // elbow
                            right_arm_encoder_values[4], right_arm_encoder_values[5], right_arm_encoder_values[6];
        joint_measured_pos *= (M_PI / 180);

        joint_regularization_task->setSetPoint(joint_measured_pos, Eigen::VectorXd::Zero(joint_measured_pos.size()));
        
        const tuple<Eigen::Vector3d, manif::SO3d, Eigen::VectorXd> state{base_position, base_orientation, joint_measured_pos};
        kinDyn->setJointPos(joint_measured_pos);
        system->setState(state);

        QPIK.advance();
        
        // cout << QPIK.getOutput().jointVelocity.transpose() << endl;
        system->setControlInput({{0, 0, 0, 0, 0, 0}, QPIK.getOutput().jointVelocity});

        integrator.integrate(0, integration_step);
        const auto &[base_position, base_orientation, joint_command_pos] = integrator.getSolution();

        Eigen::VectorXd torso_command = joint_command_pos.head(3);
        torso_command *= (180 / M_PI);
        // torso_iposd->setPositions(torso_command.data());
        Eigen::VectorXd right_arm_command = joint_command_pos.tail(7);
        right_arm_command *= (180 / M_PI);

        cout << right_arm_command << "\n\n";
        right_arm_iposd->setPositions(right_arm_command.size(), right_arm_controlled_joint_ids.data(), right_arm_command.data());
        
        cout << kinDyn->getWorldTransform("r_hand_dh_frame").asHomogeneousTransform().toString() << endl;
    }

    return 0;
}
