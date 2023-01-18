#include <iDynTree/ModelIO/ModelLoader.h>
#include <iDynTree/KinDynComputations.h>
#include <BipedalLocomotion/IK/QPInverseKinematics.h>
#include <BipedalLocomotion/IK/SE3Task.h>
#include <BipedalLocomotion/ContinuousDynamicalSystem/ForwardEuler.h>
#include <BipedalLocomotion/ContinuousDynamicalSystem/FloatingBaseSystemKinematics.h>
#include <BipedalLocomotion/ParametersHandler/YarpImplementation.h>
#include <BipedalLocomotion/System/VariablesHandler.h>
#include <Eigen/StdVector>
#include <cmath>
#include <yarp/dev/IPositionDirect.h>
#include <yarp/dev/IPositionControl.h>
#include <yarp/dev/CartesianControl.h>
#include <yarp/dev/IControlMode.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/sig/Matrix.h>
#include <yarp/math/Math.h>

using namespace std;
namespace CDS = BipedalLocomotion::ContinuousDynamicalSystem;


int main(int argc, char const *argv[])
{

    iDynTree::ModelLoader modelLoader;
    vector<string> joint_names{"torso_yaw", "torso_roll", "torso_pitch", "r_shoulder_pitch", "r_shoulder_roll", "r_shoulder_yaw",
                          "r_elbow", "r_wrist_prosup", "r_wrist_pitch", "r_wrist_yaw"};
    string device_name = "right_arm";

    if (!modelLoader.loadReducedModelFromFile("/usr/local/share/iCub/robots/iCubGenova02/model.urdf", joint_names))
    {
        throw runtime_error("Something went wrong in model loading/parsing");
    }

    auto kinDyn = make_shared<iDynTree::KinDynComputations>();

    if (!kinDyn->loadRobotModel(modelLoader.model()))
    {
        throw runtime_error("Something went wrong with model loading for kinematics/dynamics computation");
    }
    BipedalLocomotion::IK::QPInverseKinematics QPIK;

    Eigen::Vector3d base_position{0, 0, 0};
    Eigen::VectorXd initial_joint_config(10);
    initial_joint_config << 0, 0, 0, -29.4, 30.4, 0, 44.6, 0, 0, 0; // Initial joint configuration TODO
    std::vector<double> right_arm_initial_joint_config {-29.4, 30.4, 0, 44.6, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    const tuple<Eigen::Vector3d, manif::SO3d, Eigen::VectorXd> state{base_position, manif::SO3d::Identity(), initial_joint_config};
    auto system = make_shared<CDS::FloatingBaseSystemKinematics>();
    
    yarp::dev::ICartesianControl* cart_control;

    yarp::dev::IPositionControl* torso_ipos;
    yarp::dev::IPositionDirect* torso_iposd;
    yarp::dev::IControlMode* torso_imod;
    yarp::dev::PolyDriver torso_driver;
    yarp::os::Property torso_options;
    torso_options.put("device", "remote_controlboard");
    torso_options.put("remote", "/icubSim/torso");
    torso_options.put("local", "/icub_airhockey_control/torso");
    torso_driver.open(torso_options);
    torso_driver.view(torso_ipos);
    torso_driver.view(torso_iposd);
    torso_driver.view(torso_imod);

    yarp::dev::IPositionControl* right_arm_ipos;
    yarp::dev::IPositionDirect* right_arm_iposd;
    yarp::dev::IControlMode* right_arm_imod;
    yarp::dev::PolyDriver right_arm_driver;
    yarp::os::Property right_arm_options;
    right_arm_options.put("device", "remote_controlboard");
    right_arm_options.put("remote", "/icubSim/right_arm");
    right_arm_options.put("local", "/icub_airhockey_control/right_arm");
    right_arm_driver.open(right_arm_options);
    right_arm_driver.view(right_arm_ipos);
    right_arm_driver.view(right_arm_iposd);
    right_arm_driver.view(right_arm_imod);

    // yarp::dev::ICartesianControl* icart;
    // yarp::dev::PolyDriver cart_driver;

    // yarp::os::Property options_cart;
    // options_cart.put("device", "cartesiancontrollerclient");
    // options_cart.put("remote", "/icubSim/cartesianController/right_ee");
    // options_cart.put("local", "/icub_airhockey_control/right_ee");
    // cart_driver.open(options_cart);
    // cart_driver.view(icart);


    int torso_naxes;
    torso_ipos->getAxes(&torso_naxes);
    std::vector<int> torso_modes(torso_naxes, VOCAB_CM_POSITION);
    torso_imod->setControlModes(torso_modes.data()); 

    int right_arm_naxes;
    right_arm_ipos->getAxes(&right_arm_naxes);

    std::vector<int> right_arm_modes(right_arm_naxes, VOCAB_CM_POSITION);
    right_arm_imod->setControlModes(right_arm_modes.data()); 

    right_arm_ipos->positionMove(right_arm_initial_joint_config.data());

    kinDyn->setJointPos(initial_joint_config);
    system->setState(state);

    CDS::ForwardEuler<CDS::FloatingBaseSystemKinematics> integrator;

    integrator.setDynamicalSystem(system);
    integrator.setIntegrationStep(1);

    auto params_handler = make_shared<BipedalLocomotion::ParametersHandler::YarpImplementation>();
    params_handler->setFromFile("../ik.ini");

    cout << params_handler->toString() << endl;

    auto variables_handler = BipedalLocomotion::System::VariablesHandler();

    variables_handler.initialize(params_handler->getGroup("VARIABLES"));

    auto ee_task = make_shared<BipedalLocomotion::IK::SE3Task>();

    ee_task->setKinDyn(kinDyn);
    ee_task->initialize(params_handler->getGroup("EE_TASK"));
    QPIK.initialize(params_handler->getGroup("IK"));

    QPIK.addTask(ee_task, "ee_task", 0);

    QPIK.finalize(variables_handler);

    std::fill(begin(right_arm_modes), end(right_arm_modes), VOCAB_CM_POSITION_DIRECT);
    right_arm_imod->setControlModes(right_arm_modes.data());
    
    std::fill(begin(torso_modes), end(torso_modes), VOCAB_CM_POSITION_DIRECT);
    torso_imod->setControlModes(torso_modes.data());

    for (int i = 0; i < 1000; ++i)
    {
        Eigen::Vector3d T{-0.3, 0.2 * sin(M_2_PI * i / 1000), -0.1};
        ee_task->setSetPoint(manif::SE3d(T, manif::SO3d::Identity()));
        QPIK.advance();

          
        system->setControlInput({{0, 0, 0, 0, 0, 0}, QPIK.getOutput().jointVelocity});
        integrator.integrate(0, 0.01);
        const auto &[base_position, base_orientation, joint_position] = integrator.getSolution();

        torso_iposd->setPositions(joint_position.head(2).data());
        auto right_arm_pos = joint_position.tail(3);
        std::vector<int> joint_ids({0, 1, 2, 3, 4, 5, 6});
        right_arm_iposd->setPositions(right_arm_pos.size(), joint_ids.data(), right_arm_pos.data());

        kinDyn->setJointPos(joint_position);


    }

    return 0;
}
