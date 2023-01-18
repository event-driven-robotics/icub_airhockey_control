#include <iDynTree/ModelIO/ModelLoader.h>
#include <iDynTree/KinDynComputations.h>
#include <BipedalLocomotion/IK/QPInverseKinematics.h>
#include <BipedalLocomotion/IK/SO3Task.h>
#include <BipedalLocomotion/ContinuousDynamicalSystem/ForwardEuler.h>
#include <BipedalLocomotion/ContinuousDynamicalSystem/FloatingBaseSystemKinematics.h>
#include <BipedalLocomotion/ParametersHandler/TomlImplementation.h>
#include <BipedalLocomotion/System/VariablesHandler.h>
#include <Eigen/StdVector>
#include <cmath>


using namespace std;
namespace CDS = BipedalLocomotion::ContinuousDynamicalSystem;

int main(int argc, char const *argv[])
{
    
    iDynTree::ModelLoader modelLoader;
    vector<string> joints{"torso_yaw", "torso_roll", "torso_pitch", "r_shoulder_pitch", "r_shoulder_roll", "r_shoulder_yaw",
                           "r_elbow", "r_wrist_prosup", "r_wrist_pitch", "r_wrist_yaw"};
    if (!modelLoader.loadReducedModelFromFile("/usr/local/share/iCub/robots/iCubGenova02/model.urdf", joints)){
        throw runtime_error("Something went wrong in model loading/parsing");
    }

    auto kinDyn = make_shared<iDynTree::KinDynComputations>();

    if (!kinDyn->loadRobotModel(modelLoader.model())) {
        throw runtime_error("Something went wrong with model loading for kinematics/dynamics computation");
    }
    BipedalLocomotion::IK::QPInverseKinematics QPIK;
    auto task = make_shared<BipedalLocomotion::IK::SO3Task>();
    Eigen::Vector3d a {0, 0, 0};
    Eigen::VectorXd b(9); 
    b << 0, 0, 0, 0, 0, 0, 0, 0, 0;
    const tuple<Eigen::Vector3d, manif::SO3d, Eigen::VectorXd> state{a, manif::SO3d::Identity(), b};
    auto system = make_shared<CDS::FloatingBaseSystemKinematics>();
    
    
    // auto state = system.getState();

    // system.setState(state);
    
    CDS::ForwardEuler<CDS::FloatingBaseSystemKinematics> integrator;

    integrator.setDynamicalSystem(system);
    integrator.setIntegrationStep(0.01);
    

    // BipedalLocomotion::ParametersHandler::TomlImplementation params_handler;
    // auto params_handler = make_shared<BipedalLocomotion::ParametersHandler::TomlImplementation>;
    // params_handler->setFromFile("../ik.toml");
    
    // QPIK.addTask(task, "task1", 0);

    // auto variables_handler = BipedalLocomotion::System::VariablesHandler()

    // QPIK.initialize(weak_ptr<BipedalLocomotion::ParametersHandler::TomlImplementation>(params_handler));
    // QPIK.finalize()
    for (int i = 0; i < 1000; ++i){
        task->setSetPoint(manif::SO3d(10, 10*sin(M_2_PI * i / 1000), 10));
        QPIK.advance();
        cout << QPIK.getOutput().jointVelocity;
    }

    return 0;
}
