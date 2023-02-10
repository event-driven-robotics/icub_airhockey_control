#include <string>
#include <yarp/dev/IPositionDirect.h>
#include <yarp/dev/IPositionControl.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/IControlMode.h>
#include <yarp/dev/PolyDriver.h>
#include <vector>

class ControlBoard{

    private:
        yarp::dev::IPositionControl* ipos;
        yarp::dev::IPositionDirect* iposd;
        yarp::dev::IControlMode* imod;
        yarp::dev::IEncoders* enc;
        yarp::dev::PolyDriver driver;
        yarp::os::Property options;
        std::string robot_part_name;
        std::string robot_name;
        std::vector<double> enc_values;

    public:
        ControlBoard(){};
        ControlBoard(std::string robot_part_name, std::string robot_name="icub");
        void setPartControlMode(int mode);
        std::vector < double > getEncoderValues();
        void positionDirectMove(const double* command, std::vector<int> controlled_joints_ids);
        void positionDirectMove(const double * command);
        void positionMove(const double* command, std::vector<int> controlled_joints_ids);
        void positionMove(const double * command);
        void close();
};
