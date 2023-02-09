#include "controlBoardWrapper.h"
#include <stdexcept>

ControlBoard::ControlBoard(std::string robot_part_name, std::string robot_name)
{

    this->robot_name = robot_name;
    this->robot_part_name = robot_part_name;
    this->options.put("device", "remote_controlboard");
    this->options.put("remote", "/" + robot_name + "/" + robot_part_name);
    this->options.put("local", "/icub_airhockey_control/" + robot_part_name);
    
    if (!this->driver.open(options)){
        throw std::runtime_error("Could not open driver for " + robot_part_name);
    }
    
    this->driver.view(ipos);
    this->driver.view(iposd);
    this->driver.view(imod);
    this->driver.view(enc);
    int naxes;
    ipos->getAxes(&naxes);
    this->enc_values = std::vector<double>(naxes);

}

void ControlBoard::setPartControlMode(int mode)
{
    int naxes;
    ipos->getAxes(&naxes);
    std::vector<int> modes(naxes, mode);
    if (!imod->setControlModes(modes.data())) {
        throw std::runtime_error("Could not set control mode for part " + this->robot_part_name);
    };

}

std::vector<double> ControlBoard::getEncoderValues(){
    for (int i=0; i<1000; i++){
        if (this->enc->getEncoders(enc_values.data()))
            return enc_values;
    }
    throw std::runtime_error("Too many attempts of reading encoder values.");
}

void ControlBoard::positionDirectMove(const double* command, std::vector<int> controlled_joints_ids){
    if (!this->iposd->setPositions(controlled_joints_ids.size(), controlled_joints_ids.data(), command)){
        throw std::runtime_error("Could not move in position direct part " + this->robot_part_name);
    };
}

void ControlBoard::positionDirectMove(const double* command){
    if (!this->iposd->setPositions(command)){
        throw std::runtime_error("Could not move in position direct part " + this->robot_part_name);
    };
}