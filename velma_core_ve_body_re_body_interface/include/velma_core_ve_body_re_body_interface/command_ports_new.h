/*
 Copyright (c) 2014, Robot Control and Pattern Recognition Group, Warsaw University of Technology
 All rights reserved.
 
 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
     * Redistributions of source code must retain the above copyright
       notice, this list of conditions and the following disclaimer.
     * Redistributions in binary form must reproduce the above copyright
       notice, this list of conditions and the following disclaimer in the
       documentation and/or other materials provided with the distribution.
     * Neither the name of the Warsaw University of Technology nor the
       names of its contributors may be used to endorse or promote products
       derived from this software without specific prior written permission.
 
 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL <COPYright HOLDER> BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef __VELMA_CORE_VE_BODY_RE_BODY_COMMAND_PORTS_H__
#define __VELMA_CORE_VE_BODY_RE_BODY_COMMAND_PORTS_H__

#include "rtt/RTT.hpp"

#include "common_interfaces/interface_ports.h"
#include "velma_core_ve_body_re_body_msgs/Command.h"
#include "velma_core_ve_body_re_body_interface/port_data.h"

using namespace velma_core_ve_body_re_body_msgs;

using namespace interface_ports;

namespace velma_core_ve_body_re_body_interface {

template <template <typename Type> class T>
class ArmCommand_Ports : public PortsContainer<Command, CommandArm > {
public:
    ArmCommand_Ports(RTT::TaskContext &tc, const std::string &prefix, CommandArm Command::*ptr) :
    PortsContainer(ptr) {
        addPort(boost::shared_ptr<PortInterface<CommandArm > >(new Port<T, Eigen::Matrix<double,7,1>, CommandArm, CommandArm::_t_type >(tc, prefix + "_t", &CommandArm::t)));
    }
};

template <template <typename Type> class T>
class HandCommand_Ports : public PortsContainer<Command, CommandHand > {
public:
    HandCommand_Ports(RTT::TaskContext &tc, const std::string &prefix, CommandHand Command::*ptr) :
    PortsContainer(ptr) {
        addPort(boost::shared_ptr<PortInterface<CommandHand > >(new Port<T, Eigen::Matrix<double,4,1>, CommandHand, CommandHand::_q_type >(tc, prefix + "_q", &CommandHand::q)));
        addPort(boost::shared_ptr<PortInterface<CommandHand > >(new Port<T, Eigen::Matrix<double,4,1>, CommandHand, CommandHand::_dq_type >(tc, prefix + "_dq", &CommandHand::dq)));
        addPort(boost::shared_ptr<PortInterface<CommandHand > >(new Port<T, Eigen::Matrix<double,4,1>, CommandHand, CommandHand::_max_i_type >(tc, prefix + "_max_i", &CommandHand::max_i)));
        addPort(boost::shared_ptr<PortInterface<CommandHand > >(new Port<T, Eigen::Matrix<double,4,1>, CommandHand, CommandHand::_max_p_type >(tc, prefix + "_max_p", &CommandHand::max_p)));
        addPort(boost::shared_ptr<PortInterface<CommandHand > >(new Port<T, bool, CommandHand, CommandHand::_hold_type >(tc, prefix + "_hold", &CommandHand::hold)));
    }
};

template <template <typename Type> class T>
class SimpleCommand_Ports : public PortsContainer<Command, CommandSimple > {
public:
    SimpleCommand_Ports(RTT::TaskContext &tc, const std::string &prefix, CommandSimple Command::*ptr) :
    PortsContainer(ptr) {
        addPort(boost::shared_ptr<PortInterface<CommandSimple > >(new Port<T, int32_t, CommandSimple, CommandSimple::_cmd_type>(tc, prefix + "_cmd", &CommandSimple::cmd)));
        addPort(boost::shared_ptr<PortInterface<CommandSimple > >(new Port<T, bool, CommandSimple, CommandSimple::_valid_type>(tc, prefix + "_valid", &CommandSimple::valid)));
    }
};

template <template <typename Type> class T>
class MotorCommand_Ports : public PortsContainer<Command, CommandMotor > {
public:
    MotorCommand_Ports(RTT::TaskContext &tc, const std::string &prefix, CommandMotor Command::*ptr) :
    PortsContainer(ptr) {
        addPort(boost::shared_ptr<PortInterface<CommandMotor > >(new Port<T, double, CommandMotor, CommandMotor::_i_type >(tc, prefix + "_i", &CommandMotor::i)));
        addPort(boost::shared_ptr<PortInterface<CommandMotor > >(new Port<T, double, CommandMotor, CommandMotor::_q_type >(tc, prefix + "_q", &CommandMotor::q)));
        addPort(boost::shared_ptr<PortInterface<CommandMotor > >(new Port<T, double, CommandMotor, CommandMotor::_dq_type >(tc, prefix + "_dq", &CommandMotor::dq)));
    }
};


template <template <typename Type> class T>
class FTSensorCommand_Ports {
public:
    FTSensorCommand_Ports(RTT::TaskContext &tc, const std::string &prefix) {
    }
};

template <template <typename Type> class T>
class VelmaCommand_Ports : public PortsContainer<Command > {
public:
    typedef Command Container;

    ArmCommand_Ports<T > cont_rArm_;
    bool cont_rArm_valid_;
    ArmCommand_Ports<T > cont_lArm_;
    bool cont_lArm_valid_;
    Port<T, std_msgs::Int32, Container, Container::_rArmFri_type > port_rArmFri_;
    bool port_rArmFri_valid_;
    HandCommand_Ports<T > cont_rHand_;
    bool cont_rHand_valid_;
    MotorCommand_Ports<T > tMotor_;
    bool cont_tMotor_valid_;
    // etc.

    VelmaCommand_Ports(RTT::TaskContext &tc, const std::string& prefix)
        : cont_rArm_(tc, prefix + "_rArm")
        , cont_lArm_(tc, prefix + "_lArm")
        , port_rArmFri_(tc, prefix + "_rArmFri")
        , cont_rHand_(tc, prefix + "_rHand")
        , cont_tMotor_(tc, prefix + "_tMotor")
    {
    }

    virtual bool readPorts() {
        bool result = true;
        cont_rArm_valid_ = cont_rArm_->readPorts();
        cont_lArm_valid_ = cont_lArm_->readPorts();
        port_rArmFri_valid_ = port_rArmFri_->readPorts();
        cont_rHand_valid_ = cont_rHand_->readPorts();
        result &= (cont_tMotor_valid_ = tMotor_->readPorts());
        // etc.
        return result;
    }

    virtual bool writePorts() {
        if (cont_rArm_valid_) {
            cont_rArm_->writePorts();
        }
        if (cont_lArm_valid_) {
            cont_lArm_->writePorts();
        }
        if (port_rArmFri_valid_) {
            port_rArmFri_->writePorts();
        }
        if (cont_rHand_valid_) {
            cont_rHand_->writePorts();
        }
        tMotor_->writePorts();
        // etc.
        return true;
    }

    virtual void convertFromROS(const rosC &ros) {
        cont_rArm_->convertFromROS();
        cont_rArm_valid_ = ros.rArm_valid;
        cont_lArm_->convertFromROS();
        cont_lArm_valid_ = ros.lArm_valid;
        port_rArmFri_->convertFromROS();
        port_rArmFri_valid_ = ros.ArmFri_valid;
        cont_rHand_->convertFromROS();
        cont_rHand_valid_ = ros.rHand_valid;
        tMotor_->convertFromROS();
        // etc.
    }

    virtual void convertToROS(rosC &ros) {
        if (!cont_tMotor_valid_ /* || etc. */) {
            ros = rosC();
        }
        else {
            cont_rArm_->convertToROS();
            ros.rArm_valid = cont_rArm_valid_;
            cont_lArm_->convertToROS();
            ros.lArm_valid = cont_lArm_valid_;
            port_rArmFri_->convertToROS();
            ros.ArmFri_valid = port_rArmFri_valid_;
            cont_rHand_->convertToROS();
            ros.rHand_valid = cont_rHand_valid_;
            tMotor_->convertToROS();
        }        
    }


};

template class VelmaCommand_Ports<RTT::InputPort >;
template class VelmaCommand_Ports<RTT::OutputPort >;

};  // namespace velma_core_ve_body_re_body_interface

#endif  // __VELMA_CORE_VE_BODY_RE_BODY_COMMAND_PORTS_H__

