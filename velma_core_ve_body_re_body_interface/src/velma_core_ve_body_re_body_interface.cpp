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

#include <rtt/Component.hpp>

#include "common_interfaces/interface_tx.h"
#include "common_interfaces/interface_rx.h"

#include "common_interfaces/message_split.h"
#include "common_interfaces/message_concate.h"

//#include "velma_core_ve_body_re_body_interface/command_ports.h"
//#include "velma_core_ve_body_re_body_interface/status_ports.h"

#include <Eigen/Dense>
#include "velma_core_ve_body_re_body_interface/port_data.h"

#include "velma_core_ve_body_re_body_msgs/Status.h"
#include "velma_core_ve_body_re_body_msgs/Command.h"

using namespace velma_core_ve_body_re_body_msgs;
//using namespace velma_core_ve_body_re_body_interface;

typedef InterfaceTx<Status > VelmaCoreVeBodyReBodyStatusTx;
ORO_LIST_COMPONENT_TYPE(VelmaCoreVeBodyReBodyStatusTx)

typedef InterfaceRx<Status > VelmaCoreVeBodyReBodyStatusRx;
ORO_LIST_COMPONENT_TYPE(VelmaCoreVeBodyReBodyStatusRx)

typedef InterfaceTx<Command > VelmaCoreVeBodyReBodyCommandTx;
ORO_LIST_COMPONENT_TYPE(VelmaCoreVeBodyReBodyCommandTx)

typedef InterfaceRx<Command > VelmaCoreVeBodyReBodyCommandRx;
ORO_LIST_COMPONENT_TYPE(VelmaCoreVeBodyReBodyCommandRx)

typedef MessageSplit<Command_Ports > VelmaCoreVeBodyReBodyCommandSplit;
ORO_LIST_COMPONENT_TYPE(VelmaCoreVeBodyReBodyCommandSplit)

typedef MessageConcate<Status_Ports > VelmaCoreVeBodyReBodyStatusConcate;
ORO_LIST_COMPONENT_TYPE(VelmaCoreVeBodyReBodyStatusConcate)

typedef MessageSplit<Status_Ports > VelmaCoreVeBodyReBodyStatusSplit;
ORO_LIST_COMPONENT_TYPE(VelmaCoreVeBodyReBodyStatusSplit)

typedef MessageConcate<Command_Ports > VelmaCoreVeBodyReBodyCommandConcate;
ORO_LIST_COMPONENT_TYPE(VelmaCoreVeBodyReBodyCommandConcate)

ORO_CREATE_COMPONENT_LIBRARY()

