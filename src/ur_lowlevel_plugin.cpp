#include "ur_lowlevel_plugin.hpp"

// system includes
#include <stdio.h>

// library includes
#include <boost/bind.hpp>

// custom includes
#include <ahbstring.h>


namespace gazebo
{
/*---------------------------------- public: -----------------------------{{{-*/
  void
  URLowlevelPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) 
  {
    std::cout << "------------------- URLowlevelPlugin -------------------" << std::endl;

    m_model = _parent;

    m_updatePeriod = 0.001; // TODO

    if (!ros::isInitialized()) {
      ROS_FATAL_STREAM("URLowlevelPlugin: A ROS node for Gazebo has not been initialized, unable to load plugin. "
        << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }

    if (!loadParams(_sdf)) {
      std::cout << "Error during loadParams" << std::endl;
      return;
    }

    m_node = new ros::NodeHandle(m_nodeName);
    m_lastUpdateTime = ros::Time::now();

    /*
    unsigned lwrNameLen = std::string("lwrN").size();
    std::string pluginName = _sdf->GetAttribute("name")->GetAsString();
    std::string lwrName = pluginName.substr(pluginName.size() - lwrNameLen);
    */

#if 0
    std::cout << "_sdf: name=" << _sdf->GetName() << " attributeCount=" << _sdf->GetAttributeCount() << std::endl;
    std::cout << "Attributes: " << std::endl;
    for (unsigned attrIdx = 0; attrIdx < _sdf->GetAttributeCount(); attrIdx++) {
      sdf::ParamPtr param = _sdf->GetAttribute(attrIdx);
      std::cout << attrIdx << " key=" << param->GetKey() << " value=" << param->GetAsString() << std::endl;
    }

    std::cout << "LWR name: " << m_model->GetName() << " childCount=" << m_model->GetChildCount() << std::endl;
    std::cout << "Children: " << std::endl;
    for (unsigned childIdx = 0; childIdx < m_model->GetChildCount(); childIdx++) {
      physics::BasePtr child = m_model->GetChild(childIdx);
      std::cout << childIdx << " name=" << child->GetName();
      std::cout << std::endl;
    }
#endif 

    /*
    // Extract only joints belonging to current lwr, even if it is part of larger model
    physics::Joint_V joints = m_model->GetJoints();
    std::cout << lwrName << " joints:" << std::endl;
    for (size_t jointIdx = 0; jointIdx < joints.size(); jointIdx++) {
      physics::JointPtr currJoint = joints[jointIdx];
      if (lwrName == currJoint->GetName().substr(0, lwrNameLen) || currJoint->GetName().find(std::string("::") + lwrName) != std::string::npos) {
        m_joints.push_back(currJoint);
        std::cout << jointIdx << " name=" << currJoint->GetName() << " angle=" << currJoint->GetAngle(0) << " v=" << currJoint->GetVelocity(0) << std::endl;
      }
    }
    */

    /*
    m_sendUdpSocket = new boost::asio::ip::udp::socket(m_ioService);
    m_udpResolver = new boost::asio::ip::udp::resolver(m_ioService);
    boost::asio::ip::udp::resolver::query sendPortQuery(boost::asio::ip::udp::v4(),
                                                        "localhost",
                                                        ahb::string::toString(m_sendFriPort));
    m_sendUdpEndpoint = *(m_udpResolver->resolve(sendPortQuery));

    try {
      m_sendUdpSocket->connect(m_sendUdpEndpoint);
    } catch (boost::system::system_error const& e) {
      ROS_FATAL_STREAM("URLowlevelPlugin: Failed to connect UR node: " << e.what());
    }

    m_recvUdpSocket = new boost::asio::ip::udp::socket(m_ioService,
                                                       boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(),
                                                                                      m_recvFriPort));
    */

    m_updateConnection = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&URLowlevelPlugin::OnUpdate, this));
  }

  void
  URLowlevelPlugin::OnUpdate()
  {
    ros::Duration sinceLastUpdateDuration = ros::Time::now() - m_lastUpdateTime;

    if (sinceLastUpdateDuration.toSec() >= m_updatePeriod) {
      updateRobotState();
      publishRobotState();
      m_lastUpdateTime = ros::Time::now();
    } else {
      //std::cout << "Not publishing (periodic time not yet reached), only " << sinceLastUpdateDuration.toSec()  << "s passed" << std::endl;
    }
  }
/*------------------------------------------------------------------------}}}-*/

/*---------------------------------- private: ----------------------------{{{-*/
  bool
  URLowlevelPlugin::loadParams(sdf::ElementPtr _sdf)
  {
    m_nodeName = _sdf->GetParent()->Get<std::string>("name");

    m_sendURPort = ahb::string::toIntSlow<uint16_t>(_sdf->GetElement("sendURPort")->Get<std::string>());
    m_recvURPort = ahb::string::toIntSlow<uint16_t>(_sdf->GetElement("recvURPort")->Get<std::string>());

    return true;
  }

  void
  URLowlevelPlugin::updateRobotState()
  {
    /*
    if (m_recvUdpSocket->available() == 0) {
      if (m_currentFriMsrData.head.sendSeqCount != 0) {
        ROS_FATAL_STREAM("URLowlevelPlugin: No UDP data (tFriCmdData) received");
      }
      return;
    }

    boost::asio::ip::udp::endpoint sender_endpoint;
    boost::asio::socket_base::message_flags flags;
    boost::system::error_code error;
    m_recvUdpSocket->receive_from(boost::asio::buffer(&m_lastFriCmdData, sizeof(m_lastFriCmdData)),
                                  sender_endpoint,
                                  flags,
                                  error);
    if (error) {
      ROS_FATAL_STREAM("URLowlevelPlugin: Failed receive_from(): " << error);
    }

    for (size_t jointIdx = 0; jointIdx < m_joints.size(); jointIdx++) {
      if (m_lastFriCmdData.cmd.jntPos[jointIdx] < m_joints[jointIdx]->GetLowerLimit(0).Radian()) {
        ROS_FATAL_STREAM("Joint" << jointIdx << " below joint limit (" << m_joints[jointIdx]->GetLowerLimit(0).Radian() << "). Will not move robot at all.\n");
        return;
      } else if (m_lastFriCmdData.cmd.jntPos[jointIdx] > m_joints[jointIdx]->GetUpperLimit(0).Radian()) {
        ROS_FATAL_STREAM("Joint" << jointIdx << " above joint limit (" << m_joints[jointIdx]->GetUpperLimit(0).Radian() << "). Will not move robot at all.\n");
        return;
      }
    }

    for (size_t jointIdx = 0; jointIdx < m_joints.size(); jointIdx++) {
      m_joints[jointIdx]->SetAngle(0, m_lastFriCmdData.cmd.jntPos[jointIdx]);
    }
    */
  }

  void
  URLowlevelPlugin::publishRobotState()
  {
    /*
    for (size_t jointIdx = 0; jointIdx < m_joints.size(); jointIdx++) {
      physics::JointPtr currJoint = m_joints[jointIdx];
      m_currentFriMsrData.data.msrJntPos[jointIdx] = currJoint->GetAngle(0).Radian();
      //m_currentFriMsrData.data.msrCartPos[jointIdx] = TODO
    }

    try {
      m_sendUdpSocket->send(boost::asio::buffer(&m_currentFriMsrData, sizeof(m_currentFriMsrData)));
    } catch (boost::system::system_error const& e) {
      ROS_FATAL_STREAM("URLowlevelPlugin: Failed to send to FRI node: " << e.what());
    }
    */
  }
/*------------------------------------------------------------------------}}}-*/


  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(URLowlevelPlugin)
}
