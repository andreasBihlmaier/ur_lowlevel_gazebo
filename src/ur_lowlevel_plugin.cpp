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

    m_updatePeriod = 0.008;

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

    std::string urName("ur5");

#if 0
    std::cout << "_sdf: name=" << _sdf->GetName() << " attributeCount=" << _sdf->GetAttributeCount() << std::endl;
    std::cout << "Attributes: " << std::endl;
    for (unsigned attrIdx = 0; attrIdx < _sdf->GetAttributeCount(); attrIdx++) {
      sdf::ParamPtr param = _sdf->GetAttribute(attrIdx);
      std::cout << attrIdx << " key=" << param->GetKey() << " value=" << param->GetAsString() << std::endl;
    }

    std::cout << "UR name: " << m_model->GetName() << " childCount=" << m_model->GetChildCount() << std::endl;
    std::cout << "Children: " << std::endl;
    for (unsigned childIdx = 0; childIdx < m_model->GetChildCount(); childIdx++) {
      physics::BasePtr child = m_model->GetChild(childIdx);
      std::cout << childIdx << " name=" << child->GetName();
      std::cout << std::endl;
    }
#endif 

    physics::Joint_V joints = m_model->GetJoints();
    // Extract only joints belonging to ur, even if it is part of larger model
    // Two cases:
    // 1) Not part of larger model ("::" not part of any name) -> use all joints
    // 2) Part of larger model -> use only joints with urName parent/prefix
    bool urPartOfLargerModel = false;
    for (size_t jointIdx = 0; jointIdx < joints.size(); jointIdx++) {
      physics::JointPtr currJoint = joints[jointIdx];
      if (currJoint->GetName().find(std::string("::")) != std::string::npos) {
        urPartOfLargerModel = true;
        break;
      }
    }
    std::cout << urName << " joints:" << std::endl;
    for (size_t jointIdx = 0; jointIdx < joints.size(); jointIdx++) {
      physics::JointPtr currJoint = joints[jointIdx];
      if (!urPartOfLargerModel || (urName == currJoint->GetName().substr(0, urName.size()) || currJoint->GetName().find(std::string("::") + urName) != std::string::npos)) {
        m_joints.push_back(currJoint);
        std::cout << jointIdx << " name=" << currJoint->GetName() << " angle=" << currJoint->GetAngle(0) << " v=" << currJoint->GetVelocity(0) << std::endl;
      }
    }

    /*
    m_sendTcpSocket = new boost::asio::ip::tcp::socket(m_ioService);
    m_tcpResolver = new boost::asio::ip::tcp::resolver(m_ioService);
    boost::asio::ip::tcp::resolver::query sendPortQuery(boost::asio::ip::tcp::v4(),
                                                        "localhost",
                                                        ahb::string::toString(m_sendFriPort));
    m_sendTcpEndpoint = *(m_tcpResolver->resolve(sendPortQuery));

    try {
      m_sendTcpSocket->connect(m_sendTcpEndpoint);
    } catch (boost::system::system_error const& e) {
      ROS_FATAL_STREAM("URLowlevelPlugin: Failed to connect UR node: " << e.what());
    }
    */

    m_recvTcpAcceptor = new boost::asio::ip::tcp::acceptor(m_ioService,
                                                           boost::asio::ip::tcp::endpoint(boost::asio::ip::tcp::v4(),
                                                                                          m_recvURPort));

    m_nextRecvTcpSocket = new boost::asio::ip::tcp::socket(m_ioService);
    m_recvTcpSocket = NULL;
    startAsyncAccept(*m_nextRecvTcpSocket);

    m_updateConnection = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&URLowlevelPlugin::OnUpdate, this));
  }

  void
  URLowlevelPlugin::startAsyncAccept(boost::asio::ip::tcp::socket& p_recvSocket)
  {
    m_recvTcpAcceptor->async_accept(p_recvSocket,
                                    m_recvTcpEndpoint,
                                    boost::bind(&URLowlevelPlugin::onAccept, this, boost::asio::placeholders::error));
  }

  void
  URLowlevelPlugin::onAccept(const boost::system::error_code& error)
  {
    if (error) {
      ROS_FATAL_STREAM("Failed in onAccept(): " << error);
      return;
    }

    m_recvTcpSocket = m_nextRecvTcpSocket;
    m_nextRecvTcpSocket = new boost::asio::ip::tcp::socket(m_ioService);
    startAsyncAccept(*m_nextRecvTcpSocket);
    ROS_INFO_STREAM("Connected to " << m_recvTcpEndpoint.address());

    // TODO
    // So far everything is general, but we don't want to implement UR Scripting
    // language, therefore add a seperate class ProgClass that receives and sends universal_robot/ur_driver/prog
    // messages. If this prog should change only ProgClass has to be
    // modified not this more general plugin
    // Implement recv by calling a method on ProgClass; send by handing a function ptr to ProgClass
  }

  void
  URLowlevelPlugin::OnUpdate()
  {
    ros::Duration sinceLastUpdateDuration = ros::Time::now() - m_lastUpdateTime;

    m_ioService.poll();

    pollRecvUR();
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
  URLowlevelPlugin::pollRecvUR()
  {
    if (!m_recvTcpSocket || !m_recvTcpSocket->available()) {
      return;
    }

    boost::asio::socket_base::message_flags flags;
    boost::system::error_code error;
    size_t bytesReceived = m_recvTcpSocket->receive(boost::asio::buffer(&m_lastRecvUR, sizeof(m_lastRecvUR)),
                                                    flags,
                                                    error);
    if (error) {
      ROS_FATAL_STREAM("Failed receive(): " << error);
      return;
    }
    m_lastRecvUR[bytesReceived] = '\0';
    printf("received: %s\n", m_lastRecvUR);
  }

  void
  URLowlevelPlugin::updateRobotState()
  {
    /*
    if (m_recvTcpSocket->available() == 0) {
      if (m_currentFriMsrData.head.sendSeqCount != 0) {
        ROS_FATAL_STREAM("URLowlevelPlugin: No tcp data (tFriCmdData) received");
      }
      return;
    }

    boost::asio::ip::tcp::endpoint sender_endpoint;
    boost::asio::socket_base::message_flags flags;
    boost::system::error_code error;
    m_recvTcpSocket->receive_from(boost::asio::buffer(&m_lastFriCmdData, sizeof(m_lastFriCmdData)),
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
      m_sendTcpSocket->send(boost::asio::buffer(&m_currentFriMsrData, sizeof(m_currentFriMsrData)));
    } catch (boost::system::system_error const& e) {
      ROS_FATAL_STREAM("URLowlevelPlugin: Failed to send to FRI node: " << e.what());
    }
    */
  }
/*------------------------------------------------------------------------}}}-*/


  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(URLowlevelPlugin)
}
