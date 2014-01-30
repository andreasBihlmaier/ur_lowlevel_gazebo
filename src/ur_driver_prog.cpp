#include "ur_driver_prog.hpp"

// system includes

// library includes

// custom includes
#include <ahbstring.h>

/*---------------------------------- public: -----------------------------{{{-*/
UrDriverProg::UrDriverProg(gazebo::physics::Joint_V& p_joints, const std::map<std::string,double>& p_progDict, const std::string& p_sendHost, uint16_t p_sendPort)
  :m_joints(p_joints),
   m_progDict(p_progDict),
   m_sendHost(p_sendHost),
   m_sendPort(p_sendPort),
   m_updatePeriod(0.008)
{
  std::cout << "m_progDict: " << ahb::string::toString(m_progDict) << std::endl;

  ROS_INFO_STREAM("Connecting to " << m_sendHost << ":" << m_sendPort);
  m_sendTcpSocket = new boost::asio::ip::tcp::socket(m_ioService);
  m_tcpResolver = new boost::asio::ip::tcp::resolver(m_ioService);
  boost::asio::ip::tcp::resolver::query sendPortQuery(boost::asio::ip::tcp::v4(),
                                                      m_sendHost,
                                                      ahb::string::toString(m_sendPort),
                                                      boost::asio::ip::tcp::resolver::query::numeric_service);
  m_sendTcpEndpoint = *(m_tcpResolver->resolve(sendPortQuery));

  try {
    m_sendTcpSocket->connect(m_sendTcpEndpoint);
  } catch (boost::system::system_error const& e) {
    ROS_FATAL_STREAM("Failed to connect UR driver (" << m_sendHost << ":" << m_sendPort << "): " << e.what());
  }
}

void
UrDriverProg::update()
{
  ros::Duration sinceLastUpdateDuration = ros::Time::now() - m_lastUpdateTime;

  if (sinceLastUpdateDuration.toSec() >= m_updatePeriod) {
    m_lastUpdateTime = ros::Time::now();
    sendJointState();
  } else {
    //std::cout << "Not publishing (periodic time not yet reached), only " << sinceLastUpdateDuration.toSec()  << "s passed" << std::endl;
  }
}
/*------------------------------------------------------------------------}}}-*/

/*--------------------------------- protected: ---------------------------{{{-*/
/*------------------------------------------------------------------------}}}-*/

/*---------------------------------- private: ----------------------------{{{-*/
void
UrDriverProg::sendJointState()
{
  m_currentJointsMsgNetworkBO.mtype = htonl(int32_t(m_progDict["MSG_JOINT_STATES"]));
  double double2intMult = m_progDict["MULT_jointstate"];
  for (size_t jointIdx = 0; jointIdx < m_joints.size(); jointIdx++) {
    gazebo::physics::JointPtr currJoint = m_joints[jointIdx];
    m_currentJointsMsgNetworkBO.positions[jointIdx] = htonl(int32_t(currJoint->GetAngle(0).Radian() * double2intMult));
    m_currentJointsMsgNetworkBO.speeds[jointIdx] = htonl(int32_t(currJoint->GetVelocity(0) * double2intMult));
  }
  //std::cout << "Sending: " << ahb::string::toHexString((char*)&m_currentJointsMsgNetworkBO, sizeof(m_currentJointsMsgNetworkBO)) << std::endl;
  try {
    m_sendTcpSocket->send(boost::asio::buffer(&m_currentJointsMsgNetworkBO, sizeof(m_currentJointsMsgNetworkBO)));
  } catch (boost::system::system_error const& e) {
    ROS_FATAL_STREAM("Failed to send joint state: " << e.what());
  }
}
/*------------------------------------------------------------------------}}}-*/
