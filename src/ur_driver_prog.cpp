#include "ur_driver_prog.hpp"

// system includes

// library includes

// custom includes
#include <ahbstring.h>

/*---------------------------------- public: -----------------------------{{{-*/
std::string
toString(const servojType& servoj)
{
  std::stringstream ss;

  ss << "time: " << servoj.time
     << " positions: ";
  for (unsigned pIdx = 0; pIdx < 6; pIdx++) {
    ss << servoj.positions[pIdx] << " ";
  }

  return ss.str();
}

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

  m_servojStart = m_servojGoal = currentState2Servoj();
  m_servojGoal.time += ros::Duration(m_updatePeriod);
}

void
UrDriverProg::update()
{
  ros::Duration sinceLastUpdateDuration = ros::Time::now() - m_lastUpdateTime;

  if (sinceLastUpdateDuration.toSec() >= m_updatePeriod) {
    m_lastUpdateTime = ros::Time::now();
    sendJointState();
    recvMsg();
    servoJoints();
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
  memset(&m_currentJointsMsgNetworkBO, 0, sizeof(m_currentJointsMsgNetworkBO));
  m_currentJointsMsgNetworkBO.mtype_joint = htonl(int32_t(m_progDict["MSG_JOINT_STATES"]));
  m_currentJointsMsgNetworkBO.mtype_wrench = htonl(int32_t(m_progDict["MSG_WRENCH"]));
  m_currentJointsMsgNetworkBO.mtype_io = htonl(int32_t(m_progDict["MSG_GET_IO"]));
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

void
UrDriverProg::recvMsg()
{
  if (!m_sendTcpSocket->available()) {
    return;
  }

  boost::asio::socket_base::message_flags flags;
  boost::system::error_code error;
  size_t bytesReceived = m_sendTcpSocket->receive(boost::asio::buffer(&m_lastRecv, sizeof(m_lastRecv)),
                                                  flags,
                                                  error);
  if (error) {
    ROS_FATAL_STREAM("Failed receive(): " << error);
    return;
  }

  int32_t* mtypeNetworkBO = (int32_t*)m_lastRecv;
  int32_t mtype = ntohl(*mtypeNetworkBO);
  if (mtype == int32_t(m_progDict["MSG_QUIT"])) {
    std::cout << "MSG_QUIT" << std::endl;
    // TODO
  } else if (mtype == int32_t(m_progDict["MSG_STOPJ"])) {
    std::cout << "MSG_STOPJ" << std::endl;
    // TODO
  } else if (mtype == int32_t(m_progDict["MSG_SERVOJ"])) {
    //std::cout << "MSG_SERVOJ" << std::endl;
    if (bytesReceived != sizeof(servojMsgType)) {
      ROS_FATAL_STREAM("Received incomplete MSG_SERVOJ. Will ignore this message.");
      return;
    }
    servojMsgType* servoMsgNetworkBO = (servojMsgType*)m_lastRecv;

    m_servojStart = currentState2Servoj();
    m_servojGoal.time = ros::Time::now() + ros::Duration(double(ntohl(servoMsgNetworkBO->time)) / m_progDict["MULT_time"]);
    double int2doubleDiv = m_progDict["MULT_jointstate"];
    for (size_t jointIdx = 0; jointIdx < m_joints.size(); jointIdx++) {
      m_servojGoal.positions[jointIdx] = double(int32_t(ntohl(servoMsgNetworkBO->positions[jointIdx]))) / int2doubleDiv;
    }
  } else {
    ROS_FATAL_STREAM("Unknown message type: " << mtype);
  }
}

void
UrDriverProg::servoJoints()
{
  if (ros::Time::now() > m_servojGoal.time) {
    return;
  }

  // indepenently interpolate each joint between servoStart and servoGoal
  double nextJointPos[6];
  double moveDuration = (m_servojGoal.time - m_servojStart.time).toSec();
  double timeIntoMove = (ros::Time::now() - m_servojStart.time).toSec();
  double interpolationParam = timeIntoMove / moveDuration;
  //std::cout << "m_servojStart: " << toString(m_servojStart) << std::endl;
  //std::cout << "m_servojGoal: " << toString(m_servojGoal) << std::endl;
  //std::cout << "moveDuration=" << moveDuration << " timeIntoMove=" << timeIntoMove << " interpolationParam=" << interpolationParam << std::endl;
  assert(interpolationParam >= 0 && interpolationParam <= 1);
  for (size_t jointIdx = 0; jointIdx < m_joints.size(); jointIdx++) {
    double moveLength = m_servojGoal.positions[jointIdx] - m_servojStart.positions[jointIdx];
    nextJointPos[jointIdx] = m_servojStart.positions[jointIdx] + interpolationParam * moveLength;
 
    if (nextJointPos[jointIdx] < m_joints[jointIdx]->GetLowerLimit(0).Radian()) {
      ROS_FATAL_STREAM("Joint" << jointIdx << " below joint limit (" << m_joints[jointIdx]->GetLowerLimit(0).Radian() << "): " << nextJointPos[jointIdx] << ". Will not move robot at all.\n");
      return;
    } else if (nextJointPos[jointIdx] > m_joints[jointIdx]->GetUpperLimit(0).Radian()) {
      ROS_FATAL_STREAM("Joint" << jointIdx << " above joint limit (" << m_joints[jointIdx]->GetUpperLimit(0).Radian() << "): " << nextJointPos[jointIdx] << ". Will not move robot at all.\n");
      return;
    }
  }

  for (size_t jointIdx = 0; jointIdx < m_joints.size(); jointIdx++) {
    m_joints[jointIdx]->SetPosition(0, nextJointPos[jointIdx]);
  }
}


servojType
UrDriverProg::currentState2Servoj()
{
  servojType servoj;
  for (size_t jointIdx = 0; jointIdx < m_joints.size(); jointIdx++) {
    servoj.positions[jointIdx] = m_joints[jointIdx]->GetAngle(0).Radian();
  }
  servoj.time = ros::Time::now();

  return servoj;
}
/*------------------------------------------------------------------------}}}-*/
