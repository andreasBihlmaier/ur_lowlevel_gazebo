#include "ur_lowlevel_plugin.hpp"

// system includes
#include <stdio.h>

// library includes
#include <ros/package.h>
#include <boost/bind.hpp>

// custom includes
#include "ur_driver_prog.hpp"

#include <ahbstring.h>
#include <ahbre.hpp>


namespace gazebo {

/*---------------------------------- public: -----------------------------{{{-*/
void
URLowlevelPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) 
{
  std::cout << "------------------- URLowlevelPlugin -------------------" << std::endl;

  m_model = _parent;


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

  m_resetProg = ahb::string::fileToString(ros::package::getPath("ur_driver") + "/prog_reset");
  if (m_resetProg.empty()) {
    ROS_FATAL_STREAM("Could not load resetProg");
  }
  normalizeProgString(m_resetProg);
  m_driverProg = ahb::string::fileToString(ros::package::getPath("ur_driver") + "/prog");
  if (m_driverProg.empty()) {
    ROS_FATAL_STREAM("Could not load driverProg");
  }
  normalizeProgString(m_driverProg);
  //std::cout << "m_driverProg:\n" << m_driverProg << std::endl;

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
URLowlevelPlugin::OnUpdate()
{
  m_ioService.poll();
  pollRecvUR();

  if (m_runningProg) {
    m_runningProg->update();
  }
}
/*------------------------------------------------------------------------}}}-*/

/*---------------------------------- private: ----------------------------{{{-*/
bool
URLowlevelPlugin::loadParams(sdf::ElementPtr _sdf)
{
  m_nodeName = _sdf->GetParent()->Get<std::string>("name");

  m_sendURPort = ahb::string::toNumberSlow<uint16_t>(_sdf->GetElement("sendURPort")->Get<std::string>());
  m_recvURPort = ahb::string::toNumberSlow<uint16_t>(_sdf->GetElement("recvURPort")->Get<std::string>());

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
  //printf("received: %s\n", m_lastRecvUR);

  std::string recvString(m_lastRecvUR);
  normalizeProgString(recvString);
  if (recvString == m_resetProg) {
    ROS_INFO_STREAM("Received resetProg. Ignoring.\n");
  } else if (recvString == (m_resetProg + m_driverProg) || recvString == m_driverProg) {
    ROS_INFO_STREAM("Received driverProg. Starting UrDriverProg.\n");
    std::map<std::string,double> progDict = createProgDict(recvString);
    m_runningProg = new UrDriverProg(m_joints, progDict, m_recvTcpEndpoint.address().to_string(), m_sendURPort);
  } else {
    ROS_ERROR_STREAM("Received unknown data (ignoring):\n" << recvString);
  }
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
}

void
URLowlevelPlugin::normalizeProgString(std::string& p_progString)
{
  ahb::string::replace(p_progString, "\r\n", "\n");
  ahb::re::sub(p_progString, "^  HOSTNAME = \".*\"$", "  HOSTNAME = \"dummy\"");
}

std::map<std::string,double>
URLowlevelPlugin::createProgDict(const std::string& progString)
{
  std::map<std::string,double> dict;

  std::vector<std::string> assignments = ahb::re::filter(progString, "^[ ]*[A-Z]+[A-Za-z_-]*[ ]*=[ ]*[0-9.]+[ ]*$");
  for (size_t assignIndex = 0; assignIndex < assignments.size(); ++assignIndex) {
    std::string key, value;
    const std::string& currAssignment = assignments[assignIndex];
    if (!ahb::string::parseKeyValue(currAssignment, "=", key, value, true)) {
      ROS_FATAL_STREAM("Could not parse assignment: " << currAssignment);
      continue;
    }
    double valueDouble = ahb::string::toNumberSlow<double>(value);
    dict[key] = valueDouble;
  }

  return dict;
}
/*------------------------------------------------------------------------}}}-*/


// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(URLowlevelPlugin)

} // namespaces
