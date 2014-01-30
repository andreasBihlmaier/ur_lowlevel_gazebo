#ifndef _UR_LOWLEVEL_PLUGIN_H_
#define _UR_LOWLEVEL_PLUGIN_H_

// system includes

// library includes
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include <ros/ros.h>

#include <boost/asio.hpp>


// custom includes


// forward declarations
class UrDriverProg;

namespace gazebo {

class URLowlevelPlugin
  : public ModelPlugin
{
  public:
    // enums

    // typedefs

    // const static member variables
 
    // static utility functions


    // constructors

    // overwritten methods
    void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/);

    // methods
    void OnUpdate();

    // variables


  private:
    // methods
    bool loadParams(sdf::ElementPtr _sdf);
    void pollRecvUR();
    void startAsyncAccept(boost::asio::ip::tcp::socket& p_recvSocket);
    void onAccept(const boost::system::error_code& error);
    void normalizeProgString(std::string& p_progString);
    std::map<std::string,double> createProgDict(const std::string& progString);

    // variables
    // gazebo
    physics::ModelPtr m_model;
    event::ConnectionPtr m_updateConnection;
    physics::Joint_V m_joints;

    // ros
    ros::NodeHandle* m_node;
    std::string m_nodeName;

    // UR binary protocol
    uint16_t m_sendURPort;
    uint16_t m_recvURPort;
    std::string m_resetProg;
    std::string m_driverProg;
    boost::asio::io_service m_ioService;
    boost::asio::ip::tcp::acceptor* m_recvTcpAcceptor;
    boost::asio::ip::tcp::socket* m_recvTcpSocket;
    boost::asio::ip::tcp::socket* m_nextRecvTcpSocket;
    boost::asio::ip::tcp::endpoint m_recvTcpEndpoint;
    char m_lastRecvUR[64 * 1024];

    UrDriverProg* m_runningProg;

};

} // namespaces

#endif // _UR_LOWLEVEL_PLUGIN_H_
