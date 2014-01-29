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
      void updateRobotState();
      void publishRobotState();
      void onAccept(const boost::system::error_code& error);
  
      // variables
      // gazebo
      physics::ModelPtr m_model;
      event::ConnectionPtr m_updateConnection;
      physics::Joint_V m_joints;

      // ros
      ros::NodeHandle* m_node;
      std::string m_nodeName;
      ros::Time m_lastUpdateTime;

      // UR binary protocol
      uint16_t m_sendURPort;
      uint16_t m_recvURPort;
      double m_updatePeriod;
      boost::asio::io_service m_ioService;
      boost::asio::ip::tcp::acceptor* m_recvTcpAcceptor;
      boost::asio::ip::tcp::socket* m_recvTcpSocket;
      boost::asio::ip::tcp::endpoint m_recvTcpEndpoint;
      char m_lastRecvUR[64 * 1024];
      boost::asio::ip::tcp::resolver* m_tcpResolver;
      boost::asio::ip::tcp::socket* m_sendTcpSocket;
      boost::asio::ip::tcp::endpoint m_sendTcpEndpoint;
  
  };
}

#endif // _UR_LOWLEVEL_PLUGIN_H_
