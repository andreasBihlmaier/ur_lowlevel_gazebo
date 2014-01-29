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
      void updateRobotState();
      void publishRobotState();
  
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
      boost::asio::ip::udp::resolver* m_udpResolver;
      boost::asio::ip::udp::socket* m_sendUdpSocket;
      boost::asio::ip::udp::endpoint m_sendUdpEndpoint;
      boost::asio::ip::udp::socket* m_recvUdpSocket;
  
  };
}

#endif // _UR_LOWLEVEL_PLUGIN_H_
