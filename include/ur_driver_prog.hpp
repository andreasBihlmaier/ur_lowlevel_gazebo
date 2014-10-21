#ifndef _UR_DRIVER_PROG_H_
#define _UR_DRIVER_PROG_H_

// system includes

// library includes
#include <gazebo/physics/physics.hh>

#include <ros/ros.h>

#include <boost/asio.hpp>

// custom includes


// forward declarations


struct jointsMsg
{
  int32_t mtype_joint;
  int32_t positions[6];
  int32_t speeds[6];
  int32_t torques[6];
  int32_t mtype_wrench;
  int32_t wrenches[6];
  int32_t mtype_io;
  int32_t io_digital_in;
  int32_t io_digital_out;
  int32_t io_digital_flag;
  int32_t io_analog_out[2];
  int32_t io_analog_in[4];
} __attribute__ ((__packed__));
typedef struct jointsMsg jointsMsgType;

struct servojMsg
{
  int32_t mtype;
  int32_t waypoint_id;
  int32_t positions[6];
  int32_t time;
} __attribute__ ((__packed__));
typedef struct servojMsg servojMsgType;


struct servoj
{
  double positions[6];
  ros::Time time;
};
typedef struct servoj servojType;


class UrDriverProg
{
  public:
    // enums

    // typedefs

    // const static member variables
 
    // static utility functions


    // constructors
    UrDriverProg(gazebo::physics::Joint_V& p_joints, const std::map<std::string,double>& p_progDict, const std::string& p_sendHost, uint16_t p_sendPort);

    // overwritten methods

    // methods
    void update();

    // variables


  protected:
    // methods

    // variables


  private:
    // methods
    void sendJointState();
    void recvMsg();
    void servoJoints();
    servojType currentState2Servoj();

    // variables
    gazebo::physics::Joint_V& m_joints;
    std::map<std::string,double> m_progDict;
    std::string m_sendHost;
    uint16_t m_sendPort;
    double m_updatePeriod; // "frame" period
    ros::Time m_lastUpdateTime;
    boost::asio::io_service m_ioService;
    boost::asio::ip::tcp::resolver* m_tcpResolver;
    boost::asio::ip::tcp::socket* m_sendTcpSocket;
    boost::asio::ip::tcp::endpoint m_sendTcpEndpoint;
    jointsMsgType m_currentJointsMsgNetworkBO;
    char m_lastRecv[1024];
    servojType m_servojStart;
    servojType m_servojGoal;
};

#endif // _UR_DRIVER_PROG_H_
