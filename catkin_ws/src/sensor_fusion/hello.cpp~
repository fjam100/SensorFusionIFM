#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int64.h>
#include <std_msgs/String.h>
#include <Eigen/Dense>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <math.h>   
#include <tf/tf.h>

using namespace sensor_msgs;
using namespace geometry_msgs;
using namespace Eigen;
using namespace std;

double tprev =0;
geometry_msgs::Quaternion ornPrev;
geometry_msgs::Vector3 angVelPrev;
geometry_msgs::Vector3 linAccPrev;
float globalAccPrevZ=0;
float globalVelPrevZ=0;
float globalPosPrevZ=0;
float gravityBias=9.8;

Matrix3f P_ang=MatrixXf::Zero(3,3);
Matrix3f K_ang=MatrixXf::Zero(3,3);

float Q_angVel[9];
float R_ang[9];

float Q_vel=0;

float P_pos=0;
float Q_pos=0.01;
float R_pos=0.0000001;
float K_pos=0;

geometry_msgs::Quaternion multiplyQuats(geometry_msgs::Quaternion q, geometry_msgs::Quaternion r)
{
    float t0=(r.w*q.w-r.x*q.x-r.y*q.y-r.z*q.z);
    float t1=(r.w*q.x+r.x*q.w-r.y*q.z+r.z*q.y);
    float t2=(r.w*q.y+r.x*q.w+r.y*q.x-r.z*q.x);
    float t3=(r.w*q.z-r.x*q.y+r.y*q.x-r.z*q.w);
    geometry_msgs::Quaternion res;
    res.x=t1;
    res.y=t2;
    res.z=t3;
    res.w=t0;
    return res;
}

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg){

   // ROS_INFO("Acceleration: x: [%f] y:[%f] z:[%f]", msg->linear_acceleration.x,msg->linear_acceleration.y,msg->linear_acceleration.z);

    if (tprev==0){                                  //Initialize state
        ornPrev=msg->orientation;
        angVelPrev=msg->angular_velocity;
        linAccPrev=msg->linear_acceleration;
        tprev=msg->header.stamp.toSec();
        tf::Quaternion rotation(ornPrev.x, ornPrev.y, ornPrev.z, ornPrev.w);
        tf::Vector3 vector(linAccPrev.x,linAccPrev.y,linAccPrev.z);
        tf::Vector3 globalAccPrev = tf::quatRotate(rotation, vector);
        globalAccPrevZ=globalAccPrev[2];
        for (int i=0; i<9; i++){
        Q_angVel[i]=msg->angular_velocity_covariance[i];
        R_ang[i]=msg->orientation_covariance[i];
        }
        return;
    }

    //Matrix3f Quat_cov=Map<Matrix3f>(array);
    
    double dt=msg->header.stamp.toSec()-tprev;
    
    
    //Quaternion update
    
    geometry_msgs::Vector3 E;
    E.x=angVelPrev.x*dt;
    E.y=angVelPrev.y*dt;
    E.z=angVelPrev.z*dt;
    
    float w = cos(E.x/2) * cos(E.y/2) * cos(E.z/2) + sin(E.x/2) * sin(E.y/2) * sin(E.z/2);
    float x = sin(E.x/2) * cos(E.y/2) * cos(E.z/2) - cos(E.x/2) * sin(E.y/2) * sin(E.z/2);
    float y = cos(E.x/2) * sin(E.y/2) * cos(E.z/2) + sin(E.x/2) * cos(E.y/2) * sin(E.z/2);
    float z = cos(E.x/2) * cos(E.y/2) * sin(E.z/2) - sin(E.x/2) * sin(E.y/2) * cos(E.z/2);
    
    geometry_msgs::Quaternion r;
    r.x=x;
    r.y=y;
    r.z=z;
    r.w=w;
    geometry_msgs::Quaternion q=multiplyQuats(ornPrev, r);  //Prediction step
    
   
    
    for(int i=0; i<9; i++)
        Q_angVel[i]=Q_angVel[i]*dt;                         //Propagate process covariance
    P_ang=P_ang+Map<Matrix3f>(Q_angVel);                    // Should really be done with A matrix but no time, sorry
    
    //Convert predicted and measured quaternions to euler angles
    tf::Quaternion quat;
    tf::quaternionMsgToTF(q, quat);
    double roll_pred, pitch_pred, yaw_pred;
    tf::Matrix3x3(quat).getRPY(roll_pred, pitch_pred, yaw_pred);
    tf::quaternionMsgToTF(msg->orientation, quat);          //Quaternion measurement
    double roll_meas, pitch_meas, yaw_meas;
    tf::Matrix3x3(quat).getRPY(roll_meas, pitch_meas, yaw_meas);
    Vector3f pred(roll_pred, pitch_pred, yaw_pred);
    Vector3f meas(roll_meas, pitch_meas, yaw_meas);         
    K_ang=P_ang*(P_ang+Map<Matrix3f>(R_ang)).inverse();     //Measurement update
    Vector3f posteriori=pred+K_ang*(meas-pred);
    P_ang=(MatrixXf::Identity(3,3)-K_ang)*P_ang;
    q = tf::createQuaternionMsgFromRollPitchYaw( posteriori[0], posteriori[1], posteriori[2]);
    
    
    tf::Quaternion rotation(q.x, q.y, q.z, q.w);            //Prediction for position
    tf::Vector3 vector(linAccPrev.x,linAccPrev.y,linAccPrev.z);
    tf::Vector3 globalAccPrev = tf::quatRotate(rotation, vector);
    tf::Vector3 vector2 (msg->linear_acceleration_covariance[0],msg->linear_acceleration_covariance[1],msg->linear_acceleration_covariance[2]);
    tf::Vector3 globalAccCov = tf::quatRotate(rotation, vector2);
    double globalVelZ=globalVelPrevZ+(globalAccPrev[2]-gravityBias)*dt;
    globalPosPrevZ=globalPosPrevZ+globalVelZ*dt;
    Q_vel=globalAccCov[9]*dt;            //Propagate covariance -> again should be done through A ideally
    P_pos=P_pos+Q_vel*dt;
    
    ROS_INFO("Accelerometer based: [%f]", globalPosPrevZ);
    
    ornPrev=q;
    angVelPrev=msg->angular_velocity;
    linAccPrev=msg->linear_acceleration;
    for (int i=0; i<9; i++){
        Q_angVel[i]=msg->angular_velocity_covariance[i];
        R_ang[i]=msg->orientation_covariance[i];
    }
    tprev=msg->header.stamp.toSec();
}

void rangeCallback(const sensor_msgs::Range::ConstPtr& msg){
    float measured_range;
    if (tprev==0)
        return;
    ROS_INFO("Range raw measurement: [%f]", msg->range);
    //Measurement step
    tf::Quaternion rotation(ornPrev.x, ornPrev.y, ornPrev.z, ornPrev.w);
    tf::Vector3 vector(0,0,msg->range);
    tf::Vector3 rotRange = tf::quatRotate(rotation, vector);
    measured_range=rotRange[2];
    //Update step
    K_pos=P_pos*1/(P_pos+R_pos);
    globalPosPrevZ=globalPosPrevZ+K_pos*(measured_range-globalPosPrevZ);
    P_pos=(1-K_pos)*P_pos;
    ROS_INFO("Posteriori z estimate: [%f]", globalPosPrevZ);
}

int main ( int argc , char **argv ) {

    ros::init(argc , argv , "hello_ros" ) ;

    // Establish this program as a ROS node .
    ros::NodeHandle nh ;

    // Starting message
    ROS_INFO_STREAM("Cpp file running");
    ros::Subscriber sub1 = nh.subscribe("ifm_sys/imu/data", 1, imuCallback);
    ros::Subscriber sub2 = nh.subscribe("ifm_sys/distance/ground", 1, rangeCallback);
    
    ros::spin();
    
    return 0;
}
