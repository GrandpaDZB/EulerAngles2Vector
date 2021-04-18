#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/tf.h>
using namespace std;

class PC_transformer
{
private:
    /* data */
public:
    geometry_msgs::Pose pose_l515; 
    geometry_msgs::Pose pose_l515_init;
    /* offset: from L515 to PX4 and from Work point to Start point */
    geometry_msgs::Vector3 offset_PX4toL515, offset_StarttoPX4;
    geometry_msgs::Vector3 obj_position_fromL515, obj_position_fromStart;
    geometry_msgs::Vector3 Euler_angles_init, Euler_angles, Euler_angles_delta;
    float Matrix3d[3][3];
    PC_transformer(){};
    ~PC_transformer(){};


    geometry_msgs::Vector3 Quaternion2Euler(geometry_msgs::Quaternion quat){
        geometry_msgs::Vector3 Euler_angles;
        tf::Quaternion RQ2;
        tf::quaternionMsgToTF(quat,RQ2);  
        tf::Matrix3x3(RQ2).getRPY(Euler_angles.x,Euler_angles.y,Euler_angles.z);  
        return Euler_angles;
    };

    void Euler2Matrix3d(geometry_msgs::Vector3 inEuler_angles, float M[3][3]){
        float    Sx    = sinf(inEuler_angles.x);
        float    Sy    = sinf(inEuler_angles.y);
        float    Sz    = sinf(inEuler_angles.z);
        float    Cx    = cosf(inEuler_angles.x);
        float    Cy    = cosf(inEuler_angles.y);
        float    Cz    = cosf(inEuler_angles.z);

        M[0][0]=Cy*Cz;
        M[0][1]=Cz*Sx*Sy-Cx*Sz;
        M[0][2]=Cx*Cz*Sy+Sx*Sz;
        M[1][0]=Cy*Sz;
        M[1][1]=Cx*Cz+Sx*Sy*Sz;
        M[1][2]=-Cz*Sx+Cx*Sy*Sz;
        M[2][0]=-Sy;
        M[2][1]=Cy*Sx;
        M[2][2]=Cx*Cy;
        return;
    }

    // After updating data, run count()
    void count(){
        this->offset_StarttoPX4.x = this->pose_l515.position.x;
        this->offset_StarttoPX4.y = this->pose_l515.position.y;
        this->offset_StarttoPX4.z = this->pose_l515.position.z;
        cout << "Finish offset counting" << endl;
        this->Euler_angles_init = this->Quaternion2Euler(this->pose_l515_init.orientation);
        cout << "Finish Euler_angles_init counting" << endl;
        this->Euler_angles = this->Quaternion2Euler(this->pose_l515.orientation);
        cout << "Finish Euler_angles counting" << endl;
        this->Euler_angles_delta.x = this->Euler_angles.x - this->Euler_angles_init.x;
        this->Euler_angles_delta.y = this->Euler_angles.y - this->Euler_angles_init.y;
        this->Euler_angles_delta.z = this->Euler_angles.z - this->Euler_angles_init.z;
        cout << "Finish Euler_angles_delta counting" << endl;
        this->Euler2Matrix3d(this->Euler_angles_delta, this->Matrix3d);
        cout << "Finish matrix counting" << endl;

        this->obj_position_fromStart.x = this->Matrix3d[0][0]*this->obj_position_fromL515.x + 
                                         this->Matrix3d[0][1]*this->obj_position_fromL515.y + 
                                         this->Matrix3d[0][2]*this->obj_position_fromL515.z;
        this->obj_position_fromStart.y = this->Matrix3d[1][0]*this->obj_position_fromL515.x + 
                                         this->Matrix3d[1][1]*this->obj_position_fromL515.y + 
                                         this->Matrix3d[1][2]*this->obj_position_fromL515.z;
        this->obj_position_fromStart.z = this->Matrix3d[2][0]*this->obj_position_fromL515.x + 
                                         this->Matrix3d[2][1]*this->obj_position_fromL515.y + 
                                         this->Matrix3d[2][2]*this->obj_position_fromL515.z;
        cout << "Finish obj_position_fromStart counting" << endl;
        this->obj_position_fromStart.x += this->offset_StarttoPX4.x;
        this->obj_position_fromStart.y += this->offset_StarttoPX4.y;
        this->obj_position_fromStart.z += this->offset_StarttoPX4.z;
        cout << "Finish counting" << endl;

        return;
    }

};

bool is_pose_l515_init = false;
PC_transformer pc_t;
ros::Subscriber pose_sub, pointscloud_sub;


void poseCallback(const geometry_msgs::PoseStampedConstPtr &msg)
{
    if(!is_pose_l515_init)
        pc_t.pose_l515_init = msg->pose;
    pc_t.pose_l515 = msg->pose;
    return;
}

void pcCallback(const geometry_msgs::Vector3ConstPtr &msg)
{
    pc_t.obj_position_fromL515 = *msg;
    pc_t.count();
    ROS_INFO("PointsCloud: [%.2f, %.2f, %.2f]", pc_t.obj_position_fromStart.x, pc_t.obj_position_fromStart.y, pc_t.obj_position_fromStart.z);
    return;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv,"pc_trans");
    ros::NodeHandle nh;
    ros::Rate sleep(20);

    pose_sub = nh.subscribe("/mavros/vision_pose", 10, poseCallback);
    pointscloud_sub = nh.subscribe("/sun/pointscloud", 10, pcCallback);

    pc_t.offset_PX4toL515.x = 0.0;
    pc_t.offset_PX4toL515.y = 0.0;
    pc_t.offset_PX4toL515.z = 0.0;

    pc_t.offset_StarttoPX4.x = 0.0;
    pc_t.offset_StarttoPX4.y = 0.0;
    pc_t.offset_StarttoPX4.z = 0.0;
    
    ros::spin();
    return 0;
}
