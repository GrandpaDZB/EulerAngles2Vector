#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Quaternion.h"
#include "std_msgs/Float32.h"
#include "tf/tf.h"
#include <iostream>
#include <math.h>
#include <ros/ros.h>
#define pi 3.1415926
using namespace std;

// 我知道如果写英文注释你肯定就懒得看了所以这次用中文

// 旋转矩阵
struct Matrix33{ float M[3][3]; };
// 欧拉角，XYZ分别表示绕XYZ旋转的角
struct EulerAngle { double X,Y,Z; };

// 欧拉角旋转顺序枚举变量，我们应该是ZYX顺序，如果是别的顺序就要在下面的函数参数那里修改一下
enum EEulerOrder
{
    ORDER_XYZ,
    ORDER_YZX,
    ORDER_ZXY,
    ORDER_ZYX,
    ORDER_YXZ,
    ORDER_XZY
};

// 用欧拉角生成旋转矩阵，参数为欧拉角与旋转顺序枚举变量，返回计算好的旋转矩阵
Matrix33 EulerAnglesToMatrix(const EulerAngle &inEulerAngle,EEulerOrder EulerOrder)
{
    // Convert Euler Angles passed in a vector of Radians
    // into a rotation matrix.  The individual Euler Angles are
    // processed in the order requested.
    Matrix33 Mx;

    float    Sx    = sinf(inEulerAngle.X);
    float    Sy    = sinf(inEulerAngle.Y);
    float    Sz    = sinf(inEulerAngle.Z);
    float    Cx    = cosf(inEulerAngle.X);
    float    Cy    = cosf(inEulerAngle.Y);
    float    Cz    = cosf(inEulerAngle.Z);

    switch(EulerOrder)
    {
    case ORDER_XYZ:
        Mx.M[0][0]=Cy*Cz;
        Mx.M[0][1]=-Cy*Sz;
        Mx.M[0][2]=Sy;
        Mx.M[1][0]=Cz*Sx*Sy+Cx*Sz;
        Mx.M[1][1]=Cx*Cz-Sx*Sy*Sz;
        Mx.M[1][2]=-Cy*Sx;
        Mx.M[2][0]=-Cx*Cz*Sy+Sx*Sz;
        Mx.M[2][1]=Cz*Sx+Cx*Sy*Sz;
        Mx.M[2][2]=Cx*Cy;
        break;

    case ORDER_YZX:
        Mx.M[0][0]=Cy*Cz;
        Mx.M[0][1]=Sx*Sy-Cx*Cy*Sz;
        Mx.M[0][2]=Cx*Sy+Cy*Sx*Sz;
        Mx.M[1][0]=Sz;
        Mx.M[1][1]=Cx*Cz;
        Mx.M[1][2]=-Cz*Sx;
        Mx.M[2][0]=-Cz*Sy;
        Mx.M[2][1]=Cy*Sx+Cx*Sy*Sz;
        Mx.M[2][2]=Cx*Cy-Sx*Sy*Sz;
        break;

    case ORDER_ZXY:
        Mx.M[0][0]=Cy*Cz-Sx*Sy*Sz;
        Mx.M[0][1]=-Cx*Sz;
        Mx.M[0][2]=Cz*Sy+Cy*Sx*Sz;
        Mx.M[1][0]=Cz*Sx*Sy+Cy*Sz;
        Mx.M[1][1]=Cx*Cz;
        Mx.M[1][2]=-Cy*Cz*Sx+Sy*Sz;
        Mx.M[2][0]=-Cx*Sy;
        Mx.M[2][1]=Sx;
        Mx.M[2][2]=Cx*Cy;
        break;

    case ORDER_ZYX:
        Mx.M[0][0]=Cy*Cz;
        Mx.M[0][1]=Cz*Sx*Sy-Cx*Sz;
        Mx.M[0][2]=Cx*Cz*Sy+Sx*Sz;
        Mx.M[1][0]=Cy*Sz;
        Mx.M[1][1]=Cx*Cz+Sx*Sy*Sz;
        Mx.M[1][2]=-Cz*Sx+Cx*Sy*Sz;
        Mx.M[2][0]=-Sy;
        Mx.M[2][1]=Cy*Sx;
        Mx.M[2][2]=Cx*Cy;
        break;

    case ORDER_YXZ:
        Mx.M[0][0]=Cy*Cz+Sx*Sy*Sz;
        Mx.M[0][1]=Cz*Sx*Sy-Cy*Sz;
        Mx.M[0][2]=Cx*Sy;
        Mx.M[1][0]=Cx*Sz;
        Mx.M[1][1]=Cx*Cz;
        Mx.M[1][2]=-Sx;
        Mx.M[2][0]=-Cz*Sy+Cy*Sx*Sz;
        Mx.M[2][1]=Cy*Cz*Sx+Sy*Sz;
        Mx.M[2][2]=Cx*Cy;
        break;

    case ORDER_XZY:
        Mx.M[0][0]=Cy*Cz;
        Mx.M[0][1]=-Sz;
        Mx.M[0][2]=Cz*Sy;
        Mx.M[1][0]=Sx*Sy+Cx*Cy*Sz;
        Mx.M[1][1]=Cx*Cz;
        Mx.M[1][2]=-Cy*Sx+Cx*Sy*Sz;
        Mx.M[2][0]=-Cx*Sy+Cy*Sx*Sz;
        Mx.M[2][1]=Cz*Sx;
        Mx.M[2][2]=Cx*Cy+Sx*Sy*Sz;
        break;
    }
    return(Mx);
}

// vector是旋转前的单位矩阵，调用函数后vector会根据旋转矩阵变成旋转后的矩阵
void RotateProduct(Matrix33 tmatrix, float* vector)
{
    float vector_new[3] = {0,0,0};
    vector_new[0] = tmatrix.M[0][0]*vector[0] + tmatrix.M[0][1]*vector[1] + tmatrix.M[0][2]*vector[2];
    vector_new[1] = tmatrix.M[1][0]*vector[0] + tmatrix.M[1][1]*vector[1] + tmatrix.M[1][2]*vector[2];
    vector_new[2] = tmatrix.M[2][0]*vector[0] + tmatrix.M[2][1]*vector[1] + tmatrix.M[2][2]*vector[2];
    vector[0] = vector_new[0];
    vector[1] = vector_new[1];
    vector[2] = vector_new[2];
    return;
}

 
bool is_init = false;                      // 记录四元数数据是否已经初始化
float distance = 0;                        // 记录激光测距的距离长度
EulerAngle angles, Euler, Euler_init;      // 分别记录 需要旋转的欧拉角 此时的欧拉角 初始的欧拉角
Matrix33 tmatrix;                          // 旋转矩阵
tf::Quaternion quat;                       // tf的四元数类型
geometry_msgs::Quaternion q;               // 此时的四元数
geometry_msgs::Quaternion q_init;          // 初始的四元数
std_msgs::Float32 height;                  // 记录飞机高度
ros::Subscriber vision_sub;                // 获取飞机当前四元数
ros::Subscriber laser_sub;                 // 获取激光测距距离
ros::Publisher laser_pub;                  // 发送飞机当前高度

// 当vision_pose返回数据时，利用当前的四元数换算成当前的欧拉角，如果是第一次返回的值。就会保存为Euler_init
// 所以这个程序必须在主程序启动后，飞机姿态没有改变时启动
void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    if(!is_init){
        q_init = msg->pose.orientation;
        tf::quaternionMsgToTF(q_init, quat);
        tf::Matrix3x3(quat).getRPY(Euler_init.X, Euler_init.Y, Euler_init.Z);
        is_init = true;
    }
    q = msg->pose.orientation;
    tf::quaternionMsgToTF(q, quat);
    tf::Matrix3x3(quat).getRPY(Euler.X,Euler.Y,Euler.Z);
    return;
}

// 当/sun/laser_distance返回数据时调用
void distanceCallback(const std_msgs::Float32::ConstPtr &msg)
{
    // 飞机初始状态的欧拉角不是0，而是t265的放置姿态，大约是绕Y旋转66度左右
    // 所以飞机由初始状态到当前状态所旋转的欧拉角应该是做差得到，angles即为从初始到当前还需要旋转的欧拉角
    angles.X = Euler.X - Euler_init.X;
    angles.Y = Euler.Y - Euler_init.Y;
    angles.Z = Euler.Z - Euler_init.Z;
    // 由欧拉角的到旋转矩阵
    tmatrix = EulerAnglesToMatrix(angles, ORDER_ZYX);
    // 设定初始时方向向量为指向地面的单位向量
    float unit_vector[3] = {0, 0, -1};
    // 将unit_vector变成旋转后的方向向量，方向由飞机屁股垂直向外
    RotateProduct(tmatrix, unit_vector);
    // 飞机的高度应该等于激光测距在unit_vector的z轴分量
    height.data = abs(msg->data*unit_vector[2]);
    // 将计算好的飞机高度发送到/sun/laser_height
    laser_pub.publish(height);
    // 将高度打印出来方便查看
    ROS_INFO("height: %.2f", height.data);
    return;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "node_EulerTransform");
	ros::NodeHandle nh;

    laser_sub = nh.subscribe("/sun/laser_distance", 10, distanceCallback);
    vision_sub = nh.subscribe("/mavros/vision_pose/pose", 10, poseCallback);
    laser_pub = nh.advertise<std_msgs::Float32>("/sun/laser_height", 10);

    ros::spin();

    return 0;
}