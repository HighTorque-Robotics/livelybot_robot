#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include "../include/serial_struct.h"
#include "../include/hardware/robot.h"



namespace livelybot_serial
{

    enum robotType
    {
        _12dof,
        _18dof,
        _20dof
    };

    class robot_node
    {
    public:
        robot_node()
        {

            ros::NodeHandle private_nh("~");
  //topic_name & frame_id
            private_nh.param("dof_type_",     type_,  0);
            std::cout<<"type is "<< type_<<std::endl;
            switch (type_)
            {
            case 12:
                rbt = robotType::_12dof;
                map_ = map_12dof;
                break;
            case 18:
                rbt = robotType::_18dof;
                map_ = map_18dof;
                break;
            case 20:
                rbt = robotType::_20dof;
                map_ = map_20dof;
                break;
            default:
                break;
            }

            jointStateName = "walking_motor_goals";
            
            n_motors = 12;
            rkp = 12.;
            rkd = 0.2;
            
            ros::NodeHandle nh;
            joint_state_sub = nh.subscribe<sensor_msgs::JointState>(jointStateName, 1, &robot_node::jointStateCallback, this);

        };
        ~robot_node()
        {
            for (auto &thread : rb.ser_recv_threads)
            {
                thread.join();
            }
        }

        // 回调函数，每当接收到新的JointState消息时就会被调用
        void jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg)
        {
            // `msg`包含了关节名称（name）、位置（position）、速度（velocity）和力矩（effort）
            for (size_t i = 0; i < msg->position.size(); ++i)
            {
                // 对于每个关节，你可以根据需要处理它的状态
                // ROS_INFO("Joint %s - Position: %f, Velocity: %f, Effort: %f",
                //          msg->name[i].c_str(),
                //          msg->position[i],
                //          msg->velocity[i],
                //          msg->effort[i]);
                ROS_INFO_STREAM("Received " <<  msg->position[i]);
                rb.Motors[map_[i]]->fresh_cmd_int16(msg->position[i], 0.0, 0.0, rkp, 0, rkd, 0, 0, 0);
                

                // 这里添加代码来分发或处理每个关节的数据
                // 例如，发送给电机控制器等
            }
            rb.motor_send_2();
            for (size_t i = 0; i < n_motors; i++)
            {
                motor_back_t motor;
                motor = *rb.Motors[i]->get_current_motor_state();

                // ROS_INFO_STREAM("ID: " << map_[i] << " Pos: " << motor.position << " torque: " << motor.torque);
            }
        }

    private:


        int type_;//12,18,20
        robotType rbt;
        std::string jointStateName;
        ros::Subscriber joint_state_sub;
        double rkp, rkd;
        livelybot_serial::robot rb;
        int n_motors;
        std::vector<size_t> map_18dof{0, 1, 2, 3, 4, 5, 9, 10, 11, 12, 13, 14};  // 123456
        std::vector<size_t> map_20dof{0, 1, 2, 3, 4, 5, 10, 11, 12, 13, 14, 15}; // 123456
        std::vector<size_t> map_12dof{0, 1, 5, 2, 4, 3, 6, 7, 11, 8, 10, 9};     // 124653
        std::vector<size_t> map_;
    };

};
