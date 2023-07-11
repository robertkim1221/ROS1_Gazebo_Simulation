#include <ros/ros.h>
#include <vector>
#include <iostream>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/GetModelStateRequest.h>
#include <gazebo_msgs/GetModelStateResponse.h>
#include "arm_test/graspObject.h"
#include "arm_test/graspposition.h"
#include "arm_test/go.h"
#include "arm_test/goRequest.h"
#include "arm_test/goResponse.h"
#include <std_msgs/Bool.h>
#include <std_msgs/Duration.h>
#include <geometry_msgs/Pose.h>
#include <string>
#include "boost/date_time/posix_time/posix_time.hpp"


class subscribeANDpublish
{
public:

    subscribeANDpublish()
    {
        sub_ = nh_.subscribe("/gazebo/model_states", 100, &subscribeANDpublish::callback, this);
        server = nh_.advertiseService("/go", &subscribeANDpublish::servicecallback, this);
        client_ = nh_.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state", true);
        client_.waitForExistence();
    }

    bool startsWith(std::string mainStr, std::string toMatch)
    {
        if(mainStr.find(toMatch) == 0)
        {
            return true;
        }
        else
        {
            return false;
        }
    
    }

    void callback(const gazebo_msgs::ModelStates::ConstPtr& msg)
    {
        do
        {
            if (msg->name.size()!=old_msg.size())
            {
                graspObject.clear();

                old_msg = msg->name;
                for (int i=0; i < msg->name.size();i++)
                {
                    std::string mainStr = old_msg[i].c_str();
                    std::string toMatch = "test_object";
                    bool result = startsWith(mainStr, toMatch);
                // std::cout<<result<<std::endl;

                    if (result)
                    {
                       graspObject.push_back(msg->name[i]);
                    }
                }
                inst_time = ros::Time::now();
                std::cout<<"waiting for object"<<std::endl;
            }
            else
            {
                return;
            }
        }//above loop repeats until at least one object is added
        while(graspObject.size() ==0);


        std::cout<<"using grasp decider 4 (time)"<<std::endl;
        srv.request.model_name = graspObject[graspObject.size()-1];
        client_.call(srv);
        nap.object_names = graspObject[graspObject.size()-1];
        nap.position = srv.response.pose.position;

        //designate the duration in which each arm is supposed to take in requests
        arm1_time = ros::Duration(240);
        arm2_time = ros::Duration(120);

        if(graspObject.size() == 1)
        {
            init_time = ros::Time::now();
            start = inst_time;
        }
        else if((inst_time - init_time) > (arm1_time+arm2_time))
        {
            
            init_time = init_time +arm1_time +arm2_time;
        }

        ros::Time stamp = inst_time - ros::Duration(start.sec);
        if((inst_time - init_time) < arm1_time)
        {   
            std::cout<<"Object added at time [min:sec]: "<<std::endl;
            boost::posix_time::ptime my_posix_time = stamp.toBoost();
            tm iso_time_str = boost::posix_time::to_tm(my_posix_time);
            int minute = iso_time_str.tm_min;
            int second = iso_time_str.tm_sec;
            std::cout<<minute<<":"<<second<<std::endl;
            std::cout<<"using Robot 1..."<<std::endl;
            go1.go_vector.push_back(nap);
            std::cout<<nap<<std::endl;
        }
        else
        {
            std::cout<<"Object added at time [min:sec]: "<<std::endl;
            boost::posix_time::ptime my_posix_time = stamp.toBoost();
            tm iso_time_str = boost::posix_time::to_tm(my_posix_time);
            int minute = iso_time_str.tm_min;
            int second = iso_time_str.tm_sec;
            std::cout<<minute<<":"<<second<<std::endl;
            std::cout<<"using Robot 2..."<<std::endl;
            go2.go_vector.push_back(nap);
            std::cout<<nap<<std::endl;
        }
    }

    bool servicecallback(arm_test::go::Request &req, arm_test::go::Response &res)
    {   
        if (req.arm_type == "arm1")
        {
            res.go_vector = go1.go_vector;
        }
        else if (req.arm_type == "arm2")
        {
            res.go_vector = go2.go_vector;
        }
        else
        {
            ROS_INFO("Grasp Decider service requires input of arm type [arm1 or arm2]!");
        }

        sleep(1.0);
        return true;
    }


private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    arm_test::goResponse go1;
    arm_test::goResponse go2;
    ros::ServiceClient client_;
    ros::ServiceServer server;
    gazebo_msgs::GetModelState srv;
    arm_test::graspposition nap;
    std::vector<std::string> graspObject;
    std::vector<std::string> old_msg;
    ros::Time inst_time;
    ros::Time init_time;
    ros::Duration arm1_time;
    ros::Duration arm2_time;
    ros::Time start;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "grasp_decider");

    subscribeANDpublish SAP;

    ros::spin();

    return 0;
}