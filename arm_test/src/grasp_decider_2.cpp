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
#include "std_msgs/Bool.h"
#include <geometry_msgs/Pose.h>
#include <string>



class subscribeANDpublish
{
public:

    subscribeANDpublish()
    {
        sub_ = nh_.subscribe("/gazebo/model_states", 100, &subscribeANDpublish::callback, this);
        state = nh_.subscribe("/arm1/robot_state", 1, &subscribeANDpublish::statecallback, this);
        
        server = nh_.advertiseService("/go", &subscribeANDpublish::servicecallback, this);
        
        client_ = nh_.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state", true);
        client_.waitForExistence();

        // var = true;
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

    void statecallback(const std_msgs::Bool msg)
    {
        boolean = msg.data;
        if (boolean == true)
        {
           var = true;
        }
        else 
        {
            var = false;
        }
        // std::cout<<var<<std::endl;
        
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
                    if (result)
                    {
                        graspObject.push_back(msg->name[i]);
                    }
                }
            }
            else
            {
                return;
            }
        } while (graspObject.size() ==0);

        srv.request.model_name = graspObject[graspObject.size()-1];
        client_.call(srv);

        nap.object_names = graspObject[graspObject.size()-1];
        nap.position = srv.response.pose.position;

        if (graspObject.size() != 1)
        {
            if (var == true)
            {
                go1.go_vector.push_back(nap);
                std::cout<<"Robot 1 IS NOT in use"<<std::endl;
                std::cout<<"using Robot 1..."<<std::endl;
                std::cout<<"Recognized Object Position:"<<std::endl;
                std::cout<<nap<<std::endl;
            }
            
            else 
            {
                go2.go_vector.push_back(nap);
                std::cout<<"Robot 1 IS in use"<<std::endl;
                std::cout<<"using Robot 2..."<<std::endl;
                std::cout<<"Recognized Object Position:"<<std::endl;
                std::cout<<nap<<std::endl;
            }
        }
        else //the first object is always manipulated by arm1
        {
            std::cout<<"Robot 1 IS NOT in use"<<std::endl;
            std::cout<<"using Robot 1..."<<std::endl;
            std::cout<<"Recognized Object Position:"<<std::endl;
            go1.go_vector.push_back(nap);
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
    ros::Subscriber state;
    arm_test::goResponse go1;
    arm_test::goResponse go2;
    ros::ServiceClient client_;
    ros::ServiceServer server;
    gazebo_msgs::GetModelState srv;
    arm_test::graspposition nap;
    std::vector<std::string> graspObject;
    std::vector<std::string> old_msg;
    bool var;
    bool boolean;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "grasp_decider");

    subscribeANDpublish SAP;

    ros::spin();

    return 0;
}