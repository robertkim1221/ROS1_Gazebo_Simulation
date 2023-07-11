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
#include <geometry_msgs/Pose.h>
#include <string>


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

        if(graspObject.size() % 2 ==0)
        {
            go2.go_vector.push_back(nap);
            std::cout<<"Object ID is EVEN numbered"<<std::endl;
            std::cout<<"using Robot 2..."<<std::endl;
            std::cout<<"Recognized Object Position:"<<std::endl;
            std::cout<<go2.go_vector[go2.go_vector.size()-1]<<std::endl;
        }
        else
        {
            go1.go_vector.push_back(nap);
            std::cout<<"Object ID is ODD numbered"<<std::endl;
            std::cout<<"using Robot 1..."<<std::endl;
            std::cout<<"Recognized Object Position:"<<std::endl;
            std::cout<<go1.go_vector[go1.go_vector.size()-1]<<std::endl;
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

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "grasp_decider");

    subscribeANDpublish SAP;

    ros::spin();

    return 0;
}