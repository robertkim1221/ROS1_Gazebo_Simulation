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
#include <std_msgs/Bool.h>


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
        c = 0;
        d = 0;
        e = 0;
        f = 0;
        transition = false;
        oldheader.stamp = ros::Time::now();
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
        newheader.stamp = ros::Time::now();
        //getting list of grasp objects allocated in vector graspObject
        do
        {
            if (msg->name.size()!=old_msg.size())
            {
                graspObject.clear();
                condition4pos = false;
                condition4time = false;
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

        //creating allobj matrix that has information of all graspobjects
        allobj.go_vector.push_back(nap);

        //comparing position and time stamp of each vectors
        if (allobj.go_vector.size() != 1)
        {
            double position1 = allobj.go_vector[allobj.go_vector.size()-1].position.x;
            double position2 = allobj.go_vector[allobj.go_vector.size()-2].position.x;
            //condition #1 (position) counter
            if ((position1 >= 0 && position2 >= 0) || (position1 < 0 && position2 < 0))
            {
                c++;
                if (c>=3)
                {
                    e=0;
                }
            }
            //condition #4 (NOT position) counter
            else
            {
                condition4pos = true;
                if (c<3)
                {
                    c=0;
                }
            }
            //condition #2 (time>) counter
            if (newheader.stamp - oldheader.stamp > ros::Duration(30))
            {
                d++;
                if (d>=3)
                {
                    f=0;
                }
            }
            //condition #4 (NOT time>) counter
            else
            {
                condition4time = true;
                if (d<3)
                {
                    d=0;
                }
            }
            //condition #4(NOT position AND NOT time>)
            if ((condition4pos == true))
            {
                e++;
                if (e>=3)
                {
                    c=0;
                }
            }
            if ((condition4time == true))
            {
                f++;
                if (f>=3)
                {
                    d=0;
                }
            }
        }
        std::cout<<"grasp decider 1 counter: "<<c<<std::endl;
        std::cout<<"grasp decider 2 counter: "<<d<<std::endl;
        if (d<3 &&c<3)
        {
            std::cout<<"using grasp decider 3 [default]"<<std::endl;

            int numobjects = graspObject.size();

            if(transition == false)
            {
                if(numobjects % 2 != 0)//if size is odd number and transition is false then go1 vector is filled and go2 if even or true.
                {
                    std::cout<<"using Robot 1..."<<std::endl;
                    go1.go_vector.push_back(nap);
                    std::cout<<go1.go_vector[go1.go_vector.size()-1]<<std::endl;
                }
                else if(numobjects % 2 ==0)
                {
                    std::cout<<"using Robot 2..."<<std::endl;
                    go2.go_vector.push_back(nap);
                    std::cout<<go2.go_vector[go2.go_vector.size()-1]<<std::endl;
                }
            }
            else if(transition == true)
            {
                if(transition23 == false)
                {
                    std::cout<<"using Robot 1..."<<std::endl;
                    std::cout<<"Recognized Object Position:"<<std::endl;
                    go1.go_vector.push_back(nap);
                    std::cout<<nap<<std::endl;
                    transition23 = true;
                }
                else if(transition23 == true)
                {
                    std::cout<<"using Robot 2..."<<std::endl;
                    std::cout<<"Recognized Object Position:"<<std::endl;
                    go2.go_vector.push_back(nap);
                    std::cout<<nap<<std::endl;
                    transition23 = false;
                }
            }

            //above if statements are ONLY for when grasp decider 3 was used since the start
            //after grasp decider 2 or 1 was used, need new condition for designating object to either go1 or go2
                    //which is transition bool; it becomes true if either grasp decider 1 or 2 was used
        }
        else if (c >= 3 && d < 3)
        {
            std::cout<<"using grasp decider 1 (position based)"<<std::endl;
            transition = true;

            if (nap.position.x >= 0)
            {
                std::cout<<"Object placed at positive X"<<std::endl;
                std::cout<<"using Robot 1..."<<std::endl;
                std::cout<<"Recognized Object Position:"<<std::endl;
                go1.go_vector.push_back(nap);
                std::cout<<nap<<std::endl;
                transition23 = true;
            }
            else if (nap.position.x < 0)
            {
                std::cout<<"Object placed at negative X"<<std::endl;
                std::cout<<"using Robot 2..."<<std::endl;
                std::cout<<"Recognized Object Position:"<<std::endl;
                go2.go_vector.push_back(nap);
                std::cout<<nap<<std::endl;
                transition23 = false;
            }

        }
        else if (d >= 3)
        {
            transition = true;
            //use grasp_decider_2
            std::cout<<"using grasp decider 2 (Robot 1 motion based)"<<std::endl;
            if (var == true)
            {
                std::cout<<"Robot 1 IS NOT in use"<<std::endl;
                std::cout<<"using Robot 1..."<<std::endl;
                std::cout<<"Recognized Object Position:"<<std::endl;
                std::cout<<nap<<std::endl;
                go1.go_vector.push_back(nap);
                transition23 = true;
            }
            else
            {
                std::cout<<"Robot 1 IS in use"<<std::endl;
                std::cout<<"using Robot 2..."<<std::endl;
                std::cout<<"Recognized Object Position:"<<std::endl;
                std::cout<<nap<<std::endl;
                go2.go_vector.push_back(nap);
                transition23 = false;
            }
        }
      oldheader.stamp = newheader.stamp;
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
    arm_test::goResponse allobj;
    ros::ServiceClient client_;
    ros::ServiceServer server;
    gazebo_msgs::GetModelState srv;
    arm_test::graspposition nap;
    arm_test::graspposition refobj;
    std::vector<std::string> graspObject;
    std::vector<std::string> old_msg;
    std_msgs::Header newheader;
    std_msgs::Header oldheader;
    bool var;
    bool boolean;
    bool transition;
    bool transition23;
    int c;
    int d;
    int e;
    int f;
    bool condition4time;
    bool condition4pos;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "grasp_decider");

    subscribeANDpublish SAP;

    ros::spin();

    return 0;
}