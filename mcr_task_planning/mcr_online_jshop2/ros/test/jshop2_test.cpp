#include <gtest/gtest.h>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <mcr_task_planning_msgs/Plan.h>
#include <mcr_task_planning_msgs/State.h>
#include <mcr_task_planning_msgs/Task.h>


std::string g_event_in;
bool g_has_event_in = false;
mcr_task_planning_msgs::Plan g_plan;
bool g_has_plan = false;


void event_in_cb(const std_msgs::StringConstPtr &msg)
{
    g_event_in = msg->data;
    g_has_event_in = true;
}

void plan_cb(const mcr_task_planning_msgs::PlanConstPtr &plan)
{
    g_plan = *plan;
    g_has_plan = true;
}

TEST(jshop2_integration_test, test_start_event_before_data)
{
    ros::NodeHandle nh;
    ros::Publisher pub_state = nh.advertise<mcr_task_planning_msgs::State>("/mcr_task_planning/jshop2/state", 1);
    ros::Publisher pub_task = nh.advertise<mcr_task_planning_msgs::Task>("/mcr_task_planning/jshop2/task", 1);
    ros::Publisher pub_event = nh.advertise<std_msgs::String>("/mcr_task_planning/jshop2/event_in", 1);
    ros::Subscriber sub_event = nh.subscribe<std_msgs::String>("/mcr_task_planning/jshop2/event_out", 1, event_in_cb);
    ros::Subscriber sub_plan = nh.subscribe<mcr_task_planning_msgs::Plan>("/mcr_task_planning/jshop2/plan", 1, plan_cb);

    // reset global variables
    g_has_event_in = false;
    g_has_plan = false;

    // setup the start event
    std_msgs::String event;
    event.data = "e_start";

    // send out the data and wait for the plan
    while (!g_has_event_in) {
        pub_event.publish(event);

        ros::Rate(0.1).sleep();
        ros::spinOnce();
    }

    // evaluate the response
    ASSERT_EQ(g_event_in, "e_failed");
}

TEST(jshop2_integration_test, test_successful_planning)
{
    ros::NodeHandle nh;
    ros::Publisher pub_state = nh.advertise<mcr_task_planning_msgs::State>("/mcr_task_planning/jshop2/state", 1);
    ros::Publisher pub_task = nh.advertise<mcr_task_planning_msgs::Task>("/mcr_task_planning/jshop2/task", 1);
    ros::Publisher pub_event = nh.advertise<std_msgs::String>("/mcr_task_planning/jshop2/event_in", 1);
    ros::Subscriber sub_event = nh.subscribe<std_msgs::String>("/mcr_task_planning/jshop2/event_out", 1, event_in_cb);
    ros::Subscriber sub_plan = nh.subscribe<mcr_task_planning_msgs::Plan>("/mcr_task_planning/jshop2/plan", 1, plan_cb);

    // reset global variables
    g_has_event_in = false;
    g_has_plan = false;

    // setup the initial state
    mcr_task_planning_msgs::Atom atom;
    atom.name = "have";
    atom.parameters.push_back("kiwi");
    mcr_task_planning_msgs::State state;
    state.atoms.push_back(atom);

    // setup the task
    mcr_task_planning_msgs::Task task;
    task.name = "swap";
    task.parameters.push_back("banjo");
    task.parameters.push_back("kiwi");

    // setup the start event
    std_msgs::String event;
    event.data = "e_start";

    // send out the data and wait for the plan
    while (!g_has_event_in) {
        pub_state.publish(state);
        pub_task.publish(task);
        pub_event.publish(event);

        ros::Rate(0.1).sleep();
        ros::spinOnce();
    }

    // evaluate the response
    ASSERT_EQ(g_event_in, "e_done");

    ASSERT_TRUE(g_has_plan);
    ASSERT_EQ(g_plan.actions.size(), 2);

    ASSERT_EQ(g_plan.actions[0].name, "!drop");
    ASSERT_EQ(g_plan.actions[0].parameters.size(), 1);
    ASSERT_EQ(g_plan.actions[0].parameters[0], "kiwi");

    ASSERT_EQ(g_plan.actions[1].name, "!pickup");
    ASSERT_EQ(g_plan.actions[1].parameters.size(), 1);
    ASSERT_EQ(g_plan.actions[1].parameters[0], "banjo");
}


int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "jshop2_test");
    return RUN_ALL_TESTS();
}
