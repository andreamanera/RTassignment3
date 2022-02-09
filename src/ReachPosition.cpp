#include "ros/ros.h"
#include "move_base_msgs/MoveBaseActionGoal.h"
#include "actionlib_msgs/GoalStatusArray.h"
#include "actionlib_msgs/GoalID.h"
#include <time.h>

// defining two publisher and one subscriber
ros::Publisher pub_goal;
ros::Publisher pub_cancel;

ros::Subscriber subscriber;

// variables needed for goal coordinates
float x, y;

// variable needed for goal id
int id;

// variable for goal
// the package move_base_msgs contains the messages used to communicate with the move_base node. 
// The move_base package provides an implementation of an action that, given a goal in the world, will attempt to reach it
move_base_msgs::MoveBaseActionGoal goal;

// defining functions prototypes
void get_coordinates();
void Handler(const actionlib_msgs::GoalStatusArray::ConstPtr &msg);
void detect_input();
void delete_goal(int must_continue);


// flag used to detect the user input during goal navigation
int flag_for_goal;


void detect_input()
{
    //get an user input
    char input_ch = getchar();

    //if there is a goal in progress then user is allowed to cancel goal
    if ((input_ch == 'q' || input_ch == 'Q') && flag_for_goal)
    {
    	// call the function delete_goal with argument 1 to cancel the goal
        delete_goal(1);
    }
    
    else if ((input_ch == 's' ||input_ch == 'S') && flag_for_goal) 
    {
    	// call the function delete_goal with argument 0 to quit the execution
        delete_goal(0);
        system("clear");
        ros::shutdown();
        exit(0);
    }
    
    //if there aren't goal in progress then user is allowed to choose wheter to continue or not
    else if ((input_ch == 'y' || input_ch == 'Y' || input_ch == 'n' || input_ch == 'N') && !flag_for_goal)
    {
    	// The user wants to set a goal
        if (input_ch == 'y' || input_ch == 'Y')
            // call the function get_coordinates() to set the goal coordinates
            get_coordinates();
        // the user doesn't want to set a goal 
        else
            exit(1);
    }
}

void delete_goal(int must_continue)
{
    //build cancel message with goal id
    actionlib_msgs::GoalID msg_cancel;
    msg_cancel.id = std::to_string(id);
    pub_cancel.publish(msg_cancel);

    subscriber.shutdown();

    //set flag as goal in no more in progress
    flag_for_goal = 0;

    //if the program should stop then return
    if (!must_continue)
        return;

    //else clear and ask user what he wants to do
    system("clear");

    std::cout <<  "Goal is cancelled. \n\n";

    std::cout << "Do you want to set another goal? (y/n)\n";

    //detect user input
    while (true){
        detect_input();
    }
}

void get_coordinates()
{
    //wait until two floats are inserted
    
    system("clear");
    std::cout << "Automatic navigation \n\n";
    std::cout << "Insert x coordinate: \n\n";
    std::cin >> x;
    
    // cin.good() returns 0 if the stream has encountered problems.
    while (!std::cin.good())
    {
        // cin.clear() to clear the error flag on cin
        std::cin.clear();
        // cin.ignore() to ignore the previously given input
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        system("clear");
        std::cout << "Automatic navigation \n\n";
        std::cout << "Insert x coordinate: \n\n";
        std::cin >> x;
    }
    
    system("clear");
    std::cout << "Automatic navigation \n\n";
    std::cout << "Insert y coordinate: \n\n";;
    std::cin >> y;
    
    while (!std::cin.good())
    {
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        system("clear");
        std::cout << "Automatic navigation \n\n";
        std::cout << "Insert y coordinate: \n\n";
        std::cin >> y;
    }

    // build goal message using the goal coordinates given by keyboard input
    // using a random generated id
    
    //generation of random id
    id = rand();
    goal.goal.target_pose.pose.position.x = x;
    goal.goal.target_pose.pose.position.y = y;
    goal.goal.target_pose.pose.orientation.w = 1;
    goal.goal.target_pose.header.frame_id = "map";
    
    // to_string() convert the id into string type
    goal.goal_id.id = std::to_string(id);

    //publish goal message
    pub_goal.publish(goal);

    //set flag for a new goal in progress
    flag_for_goal = 1;

    //update the interface
    system("clear");

    std::cout << "\nThe goal has been set to\tx: " << x << "\ty: " << y << "\n\n";

    std::cout << "Press q and then press enter to cancel the goal, press s to quit the exexution\n\n";

    //subscribe to goal status
    ros::NodeHandle node_handle;
    // "/move_base/status" provides status information on the goal that are sent to the move_base action.
    subscriber = node_handle.subscribe("/move_base/status", 500, Handler);

    //start detecting inputs
    ros::AsyncSpinner spinner(4);

    spinner.start();
    while (true)
        detect_input();

    spinner.stop();
}

void Handler(const actionlib_msgs::GoalStatusArray::ConstPtr &msg)
{
    //status of goal
    int status = 0;

    //to read only goals with correct id
    if (msg->status_list[0].goal_id.id == std::to_string(id))
        status = msg->status_list[0].status;

    if (status != 3 && status != 4)
    {
        return;
    }

    subscriber.shutdown();

    flag_for_goal= 0;

    system("clear");

    std::cout << "Robot automatic navigation \n\n";

    if (status == 3) //status 3 corresponds to SUCCEDED

        std::cout << "Goal reached!\n";

    else // else the goal cannot be reached
        std::cout << "The goal can not be reached.\n";

    std::cout << "\nDo you want to set another goal? (y/n)\n";
}

int main(int argc, char **argv)
{
    srand(time(NULL));

    ros::init(argc, argv, "ReachPosition");
    ros::NodeHandle node_handle;

    // the goal position is published in the "/move_base/goal" topic
    pub_goal = node_handle.advertise<move_base_msgs::MoveBaseActionGoal>("/move_base/goal", 1);
    // the message to cancel the goal is published on the "/move_base/cancel" topic
    pub_cancel = node_handle.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 1);

    //get goal coordinates
    get_coordinates();

    ros::spin();

    return 0;
}
