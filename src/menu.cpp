#include "ros/ros.h"
#include "std_srvs/Empty.h"


std_srvs::Empty reset;

std::string menu = R"(
        
Select one modality and then press enter:
1 - Reach autonomousely a given position
2 - Drive the robot with the keyboard
3 - Drive the robot with the keyboard with collision avoidance
4 - Reset the simulation
0 - Quit the execution of the program
)";



int Show_Menu()
{
    // The system() function is a part of the C/C++ standard library. It is used to pass the commands that can be executed
    // in the command processor or the terminal of the operating system, and finally returns the command after it has been completed.
    
    // to clear the console
    system("clear");

    std::cout << "WELCOME!\n";
    std::cout << menu;

    int mod;

    mod = getchar();

    return mod;
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "menu");

    while (true)
    {

        switch (Show_Menu())
        {
        case '1':
            //launch reachPoint node
            system("rosrun RTassignment3 ReachPosition");
        break;
        
        case '2':
            //launch driveWithKeyboard node
            system("rosrun RTassignment3 driveWithKeyboard");
        break;
        
        case '3':
            //launch driveWithKeyboardAssisted node
            system("rosrun RTassignment3 driveWithKeyboardAssisted");
        break;
        
        case '4':
            //call gazebo/reset_simulation service and reset the simulation
            ros::service::call("/gazebo/reset_simulation", reset);
        break;
        
        case '0':
            //clear the screen and exit from main
            system("clear");
            return 0;
        break;
        
        default:
        
        break;
        }
    }

    return 0;
}
