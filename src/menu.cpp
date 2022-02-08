#include "ros/ros.h"
#include "std_srvs/Empty.h"


std_srvs::Empty reset;

std::string menu = R"(
        
Select one modality and then press enter:
1 - Reach autonomousely a given position
2 - Drive the robot with your keyboard
3 - Drive the robot with your keyboard with assisted obstacles avoidance
4 - Reset the simulation
0 - Quit the execution
)";

// function used to display the menu, take the input from keyboard

int Show_Menu()
{
    // The system() function is a part of the C/C++ standard library. It is used to pass the commands that can be executed
    // in the terminal of the operating system, returns the command after it has been completed.
    
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
	// here the input taken is associated with the correct modality
        switch (Show_Menu())
        {
		case '1':
		    //launch ReachPosition node
		    system("rosrun RTassignment3 ReachPosition");
		break;
		
		case '2':
		    //launch KeyboardDrive node
		    system("rosrun RTassignment3 KeyboardDrive");
		break;
		
		case '3':
		    //launch driveWithKeyboardAssisted node
		    system("rosrun RTassignment3 KeyboardDriveObs");
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
