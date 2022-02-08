#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include <termios.h>


ros::Publisher publisher;

//defining variables for veloocity
double velocity = 0.2;
double turn_velocity = 0.5;

// defining variables used to define the direction
int linear = 0; //linear direction
int angular =0; //angular direction

// defining threshold for wall distance
double th = 1;

// defining variable for Twist
geometry_msgs::Twist vel;

// function to have non-blocking keyboard input
// (avoid pressing enter)
int getch(void)
{
    int ch;
    struct termios oldt;
    struct termios newt;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;

    newt.c_lflag &= ~(ICANON | ECHO);
    newt.c_iflag |= IGNBRK;
    newt.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF);
    newt.c_lflag &= ~(ICANON | ECHO | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN);
    newt.c_cc[VMIN] = 1;
    newt.c_cc[VTIME] = 0;
    tcsetattr(fileno(stdin), TCSANOW, &newt);

    ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

    return ch;
}

// function used to find the closest obstacle in each section of the ranges array
float Distance(float obs_dist[], int val_min, int val_max)
{

	//as the minimum distance is set a large value that will then be compared
	//with all the values between val_min and val_max in the ranges array
	
	float dist_min = 30.0;
	
	//thanks to the for loop and the if statement all values in the ranges
	//array are compared so as to find the smallest
	
	for(int i = val_min; i <= val_max; i++){
	
		if(obs_dist[i] <= dist_min){
		
			dist_min = obs_dist[i];
		}
	}
	
	return dist_min;
}

void checkWalls(const sensor_msgs::LaserScan::ConstPtr &msg)
{
	//variables for the closest obstacle on robot right, left and front
	float dist_min_right, dist_min_left, dist_min_front;
	
	// array wich will be filled with the elements of ranges
	float scanner[721];
	
	// thanks to this loop scanner array is filled with the element of ranges
	for(int i = 0; i < 722; i++){
		scanner[i] = msg->ranges[i];
	}
	
	// Call at the Distance function to calculate the minimum
	// distance from the obstacles on robot front, right and left
	dist_min_right = Distance(scanner, 0, 159);
	dist_min_left = Distance(scanner, 561, 720);
	dist_min_front = Distance(scanner, 281, 440);
	
	 //warn user
	if ((dist_min_front < th && linear > 0) || ((dist_min_left < th) && angular > 0) || ((dist_min_right < th) && angular < 0))
		std::cout << "Wall detected!\n";
		         

	//if the nearest wall in front of the robot is too close, the robot can only turn
	if (dist_min_front < th && linear > 0){
	    
		linear = 0;
	}
	
	//if the nearest wall on the left or on the right is too close, the robot can only go straight
	if (((dist_min_left < th) && angular > 0) || ((dist_min_right < th) && angular < 0)){
	
		angular = 0;
	}
	
	// compute new velocities
   	vel.linear.x = velocity * linear;
    	vel.angular.z = turn_velocity * angular;
    
    	// publish the new velocity
    	publisher.publish(vel);
}

// function used to associate the input from keyboard with the correct command

void choose_input(char input)
{

    switch (input)
    {
	    case 'A':
	    case 'a':
		velocity *= 1.1;
		turn_velocity *= 1.1;
            break;
            
            case 'D':
	    case 'd':
		velocity *= 0.9;
		turn_velocity *= 0.9;
            break;
            
            case 'W':
	    case 'w':
		velocity *= 1.1;
	    break;
	    
	    case 'S':
	    case 's':
		velocity *= 0.9;
	    break;
	    
	    case 'Q':
	    case 'q':
		turn_velocity *= 1.1;
	    break;
	    
	    case 'C':
	    case 'c':
		turn_velocity *= 0.9;
	    break;
	    
	    case 'R':
	    case 'r':
		velocity = 0.5;
		turn_velocity = 1;
	    break;
	    
	    case 'Y':
	    case 'y':
		linear = 1;
		angular = 1;
	    break;
	    
	    case 'U':
	    case 'u':
		linear = 1;
		angular = 0;
	    break;
	    
	    case 'I':
	    case 'i':
		linear = 1;
		angular = -1;
	    break;
	    
	    case 'H':
	    case 'h':
		linear = 0;
		angular = 1;
	    break;
	    
	    case 'J':
	    case 'j':
		linear = 0;
		angular = 0;
	    break;
	    
	    case 'K':
	    case 'k':
		linear = 0;
		angular = -1;
	    break;
	    
	    case 'B':
	    case 'b':
		linear = -1;
		angular = 1;
	    break;
	    
	    case 'N':
	    case 'n':
		linear = -1;
		angular = 0;
	    break;
	    
	    case 'M':
	    case 'm':
		linear = -1;
		angular = -1;
	    break;
	    
	    case 'E':
	    case 'e': 

		//stop robot and exit
		vel.angular.z = 0;
		vel.linear.x = 0;
		publisher.publish(vel);
		system("clear");
		ros::shutdown();
		exit(0);
            break;
            
	    default:
		//stop robot
		linear = 0;
		angular = 0;
	    break;
	}
}

// function used to display the menu and compute the velocity with the input taken by the keyboard

void get_input()
{

    //display instructions
    std::cout << "Drive the robot in the envitonment using your keyboard\n";

    std::cout << R"(Move the robot using these commands as you are using a joystick
    
                y    u    i
                h    j    k
                b    n    m
                
Press:
a : increase angular and linear velocities by 10%
d : decrease angular and linear velocities by 10%
r : reset angular and linear velocities at the default values
w : increase linear velocity by 10%
s : decrease linear velocity by 10%
q : increase angular velocity by 10%
c : decrease angular velocity by 10%
press e to quit

ALL OTHER KEYS WILL STOP THE ROBOT
	)";
    // wait for user input
    char input;
    input = getch();

    // call the function choose_input to decide what to do with the input value received
    choose_input(input);

    // compute new velocities
    vel.linear.x = velocity * linear;
    vel.angular.z = turn_velocity * angular;
    
    // publish the new velocity
    publisher.publish(vel);

    system("clear");
}

int main(int argc, char **argv)
{
    system("clear");
    ros::init(argc, argv, "KeyboardDrive");
    ros::NodeHandle node_handle;
    
    //subribes to /scan topic
    ros::Subscriber subscriber = node_handle.subscribe("/scan", 500, checkWalls);

    //this node will publish updated into /cmd_vel topic
    publisher = node_handle.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    // while loop used to get input endless
    ros::AsyncSpinner spinner(4);
    spinner.start();
    while (true)
        get_input();
	spinner.stop();
    return 0;
}
