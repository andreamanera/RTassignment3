#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include <termios.h>


ros::Publisher publisher;

//defining variables for veloocity
double velocity = 0.5;
double turn_velocity = 1;

// defining variables used to define the direction
int lin=0; //linear direction
int ang =0; //angular direction

// defining variable for Twist
geometry_msgs::Twist vel;

// function to have non-blocking keyboard input
// (avoid pressing enter)
int getch(void)
{
    int ch;
    struct termios oldt;
    struct termios newt;

    // Store old settings, and copy to new settings
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;

    // Make required changes and apply the settings
    newt.c_lflag &= ~(ICANON | ECHO);
    newt.c_iflag |= IGNBRK;
    newt.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF);
    newt.c_lflag &= ~(ICANON | ECHO | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN);
    newt.c_cc[VMIN] = 1;
    newt.c_cc[VTIME] = 0;
    tcsetattr(fileno(stdin), TCSANOW, &newt);

    // Get the current character
    ch = getchar();

    // Reapply old settings
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

    return ch;
}

void choose_input(char input)
{
    //set linear and angular direction according to user input
    switch (input)
    {

    case 'a':
        velocity *= 1.1;
        turn_velocity *= 1.1;
        break;
    case 'd':
        velocity *= 0.9;
        turn_velocity *= 0.9;
        break;
    case 'w':
        velocity *= 1.1;
        break;
    case 's':
        velocity *= 0.9;
        break;
    case 'q':
        turn_velocity *= 1.1;
        break;
    case 'c':
        turn_velocity *= 0.9;
        break;
    case 'r':
        velocity = 0.5;
        turn_velocity = 1;
        break;
    case 'y':
        lin = 1;
        ang = 1;
        break;
    case 'u':
        lin = 1;
        ang = 0;
        break;
    case 'i':
        lin = 1;
        ang = -1;
        break;
    case 'h':
        lin = 0;
        ang = 1;
        break;
    case 'j':
        lin = 0;
        ang = 0;
        break;
    case 'k':
        lin = 0;
        ang = -1;
        break;
    case 'b':
        lin = -1;
        ang = 1;
        break;
    case 'n':
        lin = -1;
        ang = 0;
        break;
    case 'm':
        lin = -1;
        ang = -1;
        break;
    case '\x03': //CTRL-C

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
        lin = 0;
        ang = 0;
        break;
    }
}

void get_input()
{

    //display instructions
    std::cout << "Drive the robot in the envitonment using your keyboard\n\n";

    std::cout << R"(Use the commands below as a joystick
    
                y    u    i
                h    j    k
                b    n    m
                
Press:
a : increase angular and linear velocities by 10%
a : decrease angular and linear velocities by 10%
r : reset angular and linear velocities at the default values
w : increase linear velocity by 10%
s : decrease linear velocity by 10%
q : increase angular velocity by 10%
c : decrease angular velocity by 10%
press CTRL-C to quit

ALL OTHER KEYS WILL STOP THE ROBOT
)";
    std::cout << "\ncurrent velocities:\tspeed " << velocity << "\tturn " << turn_velocity << "\n";

    // wait for user input
    char input;
    input = getch();

    // call the function choose_input to decide what to do with the input value received
    choose_input(input);

    // compute new velocities
    vel.linear.x = velocity * lin;
    vel.angular.z = turn_velocity * ang;
    
    // publish the new velocity in the topic vel
    publisher.publish(vel);

    system("clear");
}

int main(int argc, char **argv)
{
    system("clear");
    ros::init(argc, argv, "KeyboardDrive");
    ros::NodeHandle node_handle;

    //this node will publish updated into /cmd_vel topic
    publisher = node_handle.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    // while loop used to get input endless
    while (true)
        get_input();

    return 0;
}
