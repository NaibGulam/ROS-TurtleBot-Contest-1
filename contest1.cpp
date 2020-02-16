#include <ros/console.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <stdio.h>
#include <cmath>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <fstream>
#include <string.h>
#include <cstdlib>
 
using namespace cv;
using namespace std;
 
//Define Macros
#define RAD2DEG(rad) ((rad) * 180. / M_PI)
#define DEG2RAD(deg) ((deg) * M_PI / 180.)
#define DIST(x_1,y_1,x_2,y_2) (sqrt((x_1 - x_2) * (x_1 - x_2) + (y_1 - y_2) * (y_1 - y_2)))
 
//Set the Constant for The Number of Bumper Switches
#define N_BUMPER (3)
#define Grid_Size 20
 
//Define Global Variables
float angular = 0.0, linear = 0.0;
float min_dist = 0.7;
float default_lin = 0.2, default_ang = M_PI / 3, scan_ang = M_PI / 6;
float posX = 0.0, posY = 0.0, yaw = 0.0;
int grid_x = 0, grid_y = 0, grid_dir = 0;
uint8_t bumper[3] = {kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED};
float minLaserDist = numeric_limits<float>::infinity();
float LeftLaserDist = numeric_limits<float>::infinity();
float RightLaserDist = numeric_limits<float>::infinity();
float AllminLaserDist = numeric_limits<float>::infinity();
int32_t nLasers = 0, desiredNLasers = 0, desiredAngle = 12.5, desiredNLasersSides = 0, desiredAngleSides = 12.5;
float max_angle, min_angle;
bool any_bumper_pressed;
int8_t bumper_maneuver = 0;
int temp_g_dir, scanning_g_dir;
uint64_t temp_time;
uint64_t millisecondsElapsed = 0;
uint64_t lastScanTime = 0;
uint64_t lastNewGridTime = 0;
int prev_x = 0, prev_y = 0;
bool searching = false, all_known = false;
Mat grid(Grid_Size, Grid_Size, CV_8UC1, Scalar(50));
int rotation_direction = 1;
int default_rotation_direction = 1;
bool available_scanned_dirs[8] = {false};
bool scanning = false;
vector<vector<float>> scannedlocations;
 
 
//Definition of Supplementary functions
void updateGrid(); //Updates the grid values for exploration at each ros::spin based on odometry
void handleBumper();//handels bumper events and maneuvers
void handleExloration();//handels the decision making and route selection for exploring the area
bool checkAdj(int dir);//checks adjacent grid in specific direction(int dir (0-7)) to see if the neighboring grid in this direction has been visited
void log();//logs the needed variables in a JSON file to be read by the robot monitor UI
int get_grid_direction(float angle);//Calculates the current facing adjacent grid based on an input angle(0-2PI)
void assign_blocked(int dir);//Assigns the adjacent grid in specific direction(int die (0-7)) a value of 0 meaning it is blocked
void handle_wall_avoidance();//Handles avoiding side obstacles
 
//Callback Functions
void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg)
{//Bumper Operations are based on the settings applied here
    //save bumper states
    bumper[msg->bumper] = msg->state;
    
    //activate bumper flag if any bumper is pressed
    any_bumper_pressed = false;
    for (uint32_t b_idx = 0; b_idx < N_BUMPER; ++b_idx)
        any_bumper_pressed |= (bumper[b_idx] == kobuki_msgs::BumperEvent::PRESSED);
}
 
void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{//Most important sensory data is established here
    //set minimum variables to inf for min calculations
    minLaserDist = numeric_limits<float>::infinity();
    AllminLaserDist = numeric_limits<float>::infinity();
    LeftLaserDist = numeric_limits<float>::infinity();
    RightLaserDist = numeric_limits<float>::infinity();
    
    //get the number of laser sensors and the min and max angles data is availble
    nLasers = (msg->angle_max - msg->angle_min) / msg->angle_increment;
    max_angle = msg->angle_max;
    min_angle = msg->angle_min;
    
    //determine the number of laser sensors that must be read for the desired field of view
    desiredNLasers = desiredAngle * M_PI / (180 * msg->angle_increment);
    desiredNLasersSides = desiredAngleSides * M_PI / (180 * msg->angle_increment);
    
    //See if the desired field of view is possible or not and find the minimum distance in the field of view
    if (desiredAngle * M_PI / 180 < msg->angle_max && -desiredAngle * M_PI / 180 > msg->angle_min) {
        for (uint32_t laser_idx = nLasers / 2 - desiredNLasers; laser_idx < nLasers / 2 + desiredNLasers; ++laser_idx){
            minLaserDist = min(minLaserDist, msg->ranges[laser_idx]);
        }
    }
    else {
        for (uint32_t laser_idx = 0; laser_idx < nLasers; ++laser_idx) {
            minLaserDist = min(minLaserDist, msg->ranges[laser_idx]);
        }
    }
    // Determine the minimum distance in all available directions
    for (uint32_t laser_idx = 0; laser_idx < nLasers; ++laser_idx){
            AllminLaserDist = min(AllminLaserDist, msg->ranges[laser_idx]);
    }
    
    if(desiredNLasersSides < nLasers){//see if desired field of view is possible
        //determine the minimum distance on the sides
        for (uint32_t laser_idx = 0; laser_idx < desiredNLasersSides; ++laser_idx){
            RightLaserDist = min(RightLaserDist, msg->ranges[laser_idx]);
        }
        for (uint32_t laser_idx = nLasers - desiredNLasersSides - 1; laser_idx < nLasers; ++laser_idx){
                LeftLaserDist = min(LeftLaserDist, msg->ranges[laser_idx]);
        }
    }
    else{//if not all lasers must be examined so it would be the same as AllminLaserDist
        LeftLaserDist = AllminLaserDist;
        RightLaserDist = AllminLaserDist;
    }
    
    //check to ensure laser distance readings are done appropriately (sometimes laser distances are not available(very close distance or very long ones))
    if(!isfinite(minLaserDist) || minLaserDist < 0)
        minLaserDist = 0;
    
    if(!isfinite(RightLaserDist) || RightLaserDist < 0)
        RightLaserDist = 0;
    
    if(!isfinite(LeftLaserDist) || LeftLaserDist < 0)
        LeftLaserDist = 0;
    
    if(!isfinite(AllminLaserDist) || AllminLaserDist < 0)
        AllminLaserDist = 0;
    
    //Global velocity determination based on distance measurements
    if(AllminLaserDist <= 0.8 && AllminLaserDist > 0.55){
        default_lin = 0.15;
    }
    else if(AllminLaserDist <= 0.55){
        default_lin = 0.1;
    }
    else{
        default_lin = 0.2;
    }
    if(bumper_maneuver != 0)//Set to 0.15 m/s for accurate bumper maneuver
        default_lin = 0.15;
}
 
void odomCallback (const nav_msgs::Odometry::ConstPtr& msg)
{//odometry data is analyzed and on this basis the preception is updated
    //determine the position and based on that determine the grid(setup as 10mx10m grid with a specified size)
    posX = msg->pose.pose.position.x;
    posY = msg->pose.pose.position.y;
    grid_x = (Grid_Size - 1) / 2 + round(posX / (10.0 / Grid_Size));
    grid_y = (Grid_Size - 1) / 2 + round(posY / (10.0 / Grid_Size));
    
    //determine yaw
    yaw = tf::getYaw(msg->pose.pose.orientation);
    if(yaw < 0)// since default yaw is in (-PI,PI) change the value to get a yaw in a (0-2PI) range
        yaw += 2 * M_PI;
    
    //determine the adjacent grid the robot is facing based on yaw
    grid_dir = get_grid_direction(yaw);
    
    //update the preception based on the newly attained data
    updateGrid();
    
    //check to see if all nearest neighbours are known based on the grid data and activate the appropriate flag if so
    all_known = false;
    for(int i=0;i<8;i++){
        all_known |= checkAdj(i);
    }
    all_known = !all_known;
    
    //once grid has been updated record update current readings as the previous grid values so that in the next run it can checked to see if a new grid is visited
    prev_y = grid_y;
    prev_x = grid_x;
}
 
//Supplementary Functions
void updateGrid(){//Updates the preception(grid system)
    if(grid.at<uchar>(grid_x,grid_y) == 50)//check to see if the current grid is unvisited if so make it a visited grid(50 is default value of not visited The value was chosen for the picture to look readable)
        grid.at<uchar>(grid_x,grid_y) = 128;//default value of visited once is 128(for readability of the image)
    else if(grid.at<uchar>(grid_x,grid_y) >= 128 && (prev_x != grid_x || prev_y != grid_y) && grid.at<uchar>(grid_x,grid_y) <= 235){//check to see the current grid is newly visited and is not simply the same as before
        grid.at<uchar>(grid_x,grid_y) += 20;//add 20 to the pixel value of the grid(20 based on readability)
    }
    if (prev_x != grid_x || prev_y != grid_y)//update the time of visit to the new grid is a new grid is visited
        lastNewGridTime = millisecondsElapsed;
    
    imwrite("current.png", grid);//Save an image of the robots current preception for monitoring
    
    if(millisecondsElapsed - lastNewGridTime >= 20000){//check to see if the robot is stuck at specific grid location and assign blocks to all surrounding unknown grids(which is likely why the robot is stuck anyways)
        for(int i = 0;i<8;i++)
            assign_blocked(i);
    }
    
    if(grid.at<uchar>(grid_x,grid_y) >= 188){//If at any point a grid location is visited for the fourth time(implies the robot is stuck) the default rotation direction of the robot is flipped
        default_rotation_direction = -1;
    }
}
 
int get_grid_direction(float angle){
    //find the corresponding grid direction for the current yaw value
    if((angle >= -M_1_PI/8 && angle <= M_PI/8) || (angle >= 2 * M_PI - M_PI/8 && angle <= 2 * M_PI + M_PI/8))
        return 0;
    if((angle >= M_PI/8 && angle<= 3 * M_PI/8) || (angle >= 2 * M_PI + M_PI/8))
        return 1;
    if(angle >= 3 * M_PI/8 && angle<= 5 * M_PI/8)
        return 2;
    if(angle >= 5 * M_PI/8 && angle<= 7 * M_PI/8)
        return 3;
    if(angle >= 7 * M_PI/8 && angle<= 9 * M_PI/8)
        return 4;
    if(angle >= 9 * M_PI/8 && angle<= 11 * M_PI/8)
        return 5;
    if(angle >= 11 * M_PI/8 && angle<= 13 * M_PI/8)
        return 6;
    if((angle >= 13 * M_PI/8 && angle<= 15 * M_PI/8) || (angle <= -M_PI/8))
        return 7;
}
 
void handleBumper(){
    //First check if bumper is pressed or another maneuver in progress
    if(any_bumper_pressed && bumper_maneuver == 0){
        //Reset Everything and decide which bumper was pressed and what maneuver is appropriate
        searching =false;
        temp_time = millisecondsElapsed;
        if(bumper[kobuki_msgs::BumperEvent::CENTER] == kobuki_msgs::BumperEvent::PRESSED){
            angular = 0.0;
            linear = -default_lin;
            bumper_maneuver = 1;
        }
        if(bumper[kobuki_msgs::BumperEvent::RIGHT] == kobuki_msgs::BumperEvent::PRESSED){
            linear = -default_lin;
            angular = 0.0;
            bumper_maneuver = 3;
        }
        if(bumper[kobuki_msgs::BumperEvent::LEFT] == kobuki_msgs::BumperEvent::PRESSED){
            linear = -default_lin;
            angular = 0.0;
            bumper_maneuver = 2;
        }
    }
    
    //check to see if another maneuver is in progress and act accordingly to complete the maneuver(described in the report)
    else if(bumper_maneuver == 1 && millisecondsElapsed - temp_time >= 2000){
        bumper_maneuver = 0;
        linear = 0.0;
        angular = 0.0;
    }
    else if(bumper_maneuver == 2 && millisecondsElapsed - temp_time >= 1500){
        linear = linear / 2;
        angular = -default_ang;
        if(millisecondsElapsed - temp_time >= 2000){
            linear = default_lin;
            angular = 0.0;
            if(millisecondsElapsed - temp_time >= 3000){
                bumper_maneuver = 0;
                linear = 0.0;
                angular = 0.0;
            }
        }
    }
    else if(bumper_maneuver == 3 && millisecondsElapsed - temp_time >= 1500){
        linear = linear / 2;
        angular = default_ang;
        if(millisecondsElapsed - temp_time >= 2000){
            linear = default_lin;
            angular = 0.0;
            if(millisecondsElapsed - temp_time >= 3000){
                bumper_maneuver = 0;
                linear = 0.0;
                angular = 0.0;
            }
        }
    }
}
 
bool checkAdj(int dir){
    //first determine the increments of the corresponding grid direction
    int x_f = 0, y_f = 0;
    if(dir == 0 || dir == 1 || dir == 7)
        x_f = 1;
    else if(dir == 3 || dir == 4 || dir == 5)
        x_f = -1;
    else
        x_f = 0;
    
    if(dir == 1 || dir == 2 || dir == 3)
        y_f = 1;
    else if(dir == 5 || dir == 6 || dir == 7)
        y_f = -1;
    else
        y_f = 0;
    
    if(grid_x + x_f >= 0 && grid_x + x_f < Grid_Size && grid_y + y_f >= 0 && grid_y + y_f < Grid_Size){//check to see if desired grid is within the grid system or past it's boundaries
        if(grid.at<uchar>(grid_x + x_f, grid_y + y_f) == 50){//if the grid is unvisited return a true value
            return true;
        }
    }
    return false; // in any other case return a false value indicating either visited or unavailable
}
 
void handleExloration(){
    if(searching){//if searching check if any adjacent grids where blocked and if so change the values
        //similar to scanning logic for block detection refer to handleScan function
        if(grid_dir != temp_g_dir){
            int prev_grid_dir = grid_dir - rotation_direction;
            if(prev_grid_dir < 0)
                prev_grid_dir = 7;
            if(prev_grid_dir > 7)
                prev_grid_dir = 0;
            if(prev_grid_dir != temp_g_dir)
                assign_blocked(prev_grid_dir);
        }
    }
    if(bumper_maneuver == 0){// check to see if any bumper operation in progress
        if(minLaserDist >= min_dist && checkAdj(grid_dir)){// if not blocked and facing an unexplored grid continue motion or stop searching
            angular = 0.0;
            linear = default_lin;
            searching =false;
        }
        else if(minLaserDist >= min_dist && ((millisecondsElapsed - temp_time >= 6000 && searching) || all_known || grid.at<uchar>(grid_x,grid_y) >= 168)){
            //check to see if forward direction available and either full 360 search has been completed or all adjacent grids are know or the current grid has been visited 3 or more times
            //if so move forward and don't waste time searching more
            angular = 0.0;
            linear = default_lin;
            searching = false;
        }
        else{//if the forward direction is blocked
            if(!searching){//see if a search is in progress and only go into search if not already searching
                if(!all_known){//if all neighbouring grids are not known
                    searching = true;//start search
                    temp_time = millisecondsElapsed;//reset 360 timers
                    temp_g_dir = grid_dir;//save the starting grid(for block checking)
    
                    //see if only one unexplored neighbour exists
                    int nAvailable = 0, available_dir;
                    for(int i=0;i<8;i++){
                        if(checkAdj(i)){
                            nAvailable ++;
                            available_dir = i;
                        }                   
                    }
    
                    if(nAvailable == 1){//if only one unexplored neighbour exists then find the fastest direction of rotation
                        //calculate which rotation direction would be faster
                        int forward,backward;
                        forward = available_dir - grid_dir;
                        backward = grid_dir - available_dir;
                        if(forward < 0)
                            forward += 8;
                        if(backward < 0)
                            backward += 8;
                        if(backward<forward)
                            rotation_direction = -1;
                        else if(available_dir == grid_dir){
                            //if the currently facing direction is the only available and blocked(because we are here) set the facing grid to blocked
                            assign_blocked(grid_dir);
                            
                            //rotate in the default direction
                            rotation_direction = default_rotation_direction;
                        }
                        else
                            rotation_direction = 1;//in any other case rotate in the default direction
                        
                    }
                    else{// in other cases rotate in default direction
                        rotation_direction = default_rotation_direction;
                    }
                }
            }
            if(all_known)
                rotation_direction = default_rotation_direction;//if all known rotate in default direction(this is here to reset rotation direction if one was known and it has been established that it was blocked)
            
            //at the end just set the velocities based on the logit prior
            angular = rotation_direction * default_ang;
            linear = 0.0;
        }
    }
  
}
 
void assign_blocked(int dir){//assign the neighbour grid in the corresponding grid direction as blocked
    //determine increments based on specified direction
    int x_f = 0, y_f = 0;
    if(dir == 0 || dir == 1 || dir == 7)
        x_f = 1;
    else if(dir == 3 || dir == 4 || dir == 5)
        x_f = -1;
    else
        x_f = 0;
    
    if(dir == 1 || dir == 2 || dir == 3)
        y_f = 1;
    else if(dir == 5 || dir == 6 || dir == 7)
        y_f = -1;
    else
        y_f = 0;
    
    
    if(grid_x + x_f >= 0 && grid_x + x_f < Grid_Size && grid_y + y_f >= 0 && grid_y + y_f < Grid_Size){//check to see if in bound
        if(grid.at<uchar>(grid_x + x_f, grid_y + y_f) == 50){//if grid is unvisited set the grid to blocked(0)
            grid.at<uchar>(grid_x + x_f, grid_y + y_f) = 0;
        }
    }
}
 
void log(){
    //Saves a JSON file with the relevant information later to be monitored in an HTML log page
    String temp;
    ofstream logfile("log.json");
    if (logfile.is_open())
    {
        logfile << "{";
        temp ="\"angular\": " + to_string(angular) + ",";
        logfile << temp;
        temp ="\"linear\": " + to_string(linear) + ",";
        logfile << temp;
        temp ="\"posX\": " + to_string(posX) + ",";
        logfile << temp;
        temp ="\"posY\": " + to_string(posY) + ",";
        logfile << temp;
        temp ="\"yaw\": " + to_string(yaw) + ",";
        logfile << temp;
        temp ="\"grid_x\": " + to_string(grid_x) + ",";
        logfile << temp;
        temp ="\"grid_y\": " + to_string(grid_y) + ",";
        logfile << temp;
        temp ="\"grid_dir\": " + to_string(grid_dir) + ",";
        logfile << temp;
        temp ="\"minLaserDist\": " + to_string(minLaserDist) + ",";
        logfile << temp;
        temp ="\"desiredAngle\": " + to_string(desiredAngle) + ",";
        logfile << temp;
        temp ="\"any_bumper_pressed\": " + to_string(any_bumper_pressed) + ",";
        logfile << temp;
        temp ="\"millisecondsElapsed\": " + to_string(millisecondsElapsed) + ",";
        logfile << temp;
        temp ="\"searching\": " + to_string(searching) + ",";
        logfile << temp;
        temp ="\"bumper_maneuver\": " + to_string(bumper_maneuver) + ",";
        logfile << temp;
        temp ="\"Grid_size\": " + to_string(Grid_Size) + ",";
        logfile << temp;
        temp ="\"min_angle\": " + to_string(min_angle) + ",";
        logfile << temp;
        temp ="\"max_angle\": " + to_string(max_angle) + ",";
        logfile << temp;
        temp ="\"all_min\": " + to_string(AllminLaserDist) + ",";
        logfile << temp;
        temp ="\"scores\": [" + to_string(checkAdj(0))  + "," + to_string(checkAdj(1))  + "," + to_string(checkAdj(2))  + "," + to_string(checkAdj(3))  + "," + to_string(checkAdj(4))  + "," + to_string(checkAdj(5))  + "," + to_string(checkAdj(6))  + "," + to_string(checkAdj(7)) + "],";
        logfile << temp;
        temp ="\"bumpers\": [" + to_string(bumper[kobuki_msgs::BumperEvent::RIGHT] == kobuki_msgs::BumperEvent::PRESSED) + "," + to_string(bumper[kobuki_msgs::BumperEvent::CENTER] == kobuki_msgs::BumperEvent::PRESSED) + "," + to_string(bumper[kobuki_msgs::BumperEvent::LEFT] == kobuki_msgs::BumperEvent::PRESSED) + "]}";
        logfile << temp;
        logfile.close();
    }
}
 
void handleScan()
{//Scanning is done here
    //set a temp value to infinity to check the minimum distance to any scan locations
    float temp_distance = numeric_limits<float>::infinity();
    
    //find minimum distance to any scan locations
    for(int i = 0; i<scannedlocations.size();i++){
        if(lastScanTime != 0)
            temp_distance = min(DIST(posX,posY,scannedlocations[i][0], scannedlocations[i][1]), temp_distance);
    }
    
    if(!searching && bumper_maneuver == 0 && ((temp_distance >= 1.8 && isfinite(temp_distance)) || millisecondsElapsed - lastScanTime >= 150000) && !scanning){//check to see if scanning can be done at this moment
        //scanning if more 2.5 minutes from last scan or distance more than 1.8m from any scan location
        lastScanTime = millisecondsElapsed; // reset the scan timer
        vector<float> temp_pos_vec = {posX,posY}; //make a float vector of the current position
        scannedlocations.push_back(temp_pos_vec);//save the scan location to be checked later
        scanning_g_dir = grid_dir;
    
        //set the scan velocities
        linear = 0.0;
        angular = scan_ang;
    
        //activate scanning flag
        scanning = true;

        //Adjust timer
        lastNewGridTime -= 2000 * M_PI / scan_ang;

        //reset availability flags for block checking
        for(int i=0;i<8;i++)
            available_scanned_dirs[i] = false;
    }
    else if(scanning &&  millisecondsElapsed - lastScanTime >= 2000 * M_PI / scan_ang){//check to see if scan has been completed
        scanning = false;
        angular = 0.0;
        linear = 0.0;
    }
    else if(scanning){//while scanning see if any adjacent blocks can be detected
        if(minLaserDist >= min_dist)//if not blocked activate the availability flag
            available_scanned_dirs[grid_dir] = true;
        if(grid_dir != scanning_g_dir){//check to see if an adjacent blocked a=has been fully scanned
            int prev_grid_dir = grid_dir - 1; //calculate the previous
    
        //the following is necessary correction to find prev direction in the appropriate (0-7) range
        if(prev_grid_dir < 0)
            prev_grid_dir = 7;
        if(prev_grid_dir > 7)
            prev_grid_dir = 0;
        
        //if an adjacent grid is scanned and not available set it to blocked
        if(prev_grid_dir != scanning_g_dir && !available_scanned_dirs[prev_grid_dir])
            assign_blocked(prev_grid_dir);
        }
    }
}
 
void handle_wall_avoidance(){
    //here the wall avoidance augmentation is applied
    if(bumper_maneuver == 0 && linear  > 0){//check to see if not bumper maneuver in progress and motion in linear exists
        if(LeftLaserDist < min_dist * 1.7 && RightLaserDist < min_dist * 1.7){//check to see if in a tight spot
            angular = 0;
        }
        else if(RightLaserDist < min_dist * 1.25){//check to see if too close to the right
            angular = M_PI / 12;//apply correction to avoid obstacles
        }
        else if(LeftLaserDist < min_dist * 1.25){//check to see if too close to the left
            angular = -M_PI / 12;//apply correction to avoid obstacles
        }
        else{//in any other case set to default
            angular = 0;
        }
    }
}
 
 
//Main Function
int main(int argc, char **argv)
{
    //ROS init
    ros::init(argc, argv, "maze_explorer");
    ros::NodeHandle nh;
    //Establish Subscribers
    ros::Subscriber bumper_sub = nh.subscribe("mobile_base/events/bumper", 10, &bumperCallback);
    ros::Subscriber laser_sub = nh.subscribe("scan", 10, &laserCallback);
    ros::Subscriber odom = nh.subscribe("odom", 1, &odomCallback);
    
    //Start Publishers
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);
    
    //Set Refresh Rate
    ros::Rate loop_rate(10);
    
    //Define Vel Object
    geometry_msgs::Twist vel;
    
    // contest count down timer
    chrono::time_point<chrono::system_clock> start;
    start = chrono::system_clock::now();
    while(ros::ok() && millisecondsElapsed <= 480000) {
        ros::spinOnce();
        //Primary Logic And Operations//
        //handle scan first(see if scanning is possible and appropriate)
        handleScan();
        if(!scanning){//check to ensure scanning not in progress
            //handle bumper operations
            handleBumper();
            //make decisions based on sensor and preception
            handleExloration();
            //apply the final augmentation to the decision based on sensory data
            handle_wall_avoidance();
        }
        //Setting Velocities
        vel.angular.z = angular;
        vel.linear.x = linear;
        vel_pub.publish(vel);
        //save a log of current variables
        log();
        // The last thing to do is to update the timer.
        millisecondsElapsed = chrono::duration_cast<chrono::milliseconds>(chrono::system_clock::now()-start).count();
        loop_rate.sleep();
    }
    
    return 0;
}
