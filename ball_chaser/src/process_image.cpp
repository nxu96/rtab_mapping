#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>


// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // Request a service and pass the velocities to it to drive the robot
    ROS_INFO_STREAM("Send motor command");
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;

    if (!client.call(srv)){
        ROS_ERROR("Failed to call service /command_robot");
    }
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{

    int white_pixel = 255;
    int green_channel_offset = 1;
    int blue_channel_offset = 2;
    // Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera
    int starting_pixel = img.data.size() / 3; // data is stored in a vector
    int ending_pixel = img.data.size() *2 / 3; 
    int center_pixel = 0;
    int num_white_pixels = 0;
    for (unsigned int i = starting_pixel; i+2 < ending_pixel; i+=3){
        // find white pixel
        if (img.data[i] == white_pixel && img.data[i+green_channel_offset] == white_pixel && img.data[i+blue_channel_offset] == white_pixel){
            // identify the white ball direction
            center_pixel += (i % (3 * img.width)) / 3; 
            num_white_pixels += 1;
        }
    }

    // if there is not a ball in the image, don't move anymore
    if (num_white_pixels == 0){
        drive_robot(0,0.5); // keep spining until you find the ball in the world

        // drive_robot(0,0);    
    }
    else{// take the average as the center pixel row position 
        center_pixel /= num_white_pixels;
        // if (num_white_pixels / (double) img.width * img.height > 0.8){ // when the num of white pixel is large enough, means we are very close to the ball
        //     drive_robot(0,0); // stop there when close
        // }
        // else{
            if (center_pixel < img.width / 3){
                drive_robot(0.5,1); // turn left
            }
            else if (center_pixel > img.width * 2 / 3 ){
                drive_robot(0.5,-1); //turn right
            }
            else{
                drive_robot(1,0);
            }
        // }
    }
}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}