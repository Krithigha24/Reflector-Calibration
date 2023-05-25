#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

ros::Publisher average_scan_pub;
std::vector<float> average_ranges;
std::vector<float> average_intensities;
int num_messages = 0;

// Timeout duration in seconds
double TIMEOUT_DURATION ; 

ros::Timer timeout_timer;

// Resets the average scan parameters
void resetAverageScan()
{
    average_ranges.clear();
    average_intensities.clear();
    num_messages = 0;
    timeout_timer.stop();  // Stop the timer
}

// Callback function for incoming LaserScan messages
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
    sensor_msgs::LaserScan average_scan = *scan_msg;

    const std::vector<float>& ranges = scan_msg->ranges;
    const std::vector<float>& intensities = scan_msg->intensities;

    // Check if it's the first message received
    if (num_messages == 0)
    {
        average_ranges = ranges;
        average_intensities = intensities;
        timeout_timer.start();  // Start the timeout timer
    }
    else
    {
        // Update the average ranges and intensities
        for (size_t i = 0; i < ranges.size(); ++i)
        {
            average_ranges[i] = (average_ranges[i] * num_messages + ranges[i]) / (num_messages + 1);
            average_intensities[i] = (average_intensities[i] * num_messages + intensities[i]) / (num_messages + 1);
        }
    }

    ++num_messages;

    // Publish the computed average scan
    average_scan.ranges = average_ranges;
    average_scan.intensities = average_intensities;
    average_scan_pub.publish(average_scan);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "scanAvg_node");
    ros::NodeHandle nh;

    // Load the TIMEOUT_DURATION parameter from the parameter server
    if (!nh.getParam("timeout_duration", TIMEOUT_DURATION))
    {
        ROS_WARN("Failed to load 'timeout_duration' parameter. Using default value: %.1f", TIMEOUT_DURATION);
        TIMEOUT_DURATION = 30.0;  // Default value
    }
    
    average_scan_pub = nh.advertise<sensor_msgs::LaserScan>("/average_scan", 10);
    ros::Subscriber scan_sub = nh.subscribe("/filtered_scan", 10, scanCallback);

    // Timer for checking the message timeout
    timeout_timer = nh.createTimer(ros::Duration(TIMEOUT_DURATION), [](const ros::TimerEvent&) {
        ROS_INFO("No messages received within the timeout period. Resetting average scan.");
        resetAverageScan();
    });

    timeout_timer.stop();  // Stop the timer initially

    ros::spin();

    return 0;
}
