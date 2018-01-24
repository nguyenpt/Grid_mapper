#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include <cstdlib> // Needed for rand()
#include <ctime> // Needed to seed random number generator with a time value
#include <boost/thread/mutex.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <tf/transform_listener.h>
#include <math.h>

using namespace boost::posix_time;


class GridMapper {
public:
  // Construct a new occupancy grid mapper  object and hook up
  // this ROS node to the simulated robot's pose, velocity control,
  // and laser topics
  GridMapper(ros::NodeHandle& nh) :
      canvas(200,200,CV_8UC1) {
    // Initialize random time generator
    srand(time(NULL));

    // Advertise a new publisher for the current simulated robot's
    // velocity command topic (the second argument indicates that
    // if multiple command messages are in the queue to be sent,
    // only the last command will be sent)
    commandPub = nh.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);

    // Subscribe to the current simulated robot's laser scan topic and
    // tell ROS to call this->laserCallback() whenever a new message
    // is published on that topic
    laserSub = nh.subscribe("/scan", 1, \
      &GridMapper::laserCallback, this);
    
    // Subscribe to the current simulated robot' ground truth pose topic
    // and tell ROS to call this->poseCallback(...) whenever a new
    // message is published on that topic
    poseSub = nh.subscribe("/odom", 1, \
      &GridMapper::poseCallback, this);
   
    // Timer that allows use to save snapshot every 30 seconds. 
    timer = nh.createTimer(ros::Duration(30.0), 
        &GridMapper::timerCallback,this);  

    // Create resizeable named window
  //  cv::namedWindow("Occupancy Grid Canvas", \
   //   CV_WINDOW_NORMAL | CV_WINDOW_KEEPRATIO | CV_GUI_EXPANDED);

  };
  
  void timerCallback(const ros::TimerEvent&)
  {
    saveSnapshot();
    ROS_INFO_STREAM("Snapshot of Occupancy Grid map saved!");

  }

  
  
  // Save a snapshot of the occupancy grid canvas
  // NOTE: image is saved to same folder where code was executed
  void saveSnapshot() {
    std::string filename = "grid_" + to_iso_string(second_clock::local_time()) + ".jpg";
    canvasMutex.lock();
    cv::imwrite(filename, canvas);
    canvasMutex.unlock();
  };
 

  
  
  // Update grayscale intensity on canvas pixel (x, y) (in robot coordinate frame)
  void plot(int x, int y, char value) {
    canvasMutex.lock();
    x+=canvas.rows/2;
    y+=canvas.cols/2;
    //added condition where you cannot plot over the robot's path
    if (x >= 0 && x < canvas.rows && y >= 0 && y < canvas.cols
        && canvas.at<char>(canvas.cols - y, x) != CELL_ROBOT) {
      canvas.at<char>(canvas.cols - y, x) = value;
    }
    canvasMutex.unlock();
  };

  // Update grayscale intensity on canvas pixel (x, y) (in image coordinate frame)
  void plotImg(int x, int y, char value) {
    canvasMutex.lock();
    if (x >= 0 && x < canvas.cols && y >= 0 && y < canvas.rows) {
      //canvas.at<char>(y, x) = value;
      canvas.at<char>(y, x) = value;
    }
    canvasMutex.unlock();
  };

  // Send a velocity command
  void move(double linearVelMPS, double angularVelRadPS) {
    geometry_msgs::Twist msg; // The default constructor will set all commands to 0
    msg.linear.x = linearVelMPS;
    msg.angular.z = angularVelRadPS;
    commandPub.publish(msg);
  };

  /**
   *Uses the Bresenham line algorithm in order to update the Occupancy Grid,
   *given coordinates of the two end points of a line.
   */
  void bresenhamLine(int x1, int y1, int x2, int y2) {
    int w = x2 - x1;
    int h = y2 - y1;
    int dx1 = 0, dy1 = 0, dx2 = 0, dy2 = 0;
    //boolean conditions to make sure algorithm works for all octants
    if (w<0) 
      dx1=-1;
    else if (w>0) 
      dx1 = 1;
    if (h<0) 
      dy1 = -1;
    else if (h>0)
      dy1 = 1;
    if (w<0) 
      dx2 = -1;
    else if (w>0) 
      dx2 = 1;
    int longest = abs(w);
    int shortest = abs(h);
    if (!(longest>shortest)) 
    {
       longest = abs(h);
       shortest = abs(w);
       if (h<0) dy2 = -1;
       else if (h>0) dy2 = 1;
       dx2 = 0;
    }
    int numerator = longest / 2;
    int y = y1;
 
    for(int i = 0; i <= longest; i++)
    {
       plot(x1,y1,CELL_FREE);
       numerator+=shortest;
       if (!(numerator<longest))
       {
          numerator -= longest;
          x1 += dx1;
          y1 +=dy1;
       }
       else {
          x1 += dx2;
          y1 += dy2; 
       }
    } 
    plot(x2,y2, CELL_OCCUPIED);
    

  
  }


  // Process incoming laser scan message
  void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    // TODO: parse laser data and update occupancy grid canvas
    //       (use CELL_OCCUPIED, CELL_UNKNOWN, CELL_FREE, and CELL_ROBOT values)
    // (see http://www.ros.org/doc/api/sensor_msgs/html/msg/LaserScan.html)
    int minIndex = 0;
    int maxIndex = msg->ranges.size()-1;

    for (int i = minIndex; i <= maxIndex; i++)
    {
      if (!isnan(msg->ranges[i]))
      {
        double d = msg->ranges[i];
        double angle = msg->angle_min + i * msg->angle_increment;

        int x2,y2;
        //calculate position of end point of laser data with known pose of robot
        x2 = x + d * cos(heading + angle + NINETYDEGREES);
        y2 = y + d * sin(heading + angle + NINETYDEGREES); 
     
        bresenhamLine(x,y,x2,y2);
      }
    }
    //plots the robot path
    plot(x,y,CELL_ROBOT);
    


  };

  
  
  // Process incoming ground truth robot pose message
  void poseCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    double roll, pitch;
    x = msg->pose.pose.position.x;
    y = msg->pose.pose.position.y;
    heading=tf::getYaw(msg->pose.pose.orientation);
  };
  
  
  // Main FSM loop for ensuring that ROS messages are
  // processed in a timely manner, and also for sending
  // velocity controls to the simulated robot based on the FSM state
  void spin() {
    int key = 0;
    
    // Initialize all pixel values in canvas to CELL_UNKNOWN
    canvasMutex.lock();
    canvas = cv::Scalar(CELL_UNKNOWN);
    canvasMutex.unlock();
    
    while (ros::ok()) { // Keep spinning loop until user presses Ctrl+C

      
      // NOTE: DO NOT REMOVE CODE BELOW THIS LINE
      //cv::imshow("Occupancy Grid Canvas", canvas);
      ros::spinOnce(); // Need to call this function often to allow ROS to process incoming messages
      key = cv::waitKey(1000/SPIN_RATE_HZ); // Obtain keypress from user; wait at most N milliseconds
      if (key == 'x' || key == 'X') {
        break;
      } else if (key == ' ') {
        saveSnapshot();
      }
    }
    
    ros::shutdown(); // Ensure that this ROS node shuts down properly
  };

  // Tunable motion controller parameters
  const static double FORWARD_SPEED_MPS = 2.0;
  const static double ROTATE_SPEED_RADPS = M_PI/2;
  
  const static int SPIN_RATE_HZ = 30;
  
  const static char CELL_OCCUPIED = 0;
  const static char CELL_UNKNOWN = 86;
  const static char CELL_FREE = 172;
  const static char CELL_ROBOT = 255;
  const static double NINETYDEGREES = M_PI/2;
  const static double ANGULAR_SPEED = 1.0;
  const static double GRID_SIZE = 0.1;
  const static double angleNormalizer = .5;

protected:
  ros::Publisher commandPub; // Publisher to the current robot's velocity command topic
  ros::Subscriber laserSub; // Subscriber to the current robot's laser scan topic
  ros::Subscriber poseSub; // Subscriber to the current robot's ground truth pose topic
 
  ros::Timer timer; //allows us to schedule the snapshot function every 30 seconds

  double x; // in simulated Stage units, + = East/right
  double y; // in simulated Stage units, + = North/up
  double heading; // in radians, 0 = East (+x dir.), pi/2 = North (+y dir.)
  
  cv::Mat canvas; // Occupancy grid canvas
  boost::mutex canvasMutex; // Mutex for occupancy grid canvas object
};


int main(int argc, char **argv) {
  
  ros::init(argc, argv, "grid_mapper"); // Initiate ROS node
  ros::NodeHandle n; // Create default handle
  GridMapper robbie(n); // Create new grid mapper object
  robbie.spin(); // Execute FSM loop

  return EXIT_SUCCESS;
};

