#include "ros/ros.h"
#include "vesc_msgs/VescStateStamped.h"

#include "std_msgs/Float64.h"

class Averager {
  private:
    int idxPtr; //idPtr to keep track of which item to replace in the buffer, later to compute running avg
    int n;
    double buffer[10];
  public:
    Averager();
    void chatterCallback(const vesc_msgs::VescStateStamped::ConstPtr& msg);
    double getAvgVelocity();
};

Averager::Averager() {
  this->idxPtr = 0;
  this->n = 0;

  for(int i=0; i < 10; i++){
    this->buffer[i] = 0;
  }
}

double Averager::getAvgVelocity() {
  //If readings less than 10 return 0 
  if(this->n < 9){
    return 0.0;
  }

  double sum=0;
  for(int i=0; i < 10; i++){
    sum += this->buffer[i];
  }
  //sum up past 10 readings and return avg
  //idxPtr replaces old value in the buffer to keep buffer contents to past 10 readings	
  return sum/10;
}

void Averager::chatterCallback(const vesc_msgs::VescStateStamped::ConstPtr& msg)
{
  this->n++;
  this->buffer[this->idxPtr] = msg->state.speed;
  this->idxPtr++;

  if(this->idxPtr > 9){
    this->idxPtr = 0;
   }

  ROS_INFO("Cur velocity: [%lf]; Running avg: [%lf]", msg->state.speed, (double) this->getAvgVelocity()); 

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "jawalkar_average");
  ros::NodeHandle n;
  Averager averager;

  ros::Subscriber sub = n.subscribe("/sensors/core", 1000, &Averager::chatterCallback, &averager);

  ros::Publisher pub = n.advertise<std_msgs::Float64>("/average_velocity", 1000);
  ros::Rate loop_rate(5);


  while (ros::ok())
  {
    std_msgs::Float64 msg;
    msg.data = averager.getAvgVelocity();
    pub.publish(msg);
    //ROS_INFO("Publishing computed average velocity: [%f]",(float) msg.data);
    ros::spinOnce();
    loop_rate.sleep();
  }

  ros::spin();

  return 0;
}

