#include <ros/ros.h>
//#include <std_msgs/double32.h>
#include <actionlib/server/simple_action_server.h>
#include <mav_state_machine/trackingAction.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <tf/tf.h>
#include <ail_mav/mav_interface.h>
#include <ail_mav/SetPose.h>
class TrajectoryGeneration
{
public:
    TrajectoryGeneration(void)
    {

    }
    ~TrajectoryGeneration(void)
    {

    }
    geometry_msgs::Vector3 pos_final_;
    double orientation_final_;
    geometry_msgs::Vector3 pos_init_;
    double orientation_init_;
    ros::Time time_init_;
    void generate()
    {
    //    geometry_msgs::Vector3 pos_final_init = pos_final_ - pos_init_;
    //    double length = 
    //    traj_duration
    }
    void get_setpt(ros::Time t, geometry_msgs::Vector3 &pos_sp, double &orientation_sp)
    {
        pos_sp = pos_final_;
        orientation_sp = orientation_final_;
    }
public:
    ros::Duration traj_duration;
};
class TrackingAction
{
public:
    TrackingAction(std::string name) : 
        as_(nh_, name, false),
        action_name_(name),
        serving_(false),
        traj()
    {
            //register the goal and feeback callbacks
        as_.registerGoalCallback(boost::bind(&TrackingAction::goalCB, this));
        as_.registerPreemptCallback(boost::bind(&TrackingAction::preemptCB, this));

    //subscribe to the data topic of interest
        sub_ = nh_.subscribe("/mavros/local_position/pose", 1, &TrackingAction::stateCB, this);
    //    pub_ = nh_.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1);
        ros::NodeHandle n;
        client_pos_control = n.serviceClient<ail_mav::SetPose>("/mav_interface_node/setPose");
        
        as_.start();
    }
    ~TrackingAction(void)
    {
    }
    void run(double freq)
    {
        ros::NodeHandle node;
        ros::Timer timer = node.createTimer(ros::Duration(1.0/freq), &TrackingAction::iteration, this);
        ros::spin();
    }
    void iteration(const ros::TimerEvent& e)
    {
        static double time_elapse = 0;
        double dt = e.current_real.toSec() - e.last_real.toSec();
        time_elapse += dt;
        ros::Time time_now = ros::Time::now();
        ros::Duration duration_from_traj_gen = time_now - traj.time_init_;
        if(serving_){
        //    ROS_INFO("duration %f",duration_from_traj_gen.toSec());
            double len=(pos_.x - traj.pos_final_.x)*(pos_.x - traj.pos_final_.x)+(pos_.y - traj.pos_final_.y)*(pos_.y - traj.pos_final_.y)+(pos_.z - traj.pos_final_.z)*(pos_.z - traj.pos_final_.z);
            if(len < 0.02 ){
                ROS_INFO("%s: Succeeded", action_name_.c_str());
                // set the action state to succeeded
                result_.success = true;
                as_.setSucceeded(result_);
                serving_ = false;
                return;
            }
            ail_mav::SetPose srv;
            traj.get_setpt(ros::Time::now(), srv.request.position, srv.request.orientation);
            if(client_pos_control.call(srv)){
                
            }
        }
        
        
    }
    void goalCB()
    {
        mav_state_machine::trackingGoal goal = *as_.acceptNewGoal();
        // traj generation
        traj.pos_final_ = goal.pos_final;
        traj.orientation_final_ = goal.orientation_final;
        traj.pos_init_ = pos_;
        traj.orientation_init_ = orientation_;
        traj.time_init_ = ros::Time::now();
        serving_ = true;
        ROS_INFO("accepted x: %f, y: %f, z: %f, yaw: %f",
            traj.pos_final_.x,traj.pos_final_.y,traj.pos_final_.z,traj.orientation_final_);
    }
    void preemptCB()
    {
        ROS_INFO("%s: Preempted", action_name_.c_str());
    // set the action state to preempted
        as_.setPreempted();
    }
    void stateCB(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        pos_.x = msg->pose.position.x;
        pos_.y = msg->pose.position.y;
        pos_.z = msg->pose.position.z;
        tf::Quaternion q(
        msg->pose.orientation.x,
        msg->pose.orientation.y,
        msg->pose.orientation.z,
        msg->pose.orientation.w);
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        orientation_ = yaw;
    }
protected:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<mav_state_machine::trackingAction> as_;
    std::string action_name_;

//    geometry_msgs::Twist command_;
    mav_state_machine::trackingResult result_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
    ros::ServiceClient client_pos_control;
    TrajectoryGeneration traj;
    bool serving_;
    geometry_msgs::Vector3 pos_;
    double orientation_;
};
int main(int argc, char** argv)
{
    ros::init(argc, argv, "tracking_actions");

    TrackingAction tracking(ros::this_node::getName());
 //   ros::spin();
    tracking.run(10);
    return 0;
}
