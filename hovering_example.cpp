
#include <thread>
#include <stdint.h>
#include <chrono>
#include <math.h>
#include <Eigen/Core>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <visualization_msgs/Marker.h> //include marker msg
#include <geometry_msgs/PoseStamped.h>
#include <inttypes.h>
#include <array>
/*
double distance_x, distance_y, distance_z, curr_time, error_x, error_y, error_z, last_error_x, last_error_y, last_error_z, d_error_x, d_error_y, d_error_z, sum_error_x, sum_error_y, sum_error_z, sum_error_yaw, velocity_x, velocity_y, velocity_z, velocity_pitch, velocity_roll, velocity_altitude, velocity_yaw, rotation_x, rotation_y, rotation_z, flag = 0;

float prev_time, time_elapsed = 0;

// target distances from the aruco marker
double goal_x = 0;
double goal_y = 0;
double goal_z = 2.0;

// pid values for velocity in x-axis
double kp_x = 0.5;
double ki_x = 0;
double kd_x = 5;

// pid values for velocity in y-axis
double kp_y = 1;
double ki_y = 0;
double kd_y = 5;

// pid values for velocity in z-axis
double kp_z = 2;
double ki_z = 0;
double kd_z = 0.5;

// pid values for yaw of the drone
double kp_yaw = 1;
double ki_yaw = 0;


// Function calculating pitch
inline double getPitch(const Eigen::Quaterniond& q) {
   return asin(-2.0*(q.x()*q.z() - q.w()*q.y()));
}

// Function calculating roll
inline double getRoll(const Eigen::Quaterniond& q) {
   return atan2(2.0*(q.x()*q.y() + q.w()*q.z()), q.w()*q.w() + q.x()*q.x() - q.y()*q.y() - q.z()*q.z());
}
*/
visualization_msgs::Marker current_marker; //marker object placeholder
geometry_msgs::PoseStamped current_pose;


void marker_cb(const visualization_msgs::Marker::ConstPtr& msg){ //marker callback
    current_marker = *msg;
}

void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){ //marker callback
    current_pose = *msg;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "hovering_example");
  ros::NodeHandle nh;
  // Create a private node handle for accessing node parameters.
  ros::NodeHandle nh_private("~");

  // create a subscriber to the aruco marker topic
ros::Subscriber marker_sub = nh.subscribe<visualization_msgs::Marker>("/aruco_single/marker", 100, marker_cb); //marker subscriber

  // create a subscriber to the aruco marker topic
ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/aruco_single/pose", 100, pose_cb); //marker subscriber

// publisher for aruco marker
ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("test_data", 10); //marker publisher


//publisher for Joint Trajectory
ros::Publisher trajectory_pub = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);

/*
//publisher for Eigen Point
ros::Publisher eigen_pub = nh.advertise<mav_msgs::EigenTrajectoryPoint>(mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);
*/

ROS_INFO("Started hovering example.");

//service to unpause Gazebo's physics
  std_srvs::Empty srv;
  bool unpaused = ros::service::call("/gazebo/unpause_physics", srv);
  unsigned int i = 0;

  // Trying to unpause Gazebo for 10 seconds.
  while (i <= 10 && !unpaused) {
    ROS_INFO("Wait for 1 second before trying to unpause Gazebo again.");
    std::this_thread::sleep_for(std::chrono::seconds(1));
    unpaused = ros::service::call("/gazebo/unpause_physics", srv);
    ++i;
  }

  if (!unpaused) {
    ROS_FATAL("Could not wake up Gazebo.");
    return -1;
  } else {
    ROS_INFO("Unpaused the Gazebo simulation.");
  }

  // Wait for 5 seconds to let the Gazebo GUI show up.
  ros::Duration(5.0).sleep();


 // 3D cartesian vector to denote desired position in space (x,y,z)
  Eigen::Vector3d desired_position(1.0, 0.0, 2.0);
 // desired yaw (rotation about z-axis) value
  double desired_yaw = 0.0;

// rate of ros loop (Hz)
ros::Rate loop_rate(10);

//counter variable
int count = 0;
float curr_time = 0;

	while (ros::ok()) { //start loop

		//publish marker data (updated through subscriber callback fxn)
		marker_pub.publish(current_marker);

		//Joint Trajectory empty placeholder
		trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
		//time
		trajectory_msg.header.stamp = ros::Time::now();
//START
		if (count != 0) { //second iteration of the loop

		mav_msgs::EigenTrajectoryPoint pose_trajectory;
		mav_msgs::eigenTrajectoryPointFromPoseMsg(current_pose, &pose_trajectory);


// ------------------------------------------------------------------CONTROLS PID CODE-----------------------------------------------------------------------------

/*		distance_x = pose_trajectory.position_W[0];
		distance_y = pose_trajectory.position_W[1];
		distance_z = pose_trajectory.position_W[2];
		//ROS_INFO("Distance to marker: x = %f, y = %f, z = %f\n", distance_x, distance_y, distance_z);



		double euler_r = getRoll(pose_trajectory.orientation_W_B);
		double euler_p = getPitch(pose_trajectory.orientation_W_B);
		double euler_y = pose_trajectory.getYaw();
*/
		//angles in radians

		//ROS_INFO("Euler Angles: roll = %f, pitch = %f, yaw = %f\n", euler_r, euler_p, euler_y);

	//	double theta = euler_y - pose_trajectory.orientation_W_B.z() - M_PI/2; //z rotation should be in drone ref. frame


//		curr_time = trajectory_msg.header.stamp.sec + trajectory_msg.header.stamp.nsec * pow(10, -9);
		//double d_curr_time = static_cast<double>(curr_time) * pow(10, -9);
		//ROS_INFO("Current Time: %f\n", curr_time);


//		if(flag == 0){
	//	prev_time = curr_time;
		//flag = 1;
		//}

		//time_elapsed = curr_time - prev_time;
		//ROS_INFO("Previous Time: %f\n", prev_time);
		//prev_time = curr_time;
		//ROS_INFO("Time Elapsed: %" PRIu64 "\n", time_elapsed);


		//ROS_INFO("Time elapsed: %f\n", time_elapsed);



		/*
		error_x = distance_y - goal_x;
		error_y = distance_x - goal_y;
		error_z = distance_z - goal_z;
*/
		//ROS_INFO("Error: x = %f, y = %f, z = %f\n", error_x, error_y, error_z);

/*
		// calculate differential component of error
		d_error_x = ( error_x - last_error_x ) / time_elapsed;
		d_error_y = ( error_y - last_error_y ) / time_elapsed;
		d_error_z = ( error_z - last_error_z ) / time_elapsed;

		//ROS_INFO("Differential Error: x = %f, y = %f, z = %f\n", d_error_x, d_error_y, d_error_z);

		// calculate sum of errors
		sum_error_x += (error_x * time_elapsed);
		sum_error_y += (error_y * time_elapsed);
		sum_error_z += (error_z * time_elapsed);
		sum_error_yaw += (rotation_z * time_elapsed); //could be vehicle rotation?

		ROS_INFO("Sum Error: x = %f, y = %f, z = %f\n", sum_error_x, sum_error_y, sum_error_z);

		// pid loop for x,y,z and yaw velocities
		velocity_x = kp_x * error_x + ki_x * sum_error_x + kd_x * d_error_x;
		velocity_y = kp_y * error_y + ki_y * sum_error_y + kd_y * d_error_y;
		velocity_z = kp_z * error_z + ki_z * sum_error_z + kd_z * d_error_z;

		velocity_pitch = velocity_x * cos(theta) + velocity_y * sin(theta);
		velocity_roll = velocity_y * cos(theta) - velocity_x * sin(theta);
		velocity_altitude = -1 * velocity_z;

		//ROS_INFO("Velocity : roll = %f, pitch = %f, altitude = %f\n", velocity_roll, velocity_pitch, velocity_altitude);

		pose_trajectory.velocity_W[0] = velocity_roll;
		pose_trajectory.velocity_W[1] = velocity_pitch;
		pose_trajectory.velocity_W[2] = velocity_altitude;

		pose_trajectory.angular_velocity_W[2] = 0; //yaw

    */

  /*  pose_trajectory.velocity_W[0] = 0.001;
    pose_trajectory.velocity_W[1] = 0.001;
    pose_trajectory.velocity_W[2] = 0.001;
*/



    pose_trajectory.position_W[2] = 2;
		mav_msgs::msgMultiDofJointTrajectoryFromEigen(pose_trajectory, &trajectory_msg);


    int array_size = trajectory_msg.points.size();
      ROS_INFO("array size: %f\n",array_size);

    trajectory_msg.points[1].velocities[0].linear.x = 0;
    trajectory_msg.points[1].velocities[0].linear.y = 0;
    trajectory_msg.points[1].velocities[0].linear.z = 0;


    ROS_INFO("Velocity : roll = %f, pitch = %f, altitude = %f\n", trajectory_msg.points[1].velocities[0].linear.x, trajectory_msg.points[1].velocities[0].linear.y, trajectory_msg.points[1].velocities[0].linear.z);



		trajectory_pub.publish(trajectory_msg);

	//	last_error_x = error_x;
	//	last_error_y = error_y;
	//	last_error_z = error_z;




		//pose_trajectory.velocity_W[0] = 0.05;
		//pose_trajectory.velocity_W[1] = 0.05;
		//pose_trajectory.velocity_W[2] = 0;

		//pose_trajectory.angular_velocity_W[2] = 0; //yaw

		//pose_trajectory.position_W[2] = 2;


	//	desired_position.x() = -current_pose.pose.position.y;
		//desired_position.y() = -current_pose.pose.position.x;
		//desired_yaw = pose_trajectory.getYaw();


		}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------
		if (count == 0) {

		  /* Overwrite defaults if set as node parameters.
		  nh_private.param("x", desired_position.x(), desired_position.x());
		  nh_private.param("y", desired_position.y(), desired_position.y());
		  nh_private.param("z", desired_position.z(), desired_position.z());
		  nh_private.param("yaw", desired_yaw, desired_yaw);*/

		  mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position, desired_yaw, &trajectory_msg);

		  ROS_INFO("Publishing waypoint on namespace %s: [%f, %f, %f].",nh.getNamespace().c_str(), desired_position.x(),desired_position.y(), desired_position.z());

  		// publish joint trajectory message
		trajectory_pub.publish(trajectory_msg);
			ROS_INFO("Position Update: x = %f, y = %f, yaw = %f\n", desired_position.x(),desired_position.y(), desired_yaw);
		  //ROS_INFO("Data of Marker Found: x = %f, y = %f, z = %f\n", current_pose.pose.position.x,current_pose.pose.position.y,current_pose.pose.position.z);


		// first iteration of loop

			ros::Duration(5.0).sleep();
		}

		ros::spinOnce();
		loop_rate.sleep();
		count++;
}

 // ros::shutdown();

  return 0;
}
