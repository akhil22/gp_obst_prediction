#include <sensor_msgs/PointCloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <gp_obst_prediction/gp_predict_functions.h>
/**
 * publish a temporary trajectory in the sensor frame 
 */
int main(int argc, char **argv)
{
	/**
	 * The ros::init() function needs to see argc and argv so that it can perform
	 * any ROS arguments and name remapping that were provided at the command line. For programmatic
	 * remappings you can use a different version of init() which takes remappings
	 * directly, but for most command-line programs, passing argc and argv is the easiest
	 * way to do it.  The third argument to init() is the name of the node.
	 *
	 * You must call one of the versions of ros::init() before using any other
	 * part of the ROS system.
	 */
	gp_obst_prediction::GpPredictFunctions gp_predict;	
	int num_points;
	TVector<TDoubleVector> data = gp_predict.ReadTrajectory("/home/akhil/traj/center/pt1.txt",&num_points); 
	sensor_msgs::PointCloud cloud;
	//	cloud.header.stamp = ros::Time::now();
	cloud.header.frame_id = "base_link";
	cloud.points.resize(num_points);
	cloud.channels.resize(1);
	cloud.channels[0].name = "intensities";
	cloud.channels[0].values.resize(num_points);
//	cloud.channels[0].name = "intensities";
//	cloud.channels[0].values.resize(num_points);
	for(int i=0;i<cloud.points.size();i++){
		cloud.points[i].x = data(i)(0);
		cloud.points[i].y = data(i)(1);
		cloud.points[i].z =1.0;
		cloud.channels[0].values[i] = 100 + 1;
	}
	ros::init(argc, argv, "talker");

	/**
	 * NodeHandle is the main access point to communications with the ROS system.
	 * The first NodeHandle constructed will fully initialize this node, and the last
	 * NodeHandle destructed will close down the node.
	 */
	ros::NodeHandle n;
        
	/**
	 * The advertise() function is how you tell ROS that you want to
	 * publish on a given topic name. This invokes a call to the ROS
	 * master node, which keeps a registry of who is publishing and who
	 * is subscribing. After this advertise() call is made, the master
	 * node will notify anyone who is trying to subscribe to this topic name,
	 * and they will in turn negotiate a peer-to-peer connection with this
	 * node.  advertise() returns a Publisher object which allows you to
	 * publish messages on that topic through a call to publish().  Once
	 * all copies of the returned Publisher object are destroyed, the topic
	 * will be automatically unadvertised.
	 *
	 * The second parameter to advertise() is the size of the message queue
	 * used for publishing messages.  If messages are published more quickly
	 * than we can send them, the number here specifies how many messages to
	 * buffer up before throwing some away.
	 */
	ros::Publisher traj_pub = n.advertise<sensor_msgs::PointCloud>("trajectory", 50);

	ros::Rate loop_rate(10);

	/**
	 * A count of how many messages we have sent. This is used to create
	 * a unique string for each message.
	 */

	int count = 0;
	while (ros::ok())
	{
		/**
		 * This is a message object. You stuff it with data, and then publish it.
		 */
        // 	std_msgs::String msg;

	//	std::stringstream ss;
	//	ss << "hello world " << count;
	//	msg.data = ss.str();

//		ROS_INFO("%s", msg.data.c_str());

		/**
		 * The publish() function is how you send messages. The parameter
		 * is the message object. The type of this object must agree with the type
		 * given as a template parameter to the advertise<>() call, as was done
		 * in the constructor above.
		 */
//		chatter_pub.publish(msg);i
		traj_pub.publish(cloud);

		loop_rate.sleep();
		++count;
	}


	return 0;
}


