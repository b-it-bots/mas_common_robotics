/*
 *  aravis_camera_node.cpp
 *
 *  Created on: 31.07.2013
 *      Author: Frederik Hegger
 */

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <dynamic_reconfigure/server.h>
#include <std_msgs/Int32.h>
#include <mcr_node_diagnostic/NodeDiagnostic.h>

#include "mcr_aravis_vision/AravisVisionConfig.h"
#include "mcr_aravis_vision/aravis_wrapper.h"

#define DIAGNOSTICS_ID "RGB_Camera"

AravisWrapper *gige_cam = new AravisWrapper();
image_transport::Publisher *pub;
std::string frame_id = "";

void connectToCamera()
{
	// Connect to camera
	int connection_status = -1;
	do
	{
		connection_status = gige_cam->connect();

		if (connection_status == ConnectionErrorCodes::ALREADY_CONNECTED)
		{
			ROS_ERROR("Connection to this camera is already established. Exiting ...");
			mcr::NodeDiagnostic::error(DIAGNOSTICS_ID, mcr::NodeDiagnostic::COMMUNICATION, "Connection to this camera is already established. Exiting ...");
			exit(0);
		}
		else if (connection_status == ConnectionErrorCodes::NO_CAMERA_FOUND)
		{
			ROS_ERROR("No GigE camera found. Retry every second");
			mcr::NodeDiagnostic::error(DIAGNOSTICS_ID, mcr::NodeDiagnostic::COMMUNICATION, "No GigE camera found. Retry every second");
			sleep(1);
		}
		else if (connection_status == ConnectionErrorCodes::DEVICE_BUSY)
		{
			ROS_ERROR("Could not create stream thread (check if the device is not already used)");
			mcr::NodeDiagnostic::error(DIAGNOSTICS_ID, mcr::NodeDiagnostic::COMMUNICATION,
			                            "Could not create stream thread (check if the device is not already used)");
			exit(0);
		}
	}
	while ((connection_status != ConnectionErrorCodes::CONNECTION_ESTABLISHED) && ros::ok());

	ROS_INFO_STREAM("Camera " << gige_cam->getCameraDescription() << " successfully connected");
	mcr::NodeDiagnostic::ok(DIAGNOSTICS_ID, mcr::NodeDiagnostic::COMMUNICATION, "Camera successfully connected");

}

void setDefaultParameter(ros::NodeHandle &nh)
{
	// set inital zoom position
	int init_pos = 0;
	nh.param<int>("zoom/position/init", init_pos, 0);
	gige_cam->setParameter("ZoomControlMode", "ZoomTarget");
	gige_cam->setParameter("ZoomPosition", init_pos);
	ROS_INFO_STREAM("   initial zoom pos: " << init_pos);

	// set camera resolution
	std::string resolution = "";
	nh.param < std::string > ("camera/resolution", resolution, "P720_30");
	gige_cam->setParameter("OutputMode", resolution);
	ROS_INFO_STREAM("   resolution: " << resolution);

	// set frame id
	nh.param < std::string > ("camera/frame_id", frame_id, "rgb_camera");
	ROS_INFO_STREAM("   frame_id: " << frame_id);

}

void zoomVelocityCommandCallback(const std_msgs::Int32::ConstPtr &msg)
{
	// switch to velocity control mode
	gige_cam->setParameter("ZoomControlMode", "ZoomDrive");

	if (msg->data > 0)
	{
		gige_cam->setParameter("ZoomPlusSpeed", msg->data);
		gige_cam->setParameter("ZoomPlus");
	}
	else if (msg->data < 0)
	{
		gige_cam->setParameter("ZoomMinusSpeed", (msg->data * (-1)));
		gige_cam->setParameter("ZoomMinus");
	}
	else
		gige_cam->setParameter("ZoomStop");

}

void zoomPositionCommandCallback(const std_msgs::Int32::ConstPtr &msg)
{
	// switch to position control mode
	gige_cam->setParameter("ZoomControlMode", "ZoomTarget");

	// set zoom position
	gige_cam->setParameter("ZoomPosition", msg->data);

}

void dynamicReconfigureCallback(mcr_aravis_vision::AravisVisionConfig &config, uint32_t level)
{
	gige_cam->setParameter("Rotate180", config.flip_image_vertical);
}

static void lostCameraControlCallback(ArvGvDevice *gv_device)
{
	mcr::NodeDiagnostic::error(DIAGNOSTICS_ID, mcr::NodeDiagnostic::COMMUNICATION, "Lost connection to the camera");
}

static void imageCallback(ArvStream *stream, ApplicationData *data)
{
	ArvBuffer *buffer;

	buffer = arv_stream_try_pop_buffer(stream);
	if (buffer != NULL)
	{
		if (buffer->status == ARV_BUFFER_STATUS_SUCCESS)
		{
			sensor_msgs::Image ros_img;
			int arv_row_stride;
			size_t size;

			arv_row_stride = buffer->width * ARV_PIXEL_FORMAT_BIT_PER_PIXEL(buffer->pixel_format) / 8;
			size = buffer->height * arv_row_stride;

			ros_img.data.resize(size);

			ros_img.header.frame_id = frame_id;
			ros_img.header.stamp = ros::Time::now();
			ros_img.height = buffer->height;
			ros_img.width = buffer->width;
			ros_img.encoding = sensor_msgs::image_encodings::RGB8;
			ros_img.step = buffer->width;

			ros_img.data.assign(static_cast<unsigned char *>(buffer->data), (static_cast<unsigned char *>(buffer->data)) + buffer->size);

			pub->publish(ros_img);
		}

		arv_stream_push_buffer(stream, buffer);
	}
}

int main(int argc, char **argv)
{
	int curr_num_subscriber = 0;
	int prev_num_subscriber = 0;

	// init ROS node with a name and a node handle
	ros::init(argc, argv, "mcr_aravis_vision");
	ros::NodeHandle nh("~");

	// create a Publisher
	image_transport::ImageTransport img_transport(nh);
	image_transport::Publisher pub_img_raw = img_transport.advertise("image", 1);

	pub = &pub_img_raw;

	// create velocity and position subscribe for the zoom
	ros::Subscriber sub_zoom_velocity = nh.subscribe("zoom/velocity", 1, zoomVelocityCommandCallback);
	ros::Subscriber sub_zoom_position = nh.subscribe("zoom/position", 1, zoomPositionCommandCallback);

	// set callback function for dynamic reconfigure
	dynamic_reconfigure::Server < mcr_aravis_vision::AravisVisionConfig > dyn_recfg_server;
	dyn_recfg_server.setCallback(boost::bind(dynamicReconfigureCallback, _1, _2));

	// establish connection to camera
	connectToCamera();

	// set default parameter
	setDefaultParameter(nh);

	// register callbacks for image retrival and loss of control
	gige_cam->registerImageCallback(imageCallback);
	gige_cam->registerLostControlCallback(lostCameraControlCallback);

	// start and stop image acquisition based on the number of subscriber
	ros::Rate loop_rate(30);
	while (ros::ok())
	{
		// process callbacks
		ros::spinOnce();

		// get number of current subscribers
		curr_num_subscriber = pub_img_raw.getNumSubscribers();

		// check if someone subscribed
		if (curr_num_subscriber >= 1 && prev_num_subscriber == 0)
		{
			gige_cam->startAcquisition();
			ROS_DEBUG("image acquisition started");
			mcr::NodeDiagnostic::ok(DIAGNOSTICS_ID, mcr::NodeDiagnostic::COMMUNICATION, "image acquisition started");
		}
		// check if the last subscriber unsubscribed
		else if (pub_img_raw.getNumSubscribers() == 0 && prev_num_subscriber >= 1)
		{
			gige_cam->stopAcquisition();
			ROS_DEBUG("image acquisition stopped");
			mcr::NodeDiagnostic::warn(DIAGNOSTICS_ID, mcr::NodeDiagnostic::COMMUNICATION, "image acquisition stopped");
		}
		prev_num_subscriber = curr_num_subscriber;

		// sleep if running faster then the desired frequency
		loop_rate.sleep();
	}

	// clean up
	gige_cam->disconnect();
	pub_img_raw.shutdown();
	mcr::NodeDiagnostic::reset(DIAGNOSTICS_ID);

	return 0;
}
