/*
 * aravis_wrapper.cpp
 *
 *  Created on: Aug 1, 2013
 *      Author: Frederik Hegger
 */

#include "mcr_aravis_vision/aravis_wrapper.h"

AravisWrapper::AravisWrapper()
{
	is_camera_connected_ = false;

	packet_timeout_ = 20;
	frame_retention_ = 100;
	camera_handle_ = NULL;
	stream_handle_ = NULL;
	device_handle_ = NULL;

	arv_g_thread_init (NULL);
	arv_g_type_init();
}

AravisWrapper::~AravisWrapper()
{
	disconnect();
}

int AravisWrapper::connect()
{
	return connect(NULL);
}

int AravisWrapper::connect(const char *camera_name)
{
	gint payload;

	// Connect to camera
	if (is_camera_connected_)
		return ConnectionErrorCodes::ALREADY_CONNECTED;		// camera already connected

	camera_handle_ = arv_camera_new(camera_name);
	if (camera_handle_ == NULL)
		return ConnectionErrorCodes::NO_CAMERA_FOUND;		// could not connect to camera

	// get device handle
	device_handle_ = arv_camera_get_device(camera_handle_);

	// retrieve image payload (number of bytes per image)
	payload = arv_camera_get_payload(camera_handle_);

	// create camera stream
	stream_handle_ = arv_camera_create_stream(camera_handle_, NULL, NULL);
	if (stream_handle_ == NULL)
		return ConnectionErrorCodes::DEVICE_BUSY; 		// Could not create stream thread (check if the device is not already used)

	if (ARV_IS_GV_STREAM(stream_handle_))
	{
		g_object_set(stream_handle_, "socket-buffer", ARV_GV_STREAM_SOCKET_BUFFER_FIXED, "socket-buffer-size", payload * 6, "packet-timeout",
		             packet_timeout_ * 1000, "frame-retention", frame_retention_ * 1000, "packet-resend", ARV_GV_STREAM_PACKET_RESEND_ALWAYS, NULL);
	}

	// Push 50 buffer in the stream input buffer queue
	for (int i = 0; i < 50; i++)
		arv_stream_push_buffer(stream_handle_, arv_buffer_new(payload, NULL));

	// set initial camera properties
	arv_camera_set_pixel_format(camera_handle_, ARV_PIXEL_FORMAT_RGB_8_PACKED);
	arv_camera_set_acquisition_mode(camera_handle_, ARV_ACQUISITION_MODE_CONTINUOUS);

	data_handle_.main_loop = g_main_loop_new(NULL, FALSE);

	is_camera_connected_ = true;

	return ConnectionErrorCodes::CONNECTION_ESTABLISHED;			// connection successfully
}

int AravisWrapper::disconnect()
{
	if (is_camera_connected_)
	{
		arv_camera_stop_acquisition(camera_handle_);
		g_object_unref(stream_handle_);
		g_object_unref(camera_handle_);

		is_camera_connected_ = false;

		return DisconnectionErrorCodes::DISCONNECTED;	// successfully disconnected
	}

	return DisconnectionErrorCodes::DISCONNECT_FAILED;		// there was no camera connected
}

bool AravisWrapper::registerImageCallback(ImageCallback img_cb)
{
	if (stream_handle_ == NULL)
		return false;

	g_signal_connect(stream_handle_, "new-buffer", G_CALLBACK(img_cb), &data_handle_);
	arv_stream_set_emit_signals(stream_handle_, TRUE);

	return true;
}

bool AravisWrapper::registerLostControlCallback(LostControlCallback lost_control_cb)
{
	if (camera_handle_ == NULL)
		return false;

	g_signal_connect(arv_camera_get_device(camera_handle_), "control-lost", G_CALLBACK(lost_control_cb), NULL);

	return true;
}

bool AravisWrapper::setParameter(const std::string &parameter_name, const std::string &value)
{
	if (device_handle_ == NULL)
		return false;

	arv_device_set_string_feature_value(device_handle_, parameter_name.c_str(), value.c_str());

	return true;
}

bool AravisWrapper::setParameter(const std::string &parameter_name, const gint64 &value)
{
	if (device_handle_ == NULL)
		return false;

	arv_device_set_integer_feature_value(device_handle_, parameter_name.c_str(), value);

	return true;
}

bool AravisWrapper::setParameter(const std::string &command)
{
	if (device_handle_ == NULL)
		return false;

	arv_device_execute_command(device_handle_, command.c_str());

	return true;
}

std::string AravisWrapper::getCameraDescription()
{
	if (camera_handle_ == NULL)
		return "No camera connected";

	return std::string(arv_camera_get_vendor_name(camera_handle_)) + " " + arv_camera_get_model_name(camera_handle_) + " ("
	        + arv_camera_get_device_id(camera_handle_) + ")";
}

bool AravisWrapper::startAcquisition()
{
	if (camera_handle_ == NULL)
		return false;

	arv_camera_start_acquisition(camera_handle_);

	return true;
}

bool AravisWrapper::stopAcquisition()
{
	if (camera_handle_ == NULL)
		return false;

	arv_camera_stop_acquisition(camera_handle_);

	return true;
}

