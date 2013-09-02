/*
 * aravis_wrapper.h
 *
 *  Created on: Aug 1, 2013
 *      Author: Frederik hegger
 */

#ifndef ARAVIS_WRAPPER_H_
#define ARAVIS_WRAPPER_H_

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <arv.h>

typedef struct
{
	GMainLoop *main_loop;
	int buffer_count;
} ApplicationData;

typedef void (*ImageCallback)(ArvStream *stream_handle_, ApplicationData *data_handle_);
typedef void (*LostControlCallback)(ArvGvDevice *gv_device);

namespace ConnectionErrorCodes
{
	enum error_codes
	{
		CONNECTION_ESTABLISHED = 0,
		ALREADY_CONNECTED = -1,
		NO_CAMERA_FOUND = -2,
		DEVICE_BUSY = -3,
	};
}
namespace DisconnectionErrorCodes
{
	enum error_codes
	{
		DISCONNECTED = 0,
		DISCONNECT_FAILED = -1
	};
}

class AravisWrapper
{
 public:
	AravisWrapper();
	~AravisWrapper();

	int connect();
	int connect(const char *camera_name);
	int disconnect();

	bool registerImageCallback(ImageCallback img_cb);
	bool registerLostControlCallback(LostControlCallback lost_control_cb);

	bool startAcquisition();
	bool stopAcquisition();

	bool setParameter(const std::string &parameter_name, const std::string &value);
	bool setParameter(const std::string &parameter_name, const gint64 &value);
	bool setParameter(const std::string &command);

	std::string getCameraDescription();

 private:
	bool is_camera_connected_;

	int packet_timeout_;
	int frame_retention_;

	ArvCamera *camera_handle_;
	ArvStream *stream_handle_;
	ArvDevice *device_handle_;

	ApplicationData data_handle_;
};

#endif /* ARAVIS_WRAPPER_H_ */
