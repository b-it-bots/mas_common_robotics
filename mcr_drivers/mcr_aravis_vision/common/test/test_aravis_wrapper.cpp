#include <gtest/gtest.h>

#include "mcr_aravis_vision/aravis_wrapper.h"

static void lostCameraControlCallback(ArvGvDevice *gv_device)
{
}

static void imageCallback(ArvStream *stream, ApplicationData *data)
{
}

TEST(AravisWrapperTest, connect)
{
	AravisWrapper *av_wrapper = new AravisWrapper();

	EXPECT_EQ(av_wrapper->connect(), ConnectionErrorCodes::NO_CAMERA_FOUND);
	EXPECT_EQ(av_wrapper->connect("xyz"), ConnectionErrorCodes::NO_CAMERA_FOUND);
}

TEST(AravisWrapperTest, disconnect)
{
	AravisWrapper *av_wrapper = new AravisWrapper();

	EXPECT_EQ(av_wrapper->disconnect(), DisconnectionErrorCodes::DISCONNECT_FAILED);
}

TEST(AravisWrapperTest, setParameter)
{
	AravisWrapper *av_wrapper = new AravisWrapper();

	EXPECT_EQ(av_wrapper->setParameter("ZoomControlMode", "ZoomTarget"), false);
	EXPECT_EQ(av_wrapper->setParameter("ZoomPosition", 0), false);
	EXPECT_EQ(av_wrapper->setParameter("ZoomPlus"), false);
}

TEST(AravisWrapperTest, getCameraDescription)
{
	AravisWrapper *av_wrapper = new AravisWrapper();

	EXPECT_EQ(av_wrapper->getCameraDescription(), "No camera connected");
}

TEST(AravisWrapperTest, registerCallback)
{
	AravisWrapper *av_wrapper = new AravisWrapper();

	EXPECT_EQ(av_wrapper->registerImageCallback(imageCallback), false);
	EXPECT_EQ(av_wrapper->registerLostControlCallback(lostCameraControlCallback), false);
}

TEST(AravisWrapperTest, switchAcquisitionMode)
{
	AravisWrapper *av_wrapper = new AravisWrapper();

	EXPECT_EQ(av_wrapper->startAcquisition(), false);
	EXPECT_EQ(av_wrapper->stopAcquisition(), false);
}

int main(int argc, char **argv)
{
	testing::InitGoogleTest(&argc, argv);

	return RUN_ALL_TESTS();
}
