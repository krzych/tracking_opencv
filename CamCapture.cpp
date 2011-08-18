#include "CamCapture.h"

CamCapture::CamCapture(int captureNumber)
{
	capture.open(captureNumber);
	Error initializeError (1,"Problems with initializing camera capture");
	if(!capture.isOpened())
		throw initializeError;
	if(capture.get(CV_CAP_PROP_FPS))
		m_fFps=capture.get(CV_CAP_PROP_FPS);
	else 
		m_fFps=30; //if camera not send properties correctly set m_fFps to 30
	m_iTimeBetweenFrames=1000/m_fFps;
}


CamCapture::~CamCapture(void)
{
}

cv::Mat CamCapture::catchFrame()
{
	capture>>frame;
	return frame;
}


