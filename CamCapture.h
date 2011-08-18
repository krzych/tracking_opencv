#pragma once
#include <cv.h>
#include <highgui.h>
#include "Error.h"

class CamCapture
{
private:
	double m_fFps; //frames per second
	int m_iTimeBetweenFrames;
	cv::Mat frame;
	cv::VideoCapture capture;

public:
	CamCapture(int captureNumber=0) throw(Error); //default cap number 0
	~CamCapture(void);

	cv::Mat catchFrame();
	int getTimeBetweenFrames() {return m_iTimeBetweenFrames;}
	double& setFps () {return m_fFps;}
};

