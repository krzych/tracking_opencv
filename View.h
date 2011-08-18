#pragma once
#include <cv.h>
#include <highgui.h>
#include "Window.h"
#include "CamCapture.h"
#include "Tracer.h"

class View
{
private:
	cv::Mat frame;
	cv::Mat processedFrame;
	Window* viewWindow;
	CamCapture* capture;
	Tracer* tracer;
public:
	View(void);
	~View(void);

	void signWithWindow(Window& window);
	void signWithCapture(CamCapture& cap);
	void signWithTracer(Tracer& tra);
	void update();
	//void updateFromCamera();
	void show();
};

