#pragma once
#include <cv.h>
#include <highgui.h>

class Window
{
private:
	cv::Point2d windowPosition;
	const string m_csWindowName;
	int m_iWindowFlag;
public:
	Window(const string& windowName,int flags);
	~Window(void);

	static void addPointWithMouse(int event, int x, int y, int flags, void* pointToAdd);
	const string getWindowName () const {return m_csWindowName;}
	void showImage (cv::Mat& imgToShow);
	//void setAddPointCallback (void* params);
};

