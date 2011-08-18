#include "Window.h"
#include <utility>


Window::Window(const string& windowName,int flags=1)
	:m_csWindowName(windowName), m_iWindowFlag(flags)
{
	cv::namedWindow(m_csWindowName, m_iWindowFlag);
}


Window::~Window(void)
{
}

void Window::showImage(cv::Mat& imgToShow)
{
	cv::imshow(m_csWindowName, imgToShow);
}

void Window::addPointWithMouse(int event, int x, int y, int flags, void* pointToAdd)
{
	pair<cv::Point2f,bool>* parameters(reinterpret_cast<pair<cv::Point2f,bool>*>(pointToAdd));
	if(event == CV_EVENT_LBUTTONDOWN)
	{
		parameters->first=cv::Point2f((float)x,(float)y);
		parameters->second=true;
	}
}

/*void Window::setAddPointCallback(void* params)
{
	cv::setMouseCallback(m_csWindowName,addPointWithMouse, &params);
}*/