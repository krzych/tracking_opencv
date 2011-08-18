#pragma once
#include <cv.h>
#include <highgui.h>
#include "Error.h"

class TracedObject
{
private:
	cv::Mat tracedObject;
	IplImage* m_iplTracedObject;
	const string m_csTracedObjectFilename;//path

	//points to track, contours etc.
public:
	TracedObject();
	TracedObject(const string& tracedObjectFilename) throw (Error);
	~TracedObject(void);

	cv::Mat getTracedObject() {return tracedObject;}
	IplImage* getTracedObjectIpl() {return m_iplTracedObject;}
};

