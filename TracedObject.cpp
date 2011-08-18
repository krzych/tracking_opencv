#include "TracedObject.h"


TracedObject::TracedObject(void)
{
}

TracedObject::TracedObject(const string& tracedObjectFilename)
{
    char* temp;
	temp=new char [tracedObjectFilename.size()+1];
	Error loadError(2,"Problems with loading traced object file\n");
	strcpy(temp,tracedObjectFilename.c_str()); //const char* needed in function
	if(m_iplTracedObject=cvLoadImage(temp,1))
		tracedObject=m_iplTracedObject; //IplImage header for matrix operations needed for mixing OpenCv od and new API
	else 
		throw loadError;
}

TracedObject::~TracedObject(void)
{
}


