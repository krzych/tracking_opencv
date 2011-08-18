#include "View.h"


View::View(void)
{
}


View::~View(void)
{
}

void View::signWithWindow(Window& window)
{
	viewWindow=&window;
}

void View::signWithCapture(CamCapture& cap)
{
	capture=&cap;
}

void View::signWithTracer(Tracer& tra)
{
	tracer=&tra;
}

void View::update()
{
	frame=capture->catchFrame();
	tracer->setImageToSearch()=frame;
	if(processedFrame.total())
		processedFrame=tracer->getImageWithSearchEffect();
	else 
		processedFrame=frame;

}

/*void View::updateFromCamera()
{
	frame=capture->catchFrame();
}*/

void View::show()
{
	viewWindow->showImage(processedFrame);
}


