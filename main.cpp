#include "Window.h"
#include "View.h"
#include "CamCapture.h"
#include "Tracer.h"
#include <iostream>

using std::cout;
using std::endl;
using std::cin;
int main()
{
	int menuControl;
	string fileName;
	vector<int> extractionParameters;
	try
		{
			CamCapture capture;
		}
	catch(Error initError)
		{
			cerr<<"Error occured: "<<initError.getMessage()<<endl;
			exit(initError.getErrorCode());
		}
	CamCapture capture;
	Window traceEffectWindow ("Tracing",1);
	View view;
	Tracer tracer;
	view.signWithWindow(traceEffectWindow);
	view.signWithCapture(capture);
	view.signWithTracer(tracer);
	tracer.signWithWindow(traceEffectWindow);
	//choosing tracking method
	cout<<"Choose tracking method:"<<endl;
	cout<<"1. Template matching"<<endl
		<<"2. Lucas-Kanade"<<endl
		<<"3. Find Object"<<endl;
	cin>>menuControl;
	if(menuControl==1||menuControl==3) //not nice ->need to improve control and menu
		{
			cout<<"Traced object filename:\n";
				cin>>fileName;
				try
					{
						TracedObject objectToTrack(fileName);
					}
				catch (Error loadError)
					{
						cerr<<"Error occured: "<<loadError.getMessage()<<endl;
						exit(loadError.getErrorCode());
					}
				static TracedObject objectToTrack(fileName);
				tracer.signWithTracedObject(objectToTrack);
		}
	
	//tracking
	switch(menuControl)
		{
			case 1:
				
				system("cls");
				while(1)
					{
						view.update();
						tracer.templateMatch();
						view.show();
						if(cv::waitKey(capture.getTimeBetweenFrames())==27)
							break;
					}
				break;
			case 2:
				system("cls");
				tracer.setMouseToLK();
				cout<<"ESC to quit\n"
					<<"a auto initialize tracking\n"
					<<"c delete all points\n"
					<<"to add remove feature ->left mouse button"<<endl;
				while(1)
					{
						view.update();
						tracer.lucasKanade();
						view.show();
						int control=cv::waitKey(capture.getTimeBetweenFrames());
						if(control==27)
							break;
						else if(control=='a')
							tracer.setLkNeedToInit()=true;
						else if(control=='c')
							tracer.setLkNeedToClear()=true;
					}
				break;
			case 3:
				system("cls");
				char useFlann;
				cout<<"Use Flann?(y,n)\n";
				cin>>useFlann;
				if(useFlann=='y')
					tracer.setUseFlann()=true;
				while(1)
					{
						view.update();
						extractionParameters=tracer.findObject();
						cout<<"Object descriptors: "<<extractionParameters[0]<<endl;
						cout<<"Frame descriptors: "<<extractionParameters[1]<<endl;
						cout<<"Extraction time: "<<extractionParameters[2]<<" ms"<<endl;
						cout<<"---"<<endl;
						view.show();
						if(cv::waitKey(capture.getTimeBetweenFrames())==27)
							break;
					}
				break;
			default:
				cerr<<"Incorrect choose!\n";
				break;
	}
}