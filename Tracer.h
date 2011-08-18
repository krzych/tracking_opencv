#pragma once
#include <cv.h>
#include <highgui.h>
#include "TracedObject.h"
#include "Window.h"

class Tracer
{
private:
	TracedObject* tracedObject;
	Window* window;
	cv::Mat templateMatchEffect;
	cv::Mat imageToSearch;
	cv::Mat imageWithSearchEffect;
	cv::Mat grayImage; //imageToSearchGray
	cv::Mat prevGrayImage; //previous frame gray needed to compare
	vector<cv::Point2f> points[2];//points[0] vector of points for which flow needs to be found
							      //points[1] output points with calcuated new positions of input features
	pair<cv::Point2f,bool> addedPointToTrack;
	CvMemStorage* storageForFo;
	
	bool lKneedToInit;
	bool lKneedToClear;
	bool useFlann;

public:
	Tracer(void);
	~Tracer(void);

	void signWithTracedObject(TracedObject& trobj);
	void signWithWindow (Window& win);
	cv::Mat& setImageToSearch() {return imageToSearch;} 
	cv::Mat getImageWithSearchEffect() {return imageWithSearchEffect;}
	void templateMatch();
	void setMouseToLK();
	void lucasKanade();
	vector<int> findObject(int hessianTreshold=500);
	double compareSurfDescriptors(const float* d1, const float* d2, double best, int length);
	int naiveNearestNeighbor( const float* vec, int laplacian,const CvSeq* model_keypoints,
                              const CvSeq* model_descriptors );
	void findPairs( const CvSeq* objectKeypoints, const CvSeq* objectDescriptors,
                            const CvSeq* imageKeypoints, const CvSeq* imageDescriptors, vector<int>& ptpairs);
	void flannFindPairs( const CvSeq*, const CvSeq* objectDescriptors,
                         const CvSeq*, const CvSeq* imageDescriptors, vector<int>& ptpairs);
	int locatePlanarObject( const CvSeq* objectKeypoints, const CvSeq* objectDescriptors,
                            const CvSeq* imageKeypoints, const CvSeq* imageDescriptors,
                            const CvPoint src_corners[4], CvPoint dst_corners[4],bool useFlann=1 );
	bool& setLkNeedToInit() {return lKneedToInit;}
	bool getLkNeedToInit () const {return lKneedToInit;}
	bool& setLkNeedToClear() {return lKneedToClear;}
	bool getLkNeedToClear() const {return lKneedToClear;}
	bool& setUseFlann() {return useFlann;}
	bool getUseFlann() const {return useFlann;}

};

