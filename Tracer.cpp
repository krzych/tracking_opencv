#include "Tracer.h"
#include <iostream>

using namespace cv;

Tracer::Tracer(void)
	:lKneedToInit(0),lKneedToClear(0),useFlann(0)
{
	storageForFo=cvCreateMemStorage(0);
}


Tracer::~Tracer(void)
{
}

void Tracer::signWithTracedObject(TracedObject& trobj)
{
	if(&trobj)
		tracedObject = &trobj;
}

void Tracer::signWithWindow(Window& win)
{
	window=&win;
}

void Tracer::templateMatch()
{
	double minVal, maxVal;
	CvPoint minLoc, maxLoc;
	IplImage* templatematch=cvCreateImage(cv::Size(imageToSearch.cols-tracedObject->getTracedObject().cols+1,
							imageToSearch.rows- tracedObject->getTracedObject().rows+1),IPL_DEPTH_32F,1);
	templateMatchEffect=templatematch;
	matchTemplate(imageToSearch,tracedObject->getTracedObject(),templateMatchEffect,CV_TM_SQDIFF_NORMED);//correct assured by loading
																										 //other errors catched by OpenCV
	cvMinMaxLoc(templatematch, &minVal, &maxVal, &minLoc, &maxLoc, 0);
	imageWithSearchEffect=imageToSearch;
	rectangle(imageWithSearchEffect,Point(minLoc.x,minLoc.y),Point(minLoc.x+tracedObject->getTracedObject().cols,
			  minLoc.y+tracedObject->getTracedObject().rows),CV_RGB(255,0,0),1,0,0);
}

void Tracer::lucasKanade()
{
	TermCriteria termcrit(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,20,0.03); //termination criteria max iter=20, precision=0.03
	Size winSize(10,10);// size of search window at each pyramide level
	const int MAX_COUNT = 500;
	imageToSearch.copyTo(imageWithSearchEffect);
	cvtColor(imageWithSearchEffect, grayImage, CV_BGR2GRAY);
	if( lKneedToInit )
        {
            // automatic initialization
            goodFeaturesToTrack(grayImage, points[1], MAX_COUNT, 0.01, 10, Mat(), 3, 0, 0.04);
            cornerSubPix(grayImage, points[1], winSize, Size(-1,-1), termcrit);
			addedPointToTrack.second=false;
			
			
        }
	else if( !points[0].empty() )
        {
            vector<uchar> status; //1 if flow for corresponding feature found, 0 otherwise
            vector<float> err; //difference between patches around orginal and moved points
            if(prevGrayImage.empty())
                grayImage.copyTo(prevGrayImage);
            calcOpticalFlowPyrLK(prevGrayImage, grayImage, points[0], points[1], status, err, winSize,
                                 3, termcrit, 0);
            size_t i, k;
            for( i = k = 0; i < points[1].size(); i++ )
            {
				if(addedPointToTrack.second)
                {
					if( norm(addedPointToTrack.first - points[1][i]) <= 5 )
                    {
						addedPointToTrack.second=false;
                        continue;
                    }
                }

                if( !status[i] )
                    continue;

                points[1][k++] = points[1][i];
                circle( imageWithSearchEffect, points[1][i], 3, Scalar(0,255,0), -1, 8);
            }
            points[1].resize(k);
        }
		if( addedPointToTrack.second && points[1].size() < (size_t)MAX_COUNT )
        {
            vector<Point2f> tmp;
			tmp.push_back(addedPointToTrack.first);
            cornerSubPix( grayImage, tmp, winSize, cvSize(-1,-1), termcrit);
            points[1].push_back(tmp[0]);
			addedPointToTrack.second=false;
        }
        lKneedToInit = false;
		if(lKneedToClear)
		{
			points[1].clear();
			points[0].clear();
			lKneedToClear=false;
		}
		std::swap(points[1], points[0]);
        swap(prevGrayImage, grayImage);
		
}

void Tracer::setMouseToLK()
{
	addedPointToTrack.second=false;
	setMouseCallback(window->getWindowName(),Window::addPointWithMouse,&addedPointToTrack);
}

double Tracer::compareSurfDescriptors( const float* d1, const float* d2, double best, int length )
{
    double total_cost = 0;
    assert( length % 4 == 0 );
    for( int i = 0; i < length; i += 4 )
    {
        double t0 = d1[i] - d2[i];
        double t1 = d1[i+1] - d2[i+1];
        double t2 = d1[i+2] - d2[i+2];
        double t3 = d1[i+3] - d2[i+3];
        total_cost += t0*t0 + t1*t1 + t2*t2 + t3*t3;
        if( total_cost > best )
            break;
    }
    return total_cost;
}	

int Tracer::naiveNearestNeighbor( const float* vec, int laplacian,const CvSeq* model_keypoints,
                          const CvSeq* model_descriptors )
{
	int length = (int)(model_descriptors->elem_size/sizeof(float));
    int i, neighbor = -1;
    double d, dist1 = 1e6, dist2 = 1e6;
    CvSeqReader reader, kreader;
    cvStartReadSeq( model_keypoints, &kreader, 0 );
    cvStartReadSeq( model_descriptors, &reader, 0 );

    for( i = 0; i < model_descriptors->total; i++ )
    {
        const CvSURFPoint* kp = (const CvSURFPoint*)kreader.ptr;
        const float* mvec = (const float*)reader.ptr;
    	CV_NEXT_SEQ_ELEM( kreader.seq->elem_size, kreader );
        CV_NEXT_SEQ_ELEM( reader.seq->elem_size, reader );
        if( laplacian != kp->laplacian )
            continue;
        d = compareSurfDescriptors( vec, mvec, dist2, length );
        if( d < dist1 )
        {
            dist2 = dist1;
            dist1 = d;
            neighbor = i;
        }
        else if ( d < dist2 )
            dist2 = d;
    }
    if ( dist1 < 0.6*dist2 )
        return neighbor;
    return -1;
}

void Tracer::findPairs( const CvSeq* objectKeypoints, const CvSeq* objectDescriptors,
                const CvSeq* imageKeypoints, const CvSeq* imageDescriptors, vector<int>& ptpairs )
{
    int i;
    CvSeqReader reader, kreader;
    cvStartReadSeq( objectKeypoints, &kreader );
    cvStartReadSeq( objectDescriptors, &reader );
    ptpairs.clear();

    for( i = 0; i < objectDescriptors->total; i++ )
    {
        const CvSURFPoint* kp = (const CvSURFPoint*)kreader.ptr;
        const float* descriptor = (const float*)reader.ptr;
        CV_NEXT_SEQ_ELEM( kreader.seq->elem_size, kreader );
        CV_NEXT_SEQ_ELEM( reader.seq->elem_size, reader );
        int nearest_neighbor = naiveNearestNeighbor( descriptor, kp->laplacian, imageKeypoints, imageDescriptors );
        if( nearest_neighbor >= 0 )
        {
            ptpairs.push_back(i);
            ptpairs.push_back(nearest_neighbor);
        }
    }
}

void Tracer::flannFindPairs( const CvSeq*, const CvSeq* objectDescriptors,
                     const CvSeq*, const CvSeq* imageDescriptors, vector<int>& ptpairs )
{
	int length = (int)(objectDescriptors->elem_size/sizeof(float));

    cv::Mat m_object(objectDescriptors->total, length, CV_32F);
	cv::Mat m_image(imageDescriptors->total, length, CV_32F);


	// copy descriptors
    CvSeqReader obj_reader;
	float* obj_ptr = m_object.ptr<float>(0);
    cvStartReadSeq( objectDescriptors, &obj_reader );
    for(int i = 0; i < objectDescriptors->total; i++ )
    {
        const float* descriptor = (const float*)obj_reader.ptr;
        CV_NEXT_SEQ_ELEM( obj_reader.seq->elem_size, obj_reader );
        memcpy(obj_ptr, descriptor, length*sizeof(float));
        obj_ptr += length;
    }
    CvSeqReader img_reader;
	float* img_ptr = m_image.ptr<float>(0);
    cvStartReadSeq( imageDescriptors, &img_reader );
    for(int i = 0; i < imageDescriptors->total; i++ )
    {
        const float* descriptor = (const float*)img_reader.ptr;
        CV_NEXT_SEQ_ELEM( img_reader.seq->elem_size, img_reader );
        memcpy(img_ptr, descriptor, length*sizeof(float));
        img_ptr += length;
    }

    // find nearest neighbors using FLANN
    cv::Mat m_indices(objectDescriptors->total, 2, CV_32S);
    cv::Mat m_dists(objectDescriptors->total, 2, CV_32F);
    cv::flann::Index flann_index(m_image, cv::flann::KDTreeIndexParams(4));  // using 4 randomized kdtrees
    flann_index.knnSearch(m_object, m_indices, m_dists, 2, cv::flann::SearchParams(64) ); // maximum number of leafs checked

    int* indices_ptr = m_indices.ptr<int>(0);
    float* dists_ptr = m_dists.ptr<float>(0);
    for (int i=0;i<m_indices.rows;++i) {
    	if (dists_ptr[2*i]<0.6*dists_ptr[2*i+1]) {
    		ptpairs.push_back(i);
    		ptpairs.push_back(indices_ptr[2*i]);
    	}
    }
}

int Tracer::locatePlanarObject( const CvSeq* objectKeypoints, const CvSeq* objectDescriptors,
                        const CvSeq* imageKeypoints, const CvSeq* imageDescriptors,
                        const CvPoint src_corners[4], CvPoint dst_corners[4],bool useFlann )
{
    double h[9];
    CvMat _h = cvMat(3, 3, CV_64F, h);
    vector<int> ptpairs;
    vector<CvPoint2D32f> pt1, pt2;
    CvMat _pt1, _pt2;
    int i, n;

if(useFlann)
    flannFindPairs( objectKeypoints, objectDescriptors, imageKeypoints, imageDescriptors, ptpairs );
else
    findPairs( objectKeypoints, objectDescriptors, imageKeypoints, imageDescriptors, ptpairs );

    n = (int)(ptpairs.size()/2);
    if( n < 4 )
        return 0;

    pt1.resize(n);
    pt2.resize(n);
    for( i = 0; i < n; i++ )
    {
        pt1[i] = ((CvSURFPoint*)cvGetSeqElem(objectKeypoints,ptpairs[i*2]))->pt;
        pt2[i] = ((CvSURFPoint*)cvGetSeqElem(imageKeypoints,ptpairs[i*2+1]))->pt;
    }

    _pt1 = cvMat(1, n, CV_32FC2, &pt1[0] );
    _pt2 = cvMat(1, n, CV_32FC2, &pt2[0] );
    if( !cvFindHomography( &_pt1, &_pt2, &_h, CV_RANSAC, 5 ))
        return 0;

    for( i = 0; i < 4; i++ )
    {
        double x = src_corners[i].x, y = src_corners[i].y;
        double Z = 1./(h[6]*x + h[7]*y + h[8]);
        double X = (h[0]*x + h[1]*y + h[2])*Z;
        double Y = (h[3]*x + h[4]*y + h[5])*Z;
        dst_corners[i] = cvPoint(cvRound(X), cvRound(Y));
    }

    return 1;
}

vector<int> Tracer::findObject(int hessianTreshold)
{
	cvtColor(imageToSearch, grayImage, CV_BGR2GRAY);
	CvSeq *objectKeypoints = 0, *objectDescriptors = 0;
	CvSeq *imageKeypoints = 0, *imageDescriptors = 0;
	vector<int> parameters; //parameters[1]->object descriptors, parameters[2]->frame descriptors, parameters[3]->extraction time
	int i;
	CvSURFParams params = cvSURFParams(hessianTreshold, 1);
	double tt = (double)cvGetTickCount();//time measure
	IplImage* objectGray=cvCreateImage(cvGetSize(tracedObject->getTracedObjectIpl()),8,0);
	cvCvtColor(tracedObject->getTracedObjectIpl(),objectGray,CV_BGR2GRAY);
	cvExtractSURF( objectGray, 0, &objectKeypoints, &objectDescriptors, storageForFo, params );
	parameters.push_back(objectDescriptors->total);
	//grayimage to ipl conver needed
	IplImage grayIplImage=grayImage;//may be problems
	cvExtractSURF( &grayIplImage, 0, &imageKeypoints, &imageDescriptors, storageForFo, params );
	parameters.push_back(imageDescriptors->total);
	tt=(double)cvGetTickCount()-tt;
	parameters.push_back(tt/(cvGetTickFrequency()*1000.));

	CvPoint src_corners[4] = {{0,0}, {objectGray->width,0}, {objectGray->width, objectGray->height}, {0, objectGray->height}};
	CvPoint dst_corners[4];
	IplImage* correspond = cvCreateImage( cvSize(grayIplImage.width, grayIplImage.height + objectGray->height), grayIplImage.depth,0);
	cvSetImageROI( correspond, cvRect( 0, 0, objectGray->width, objectGray->height ) );
	cvCopy( objectGray, correspond );
	cvSetImageROI( correspond, cvRect( 0, objectGray->height, correspond->width, correspond->height ) );
	cvCopy( &grayIplImage, correspond );
	cvResetImageROI( correspond );
    
	if( locatePlanarObject( objectKeypoints, objectDescriptors, imageKeypoints,
								 imageDescriptors, src_corners, dst_corners,useFlann ))
			 {
			   for( i = 0; i < 4; i++ )
				   {
				     CvPoint r1 = dst_corners[i%4];
				     CvPoint r2 = dst_corners[(i+1)%4];
				     cvLine( correspond, cvPoint(r1.x, r1.y+objectGray->height ),
			         cvPoint(r2.x, r2.y+objectGray->height ), Scalar(0,0,255) );
					}
			 }
	vector<int> ptpairs;
	if(useFlann)
		flannFindPairs( objectKeypoints, objectDescriptors, imageKeypoints, imageDescriptors, ptpairs );
	else 
		findPairs( objectKeypoints, objectDescriptors, imageKeypoints, imageDescriptors, ptpairs );
	
	imageWithSearchEffect=correspond;
	return parameters;
}