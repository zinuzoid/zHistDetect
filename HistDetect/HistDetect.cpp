#include "stdafx.h"

#include <opencv2/video/tracking.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <iostream>

#define CAM_X	288
#define CAM_Y	352

void onMouse(int event, int x, int y, int, void*);
void SerialInit(HANDLE *hPort,DCB *dcb);

cv::Point select_origin;
cv::Rect select_rect;
bool select_on=false;

int _tmain(int argc, _TCHAR* argv[])
{
	int
		hmin=175,
		hmax=185,
		smin=65,
		smax=108,
		vmin=136,
		vmax=253;

	cv::VideoCapture cap;

	DCB dcb;

	DWORD byteswritten;

	char a[]="abcdefg";

	HANDLE hPort;
	SerialInit(&hPort,&dcb);
	WriteFile(
		hPort,
		a,	
		sizeof(a),	
		&byteswritten,
		NULL);

	std::cout<<byteswritten;

	CloseHandle(hPort);
	exit(1);

	std::cout << "initial webcam...";
	cap.open(0);
	if(!cap.isOpened())
	{
		std::printf("err init webcam!");
		std::getchar();
		return 0;
	}
	std::cout << "OK\n";

	cv::namedWindow("Raw",cv::WINDOW_AUTOSIZE);
	cv::namedWindow("Histrogram",cv::WINDOW_AUTOSIZE);
	cv::namedWindow("Filtered",cv::WINDOW_AUTOSIZE);
	cv::namedWindow("Setting",cv::WINDOW_AUTOSIZE);
	cv::setMouseCallback("Raw",onMouse,0);
	cv::createTrackbar("Hmin","Setting",&hmin,255,0);
	cv::createTrackbar("Hmax","Setting",&hmax,255,0);
	cv::createTrackbar("Smin","Setting",&smin,255,0);
	cv::createTrackbar("Smax","Setting",&smax,255,0);
	cv::createTrackbar("Vmin","Setting",&vmin,255,0);
	cv::createTrackbar("Vmax","Setting",&vmax,255,0);

	int keycode=-1;
	while(keycode!=27)
	{
		cv::Mat frame,hsv,hist,histshow;
		cv::Mat hue,sat,val;
		histshow=cv::Mat::zeros(CAM_X,CAM_Y,CV_8UC3);
		hue=cv::Mat::zeros(CAM_X,CAM_Y,CV_8UC1);
		sat=cv::Mat::zeros(CAM_X,CAM_Y,CV_8UC1);
		val=cv::Mat::zeros(CAM_X,CAM_Y,CV_8UC1);
		cv::Mat threshold=cv::Mat::zeros(CAM_X,CAM_Y,CV_8UC1);

		double tt=(double)cv::getTickCount();

		cap >> frame;
		if(frame.empty())
		{
			std::cout<<"err!!!";
			cv::waitKey(0);
			break;
		}

		cv::cvtColor(frame,hsv,CV_BGR2HSV);

		//split channels
		cv::Mat out[]={hue,sat,val};
		int channelmapping[]={0,0,1,1,2,2};
		cv::mixChannels(&hsv,3,out,3,channelmapping,3);
		//end split channels

		/*
		frame.adjustROI(
			select_rect.y,
			select_rect.x+select_rect.height,
			select_rect.x,
			select_rect.x+select_rect.width);
		*/

		float histranges[] = {0,180};
		const float* phistranges = histranges;
		int ch[]={0};
		int histdim=64;
		cv::Mat mask;
		cv::Mat roi(hue,select_rect);
		cv::Mat maskroi(mask,select_rect);
		cv::calcHist(&roi,1,0,maskroi,hist,1,&histdim,&phistranges,true,false);

		//threshold
		cv::inRange(hue,hmin,hmax,hue);
		cv::inRange(sat,smin,smax,sat);
		cv::inRange(val,vmin,vmax,val);
		//end threshold

		//and
		cv::bitwise_and(hue,sat,threshold,hue);
		cv::bitwise_and(threshold,val,threshold,val);
		//end and

		//erode&dilate
		cv::erode(threshold,threshold,cv::getStructuringElement(cv::MORPH_RECT,cv::Size(3,3)),cv::Point(-1,-1),1,0);
		cv::dilate(threshold,threshold,cv::getStructuringElement(cv::MORPH_RECT,cv::Size(5,5)),cv::Point(-1,-1),10,0);
		//end erode&dilate

		//find corner
		std::vector<cv::Point2f> eig;
		cv::goodFeaturesToTrack(threshold,eig,50,0.01,10,cv::Mat(),3,false,0.04);
		cv::cornerSubPix(threshold,eig,cv::Size(10,10),cv::Size(-1,-1),cv::TermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,20,0.03));
		cv::Point obj_origin;
		cv::Point obj_corner;
		cv::Rect obj_rect;
		obj_origin.x=(int)eig[0].x;
		obj_origin.y=(int)eig[0].y;
		obj_corner.x=(int)eig[0].x;
		obj_corner.y=(int)eig[0].y;
		for(int i=0;i<(int)eig.size();i++)
		{
			obj_origin.x=MIN(obj_origin.x,(int)eig[i].x);
			obj_origin.y=MIN(obj_origin.y,(int)eig[i].y);
			obj_corner.x=MAX(obj_corner.x,(int)eig[i].x);
			obj_corner.y=MAX(obj_corner.y,(int)eig[i].y);

			cv::circle(frame,eig[i],1,cv::Scalar(255,0,0),1,8,0);
		}
		obj_rect.x=obj_origin.x;
		obj_rect.y=obj_origin.y;
		obj_rect.width=abs(obj_rect.x-obj_corner.x);
		obj_rect.height=abs(obj_rect.y-obj_corner.y);
		//end find corner


		cv::rectangle(frame,obj_rect,cv::Scalar(0,0,255),1,8,0);
		cv::rectangle(frame,select_rect,cv::Scalar(0,255,0),1,8,0);

		cv::imshow("Raw",frame);
		cv::imshow("Filtered",threshold);

		tt=(double)cv::getTickCount()-tt;
		tt=tt/cv::getTickFrequency()*1000;
		std::cout<< "t:" << tt << "ms\n";

		keycode=cv::waitKey(1);
	}

	std::cout << keycode;
	cv::destroyWindow("Raw");
	cv::destroyWindow("Histrogram");
	cv::destroyWindow("Filtered");
	cv::destroyWindow("Setting");

	cv::waitKey(0);
	return 0;
}

void onMouse(int event, int x, int y, int, void*)
{
	std::cout << "x:" << x << " y:" << y << '\n';
	if(select_on)
	{
		select_rect=cv::Rect(
			MIN(select_origin.x,x),
			MIN(select_origin.y,y),
			abs(select_origin.x-x),
			abs(select_origin.y-y));
	}
	switch(event)
	{
	case CV_EVENT_LBUTTONDOWN	:
		select_origin=cv::Point(x,y);
		select_on=true;
		break;
	case CV_EVENT_LBUTTONUP		:
		select_on=false;
		break;
	}
}

void SerialInit(HANDLE *hPort,DCB *dcb)
{
    *hPort=CreateFile(
		_T("COM1"),
		GENERIC_WRITE,
		0,
		NULL,
		OPEN_EXISTING,
		0,
		NULL);

	if(!GetCommState(*hPort,dcb))
	{
		std::cout<< "err comm";
		exit(0);
	}
	dcb->BaudRate=CBR_115200;
	dcb->ByteSize=8;
	dcb->Parity=0;
	dcb->StopBits=0;

    if(!GetCommState(*hPort,dcb))
	{
		std::cout<< "err comm";
		exit(0);
	}
}

