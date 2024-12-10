#pragma once
#include "SetupAndSegment.h"
#include <Windows.h>
#include <condition_variable>
#include <queue>

//To Do: Allow Dyanmic Changes of scrcpy window
// Also check if current frame is different than previous and update only when change

//*********Constants************
const std::wstring WINDOWDISPLAYNAME = L"SM-T870"; //Name of window that scrcpy is writing to
const std::string USSTREAMDISPLAYNAME = "Ultrasound Stream"; //Name of window that this application writes to


class USVideoStreaming
{
public:

	//Class Vars
	bool _showStream = false; //Whether we are printing the stream to an OpenCV window or not (this window can be shown to the patient instead of the scrcpy window)
	HWND _windowHandle;
	int _windowWidth = 0;
	int _windowHeight=0;
	int _windowWidth_original = 0;
	int _windowHeight_original = 0;
	int _screenX = 0;
	int _screenY = 0;
	HDC _hwindowDC, _hwindowCompatibleDC; // handle to device content
	HBITMAP _hbwindow; //The bitmap object
	BITMAPINFOHEADER  _bi;

	//USFrame is what we read
	cv::Mat USFrameScaled,USFrame; //USFrameScaled is dpi scaled, then USFrame is rescaled to match the proper display size
	
	

	//Set up the streaming of the US video
	//Create the streaming object
	explicit USVideoStreaming(bool show_stream); //To Do: Allow Dynamic size changes on the scrcpy window
	~USVideoStreaming();

	

	//Methods to check if a new frame has arrived and to get that frame
	//bool hasNewFrame();
	cv::Mat getFrame(); //Gets the frame
	bool showFrame(); //Method to show the frame

private:
	void createBitmapHeader(int width, int height);
	//Method to wait for new frame arrival, and convert the data to a cv::Mat object
	void ReadFrames();

	//Threading Setup
	//This is queue that is used to push frames to in the thread
	std::queue<cv::Mat> _frameQueue;
	std::mutex _frameMutex;
	std::condition_variable _frameArrivedVar;
	std::shared_ptr<std::thread> _frame_Thread;
	std::atomic<bool> _runFrame_Thread=true;
	//std::atomic<bool> _hasNew_Frame=false; //Do we have a new frame


};

