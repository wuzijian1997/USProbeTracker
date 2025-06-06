#include "USVideoStreaming.h"
#include <spdlog/spdlog.h>
#include "configs.hpp"

//****************Setup Methods******************
USVideoStreaming::USVideoStreaming(bool show_stream, int timeout)
	: _showStream(show_stream),
	_windowHandle(nullptr),
	_windowWidth(0),
	_windowHeight(0),
	_windowWidth_original(0),
	_windowHeight_original(0),
	_screenX(0),
	_screenY(0),
	_hwindowDC(nullptr),
	_hwindowCompatibleDC(nullptr),
	_hbwindow(nullptr),
	_bi{},
	_timeout(timeout)
{
    const std::string winName = gCoreConfig.usWindowDisplayName;
    spdlog::info("Initializing USVideoStreaming with window name: {}", winName);
	_windowHandle = FindWindow(0, winName.c_str());
	if (!_windowHandle)
	{
		spdlog::warn("Error: Could Not find window with name: {}", winName.c_str());
		//Sets recording and showing booleans to false
		_showStream = false;
		return;
	}

	//Gets handles of device context
	_hwindowDC = GetDC(_windowHandle);
	_hwindowCompatibleDC = CreateCompatibleDC(_hwindowDC);
	//SetStretchBltMode(_hwindowCompatibleDC, HALFTONE);

	//Gets the screen area that we are streaming
	RECT rect;
	GetClientRect(_windowHandle, &rect);
	//GetWindowRect(_windowHandle, &rect);
	//AdjustWindowRectEx(&rect, WS_OVERLAPPEDWINDOW, FALSE, 0);
	POINT topLeft = { rect.left, rect.top };
	ClientToScreen(_windowHandle, &topLeft);

	_windowWidth_original = rect.right - rect.left;
	_windowHeight_original = rect.bottom - rect.top;
	/*_screenX = rect.left;
	_screenY = rect.top;*/
	_screenX = topLeft.x;
	_screenY = topLeft.y;


	//Scaling the image to match scaling of screen:
	UINT dpi = GetDpiForWindow(_windowHandle);
	float scaleFactor = dpi / 96.0f;
	_windowWidth = static_cast<int>(_windowWidth_original * scaleFactor);
	_windowHeight = static_cast<int>(_windowHeight_original * scaleFactor);
	//_screenX = static_cast<int>(_screenX * scaleFactor);
	//_screenY = static_cast<int>(_screenY * scaleFactor);

	std::cout << "window width: " << _windowWidth << std::endl;
	std::cout << "window height: " << _windowHeight << std::endl;
	std::cout << "scale factor: " << scaleFactor << std::endl;

	//Inits the cv Mat frame that we read data into
	USFrameScaled.create(_windowHeight, _windowWidth, CV_8UC4); //Where bits are intially read to
	USFrame.create(_windowHeight_original, _windowWidth_original, CV_8UC3); //Frame that is stored with actual size

	//Creates the bitmap object
	_hbwindow = CreateCompatibleBitmap(_hwindowDC, _windowWidth, _windowHeight);
	createBitmapHeader(_windowWidth, _windowHeight); //Gets the bitmap header (_bi)

	//Use the previously created device context with the bitmap
	SelectObject(_hwindowCompatibleDC, _hbwindow);

	//Captures the screen to a bitmap
	//BitBlt(_hwindowCompatibleDC, 0, 0, _windowWidth, _windowHeight, _hwindowDC, 0, 0, SRCCOPY);

	_showStream = show_stream;
	//Inits the opencv window
	if (_showStream) //If it is true, we create the window
	{
		cv::namedWindow(USSTREAMDISPLAYNAME, cv::WINDOW_NORMAL);
		cv::resizeWindow(USSTREAMDISPLAYNAME, _windowWidth_original, _windowHeight_original);
	}


	//Start the thread
	_runFrame_Thread = true;
	_frame_Thread = std::make_shared<std::thread>([this]() {ReadFrames(); });



}

//Destroying objects to protect against memory leaks
USVideoStreaming::~USVideoStreaming()
{
	{
		std::lock_guard<std::mutex> lock(_frameMutex);
		_runFrame_Thread = false;
	}

	if (_frame_Thread && _frame_Thread->joinable())
	{
		_frame_Thread->join();
	}

	//Deletes the streaming objects from window API
	DeleteObject(_hbwindow);
	DeleteDC(_hwindowCompatibleDC);
	ReleaseDC(_windowHandle, _hwindowDC);
	cv::destroyWindow(USSTREAMDISPLAYNAME);


	while (!_frameQueue.empty()) {
		_frameQueue.pop();
	}

}


void USVideoStreaming::createBitmapHeader(int width, int height)
{
	// create a bitmap
	_bi.biSize = sizeof(BITMAPINFOHEADER);
	_bi.biWidth = width;
	_bi.biHeight = -height;  //this is the line that makes it draw upside down or not
	_bi.biPlanes = 1;
	_bi.biBitCount = 32;
	_bi.biCompression = BI_RGB;
	_bi.biSizeImage = width*height*4;
	_bi.biXPelsPerMeter = 3780;
	_bi.biYPelsPerMeter = 3780;
	_bi.biClrUsed = 0;
	_bi.biClrImportant = 0;


}


//******************Methods to Read US Frames*********************
void USVideoStreaming::ReadFrames()
{
	while (_runFrame_Thread)
	{

		//Gets the new frame in "USFrame"

		//StretchBlt(_hwindowCompatibleDC, 0, 0, _windowWidth, _windowHeight, _hwindowDC, 0, 0, _windowWidth, _windowHeight, SRCCOPY);
		BitBlt(_hwindowCompatibleDC, 0, 0, _windowWidth, _windowHeight, _hwindowDC, 0, 0, SRCCOPY);
		GetDIBits(_hwindowCompatibleDC, _hbwindow, 0, _windowHeight, USFrameScaled.data, (BITMAPINFO*)&_bi, DIB_RGB_COLORS);
		cv::resize(USFrameScaled, USFrame, cv::Size(_windowWidth_original, _windowHeight_original), 0, 0, cv::INTER_CUBIC);

		//Updates the frame queue and locks the mutex before writing to the queue and notifying
		//cv::Mat NewFrame = USFrame.clone();

		{
			std::lock_guard<std::mutex> l{ _frameMutex };
			//Clears the queue before pushing the newest reading (we only have one reading in the queue at a time)
			while (!_frameQueue.empty()) {
				_frameQueue.pop();
			}
			_frameQueue.push(USFrame.clone());
		}

		//Notifies recipient
		_frameArrivedVar.notify_one();


	}

}
// Gets most recent frame from the thread
cv::Mat USVideoStreaming::getFrame()
{
	cv::Mat currFrame;
	std::unique_lock<std::mutex> lock{ _frameMutex };
	//Returns empty frame if waiting longer than 20 ms (50 Hz)
	if (!_frameArrivedVar.wait_for(lock, std::chrono::milliseconds(_timeout), [this]() { return !_frameQueue.empty(); }))
	{
		//We had a timeout event
		return currFrame;
	}
	if (!_frameQueue.empty())
	{
		//Grab frame if not empty queue
		currFrame = _frameQueue.front();
		_frameQueue.pop();
	}
	lock.unlock();

	return currFrame;
}

bool USVideoStreaming::showFrame()
{
	bool returnBool = true;
	//Shows the frame if we want to
	if (_showStream)
	{

		cv::imshow(USSTREAMDISPLAYNAME, USFrame); //Shows the frame
		if (cv::waitKey(1) == 27) { // Press 'Esc' to exit
			returnBool=false;
		}

	}

	return returnBool;

}
