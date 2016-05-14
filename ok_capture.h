#ifndef OK_CAPTURE_H
#define OK_CAPTURE_H
#include <opencv2/opencv.hpp>
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/videoio.hpp"
#include "opencv/cv.h"
#include "opencv/cxmisc.h"
#include "opencv/highgui.h"
#include <opencv2/calib3d/calib3d.hpp>
#include <Windows.h>
#include <Okapi64.h>

class Ok_capture
{
public:
    Ok_capture();
    ~Ok_capture();
    int Capture_single(int);
    int Capture_series();
    int Callsave();

private:
    HINSTANCE hInstLibrary;
    int	mBufRgbFrom;
    int mAutoGain;
    long lParam;
    HANDLE hBoard;

};

BOOL Convert_to_Cvmat(BLOCKINFO, CvMat);
HANDLE OpenCard(LPBYTE, MLONG );
BOOL CALLBACK EndCapture(HANDLE );
BOOL CALLBACK Process(HANDLE ,MLONG );
BOOL CALLBACK BeginCapture(HANDLE );
long GetBitmapHeader(HANDLE , TARGET , LPBITMAPINFOHEADER );
long SetBitmapHeader(LPBITMAPINFOHEADER ,short, short , short , short);
long GetTargetSize(HANDLE , TARGET , short *, short *);

#endif // OK_CAPTRURE_H
