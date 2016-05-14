/*
 All copyright reserved
 Contributed By Lsxy
 March, 24, 2016
*/
#include "ok_capture.h"
#include <iostream>
#include <string.h>

#define MxW 640
#define MxH 480
LPBYTE lpdib;
BLOCKINFO blk;
static int totalframe = 0;
static int elapsed = 0;

BOOL Convert_to_Cvmat(BLOCKINFO, CvMat* )
{

}

Ok_capture::Ok_capture()
{
    lParam = 0x3;
    OKDEVTYPE *lpOkDevInfo;
    MLONG iIndex = -1;
    hInstLibrary = LoadLibraryA("okapi64");
    if (hInstLibrary == NULL) {
        printf("Error load okapi64.DLL!\n");
        FreeLibrary(hInstLibrary);
    }
    int TotalBoard = okGetImageDevice(&lpOkDevInfo, (LPARAM)&lParam);
    printf("%d\n",TotalBoard);
    hBoard=OpenCard(lpdib, iIndex);
    while (hBoard == 0) {
        printf("Wrong detect OK Board!\n");
        Sleep(2000);
        hBoard = OpenCard(lpdib, iIndex);
    }
    printf("Initial Success!\n");
}

Ok_capture::~Ok_capture()
{
    if (hInstLibrary != NULL) {
        FreeLibrary(hInstLibrary);
    }
    std::cout<<"GOOD BYE!"<<std::endl;
}

long GetTargetSize(HANDLE hBoard, TARGET tgt, short *width, short *height)
{
    RECT	rect;
    long	form;

    SetRect(&rect, 0, 0, 1280, 1024);   //Change the Imagine size
    if ( (tgt == SCREEN) || (tgt == BUFFER) ) {
      //  rect.right=-1;
        okSetTargetRect(hBoard, tgt, &rect); //get current rect
        *width = (short)(rect.right - rect.left);
        *height = (short)(rect.bottom - rect.top);

        if(tgt == SCREEN ) {
            form = okSetCaptureParam(hBoard, CAPTURE_SCRRGBFORMAT, GETCURRPARAM); //-1
            //limit to video rect
            rect.right = -1; //max. captured rect
            okSetTargetRect(hBoard, VIDEO, &rect); //get video rect
            *width = (short)min(*width, rect.right - rect.left);
            *height = (short)min(*height, rect.bottom - rect.top);
        }
        else if(tgt == BUFFER)
            form = okSetCaptureParam(hBoard, CAPTURE_BUFRGBFORMAT, FORM_GRAY8); //-1--------GETCURRPARAM---------------------------------

    } else if (tgt > BUFFER) { //from blkinfo
        LPBLOCKINFO	lpblk;
        lpblk = (LPBLOCKINFO)tgt;

        *width = lpblk->iWidth;
        *height = abs(lpblk->iHeight);
        form = MAKELONG(lpblk->iFormType, lpblk->iBitCount);
    }

    return form;
}

BOOL CALLBACK EndCapture(HANDLE hBoard)
{
    okSetSeqCallback(hBoard, NULL, NULL, NULL);
    return 0;
}

char* GiveNames(MLONG no)
{
    int i, loop = 0, dt[10];
    MLONG frame = no;
    char szFileName[128] = "Image",tail[10] = ".bmp"; //
    
    while (frame > 0) {
        dt[loop] = frame % 10 ;
        frame = frame / 10;
        loop ++;
    }
    for (i=0; i<loop; i++) {
        szFileName[5+i] = dt[loop-i-1] + '0';
    }
    szFileName[loop+5] = '\0';
    strcat(szFileName, tail);
    return szFileName;
}

BOOL CALLBACK Process(HANDLE hBoard, MLONG no)
{

    if ((totalframe >= 12) || ((okGetTickCount() - elapsed) > 2000)&&(totalframe >=2 )) {
        elapsed=okGetTickCount() - elapsed;
        printf("Running time = %lf\n",((float)totalframe*1000/elapsed));
    }
    if (totalframe >= 6) {
        okConvertRect(hBoard, (TARGET)&blk, 0, BUFFER, 0, 1);
        Convert_to_Cvmat(&blk, );
    }
    okSaveImageFile(hBoard, GiveNames(no), 0, BUFFER, no, 1);
    if (totalframe == 0) elapsed = okGetTickCount();
    printf("Save Success %d images!!\n", totalframe);
    totalframe++; //总采集帧数

    return 0;
}

BOOL CALLBACK BeginCapture(HANDLE hBoard)
{
    short	width, height;
    long	blkform, bufform;
    bufform = GetTargetSize(hBoard, BUFFER, &width, &height);
    if ((LOWORD(bufform) == FORM_GRAY10) || (LOWORD(bufform) == FORM_GRAY12)) { //special
     //blkform=okSetCaptureParam(hBoard,CAPTURE_SCRRGBFORMAT,GETCURRPARAM); //-1
        blkform = MAKELONG(FORM_GRAY8, 8);
        }
        else //take same bits as buffer
            blkform = bufform;
    blk.lpBits = lpdib;
    blk.iBitCount = HIWORD(blkform);
    blk.iWidth = width;
    
    if (okSetCaptureParam(hBoard, CAPTURE_SAMPLEFIELD, -1) == 0 ) //sample in field by field
        if (LOWORD(okSetVideoParam(hBoard, VIDEO_SIGNALTYPE, -1)) == 1 ) //interlace video
            height*= 2; //double size for ht

        blk.iHeight = height; //note: minus is for invert dib in y by ConvertRect
        elapsed = GetTickCount();
       // totalframe=0;
        return 1;
}

HANDLE OpenCard(LPBYTE lpdib, MLONG iIndex)
{
    HANDLE hBoard;
//	short	typecode;
    char	string[100];
    //MEMORYSTATUS Status;
    DWORD dwMaxMemSize = 4*MxW*MxH;
    //just starting, then allocated memory for dib
    if(!lpdib) lpdib = (LPBYTE)malloc(dwMaxMemSize);
         else sprintf(string,"Allocate memory failed !");
    //	MessageBox(hwnd,string,"Message",MB_OK);
//    for(int i =0; i<TotalBoard; i++)
//        {
//            printf("Device %d: %s\n", i, lpOkDevInfo[i].szBoardName);
//        }
    hBoard = okOpenBoard(&iIndex);
    return hBoard;
}

int Ok_capture::Capture_single(int frame)
{
    int i, loop = 0, dt[10];
    char szFileName[128] = "Image",tail[10] = ".bmp"; //
    while (frame > 0) {
        dt[loop] = frame % 10 ;
        frame = frame / 10;
        loop ++;
    }
    for (i=0; i<loop; i++) {
        szFileName[5+i] = dt[loop-i-1] + '0';
    }
    szFileName[loop+5] = '\0';
    strcat(szFileName, tail);
    int ret = 0;

    okCaptureSingle(hBoard, BUFFER, 0);

    okConvertRect(hBoard, (TARGET)&blk, 0, BUFFER, 0, 1);

    ret = okGetCaptureStatus(hBoard, 1);

    printf("%d\n", ret);

    printf("Single frame is Finished! \n"); //Success
    okSaveImageFile(hBoard, szFileName, 0, BUFFER, 0, 1);

    okStopCapture(hBoard);
    //okConvertRect()
    //okStopCapture(hBoard);
    return 0;
}

int Ok_capture::Capture_series()
{
    okSetSeqCallback(hBoard, BeginCapture, Process, EndCapture);
    printf("Endcapture! The max frame number is %d! \n",okCaptureTo(hBoard,BUFFER,0,-1)); //连续
    okStopCapture(hBoard);
    //okStopCapture(hBoard);
    return 0;
}

