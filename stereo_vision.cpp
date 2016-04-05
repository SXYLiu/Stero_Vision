#include <stereo_vision.h>

void OnMouse(int event, int x, int y, int flags, void* tmp)
{
    CvPoint pt =cvPoint(x, y);
    printf("x=%d,y=%d.\n", x, y);
    printf("Begin:x=%d,y=%d.\n", pt_beg.x, pt_beg.y);
    printf("END:x=%d,y=%d.\n", pt_end.x, pt_end.y);
    //if (!tmp) {return;}
    if ((event == CV_EVENT_LBUTTONUP)&&(flags != CV_EVENT_FLAG_LBUTTON)) {
        prev_pt = cvPoint(-1, -1);
    }
    if (event == CV_EVENT_LBUTTONDOWN) {
        prev_pt = cvPoint(x,y);
        pt_beg = cvPoint(x,y);
     }
    if ((event == CV_EVENT_MOUSEMOVE)&&(flags == CV_EVENT_FLAG_LBUTTON)) {
    //cvCopy(src,tmp); QQQQQQQQQQQQQQQQQQQQQQQQQQQ
        pt1.x = pt.x;
        pt1.y = prev_pt.y;
        pt2.x = prev_pt.x;
        pt2.y = pt.y;
        cvLine( tmp, prev_pt, pt1, cvScalarAll(255), 2, 8, 0 );
        cvLine( tmp, prev_pt, pt2, cvScalarAll(255), 2, 8, 0 );
        cvLine( tmp, pt1, pt, cvScalarAll(255), 2, 8, 0 );
        cvLine( tmp, pt2, pt, cvScalarAll(255), 2, 8, 0 );
        pt_end = pt;
        cvShowImage("SRC", tmp);
    }
}

void Select_an_Area(IplImage *src)
{
    prev_pt = {-1, -1}; pt_beg = {-1, -1};
    pt_end = {-1, -1}; pt1 = {-1, -1}; pt2 = {-1, -1};
    IplImage *dst, *tmp;
    CvRect rect;
    int c;
    cvNamedWindow("SRC", 1);
    cvShowImage("SRC", src);
    tmp = cvCloneImage(src);
    cvSetMouseCallback("SRC", OnMouse, tmp);
    for (;;) {
        c = cvWaitKey(10);
        if (c == 'c') {
            if (pt_beg.x != -1) {
                rect.x = min(pt_beg.x, pt_end.x);
                rect.y = min(pt_beg.y, pt_end.y);
                rect.height = abs(pt_end.y - pt_beg.y);
                rect.width = abs(pt_end.x - pt_beg.x);
                printf("RECT:height=%d,width=%d\n", rect.height, rect.width);
                cvSetImageROI(src, rect);
                printf("ROI:Height=%d,Width=%d.\n", src->roi->height, src->roi->width);
                dst = cvCreateImage(cvSize(rect.width, rect.height), src->depth, src->nChannels);
                dst = cvCloneImage(src);
                printf("DST Height=%d,Width=%d\n", dst->height, dst->width);
                cvNamedWindow("Selected", 1);
                cvShowImage("Selected", dst);
                cvSaveImage("selected.jpg", dst);
                printf("Finished!\n");
                cvWaitKey(0);
                cvDestroyWindow("Selected");
                break;
            }
        }
        if (c == 27) //ESC to exit
        {
            cvReleaseImage(&tmp);
            cvReleaseImage(&dst);
            cvDestroyWindow("SRC");
            break;
        }
    }
   // return rect;
}

CvPoint floods(int y, int x, char * ptr, int lg)
{
    int qux[101],quy[101],tail=1,head=0,temx,temy,t,tot=0;
    qux[head]=x;
    quy[head]=y;
    flags[y][x]=1;
    CvPoint ave=cvPoint(x,y);
    ave.x=0;
    ave.y=0;
    while (tail>head) {
        temx=qux[head];
        temy=quy[head];
        ave.x+=temx;
        ave.y+=temy;
        tot++;
        head++;
        if (tail>=100) {
            ave.x=-1;
            ave.y=-1;
            return ave;
        }
        for (t=0; t<8; t++)
            if (temy+dy[t]>=0&&temy+dy[t]<480&&temx+dx[t]>=0&&temx+dx[t]<640&&
                    !flags[temy+dy[t]][temx+dx[t]]&&ptr[(temy+dy[t])*lg+temx+dx[t]]>=CRT) {//attention maybe not 110 ,according to actual value
            qux[tail]=temx+dx[t];
            quy[tail]=temy+dy[t];
            flags[quy[tail]][qux[tail]]=1;
            tail++;
            }
    }
    ave.x=ave.x/tot;
    ave.y=ave.y/tot;
    return ave;
}

void FindthePoints(IplImage *dst, const CvMat* xyz)
{
    spoint.clear();
    memset(flags, 0, sizeof(flags));
    int x, y, i, j;
    char *ptr;
    float fl;
    IplImage *exds;
    CvPoint temp;
    for (y = 0; y < dst->height; y++) {
       ptr = (dst->imageData) + y * dst->widthStep;
        for (x = 0; x < dst->width; x++) {
         if (!flags[y][x] && ptr[dst->nChannels * x] >= CRT) {
             temp = floods(y, x, dst->imageData, dst->widthStep);
             if (temp.x != -1) {
                 fl = xyz->data.fl[temp.y * (xyz->step/4) + 3*temp.x + 2];
                 if (abs(fl-10000.0) > 0.1) spoint.push_back(temp); //Not equal to 10000.0
            }
         }
       }
    }
    exds = cvCloneImage(dst);
    for (i = 0; i < spoint.size(); i++) {
        ptr = (exds->imageData) + spoint[i].y*dst->widthStep + spoint[i].x;
        for (j = 0; j < 12; j++) *(ptr + j) = 255;
        printf("%f\n", xyz->data.fl[spoint[i].y * (xyz->step/4) + 3*spoint[i].x + 2]);
    }
    cvNamedWindow("POINT", 1);
    cvShowImage("POINT", exds);
}

