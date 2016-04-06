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

void Find_Points(IplImage *dst, const CvMat* xyz)
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

void StereoCalib(const char* imageList, int nx, int ny, int useUncalibrated)
{
    int displayCorners = 0;            //display corners
    char kyinp = -1, chooseinp, runinp;
    int showUndistorted = 1;
    bool isVerticalStereo = false;    //OpenCV can handle left-right
                                      //or up-down camera arrangements
    const int maxScale = 1;
    const float squareSize = 22.0f;   //Set this to your actual square size
    int i, j, lr, nframes, n = nx*ny, N = 0;
    vector<string> imageNames[2];     //File Name
    vector<CvPoint3D32f> objectPoints;
    vector<CvPoint2D32f> points[2];
    vector<int> npoints;
    vector<uchar> active[2];
    vector<CvPoint2D32f> temp(n);
    CvSize imageSize = {640, 480};    //change to your real size
    // ARRAY AND VECTOR STORAGE:
    double M1[3][3], M2[3][3], D1[5], D2[5];
    double R[3][3], T[3], E[3][3], F[3][3], Q[4][4];
    CvMat _M1 = cvMat(3, 3, CV_64F, M1);
    CvMat _M2 = cvMat(3, 3, CV_64F, M2);
    CvMat _D1 = cvMat(1, 5, CV_64F, D1);
    CvMat _D2 = cvMat(1, 5, CV_64F, D2);
    CvMat _R = cvMat(3, 3, CV_64F, R);
    CvMat _T = cvMat(3, 1, CV_64F, T);
    CvMat _E = cvMat(3, 3, CV_64F, E);
    CvMat _F = cvMat(3, 3, CV_64F, F);
    CvMat _Q  = cvMat(4, 4, CV_64F, Q);
    CvMat* mx1 = cvCreateMat( imageSize.height,
        imageSize.width, CV_32F );
    CvMat* my1 = cvCreateMat( imageSize.height,
        imageSize.width, CV_32F );
    CvMat* mx2 = cvCreateMat( imageSize.height,
        imageSize.width, CV_32F );
    CvMat* my2 = cvCreateMat( imageSize.height,
        imageSize.width, CV_32F );
    CvMat* pair;
   // CvMat _F = cvMat(3, 3, CV_64F, F );
    if( displayCorners )
        cvNamedWindow( "corners", 1 );
     FILE* f = fopen(imageList, "rt");
// READ IN THE LIST OF CHESSBOARDS:
    if( !f )
    {
        fprintf(stderr, "can not open file %s\n", imageList );
        return;
    }
    for(i = 0 ;; i++)
    {
        char buf[1024];
        int count = 0, result=0;
        lr = i % 2;
        vector<CvPoint2D32f>& pts = points[lr];
        if( !fgets( buf, sizeof(buf)-3, f ))
            break;
        size_t len = strlen(buf);
        while( len > 0 && isspace(buf[len-1]))
            buf[--len] = '\0';
        if( buf[0] == '#')
            continue;
        IplImage* img = cvLoadImage( buf, 0 );
        if (!img) break;
        imageSize = cvGetSize(img);
        imageNames[lr].push_back(buf);     //Put File Names into list
    //FIND CHESSBOARDS AND CORNERS THEREIN:
        for( int s = 1; s <= maxScale; s++ )
        {
            IplImage* timg = img;
            if( s > 1 )
            {
                timg = cvCreateImage(cvSize(img->width*s, img->height*s),
                    img->depth, img->nChannels );
                cvResize( img, timg, CV_INTER_CUBIC );
            }
            result = cvFindChessboardCorners( timg, cvSize(nx, ny),
                &temp[0], &count,
                CV_CALIB_CB_ADAPTIVE_THRESH |
                CV_CALIB_CB_NORMALIZE_IMAGE);
            if( timg != img )
                cvReleaseImage( &timg );
            if( result || s == maxScale )
                for( j = 0; j < count; j++ )
            {
                temp[j].x /= s;
                temp[j].y /= s;
            }
            if( result )
                break;
        }
        if( displayCorners )
        {
            printf("%s\n", buf);
            IplImage* cimg = cvCreateImage( imageSize, 8, 3 );
            cvCvtColor( img, cimg, CV_GRAY2BGR );
            cvDrawChessboardCorners( cimg, cvSize(nx, ny), &temp[0],
                count, result );
            cvShowImage( "corners", cimg );
            cvReleaseImage( &cimg );
            if( cvWaitKey(0) == 27 ) //Allow ESC to quit
                exit(-1);
        }
        else
            putchar('.');
        N = pts.size();
        pts.resize(N + n, cvPoint2D32f(0,0));
        active[lr].push_back((uchar)result);
    //assert( result != 0 );
        if( result )
        {
         //Calibration will suffer without subpixel interpolation
            cvFindCornerSubPix( img, &temp[0], count,
                cvSize(11, 11), cvSize(-1,-1),
                cvTermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS,
                30, 0.01) );
            copy( temp.begin(), temp.end(), pts.begin() + N );
        }
        cvReleaseImage( &img );
    }
    fclose(f);
    printf("\n");
// HARVEST CHESSBOARD 3D OBJECT POINT LIST:
    nframes = active[0].size();//Number of good chessboads found
    objectPoints.resize(nframes*n);
    for( i = 0; i < ny; i++ )
        for( j = 0; j < nx; j++ )
        objectPoints[i*nx + j] =
        cvPoint3D32f(i*squareSize, j*squareSize, 0);
    for( i = 1; i < nframes; i++ )
        copy( objectPoints.begin(), objectPoints.begin() + n,
        objectPoints.begin() + i*n );
    npoints.resize(nframes,n);
    N = nframes*n;
    CvMat _objectPoints = cvMat(1, N, CV_32FC3, &objectPoints[0] );
    CvMat _imagePoints1 = cvMat(1, N, CV_32FC2, &points[0][0] );
    CvMat _imagePoints2 = cvMat(1, N, CV_32FC2, &points[1][0] );
    CvMat _npoints = cvMat(1, npoints.size(), CV_32S, &npoints[0] );
    cvSetIdentity(&_M1);
    cvSetIdentity(&_M2);
    cvZero(&_D1);
    cvZero(&_D2);

// CALIBRATE THE STEREO CAMERAS
    printf("Running stereo calibration ...");
    fflush(stdout);
    cvStereoCalibrate( &_objectPoints, &_imagePoints1,
        &_imagePoints2, &_npoints,
        &_M1, &_D1, &_M2, &_D2,
        imageSize, &_R, &_T, &_E, &_F,
        CV_CALIB_FIX_ASPECT_RATIO +
        CV_CALIB_ZERO_TANGENT_DIST +
        CV_CALIB_SAME_FOCAL_LENGTH,
        cvTermCriteria(CV_TERMCRIT_ITER+
        CV_TERMCRIT_EPS, 10, 1e-5)
        );
    printf(" done\n");
// CALIBRATION QUALITY CHECK
// because the output fundamental matrix implicitly
// includes all the output information,
// we can check the quality of calibration using the
// epipolar geometry constraint: m2^t*F*m1=0
    vector<CvPoint3D32f> lines[2];
    points[0].resize(N);
    points[1].resize(N);
    _imagePoints1 = cvMat(1, N, CV_32FC2, &points[0][0] );
    _imagePoints2 = cvMat(1, N, CV_32FC2, &points[1][0] );
    lines[0].resize(N);
    lines[1].resize(N);
    CvMat _L1 = cvMat(1, N, CV_32FC3, &lines[0][0]);
    CvMat _L2 = cvMat(1, N, CV_32FC3, &lines[1][0]);
//Always work in undistorted space
    cvUndistortPoints( &_imagePoints1, &_imagePoints1,
        &_M1, &_D1, 0, &_M1 );
    cvUndistortPoints( &_imagePoints2, &_imagePoints2,
        &_M2, &_D2, 0, &_M2 );
    Cv_Save_Vector(&_F,"_F.txt",2);
    cvComputeCorrespondEpilines( &_imagePoints1, 1, &_F, &_L1 );
    cvComputeCorrespondEpilines( &_imagePoints2, 2, &_F, &_L2 );
    double avgErr = 0;
    for( i = 0; i < N; i++ )
    {
        double err = fabs(points[0][i].x*lines[1][i].x +
            points[0][i].y*lines[1][i].y + lines[1][i].z)
            + fabs(points[1][i].x*lines[0][i].x +
            points[1][i].y*lines[0][i].y + lines[0][i].z);
        avgErr += err;
    }
    printf( "avg err = %g\n", avgErr/(nframes*n) );
//COMPUTE AND DISPLAY RECTIFICATION
    if( showUndistorted )
    {
        double R1[3][3], R2[3][3], P1[3][4], P2[3][4];
        CvMat _R1 = cvMat(3, 3, CV_64F, R1);
        CvMat _R2 = cvMat(3, 3, CV_64F, R2);
// IF BY CALIBRATED (BOUGUET'S METHOD)
        if( useUncalibrated == 0 )
        {
            CvMat _P1 = cvMat(3, 4, CV_64F, P1);
            CvMat _P2 = cvMat(3, 4, CV_64F, P2);
            cvStereoRectify( &_M1, &_M2, &_D1, &_D2, imageSize,
                &_R, &_T,
                &_R1, &_R2, &_P1, &_P2, &_Q,
                0/*CV_CALIB_ZERO_DISPARITY*/ );
            isVerticalStereo = fabs(P2[1][3]) > fabs(P2[0][3]);
    //Precompute maps for cvRemap()
            cvInitUndistortRectifyMap(&_M1,&_D1,&_R1,&_P1,mx1,my1);
            cvInitUndistortRectifyMap(&_M2,&_D2,&_R2,&_P2,mx2,my2);
        }
//OR ELSE HARTLEY'S METHOD
        else if( useUncalibrated == 1 || useUncalibrated == 2 )
     // use intrinsic parameters of each camera, but
     // compute the rectification transformation directly
     // from the fundamental matrix
        {
            double H1[3][3], H2[3][3], iM[3][3];
            CvMat _H1 = cvMat(3, 3, CV_64F, H1);
            CvMat _H2 = cvMat(3, 3, CV_64F, H2);
            CvMat _iM = cvMat(3, 3, CV_64F, iM);
    //Just to show you could have independently used F
            if( useUncalibrated == 2 )
                cvFindFundamentalMat( &_imagePoints1,
                &_imagePoints2, &_F);
            cvStereoRectifyUncalibrated( &_imagePoints1,
                &_imagePoints2, &_F,
                imageSize,
                &_H1, &_H2, 3);
            cvInvert(&_M1, &_iM);
            cvMatMul(&_H1, &_M1, &_R1);
            cvMatMul(&_iM, &_R1, &_R1);
            cvInvert(&_M2, &_iM);
            cvMatMul(&_H2, &_M2, &_R2);
            cvMatMul(&_iM, &_R2, &_R2);
    //Precompute map for cvRemap()
            cvInitUndistortRectifyMap(&_M1, &_D1, &_R1, &_M1, mx1, my1);

            cvInitUndistortRectifyMap(&_M2, &_D1, &_R2, &_M2, mx2, my2);
        }
    }
        else
            assert(0);
        cvNamedWindow( "rectified", 1 );
// RECTIFY THE IMAGES AND FIND DISPARITY MAPS
        if( !isVerticalStereo )
            pair = cvCreateMat( imageSize.height, imageSize.width*2,
            CV_8UC3 );
        else
            pair = cvCreateMat( imageSize.height*2, imageSize.width,
            CV_8UC3 );
        Cv_Save_Vector(&_Q, "_Q.txt", 2);
        Cv_Save_Vector(mx1, "mx1.txt", 1);
        Cv_Save_Vector(mx2, "mx2.txt", 1);
        Cv_Save_Vector(my1, "my1.txt", 1);
        Cv_Save_Vector(my2, "my2.txt", 1);
}

int Cv_Save_Vector(CvMat *mat, char *filename, int type)
{
    FILE *fp = fopen(filename, "w+");
    if(fp != NULL){
    fprintf(fp, "%d %d\n", mat->rows, mat->cols);
    for(int i = 0; i<mat->rows; i++) {
        for(int j = 0; j<mat->cols; j++) {
        switch(type)
        {
       case 0:
        fprintf(fp, "%d ", (mat->data.ptr + i*mat->step)[j]);
        break;
       case 1:
        fprintf(fp, "%f ", (mat->data.fl + i*mat->step/4)[j]);
        break;
       case 2:
        fprintf(fp, "%lf ", (mat->data.db + i*mat->step/8)[j]);
        break;
       case 3:
        fprintf(fp, "%d ", (mat->data.s + i*mat->step/2)[j]);
         break;
       }
    }
    fprintf(fp,"\n");
  }
   fclose(fp);
 }
    return 0;
}

CvMat *Cv_Load_Vector(char* filename, int type)
{
    int rows, cols;
    CvMat *mat = NULL;
    FILE *fp = fopen(filename, "r+");
    if (fp != NULL) {
    fscanf(fp, "%d %d", &rows, &cols);
    switch(type)
    {
    case 0:
        mat = cvCreateMat(rows, cols, CV_8UC1);
        break;
    case 1:
        mat = cvCreateMat(rows, cols, CV_32FC1);
        break;
    case 2:
        mat = cvCreateMat(rows, cols, CV_64FC1);
    break;
  }
  for(int i = 0; i< mat->rows; i++) {
   for(int j = 0; j< mat->cols; j++) {
    switch(type)
    {
     case 0:
      fscanf(fp, "%d", &(mat->data.ptr + i*mat->step)[j]);
      break;
     case 1:
      fscanf(fp, "%f", &(mat->data.fl + i*mat->step/4)[j]);
      break;
     case 2:
      fscanf(fp, "%lf", &(mat->data.db + i*mat->step/8)[j]);
      break;
    }
   }
  }
  fclose(fp);
 }
 return mat;
}
