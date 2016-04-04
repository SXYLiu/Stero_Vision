/* *************** License:**************************
   Nov. 11, 2015
   Right to use this code in any way you want without warrenty, support or any guarentee of it working.


   * Contributed By LSXY
   ************************************************* */
#include <opencv2/opencv.hpp>
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/videoio.hpp"
#include "opencv/cv.h"
#include "opencv/cxmisc.h"
#include "opencv/highgui.h"
#include <opencv2/calib3d/calib3d.hpp>
#include <vector>
#include <string>
#include <algorithm>
#include <stdio.h>
#include <ctype.h>
#include <Windows.h>
#include <Okapi64.h>
#include <QString>
#include <string>
#include <D:/PosNo2/Capture/ok_capture.h>
#include <sstream>
#ifdef ide
 #include<iostream>
#endif
using namespace std;
#define Pi 3.1415926
#define NT 300
#define CRT 27
int totpoints;
CvPoint prev_pt;
CvPoint pt_beg,pt_end;
CvPoint pt1,pt2;
bool flags[480][640];
vector<CvPoint>spoint[2];//Solutions
const int dx[8] = {0,0,-1,-1,-1,1,1,1};
const int dy[8] = {1,-1,-1,0,1,-1,0,1};
bool stdflags[NT];
float MidDistance[NT]; //square ^2;
vector<CvPoint3D32f> origin;
float CompDarr[NT][NT];

//IplImage *tmp;
//
// Given a list of chessboard images, the number of corners (nx, ny)
// on the chessboards, and a flag: useCalibrated for calibrated (0) or
// uncalibrated (1: use cvStereoCalibrate(), 2: compute fundamental
// matrix separately) stereo. Calibrate the cameras and display the
// rectified results along with the computed disparity images.
//

void onMouse(int event,int x,int y,int flags,void *tmp)
{
    CvPoint pt =cvPoint(x,y);
    printf("x=%d,y=%d.\n",x,y);
    printf("Begin:x=%d,y=%d.\n",pt_beg.x,pt_beg.y);
    printf("END:x=%d,y=%d.\n",pt_end.x,pt_end.y);
    //if (!tmp) {return;}

if ((event == CV_EVENT_LBUTTONUP)&&(flags != CV_EVENT_FLAG_LBUTTON))
{

prev_pt=cvPoint(-1,-1);
}

if (event == CV_EVENT_LBUTTONDOWN)
{
    prev_pt = cvPoint(x,y);
    pt_beg = cvPoint(x,y);
}

if ((event == CV_EVENT_MOUSEMOVE)&&(flags == CV_EVENT_FLAG_LBUTTON) )
    {
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
    cvShowImage("SRC",tmp);
    }
}

void Select_an_area(IplImage *src)
{
    prev_pt={-1,-1};pt_beg={-1,-1};
    pt_end = {-1,-1}; pt1={-1,-1}; pt2={-1,-1};
    IplImage *dst,*tmp;
    CvRect rect;
    int c;
    cvNamedWindow("SRC",1);
    cvShowImage("SRC",src);
    tmp = cvCloneImage(src);
    cvSetMouseCallback("SRC",onMouse,tmp);
    for (;;)
    {
    c = cvWaitKey(10);
    if (c=='c')
    {
    if (pt_beg.x!=-1)
    {
        rect.x = min(pt_beg.x,pt_end.x);
        rect.y = min(pt_beg.y,pt_end.y);
        rect.height = abs(pt_end.y-pt_beg.y);
        rect.width = abs(pt_end.x-pt_beg.x);
        printf("RECT:height=%d,width=%d\n",rect.height,rect.width);
        cvSetImageROI(src,rect);
        printf("ROI:Height=%d,Width=%d.\n",src->roi->height,src->roi->width);
        dst = cvCreateImage(cvSize(rect.width,rect.height),src->depth,src->nChannels);
        dst = cvCloneImage(src);
        printf("DST Height=%d,Width=%d\n",dst->height,dst->width);
        cvNamedWindow("Selected",1);
        cvShowImage("Selected",dst);
        cvSaveImage("selected.jpg",dst);
        printf("Finished!\n");
        cvWaitKey(0);
        cvDestroyWindow("Selected");
        break;
        }
    }
    if (c==27) //ESC to exit
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


void FindthePoints(IplImage *dst,const CvMat* xyz)
{
    spoint.clear();
    memset(flags,0,sizeof(flags));
    int x,y,i,j;
    char *ptr;
    float fl;
    IplImage *exds;
    CvPoint temp;
    for (y=0; y<dst->height; y++) {
       ptr=(dst->imageData)+y*dst->widthStep;
        for (x=0; x<dst->width; x++) {
         if (!flags[y][x]&&ptr[dst->nChannels*x]>=CRT) {
             temp=floods(y,x,dst->imageData,dst->widthStep);
             if (temp.x!=-1) {
                 fl=xyz->data.fl[temp.y*(xyz->step/4)+3*temp.x+2];
                 if (abs(fl-10000.0)>0.1) spoint.push_back(temp); //Not equal to 10000.0
            }
         }
        }
    }
    exds=cvCloneImage(dst);
    for (i=0; i<spoint.size(); i++) {
      ptr=(exds->imageData)+spoint[i].y*dst->widthStep+spoint[i].x;
      for (j=0; j<12; j++) *(ptr+j)=255;
      printf("%f\n",xyz->data.fl[spoint[i].y*(xyz->step/4)+3*spoint[i].x+2]);
    }
    cvNamedWindow("POINT",1);
    cvShowImage("POINT",exds);
}

void FindthePoints(IplImage *dst)
{
    spoint.clear();
    memset(flags,0,sizeof(flags));
    int x,y,i,j;
    char *ptr;
    IplImage *exds;
    CvPoint temp;
    for (y=0; y<dst->height; y++) {
       ptr=(dst->imageData)+y*dst->widthStep;
        for (x=0; x<dst->width; x++) {
         if (!flags[y][x]&&ptr[dst->nChannels*x]>=CRT) {
             temp=floods(y,x,dst->imageData,dst->widthStep);
             spoint.push_back(temp);
            }
         }
    }
    exds=cvCloneImage(dst);
    for (i=0; i<spoint.size(); i++) {
      ptr=(exds->imageData)+spoint[i].y*dst->widthStep+spoint[i].x;
      for (j=0; j<12; j++) *(ptr+j)=255;
      printf("%f\n",xyz->data.fl[spoint[i].y*(xyz->step/4)+3*spoint[i].x+2]);
    }
    cvNamedWindow("POINT",1);
    cvShowImage("POINT",exds);
}

void CalculateDist(float temp[15][15],const CvMat* xyz)
{
    int i,j,tei,tej;
    for (i=0; i<spoint.size()-1; i++) {
        temp[i][i]=0;
        for (j=i+1; j<spoint.size(); j++) {
 //Now Q
        tei=spoint[i].y*(xyz->step/4)+3*spoint[i].x;
        tej=spoint[j].y*(xyz->step/4)+3*spoint[j].x;
      //  if (xyz->data.fl[tei+2]<9999.0&&xyz->data.fl[tej+2]<9999.0) {
            temp[i][j]=sqrt(xyz->data.fl[tei]*xyz->data.fl[tei]+
                            xyz->data.fl[tei+1]*xyz->data.fl[tei+1]+
                            xyz->data.fl[tei+2]*xyz->data.fl[tei+2]);
            temp[j][i]=temp[i][j];
     //   } else {
     //       temp[i][j]=10000.0;
      //      temp[j][i]=10000.0;
        //}
      }
   }
}

bool cmp(const int aa, const int bb)
{
    return aa>bb;
}

int FindtheSimilarD(float dis[15],int tot)
{
    int i,j,k,t;
    float ds[60];
    for (i=0; i<totpoints; i++)
       if (!stdflags[i]){
        j=0; k=0; t=0;
        for (j=0; j<totpoints; j++) ds[t++]=CompDarr[i][j];
        sort(ds,ds+t,cmp);
        j=0;
        while (j<totpoints) {
            if (abs(dis[k]-ds[j])<=1.5) k++;
            if (abs(dis[k]-ds[j])>1.5&&dis[k]>ds[j]) break;
            if (k==tot-1) break;
            j++;
        }
        if (k==tot-1) {
            stdflags[i]=1;
            return i;
        }
    }
    return -1;
}


void ComparetoStandardD(const float temp[15][15],int relat[15])
{
    int i,j,tot=0;
    float ds[15];
    memset(stdflags,0,sizeof(stdflags));
    for (i=0; i<spoint.size(); i++) {
        memset(ds,0,sizeof(ds));
        tot=0;
        for (j=0; j<spoint.size(); j++) if (temp[i][j]<9999.0) ds[tot++]=temp[i][j];
        sort(ds,ds+tot,cmp);
        relat[i]=FindtheSimilarD(ds,tot);
   }
}

void GetthePosition(int relat[],float *posx,float *posy,float *posz,float *movx,float *movy,float *movz,const CvMat* xyz)
{
    //Select 3 Points
    int i,j,k,te1,te2,offset,fl3=9999,fl1=0,fl2=9999;
    float a,b,c,anx,any,l1,l2,l3,x1,x2,a1,b1,a2,b2,y1,y2,x12,x13,x14,y12,y13,y14,x22,x23,x24,y22,y23,y24,l12,l22,tem,tail,mid,z1,z2,z3,x3,y3;
 /*   for (i=0; i<spoint.size()-2; i++) {
      tei=spoint[i].y*(xyz->step/4)+3*spoint[i].x;
      if (xyz->data.fl[tei+2]<9999.0&&relat[i]!=-1) {
      for (j=i+1; j<spoint.size()-1; j++) {
         tej=spoint[j].y*(xyz->step/4)+3*spoint[j].x;
         if (xyz->data.fl[tej+2]<9999.0&&relat[j]!=-1) {
           for (k=j+1; k<spoint.size(); k++) {
             tek=spoint[k].y*(xyz->step/4)+3*spoint[k].x;
              if (xyz->data.fl[tek+2]<9999.0&&relat[k]!=-1) {
                fl=1;
                break;
              }
              if (fl==1) break;
            }
         }
         if (fl==1) break;
        }
      }
      if (fl==1) break;
    } where error happens*/
    k=-1;
    for (i=0; i<spoint.size(); i++) {
       if (spoint[i].y>fl1) fl1=spoint[i].y;
       if (spoint[i].y<fl2) fl2=spoint[i].y;
    }
    fl1=(fl1+fl2)/2;
    fl2=9999; fl3=9999;
    for (i=0; i<spoint.size(); i++) {
      if (abs(spoint[i].y-fl1)<fl2) {
          te1=i;
          fl2=abs(spoint[i].y-fl1);
      }
         else if (abs(spoint[i].y-fl1)<fl3) {
         te2=i;
         fl3=abs(spoint[i].y-fl1);
      }
    }
    //Select 2 Points to reconstruct Mid Point
    //ds=distanc[i][j];
    l1=MidDistance[relat[te1]];
    l2=MidDistance[relat[te2]];
    offset=spoint[te1].y*(xyz->step/4)+3*spoint[te1].x;
    x1=xyz->data.fl[offset];
    y1=xyz->data.fl[offset+1];
    z1=xyz->data.fl[offset+2];
    c=xyz->data.fl[offset+2];
    offset=spoint[te2].y*(xyz->step/4)+3*spoint[te2].x;
    x2=xyz->data.fl[offset];
    y2=xyz->data.fl[offset+1];
    z2=xyz->data.fl[offset+2];
    c=(c+xyz->data.fl[offset+2])/2;
    x12=x1*x1;  x22=x2*x2;
    x13=x12*x1; x23=x22*x2;
    x14=x13*x1; x24=x23*x2;
    y12=y1*y1;  y22=y2*y2;
    y13=y12*y1; y23=y22*y2;
    y14=y13*y1; y24=y23*y2;
    l12=l1*l1;  l22=l2*l2;
    tem=sqrt(-l12+2*l1*l2+2*l1*x12-4*l1*x1*x2+2*l1*x22+2*l1*y12-4*l1*y1*y2+2*l1*y22-l22+2*l2*x12-4*l2*x1*x2+2*l2*x22+2*l2*y12-4*l2*y1*y2
               +2*l2*y22-x14+4*x13*x2-6*x12*x22-2*x12*y12+4*x12*y1*y2-2*x12*y22+4*x1*x23+4*x1*x2*y12-8*x1*x2*y1*y2+4*x1*x2*y22-x24-2*x22*y12
               +4*x22*y1*y2-2*x22*y22-y14+4*y13*y2-6*y12*y22+4*y1*y23-y24);
    tail=2*(x12-2*x1*x2+x22+y12-2*y1*y2+y22);
    mid=-l1*y1+l1*y2+l2*y1-l2*y2+x12*y1+x12*y2+x22*y1+x22*y2-y1*y22-y12*y2+y13+y23-2*x1*x2*y1-2*x1*x2*y2;
    a1=(x2*tem-x1*tem+mid)/tail;
    a2=(x1*tem-x2*tem+mid)/tail;
    mid=-l1*x1+l1*x2+l2*x1-l2*x2-x1*x22-x12*x2+x1*y12+x1*y22+x2*y12+x2*y22+x13+x23-2*x1*y1*y2-2*x2*y1*y2;
    b1=(y1*tem-y2*tem+mid)/tail;
    b2=(y2*tem-y1*tem+mid)/tail;
    for (i=0; i<spoint.size(); i++)
        if (k!=te1&&k!=te2) {
            k=i;
            break;
        }
    anx=xyz->data.fl[spoint[k].y*(xyz->step/4)+3*spoint[k].x];
    any=xyz->data.fl[spoint[k].y*(xyz->step/4)+3*spoint[k].x+1];
    z3=xyz->data.fl[spoint[k].y*(xyz->step/4)+3*spoint[k].x+2];
    if (abs((a1-anx)*(a1-anx)+(b1-any)*(b1-any)-MidDistance[relat[k]])<abs((a2-anx)*(a2-anx)+(b2-any)*(b2-any)-MidDistance[relat[k]]))
    {
        a=a1; b=b1;
    }
    else {
        a=a2; b=b2;
    }
    x3=anx; y3=any;
    x1-=a; y1-=b; z1-=c;
    *movx=a; *movy=b; *movz=c;
    a=x1;  b=y1; c=z1;
    fl1=99999.0;
    for (i=0; i<360; i+=2) {
        x3=cos(i/(2*Pi));
        y3=sin(i/(2*Pi));
      for (j=0; j<360; j+=2) {
          x2=cos(j/(2*Pi));
          y2=sin(j/(2*Pi));
         for (k=0; k<360; k+=2) {
           x1=cos(k/(2*Pi));
           y1=sin(k/(2*Pi));
           l1=abs(x3*(y2*(c*x1+b*y1)+a*x2)-y3*(b*x1-c*y1)-origin[relat[te1]].x);
           l2=abs(x3*(b*x1-c*y1)+y3*(y2*(c*x1+b*y1)+a*x2)-origin[relat[te1]].y);
           l3=abs(x2*(c*x1+b*y1)-a*y2-origin[relat[te1]].z);
           if (l1+l2+l3<1) {
               *posx=k;
               *posy=j;
               *posz=i;
               return;
           }
           if (l1+l2+l3<fl1) {
              fl1=l1+l2+l3;
              *posx=k;
              *posy=j;
              *posz=i;
           }
         }
      }
    }
    return;
}
void GetArrangement(float *movx,float *movy,float *movz,const CvMat* xyz)
{
  //  int i,j,k,te1,te2,offset,fl3=9999,fl1=0,fl2=9999;


}

void Rotate(const CvMat* xyz,int axs,int angel,float movx,float movy,float movz)
{
    int i,j,k,offset;
    float x,y,z,sint,cost;
    if (angel==0) {sint=0; cost=1.0;}
    if (angel==3) {sint=1.0; cost=0;}
    if (angel==2) {sint=0; cost=-1.0;}
    if (angel==1) {sint=-1.0; cost=0;}
    for (i=0; i<spoint.size(); i++) {
        offset=spoint[i].y*(xyz->step/4)+3*spoint[i].x;
        x=xyz->data.fl[offset]-movx;
        y=xyz->data.fl[offset+1]-movy;
        z=xyz->data.fl[offset+2]-movz;
        if (axs==3) {
        x=x*cost-y*sint;
        y=y*cost+x*sint;
        }
        if (axs==2) {
        x=x*cost+z*sint;
        z=z*cost-x*sint;
        }
        if (axs==1) {
        y=y*cost-z*sint;
        z=z*cost+y*sint;
        }
        k=0;
        for (j=0; j<origin.size(); j++)
          if (abs(z-origin[j].z)+abs(y-origin[j].y)+abs(x-origin[j].x)<10.0) {//
             k=1;
             break;
          }
        if (k!=1) {
          ++totpoints;
          origin.push_back(cvPoint3D32f(x,y,z));
          MidDistance[totpoints]=x*x+y*y+z*z;
          for (j=0; j<origin.size()-1; j++) {
              CompDarr[j][totpoints]=(x-origin[j].x)*(x-origin[j].x)+(y-origin[j].y)*(y-origin[j].y)+(z-origin[j].z)*(z-origin[j].z);//
              CompDarr[totpoints][j]=CompDarr[j][totpoints];
          }

      }
    }
}

void Savearr()
{
    FILE *file=fopen("Arr.txt","w");
    int i,j;
    fprintf(file,"%d\n",totpoints);
    for (i=0; i<origin.size(); i++) {
        fprintf(file,"%f %f %f %f ",origin[i].x,origin[i].y,origin[i].z,MidDistance[i]);
        for (j=0; j<origin.size()-1; j++) fprintf(file,"%f ",CompDarr[i][j]);
        fprintf(file,"%f\n",CompDarr[i][totpoints]);
    }
    fclose(file);
}

void Loadarr()
{
    FILE *file=fopen("Arr.txt","a+");
    int i,j;
    float x,y,z;
    fscanf(file,"%d",&totpoints);
    for (i=0; i<totpoints; i++) {
        fscanf(file,"%f%f%f%f",&x,&y,&z,&MidDistance[i]);
        origin.push_back(cvPoint3D32f(x,y,z));
        for (j=0; j<totpoints; j++) fscanf(file,"%f",&CompDarr[i][j]);
    }
    fclose(file);
}

int cv_save_vector(CvMat *mat,char *filename,int type)
{
    int i,j;
    FILE *fp=fopen(filename,"w+");
    if(fp!=NULL){
    fprintf(fp,"%d %d\n",mat->rows,mat->cols);
    for(i=0;i<mat->rows;i++){
        for(j=0;j<mat->cols;j++){
        switch(type)
        {
       case 0:
        fprintf(fp,"%d ",(mat->data.ptr+i*mat->step)[j]);
        break;
       case 1:
        fprintf(fp,"%f ",(mat->data.fl+i*mat->step/4)[j]);
        break;
       case 2:
        fprintf(fp,"%lf ",(mat->data.db+i*mat->step/8)[j]);
        break;
       case 3:
        fprintf(fp,"%d ",(mat->data.s+i*mat->step/2)[j]);
         break;
       }
    }
    fprintf(fp,"\n");
  }
   fclose(fp);
 }
    return 0;
}

CvMat *cv_load_vector(char* filename,int type)
{
    int rows,cols,i,j;
    CvMat *mat=NULL;
    FILE *fp=fopen(filename,"r+");
    if(fp!=NULL){
    fscanf(fp,"%d %d",&rows,&cols);
    switch(type)
    {
   case 0:
    mat=cvCreateMat(rows,cols,CV_8UC1);
    break;
   case 1:
    mat=cvCreateMat(rows,cols,CV_32FC1);
    break;
   case 2:
    mat=cvCreateMat(rows,cols,CV_64FC1);
    break;
  }
  for(i=0;i<mat->rows;i++){
   for(j=0;j<mat->cols;j++){
    switch(type)
    {
     case 0:
      fscanf(fp,"%d",&(mat->data.ptr+i*mat->step)[j]);
      break;
     case 1:
      fscanf(fp,"%f",&(mat->data.fl+i*mat->step/4)[j]);
      break;
     case 2:
      fscanf(fp,"%lf",&(mat->data.db+i*mat->step/8)[j]);
      break;
    }
   }

  }
  fclose(fp);
 }
 return mat;
}


static void
StereoCalib(const char* imageList, int nx, int ny, int useUncalibrated)
{
    int displayCorners = 0;
    char kyinp=-1,chooseinp,runinp;
    int showUndistorted = 1;
    bool isVerticalStereo = false;//OpenCV can handle left-right
                                   //or up-down camera arrangements
    const int maxScale = 1;
    const float squareSize = 22.0f; //Set this to your actual square size
    int i, j, lr, nframes, n = nx*ny, N = 0;
    float dist[15][15];
    vector<string> imageNames[2];
    vector<CvPoint3D32f> objectPoints;
    vector<CvPoint2D32f> points[2];
    vector<int> npoints;
    vector<uchar> active[2];
    vector<CvPoint2D32f> temp(n);
    CvSize imageSize = {640,480};
    // ARRAY AND VECTOR STORAGE:
    double M1[3][3], M2[3][3], D1[5], D2[5];
    double R[3][3], T[3], E[3][3], F[3][3];
    CvMat _M1 = cvMat(3, 3, CV_64F, M1 );
    CvMat _M2 = cvMat(3, 3, CV_64F, M2 );
    CvMat _D1 = cvMat(1, 5, CV_64F, D1 );
    CvMat _D2 = cvMat(1, 5, CV_64F, D2 );
    CvMat _R = cvMat(3, 3, CV_64F, R );
    CvMat _T = cvMat(3, 1, CV_64F, T );
    CvMat _E = cvMat(3, 3, CV_64F, E );
    CvMat* mx1 = cvCreateMat( imageSize.height,
        imageSize.width, CV_32F );
    CvMat* my1 = cvCreateMat( imageSize.height,
        imageSize.width, CV_32F );
    CvMat* mx2 = cvCreateMat( imageSize.height,
        imageSize.width, CV_32F );
    CvMat* my2 = cvCreateMat( imageSize.height,
        imageSize.width, CV_32F );
    CvMat* img1r = cvCreateMat( imageSize.height,
        imageSize.width, CV_8U );
    CvMat* img2r = cvCreateMat( imageSize.height,
        imageSize.width, CV_8U );
    CvMat* disp = cvCreateMat( imageSize.height,
        imageSize.width, CV_16S );
    CvMat* vdisp = cvCreateMat( imageSize.height,
        imageSize.width, CV_8U );
    CvMat* pair;
    double Q[4][4];
    CvMat _Q  = cvMat(4, 4, CV_64F, Q );
   // CvMat _F = cvMat(3, 3, CV_64F, F );
    if( displayCorners )
        cvNamedWindow( "corners", 1 );
    printf("Run Calib ? (Y) or load the last calib result? (N)");
    scanf("%c",&chooseinp);
    if (chooseinp=='y'||chooseinp=='Y') {
     CvMat _F = cvMat(3, 3, CV_64F, F );
     FILE* f = fopen(imageList, "rt");
// READ IN THE LIST OF CHESSBOARDS:
    if( !f )
    {
        fprintf(stderr, "can not open file %s\n", imageList );
        return;
    }
    for(i=0;;i++)
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
        if( !img )
            break;
        imageSize = cvGetSize(img);
        imageNames[lr].push_back(buf);//can use
    //FIND CHESSBOARDS AND CORNERS THEREIN:
        for( int s = 1; s <= maxScale; s++ )
        {
            IplImage* timg = img;
            if( s > 1 )
            {
                timg = cvCreateImage(cvSize(img->width*s,img->height*s),
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
    cv_save_vector(&_F,"_F.txt",2);
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
            cvInitUndistortRectifyMap(&_M1,&_D1,&_R1,&_M1,mx1,my1);

            cvInitUndistortRectifyMap(&_M2,&_D1,&_R2,&_M2,mx2,my2);
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
        cv_save_vector(&_Q,"_Q.txt",2);
        cv_save_vector(mx1,"mx1.txt",1);
        cv_save_vector(mx2,"mx2.txt",1);
        cv_save_vector(my1,"my1.txt",1);
        cv_save_vector(my2,"my2.txt",1);
    }
    else {
        //load Q load mx1,mx2,my1,my2
       // cvLoad() 
       mx1=cv_load_vector("mx1.txt",1);
       mx2=cv_load_vector("mx2.txt",1);
       my1=cv_load_vector("my1.txt",1);
       my2=cv_load_vector("my2.txt",1);
       _Q=*cv_load_vector("_Q.txt",2);
    }
    getchar();
    printf("Do the First-time Exercise (Y) or Run Position Calculation (N)?");
    scanf("%c",&runinp);
    if (runinp=='Y'||runinp=='y') {     
            //addins,totpoints;
        //calculate Mid Point & all distance
       // IplImage *img1,*img2;
        char buf1[1024],buf2[1024];
        CvMat* xyz;
        float movx,movy,movz;
        CvStereoBMState *BMState = cvCreateStereoBMState();
//        BMState->preFilterType=0;
//        BMState->preFilterSize=21;
//        BMState->preFilterCap=31;
//        BMState->SADWindowSize=11;
//        BMState->minDisparity=0;
//        BMState->numberOfDisparities=128;
//        BMState->textureThreshold=20;
//        BMState->uniquenessRatio=5;
        printf("Please Input the Middle Point of the Object...");
        scanf("%f %f %f",&movx,&movy,&movz);
        totpoints=0;
        memset(CompDarr,0,sizeof(CompDarr));
        memset(MidDistance,0,sizeof(MidDistance));
        origin.clear();
//        for (i=0; i<4; i++) {
//           scanf("%s",buf1);
//           scanf("%s",buf2);
//           imageNames[0].push_back(buf1);
//           imageNames[1].push_back(buf2);
//        }
 //       char *ptr;
        for( i=0; i<1; i++)//z axis rotation
        {
            kyinp=-1;
            IplImage *img1=cvLoadImage("left.bmp",0);
            IplImage *img2=cvLoadImage("right.bmp",0);
//            FILE *fs=fopen("imgine.txt","w+");
//            for (int j=0; j<img1->height; j++) {
//               ptr=(img1->imageData)+j*img1->widthStep;
//                for (int k=0; k<img1->width; k++) fprintf(fs,"%d ",*(ptr++));
//                fprintf(fs,"\n");
//            }
//            fclose(fs);
           // CvMat mathdr, *sav = cvGetMat( img1, &mathdr);
            if( img1 && img2 )
            {
//               //cv_save_vector(sav,"img1.txt",0);
                cvRemap( img1, img1r, mx1, my1 );
                cvRemap( img2, img2r, mx2, my2 );
//                cv_save_vector(img1r,"img1r.txt",0);
//                cv_save_vector(img2r,"img2r.txt",0);
//
//                cvShowImage( "img1r", img1r );
//                cvShowImage( "img2r", img2r );
//                cvSaveImage("img1r.bmp",img1r,0);
//                cvSaveImage("img2r.bmp",img2r,0);
//                cvFindStereoCorrespondenceBM( img2r, img1r, disp,
//                    BMState);
//                xyz=cvCreateMat(disp->height,disp->width,CV_32FC3);
//                cvReprojectImageTo3D(disp, xyz, &_Q, true);
//                printf("1111 %lf\n",_Q.data.db[14]);
//                cvSave("xyz.xml",xyz);
//                cvNormalize( disp, vdisp, 0, 150, CV_MINMAX );
//                cvSaveImage("disp.bmp",disp,0);
//                cv_save_vector(disp,"disp.txt",3);
//                cvNamedWindow( "disparity" );
//                cvShowImage( "disparity", disp );
                FindthePoints(img2r);  //According to ROI to fix
                //GetArrangement();
                printf("%d\n",spoint.size());
                Rotate(xyz,2,i,movx,movy,movz); //1:x 2:y 3:z i:angle
                if( cvWaitKey() == 27 )
                    break;
                }
            cvReleaseImage( &img1 );
            cvReleaseImage( &img2 );
            }
        for( i = 0; i < 0; i++ )//x axis rotation
        {
            kyinp=-1;
            IplImage *img1=cvLoadImage(imageNames[0][i].c_str(),0);
            IplImage *img2=cvLoadImage(imageNames[1][i].c_str(),0);
            if( img1 && img2 )
            {
                cvRemap( img1, img1r, mx1, my1 );
                cvRemap( img2, img2r, mx2, my2 );
                cvFindStereoCorrespondenceBM( img1r, img2r, disp,
                    BMState);
                xyz=cvCreateMat(disp->height,disp->width,CV_32FC3);
                cvReprojectImageTo3D(disp, xyz, &_Q, true);
                cvNormalize( disp, vdisp, 0, 256, CV_MINMAX );
                cvNamedWindow( "disparity" );
                cvShowImage( "disparity", disp );
                FindthePoints(img2,xyz);  //According to ROI to fix
                //GetArrangement();
                Rotate(xyz,1,i,movx,movy,movz); //1:x 2:y 3:z i:angle mm
                }
            cvReleaseImage( &img1 );
            cvReleaseImage( &img2 );
            }
        Savearr();
        cvReleaseStereoBMState(&BMState);
        cvReleaseMat(&xyz);
        cvReleaseMat( &mx1 );
        cvReleaseMat( &my1 );
        cvReleaseMat( &mx2 );
        cvReleaseMat( &my2 );
        cvReleaseMat( &img1r );
        cvReleaseMat( &img2r );
        cvReleaseMat( &disp );

    } else
       {
        int Relation[15];
        char buf1[1024],buf2[1024];
        float posx,posy,posz,movx,movy,movz;
        //load CompDarr[]距离矩阵; origin坐标矩阵 MiddPoint & distance
//Setup for finding stereo corrrespondences
        printf("Please input your images~ Left and Right\n");
        //scanf("%d\n",&nframes);
        scanf("%s",buf1);
        scanf("%s",buf2);
        nframes=1; //changgggggggggggggggg
        Loadarr();
        CvStereoBMState *BMState = cvCreateStereoBMState();
        assert(BMState != 0);
        BMState->preFilterSize=41;
        BMState->preFilterCap=31;
        BMState->SADWindowSize=41;
        BMState->minDisparity=-64;
        BMState->numberOfDisparities=128;
        BMState->textureThreshold=10;
        BMState->uniquenessRatio=15;
        for( i = 0; i < nframes; i++ )//nframes fix
        {
            kyinp=-1;
            IplImage* img1=cvLoadImage(buf1,0);//imageNames[0][i].c_str()
            IplImage* img2=cvLoadImage(buf2,0);//imageNames[1][i].c_str()
            printf("Need Select an area?(Y/N)\n");
            scanf("%c",&kyinp);
            if (kyinp=='Y') {
             Select_an_area(img1);
             Select_an_area(img2);
            }
            memset(dist,0,sizeof(dist));
            if( img1 && img2 )
            {
                CvMat part;
                cvRemap( img1, img1r, mx1, my1 );
                cvRemap( img2, img2r, mx2, my2 );
                cvShowImage("eeeee",img1r);
                if( !isVerticalStereo || useUncalibrated != 0 )
                {
              // When the stereo camera is oriented vertically,
              // useUncalibrated==0 does not transpose the
              // image, so the epipolar lines in the rectified
              // images are vertical. Stereo correspondence
              // function does not support such a case.
                    cvFindStereoCorrespondenceBM( img1r, img2r, disp,
                        BMState);
                    CvMat* xyz=cvCreateMat(disp->height,disp->width,CV_32FC3);
                    cvReprojectImageTo3D(disp, xyz, &_Q, true);
              //      FindthePoints(img1,xyz);//According to ROI to fix
              //      CalculateDist(dist,xyz);
                    //load standard mat
              //      ComparetoStandardD(dist,Relation);
             //      GetthePosition(Relation,&posx,&posy,&posz,&movx,&movy,&movz,xyz);
               //     printf("位移（像素）：%f %f %f\n",movx,movy,movz);
               //     printf("旋转（°）：x轴%f y轴%f z轴%f\n",posx,posy,posz);
                    cvNormalize( disp, vdisp, 0, 256, CV_MINMAX );
                    cvNamedWindow( "disparity" );
                    cvShowImage( "disparity", vdisp );
                    cvSaveImage("disparity.bmp",vdisp);
                    cvShowImage("3D",xyz);
                    cvSave("xyz.xml",xyz);
                    //assert(right or wrong);

                    cvReleaseMat(&xyz);

                }
//                if( !isVerticalStereo )
//                {
//                    cvGetCols( pair, &part, 0, imageSize.width );
//                    cvCvtColor( img1r, &part, CV_GRAY2BGR );
//                    cvGetCols( pair, &part, imageSize.width,
//                        imageSize.width*2 );
//                    cvCvtColor( img2r, &part, CV_GRAY2BGR );
//                    for( j = 0; j < imageSize.height; j += 16 )
//                        cvLine( pair, cvPoint(0,j),
//                        cvPoint(imageSize.width*2,j),
//                        CV_RGB(0,255,0));
//                }
//                else
//                {
//                    cvGetRows( pair, &part, 0, imageSize.height );
//                    cvCvtColor( img1r, &part, CV_GRAY2BGR );
//                    cvGetRows( pair, &part, imageSize.height,
//                        imageSize.height*2 );
//                    cvCvtColor( img2r, &part, CV_GRAY2BGR );
//                    for( j = 0; j < imageSize.width; j += 16 )
//                        cvLine( pair, cvPoint(j,0),
//                        cvPoint(j,imageSize.height*2),
//                        CV_RGB(0,255,0));
//                }
//                cvShowImage( "rectified", pair );
                if( cvWaitKey() == 27 )
                    break;
            }
            cvReleaseImage( &img1 );
            cvReleaseImage( &img2 );

        }
        cvReleaseStereoBMState(&BMState);
        cvReleaseMat( &mx1 );
        cvReleaseMat( &my1 );
        cvReleaseMat( &mx2 );
        cvReleaseMat( &my2 );
        cvReleaseMat( &img1r );
        cvReleaseMat( &img2r );
        cvReleaseMat( &disp );
    }
}

int main(void)
{
    StereoCalib("asd.txt", 13, 11, 0);
    return 0;
}
