/*
 Contributed by Lsxy
 March, 25, 2016
 */ 
#ifndef STEREO_VISION_H
#define STEREO_VISION_H

#include <vector>
//#include <Windows.h>
#include <algorithm>
#include <cstring>
#include "ok_capture.h"

class Monocular
{
    private:
        double posx, posy, posz;
        double movx, movy, movz;
    public:
        Monocular();
        ~Monocular();
};

class Stereo
{
    private:
        float aDist[15][15];
        double m_dPosX, m_dPosY, m_dPosZ;
        double m_dMovX, m_dMovY, m_dMovZ;
        CvMat* m_pcmDisp;
        CvMat* m_pcmXyz;
        vector<int> vecPoint;
        IplImage* m_pGraphic_left, m_pGraphic_right;

    public:

        Stereo();
        ~Stereo();
        friend Select_an_Area(IplImage * );





    

};

void Select_an_Area(IplImage * )
CvPoint floods(int, int, char * , int )
void Find_Points(IplImage *, const CvMat* )
void Stereo_Calib(const char* , int, int, int )
CvMat* Cv_Load_Vector(char*, int )
int Cv_Save_Vector(CvMat*, char*, int )
void Load_Parameters()
void Stereo_Position()
void CalculateDist(float temp[][15], const CvMat*)
inline bool cmp(const int aa, const int bb)
void ComparetoStandardD(const float temp[][15], int relat[15])
void GetthePosition(int relat[], float *, float *, float *, float *, float *, float *, const CvMat* )
void Rotate(const CvMat*, int, int, float, float, float)
void GetArrangement(float *, float *, float *, const CvMat* )

#endif //Stereo.h






