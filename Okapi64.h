
//---------------okapi64.h---------------------------------
//
// ok api32 header file for user
//
//---------------------------------------------------------

#ifndef __JOINHOPE__
#define __JOINHOPE__


//----data type defines ------------------------------------------------------------------------------------

//--datatype defines and bytes 
//	signed			signed pointer		unsigned		unsigned pointer
//	CHAR (1)		LPSTR (4/8)			BYTE (1)		LPBYTE (4/8)
//	SHORT (2)		LPSHORT	((4/8)		WORD (2)		LPWORD (4/8)
//	INT32 (4)		LPINT32	(4/8)		DWORD (4)		LPDWORD (4/8)
//	LONG64 (8)		LPLONG64 (4/8)		QWORD (8)		LPQWORD (4/8)
//  MLONG (4/8)		LPMLONG (4/8)		MWORD (4/8)		LPMWORD (4/8)
//  VOID			HANDLE, HWND, LPVOID (4/8)
//	note: (4/8) mean: 4 bytes in 32bits OS, 8 in 64bits OS 

//-----fixed size data define (exclude pointers)
//1 byte prototype: (unsigned) char, __int8
typedef		char				CHAR,	*PCHAR, *LPSTR;  
typedef		unsigned char		BYTE,	*PBYTE, *LPBYTE;  

//2 byte prototype: (unsigned) short, __int16
typedef		short				SHORT,	*PSHORT, *LPSHORT;  
typedef		unsigned short		WORD,	*PWORD,	*LPWORD;  

//4 byte prototype: (unsigned) long, int, __int32
typedef		int					INT32,	*PINT32, *LPINT32;;  
typedef		unsigned long		DWORD, *PDWORD, *LPDWORD;  
//typedef		unsigned __int32	DWORD; //

typedef		float				FLOAT,	*PFLOAT, *LPFLOAT ; //float 

//8 byte prototype:  __int64, long, int
typedef		__int64				LONG64,	*PLONG64, *LPLONG64;
typedef		unsigned __int64	QWORD,	*PQWORD, *LPQWORD;

typedef		double				DOUBLE, *PDOUBLE, *LPDOUBLE;   //float

typedef		void				*HANDLE, *LPVOID;    //which are 4 bytes in 32 system, 8 bytes in 64 system 
//typedef	void				*HWND;    //which are 4 bytes in 32 system, 8 bytes in 64 system 

//-----unfixed size data defines 
typedef		int					BOOL;	//32 or 64 not sure


//-----4 or 8 byte size depent on 32 or 64 OS
//#if     _INTEGRAL_MAX_BITS >= 64
#define		__64OS
//#endif

#ifdef		__64OS
	typedef		__int64				MLONG,	*PMLONG, *LPMLONG;  //which are 4 bytes in 32 system, 8 bytes in 64 system 
	typedef		unsigned __int64	MWORD,	*PMWORD, *LPMWORD;  //which are 4 bytes in 32 system, 8 bytes in 64 system 
	#define		MAX_VALID_FRAME 0x400000000000000
#else
	typedef		long long			MLONG,	*PMLONG, *LPMLONG;	//which are 4 bytes in 32 system, 8 bytes in 64 system 
	typedef		unsigned long long	MWORD,	*PMWORD, *LPMWORD;  //which are 4 bytes in 32 system, 8 bytes in 64 system 
	#define		MAX_VALID_FRAME 0x40000000
#endif


//----contant defines---------------------------------------------------------------------------------------------


//--defines of ok series image board identity
//Mono series
#define		OK_M10					1010
#define		OK_M10N					1010
#define		OK_M10M					1013
#define		OK_M10F					1011
#define		OK_M10L					1014
#define		OK_M10H					1012
#define		OK_M20					1020
#define		OK_M2KC					1021
#define		OK_M20H					1022
#define		OK_M30					1030
#define		OK_M40					1040
#define		OK_M50					1050
#define		OK_M60					1060
#define		OK_M70					1070
#define		OK_M80					1080
#define		OK_M80K					1081

//--new updated series
#define		OK_M10A					1212	//OK_M10M:1013
#define		OK_M10B					1213	//OK_M10L/N:1014
#define		OK_M10C					1214	//
#define		OK_M10D					1215
#define		OK_M10K					1218	//OK_M80K
#define		OK_M11K					1217	//


#define		OK_M20A					1222	//OK_M20H:1022
#define		OK_M20B					1223	//110M
#define		OK_M20C					1224	//160M
#define		OK_M20D					1225	//205M
#define		OK_M20G					1227	//oem

#define		OK_M40A					1240	//OK_M40:1041
#define		OK_M40B					1243	//110M
#define		OK_M40C					1244	//160M
#define		OK_M40D					1245	//205M

#define		OK_M60A					1260	//OK_M60:1060
#define		OK_M60B					1263	//110M
#define		OK_M60C					1264	//160M
#define		OK_M60D					1265	//205M

#define		OK_M50A					1250
#define		OK_M50B					1253
#define		OK_M50K					1258
#define		OK_M51K					1257

#define		OK_M30A					1230
#define		OK_M30B					1233
#define		OK_M30K					1238
#define		OK_M31K					1237
#define		OK_M70A					1270
#define		OK_M70B					1273



//Color series
#define		OK_C20					2020
#define		OK_C20C					2021
#define		OK_C30					2030
#define		OK_C32					2032
#define		OK_C30S					2031
#define		OK_C40					2040
#define		OK_C50					2050
#define		OK_C70					2070
#define		OK_C80					2080
#define		OK_C80M					2081
#define		OK_M90					1090

//RGB series
#define		OK_RGB10				3010
#define		OK_RGB20				3020
#define		OK_RGB30				3030

//Monitor Control series
#define		OK_MC01					4001
#define		OK_MC03					4003
#define		OK_MC10					4010
#define		OK_MC16					4016
#define		OK_MC20					4020
#define		OK_MC30					4030
#define		OK_MC32					4032


//--new updated series
#define		OK_C20A					2220
#define		OK_C23A					2221
#define		OK_C20B					2223
#define		OK_C21B					2225

#define		OK_C30A					2230
#define		OK_C31A					2231
#define		OK_C30B					2233
#define		OK_C40A					2240
#define		OK_C50A					2250
#define		OK_C60A					2260
#define		OK_C61A					2261
#define		OK_C60B					2263
#define		OK_C60C					2264
#define		OK_C80A					2280
#define		OK_C80K					2288

#define		OK_RGB10A				3210
#define		OK_RGB11A				3211
#define		OK_RGB10B				3213

#define		OK_RGB20A				3220
#define		OK_RGB21A				3221
#define		OK_RGB20B				3223
#define		OK_RGB20C				3225

#define		OK_RGB30A				3230
#define		OK_RGB30B				3233
#define		OK_RGB30C				3235

#define		OK_RGB60A				3260
#define		OK_RGB60B				3263
#define		OK_RGB60C				3265


#define		OK_VGA40A				3240
#define		OK_VGA40B				3243


#define		OK_MC10A				4210
#define		OK_MC12A				4212
#define		OK_MC16A				4216

#define		OK_MC20A				4220

#define		OK_MC20B				4223
#define		OK_MC40B				4243
#define		OK_MC40B_E				4343



//---pc/104+ series
#define		OK_PC10A				5210 // color
#define		OK_PC16A				5216 //16 channles
#define		OK_PC20A				5221 //two cards

//PC 51--
#define		OK_PC20B				5123 //dvd compress


//PR 53--
#define		OK_PR10B				5313 //->rgb10b 
#define		OK_PR20B				5323 //->rgb20b 
#define		OK_PR30A				5330 //->rgb30a 
#define		OK_PR170				5317 //
#define		OK_PR30B				5333 //->rgb30b 

#define		OK_PR30C				5334 //->rgb30b/two input  **
#define		OK_PR40A				5340 //out rgb signal **
#define		OK_PR43A				5343 //out 2 comp signal 
#define		OK_PR50A				5350 //standard out rgb 

#define		OK_PR60C				5364  //rgb60c with cmp


//PM 54--
#define		OK_PM10A				5410 //->m10a 
#define		OK_PM10B				5413 //->m10b 
#define		OK_PM20A				5420 //->m20a 
#define		OK_PM20B				5423 //->m20b 

#define		OK_PM20D				5426 //->613 special signal cap 

#define		OK_PM50G				5457 //->m50G special 


//---usb series
#define		OK_USB10A				5110	//line scan capture

//USB 52--
#define		OK_USB20A				5220	//std color
#define		OK_USB20B				5223	//non-std b/w capture

//--3.0
//USB3.0 52-- (std)
#define		OK_USB30A				5231	//USB3.0 capture and compress
#define		OK_USB30B				5233	//USB3.0 capture, compress and support sdi


//---cPCI series
#define		OK_CPC16A				5230  //color 16 channels
#define		OK_CPC50B				5253  //m50b 
#define		OK_CPC50G				5257  //m20g oem
#define		OK_CPC66C				5266  //rgb66c /6u with cmp

#define		OK_CPC60C				5365  //rgb60c /3u with cmp 53S
#define		OK_CPC61C				5361  //rgb61c //3u rgb/dvi. comp with 60c
#define		OK_CPC62C				5362  //rgb62c //3u camlink
#define		OK_CPC63C				5363  //rgb61c //3u lvds

#define		OK_CPC61E				5367  //same as cpc61c (eqaul to PCI-4E)


//CPCI 55--
#define		OK_CPC12A				5512  //color 12 difference channels

#define		OK_CPC40C				5540  //rgb41c lvds(camlink) output /3u 
#define		OK_CPC41C				5541  //rgb41c div output /3u 
#define		OK_CPC41D				5544  //incl. OK_CPC41C
#define		OK_CPC40D				5542  //(cpc40c revised) lvds output /3u 

#define		OK_CPC43C				5543  //rgb41c /3u 4 comp  



//---Digital series
#define		OK_CL20A				6120	//CameraLink 
#define		OK_CL40A				6140	//
#define		OK_CL60A				6160	//
#define		OK_CL20B				6123	//
#define		OK_CL40B				6143	//
#define		OK_CL60B				6163	//

#define		OK_CL20C				6125	//
#define		OK_CL40C				6145	//


#define		OK_LV20A				6220	//LVDS
#define		OK_LV21B				6224	//LVDS mpeg-II like c21b 
#define		OK_LV40A				6240	//

#define		OK_LV50A				6250	//just output

#define		OK_LV60A				6260	//
#define		OK_LV20B				6223	//
#define		OK_LV40B				6243	//
#define		OK_LV60B				6263	//

#define		OK_LV20C				6225	//
#define		OK_LV40C				6245	//
#define		OK_LV20G				6227	//



//---Line scan series
#define		OK_LS100A				7100	//LS basic type for Analog
#define		OK_LS100B				7103	//LS second type for Analog
#define		OK_LS120A				7120	//LS basic type for CameraLink
#define		OK_LS120B				7123	//LS second type for CameraLink
#define		OK_LS200A				7200	//LS second class for Analog



//---FireWare (1394) series
#define		OK_FW10A				8410

//---fibre optical series
#define		OK_FB10A				8510

//---GigE net series
#define		OK_GN10A				8610
#define		OK_GN20A				8620
#define		OK_GN40A				8640  //4 channs
#define		OK_GN40A_E				8643  //PCI_E 4 channs
#define		OK_GN40A_4E				8644  //PCI_E 4 channs



//---DSP series
#define		OK_DSP100A				9100	//DSP capture process




//---PCI-Express and PCI_X series
//Mono series

//#define		OK_M40A_E				1340	//
//#define		OK_M40B_E				1343	//

//#define		OK_M20B_4E				1423	//
//#define		OK_M40B_4E				1443	//

#define		OK_M10B_E				1313	//  
#define		OK_M10K_E				1318	//
#define		OK_M20A_E				1322	//
#define		OK_M20B_E				1323	//

#define		OK_M30B_E				1333	//
#define		OK_M50B_E				1353	//


//Color series
#define		OK_C20A_E				2320
#define		OK_C21B_E				2325
#define		OK_C30A_E				2330

#define		OK_C61A_E				2361	
#define		OK_C60B_E				2363

//RGB series
#define		OK_RGB20A_E				3320
#define		OK_RGB20B_E				3323
#define		OK_RGB30A_E				3330
#define		OK_RGB30B_E				3333

#define		OK_RGB60A_E				3360 //cmp
#define		OK_RGB60B_E				3363 //cmp
#define		OK_RGB60C_E				3365 //cmp
#define		OK_RGB61C_E				3366 //DVI/cmp
#define		OK_RGB60C_X				3565 //cmp
#define		OK_RGB61C_X				3566 //DVI/cmp

//4E
#define		OK_RGB61C_4E			3466 //DVI/VGA/cmp


#define		OK_VGA40A_E				3340
#define		OK_VGA41A_E				3341 //DVI/VGA 
#define		OK_VGA40B_E				3343
#define		OK_VGA41B_E				3344 //DVI/VGA

#define		OK_VGA51A_E				3351 //DVI/AUDIO


#define		OK_VGA40B_4E			3443 //PCI-4E
#define		OK_VGA40B_4E			3443 //PCI-4E two vga
#define		OK_VGA41A_4E			3441 //PCI-4E single DVI/VGA
#define		OK_VGA41B_4E			3444 //PCI-4E two DVI/VGA

#define		OK_VGA41C_4E			3445 //PCI-4E HR DVI

//X
#define		OK_VGA40A_X				3540
#define		OK_VGA41A_X				3541 //DVI
#define		OK_VGA40B_X				3543
#define		OK_VGA41B_X				3544 //DVI


//Monitor Control series
#define		OK_MC10A_E				4310	//
#define		OK_MC12A_E				4312	//

#define		OK_MC30A_E				4330	
#define		OK_MC20B_E				4323  
#define		OK_MC40B_E				4343

//4E
#define		OK_MC40A_4E				4440	//
#define		OK_MC40C_4E				4445	// with comp

#define		OK_MC50A_4E				4450	// 4+1


//CameraLink 
#define		OK_CL20A_E				6320	//CameraLink 
#define		OK_CL40A_E				6340	//
#define		OK_CL60A_E				6360	//
#define		OK_CL20B_E				6323	//
#define		OK_CL40B_E				6343	//
#define		OK_CL60B_E				6363	//

//LVDS-RS644
#define		OK_LV20A_E				6420	//LVDS
#define		OK_LV40A_E				6440	//
#define		OK_LV60A_E				6460	//
#define		OK_LV20B_E				6423	//
#define		OK_LV40B_E				6443	//
#define		OK_LV50A_E				6450
#define		OK_LV60B_E				6463	//

//SDI
#define		OK_SDI40A_E				6540	//SDI 
#define		OK_SDI40B_E				6543	//H_SDI 


//---Ok sereis Camera-----------------------------------
//LVDS-RS644
#define		OK_AM1530				201530 //LVDS


//USB2.0 series
#define		OK_AM1110				201110 //USB
#define		OK_AM1111				201111
#define		OK_AM1310				201310

#define		OK_AM1410				201410
#define		OK_AM1510				201510

#define		OK_AM9010				209010 //for DR
#define		OK_GAM1610				601610 //for DR
#define		SUPERCCD_1680			201680	//$
#define		SUPERCCD_1681			201681	//$

#define		OK_AC1210				211210
#define		OK_AC1310				211310

#define		OK_AC168				210168 //oem
#define		OK_AC168N				210169 //oem

#define		OK_SM1310				301310
#define		OK_SC1310				311310
#define		OK_SC2010				312010
#define		OK_SC3010				313010

#define		OK_SM130				300130 //oem

#define		OK_GSM1410				701410
#define		OK_GSC1410				711410
#define		NS_DR16					700116

//FireWare(1394)
#define		OK_AM1340				201340 //1394

//Optical
#define		OK_AC1350				211350 //

//GigE series (Remote net camera)
#define		OK_AM1160				201160 //
#define		OK_AM1161				201161 //
#define		OK_AM1360				201360 //
#define		OK_AM1460				201460 //
#define		OK_AM1560				201560 //
#define		OK_AM1561				201561 // plus

#define		OK_AM1565				201565 // special for x-ray for GE
#define		OK_AM1566				201566 // special for x-ray 

#define		OK_AM1666				201666 // Kodak sereis

//--67 (GIGE 29x29 sereis)

#define		OK_AM2067				202067 // ccd b/w 2.0 M Pixels
#define		OK_AC2067				212067 // ccd color 2.0 M Pixels
#define		OK_AM2167				202167 // ccd b/w 2.0 M Pixels (double channels)
#define		OK_AC2167				212167 // ccd color 2.0 M Pixels (double channels)

#define		OK_AM5067				205067 // ccd b/w 5.0 M Pixels
#define		OK_AC5067				215067 // ccd color 5.0 M Pixels

#define		OK_GAM2967				602967 // ccd b/w 29 M Pixels

#define		OK_SM1367				301367 // cmos b/w 1.3 M Pixels
#define		OK_SC1367				311367 // cmos color 1.3 M Pixels

#define		OK_SM5067				305067 // cmos b/w 5.0 M Pixels
#define		OK_SC5067				315067 // cmos color 5.0 M Pixels



//--68 (kodak ccd) (GIGE 2-channel sereis )

//--1023x1024
#define		OK_AM1168				201168 // (5.5, b/w, 1 MPix, single net interface) # 
#define		OK_AC1168				211168 // (5.5, color, 1 MPix, single) #
#define		OK_AM1268				201268 // (5.5, b/w, 1 MPix, double)  # 
#define		OK_AC1268				211268 // (5.5, color, 1 MPix, double) #

//--(1600x1200)
#define		OK_AM2168				202168 // (5.5, b/w, 2 MPix, single)  
#define		OK_AC2168				212168 // (5.5, color, 2 MPix, single)
#define		OK_AM2268				202268 // (5.5, b/w, 2 MPix, double)  
#define		OK_AC2268				212268 // (5.5, color, 2 MPix, double)

//--(1920x1080)
#define		OK_AM2368				202368 // (5.5, b/w, 2 MPix, single) # 
#define		OK_AC2368				212368 // (5.5, color, 2 MPix, single) 
#define		OK_AM2468				202468 // (5.5, b/w, 2 MPix, double)  #
#define		OK_AC2468				212468 // (5.5, color, 2 MPix, double) #

//--(1920x1080)
#define		OK_AM2568				202568 // (7.4, b/w, 2 MPix, single)  
#define		OK_AC2568				212568 // (7.4, color, 2 MPix, single)
#define		OK_AM2668				202668 // (7.4, b/w, 2 MPix, double) # 
#define		OK_AC2668				212668 // (7.4, color, 2 MPix, double) #

//--(2336x1752)
#define		OK_AM4168				204168 // (5.5, b/w, 4 MPix, single)  
#define		OK_AC4168				214168 // (5.5, color, 4 MPix, single)
#define		OK_AM4268				204268 // (5.5, b/w, 4 MPix, double) # 
#define		OK_AC4268				214268 // (5.5, color, 4 MPix, double) #

//--(2048x2048)
#define		OK_AM4568				204568 // (7.4, b/w, 4 MPix, single)  
#define		OK_AC4568				214568 // (7.4, color, 4 MPix, single)
#define		OK_AM4668				204668 // (7.4, b/w, 4 MPix, double) # 
#define		OK_AC4668				214668 // (7.4, color, 4 MPix, double) #

//--(3296x2472)
#define		OK_AM8168				208168 // (5.5, b/w, 8 MPix, single)  
#define		OK_AC8168				218168 // (5.5, color, 8 MPix, single)
#define		OK_AM8268				208268 // (5.5, b/w, 8 MPix, double) # 
#define		OK_AC8268				218268 // (5.5, color, 8 MPix, double) #


//--(4896x3264)
#define		OK_GAM1568				601568 // (5.5, b/w, 16 MPix, single)  
#define		OK_GAC1568				611568 // (5.5, color, 16 MPix, single)
#define		OK_GAM1668				601668 // (5.5, b/w, 16 MPix, double) # 
#define		OK_GAC1668				611668 // (5.5, color, 16 MPix, double) 

//--(4864x3232)
#define		OK_GAM1768				601768 // (7.4, b/w, 16 MPix, single)  
#define		OK_GAC1768				611768 // (7.4, color, 16 MPix, single)
#define		OK_GAM1868				601868 // (7.4, b/w, 16 MPix, double) # 
#define		OK_GAC1868				611868 // (7.4, color, 16 MPix, double) 


//--(6576x4384)
#define		OK_GAM2568				602568 // (5.5, b/w, 29 MPix, single)  
#define		OK_GAC2568				612568 // (5.5, color, 29 MPix, single)
#define		OK_GAM2668				602668 // (5.5, b/w, 29 MPix, double) # 
#define		OK_GAC2668				612668 // (5.5, color, 29 MPix, double) 


//--

#define		OK_AM1751				201751 // e2v sereis, customized

#define		OK_AC1260				211260 //
#define		OK_AC1360				211360 //
#define		OK_AC1361				211361 //

//--Linear scan with GigE interface
#define		OK_LM1060				101060 //1K
#define		OK_LM2060				102060 //2K
#define		OK_LC2060				112060 //2K color
#define		OK_LC4060				114060 //4K color


//------------------------------------
//Intellegent Camera 
#define		OK_IC1200				181200 //
#define		OK_IC1500				181500 //

#define		OK_IM1161				171161 // b/w 640x480, Gnet
#define		OK_IM1160				171160 // b/w 7680x576, Gnet


//------Embeded parts-------------------------------------
//GigE converter 
#define		OK_GE130				10130 //
#define		OK_GE400				10400 //

#define		OK_DX160				21160 // GigE CCU (type I)
#define		OK_DX260				21260 // GigE CCU (type II)


//EtherNet  
#define		OK_HD120				41120 //HighDensity stream


//--error code

#define		ERR_NOERROR				0	//no error 
#define		ERR_NOTFOUNDBOARD		1	//not found available ok board 

#define		ERR_NOTFOUNDVXDDRV		2	//not found ok vxd/ntsys driver
#define		ERR_NOTALLOCATEDBUF		3	//not pre-allocated buffer from host memory
#define		ERR_BUFFERNOTENOUGH		4	//available buffer not enough for requirment
#define		ERR_BEYONDFRAMEBUF		5	//capture iamge size beyond buffer

#define		ERR_NOTFOUNDDRIVER		6	//not found the driver responded the card
#define		ERR_NOTCORRECTDRIVER	7	//the needed driver not correct

#define		ERR_MEMORYNOTENOUGH		8	//host memory not enough for DLL
#define		ERR_FUNNOTSUPPORT		9	//the function is not supported
#define		ERR_OPERATEFAILED		10	//something wrong with this function call

#define		ERR_HANDLEAPIERROR		11	//the handle to okapi32 function wrong
#define		ERR_DRVINITWRONG		12	//something wrong with this card's driver on initializing

#define		ERR_RECTVALUEWRONG		13	//the rect's parameters set wrong
#define		ERR_FORMNOTSUPPORT		14	//the format set not supported by this board

#define		ERR_TARGETNOTSUPPORT	15	//the target not support by this function

#define		ERR_NOSPECIFIEDBOARD	16	//not found specified board correctly sloted


//--format defines
#define		FORM_RGB888				1	//rgb format
#define		FORM_RGB565				2
#define		FORM_RGB555				3
#define		FORM_RGB8888			4
#define		FORM_RGB332				5	
#define		FORM_RGB8886			18	

#define		FORM_YUV422				6	//yuu format
#define		FORM_YUV411				7	
#define		FORM_YUV16				8	
#define		FORM_YUV12				9	
#define		FORM_YUV9				10	
#define		FORM_YUV8				11	

#define		FORM_GRAY888			12	//gray format
#define		FORM_GRAY8888			13
#define		FORM_GRAY8				14
#define		FORM_GRAY10				15
#define		FORM_GRAY12				16
#define		FORM_GRAY14				17
#define		FORM_GRAY16				19
#define		FORM_GRAY4				20	 //added 090822
#define		FORM_GRAY12PACKED		21	 //added 110602
#define		FORM_GRAY10PACKED		22	 //added 110602

#define		FORM_BAYGR8				23	//bayer format
#define		FORM_BAYRG8				24	//added on 20140926
#define		FORM_BAYGB8				25	
#define		FORM_BAYBG8				26	

#define		FORM_BAYGR16			27	
#define		FORM_BAYRG16			28	
#define		FORM_BAYGB16			29	
#define		FORM_BAYBG16			30	



//--mask command
#define		MASK_DISABALE			0	//turn of mask
#define		MASK_POSITIVE			1	//0 win clients visible, 1 video visible
#define		MASK_NEGATIVE			2	//0 for video 1 for win client (graph)


//--tv system standard
#define		TV_PALSTANDARD			0	//PAL
#define		TV_NTSCSTANDARD			1	//NTSC
#define		TV_NONSTANDARD			2	//NON_STD
#define		TV_HDTVSTANDARD			3	//HDTV_STD
//#define		TV_SECAMSTANDARD		4	//SECAM

//--HDTV sub mode
#define		HDTV_480P				1  //480 Lines Progressive scan
#define		HDTV_720P				2  //720 Lines Progressive scan
#define		HDTV_1080I				3  //1080 Lines Interlaced scan
#define		HDTV_1080P				4  //1080 Lines Interlaced scan


#define		TV_PALMAXWIDTH			768
#define		TV_PALMAXHEIGHT			576

#define		TV_NTSCMAXWIDTH			640 //720
#define		TV_NTSCMAXHEIGHT		480


//-----defines lParam for get param
#define		GETCURRPARAM			-1 

//-----sub-function defines for wParam of SetVideoParam
		//wParam cab be one of the follow
#define		VIDEO_RESETALL			0 //reset all video params (including lut) to sys default
#define		VIDEO_SOURCECHAN		1 //lParam=0,1.. Comp.Video; 0x100,101...to Y/C(S-Video), 0x200,0x201 to RGB/VGA Chan.Input,
									  //0x400,0x401 to YPrPb, 0x800,0x801 to DVI/HDMI, 0x900,0x901 to HDMI(YPbPr)
									// return max support Comp. Video channles when input lParam=0xff.
									// as same way return max s-video channeles when input lParam=0x1ff.
									// and return max rgb/vga video channeles when input lParam=0x2ff.
									// and return max YPrPb video channeles when input lParam=0x4ff.
									// and return max DVI/HDMI(RGB) channeles when input lParam=0x8ff.
									// and return max HDMI(YPbPr) channeles when input lParam=0x9ff.
									// and return max SDI channeles when input lParam=0xAff.

#define		VIDEO_BRIGHTNESS		2  //LOWORD is brightness, for RGB HIWORD is channel (0:red, 1:green, 2:blue)
#define		VIDEO_CONTRAST			3  //LOWORD is contrast, for RGB HIWORD is channel (0:red, 1:green, 2:blue)
							
#define		VIDEO_COLORHUE			4  //LOWORD is hue of color,  for YUV/YPbPr HIWORD=0 is for V/Pr, =1 for U/Pb 
#define		VIDEO_SATURATION		5  //LOWORD is saturation of color,  for YUV/YPbPr HIWORD=0 is for V/Pr, =1 for U/Pb
#define		VIDEO_GAINADJUST		18 //gain adjust, some card support range 0~255, some just 0~3
#define		VIDEO_PHASEADJUST		20 //phase adjust  

#define		VIDEO_RGBFORMAT			6  //when return, low word is format code 
#define		VIDEO_TVSTANDARD		7  //Main Mode:LOWORD 0 PAL, 1 NTSC, 2 Non-stadard, HDTVSTANDARD,
									   //Sub Mode: HIWORD 1 HDTV_480P, ...
#define		VIDEO_SIGNALTYPE		8  //LOWORD 0: non-interlaced(progressive), 1: interlaced
									   //2:single pixel, 3:double pixels (for digital signal), 4:double pixels (inverse pixel order)
									   //5:treble pixels, 6:quad pixels
									   //HIWORD: trigger mode(/slot in field header for standard tv video). 
									   //0= continu. signal, =1: edge-trigger(hardware or software)
									   //for camera, =2: pulse width trigger, =3: extern sync mode, =4: keep one after trigger, 
									   //=5: keep last one after trigger,   
									   //when lParam==-2, just get capability of LOWORD, if return -1: not support; 
									   //when lParam==-3, just get capability of HIWORD, if return -1: not support
#define		VIDEO_SYNCSIGCHAN		10 //LOWORD: sync line of source channel, =0:Red,1:Grn,2:Blue, 3:Sync,4 H.V.Sync; 
									   //usually =1, 3 or 4 when vidoe input is rgb source
									   //HIWORD: which source the sync line is in. =0,1,..for RGB source 0,1,  
									   //=0x100,0x101,... for comp.video source 0,1, (in this case(comp. video) LOWORD has no mean more)
#define		VIDEO_IMAGESOURCE		36 //LOBYTE(BYTE0): 0(defalut): alive(continuous), 1: last image, 2:black, 3: white, 4: vertical-lines
									   //5: vertical-lines changing
										//BYTE1: select chann(based 0) to store or process 
										//b16=1: test light on, b17:histogram in pic

#define		VIDEO_AUXMONCHANN		11 //BYTE0(LOBYTE): monitor video source chann on aux monitor for MC30
									   //BYTE1(B8:15) special for LV20G
									   //HIWORD select out video channel to set (when more than 1, e.g for CCU)

#define		VIDEO_RECTSHIFT			9  //video rect shift,  makelong (x,y)
#define		VIDEO_RECTSHIFTEX		19 //general video active rect shift for all,  makelong (x,y)
									   //replace for VIDEO_RECTSHIFT 

#define		VIDEO_AVAILRECTSIZE		12 //makelong(horz,vert). horz available pixels per scan line and
									   //vert available lines per frame
#define		VIDEO_FREQSEG			13 // set video pixel frequency range
									   //LOWORD, 0:Low(7.5~15MHz),  1:middle(15~30), 2:High(30~60), 3:ultra(60~120), 4:very(>120)
									   //HIWORD =1: mean LOWORD is specified frequency value (like 120 mean 120MHz)

#define		VIDEO_MISCCONTROL		16 //miscellaneous control bits
									   //b0:-satur, b1:- contr (for c20, c30)
									   //b2:agc, b3:gama, b4:b/w, 
									   //b5:fast mode (for c30b)
									   //b6: 1: std high bit mode (like 12...), 0: compatible 10b mode for LVDS digital input
									   //b7: 1: switch off auto zoom mode when output is different from source  
									   //b8: 1:VTR mode, 0:auto mode
									   //b9: 1: close language select, 0:open
									   //b10: 1: inverse gray of out video
									   //b11: 1: enable electronic circle
									   //b12: 1: enable rect to measure visible


#define		VIDEO_ENABLEGRAPHS		17 //enable graph. LOBYTE,b0: enable cap-graph; b1: enb out-graph.  HIBYTE graph frm sel  

//--set local device 
#define		VIDEO_SETBADPTNUM		25 // set number of bad point, LOWORD= number(based 1).  and  HIWORD=1 mean enable removing bad points, 0: stay there	    //
#define		VIDEO_SETBADPTPOS		26 // set position of bad point, LOWORD=x, HIWORD=y

#define		VIDEO_TAGFRMCOUNT		27 // LOWORD=1: turn on frame counter and tag it to each frame, =0 off;
									   // HIWORD=1 reset frame counter
//
#define		VIDEO_SETWATCHDOG		28 // set watchdog, LOWORD= 0: off wotchdog, >0: enable, and it is timer in seconds(1~255)
#define		VIDEO_SETDATETIME		29 // set and get date and time embeded in hardware. lParam should be pointer to struct tm (include date and time) 
									   //if lParam=0, it will take current system date and time; You can get current datetime of device by setting tm.tm_year=-1 
#define		VIDEO_RESETSYSTEM		30 // bits0=1 restart system timer, b1=1: clear system's flash memory

#define		VIDEO_SETDELAYLINES		39 //(for LineScan camera) LOWORD is delay lines between two channels(R,G, or B), HIWORD =0 mena R is first channel, =1: B is first one  	

//--set params of video out to the other device

#define		VIDEO_OUTLINEPERIOD		14 //LOWORD out line period (in 0.54 us) generated by board, when HIWORD==0:in 0.54 us, =1: in 0.1 vs, =2: in 0.01vs ...
#define		VIDEO_LINEPERIOD		14 //LOWORD out line period (in 0.54 us) generated by board, when HIWORD==1:in 0.1 vs, ...
										

#define		VIDEO_OUTFRAMELINES		15 //out lines per frame generated by board, HIWORD is local sync
#define		VIDEO_FRAMELINES		15 //when outlines<2/3* caspture vert lines, it will zoom vert to half lines
										//for digtal cards
#define		VIDEO_OUTSIGNALTYPE		22 //out signal type, b0:in grn, b1:comp sync, b2:interlace
									   //b3:neg pole of horz sync;  
									   //b4~5:vert half or double. 0:auto set (when VIDEO_FRAMELINES set is 1/3 difference from CAPTURE_VERTLINES,
									   //0: auto-set, 1=forced half, 2=forced double, 3: forced set no
									   //b6:neg pole of vert sync
										//HIWORD: b16~19: b16=1: off out video channel 0, b17=1: off out video channel 1. etc.
										//(only two channs for cpc50b, 4 channs for PR43A)

#define		VIDEO_OUTHORZPIX		23 // set out horz total pixel on the monitor

#define		VIDEO_OUTXYOFFSET		24 // set offset of pixels in horz and vert directions on monitor 

#define		VIDEO_OUTHVSYNCWID		41 // set out horz and vert sync width (in unit pixle and line). LOWORD is for horz sync and HIWORD is for vert sync.

#define		VIDEO_OUTWINDOWLEVEL	40 // set gray level window's location (start pos) and its width for high bits (>8) 
									   // LOWORD = start gray level, HIWORD = width, turn of when HIWORD=0;


#define		VIDEO_OUTMODESET		38 //(for cpc41c)set mode of out video. LOWORD(mode)=0: no action, =1: pause, =2: resume, =3: forward when HIWORD>0, backward when HIWORD<0 
									   //=4: go to specified frame number(=HIWORD), =5: step to number(=HIWORD, may be negative) of frames relative to current frame 

#define		VIDEO_PARTIALMODE		31 //partial mode, 0(default): normal(global) mode, non 0: partial mode
									   //when partial mode LOWORD= start line number, HIWORD= end line number

//--set some rect to process
#define		VIDEO_ELLIPSEOFFSET		32	//ellipse(electronic circle) offset (left,top), makelong(horz left offset,vert top offset)
#define		VIDEO_ELLIPSESIZE		33	//ellipse(electronic circle) size (width, height), makelong(horz width, vert hight)

#define		VIDEO_MEASUREOFFSET		34	//for IBS, measure rect's offset (left,top), makelong(horz left offset,vert top offset) for IBS...
#define		VIDEO_MEASURESIZE		35	//for IBS, measure rect's size (width, height), makelong(horz width, vert hight)

#define		VIDEO_CURRHISTOGRAM		37	//just get current histogram of specified measure rect (default for whole image). return length (256 for gray 8).
										//lParam=1: enable evaluate histogram; =0; disable; =-1; just return length,; else is pointer to address to store histogram in INT32, =1;
//max=37


//-----sub-function defines for wParam of SetCaptureParam
		//wParam cab be one of the follow
#define		CAPTURE_RESETALL		0 //reset all cap params to sys default
#define		CAPTURE_INTERVAL		1 //LOWORD is number of frm interval, mean take one frm every LOWORD+1 frames;
									  //HIWORD is about number per 10 seconds (frm rate). LOWORD is valid only HIWORD is 0 
#define		CAPTURE_CLIPMODE		2 //LOWORD: clip mode when video and dest rect not same size
									  //HIWORD: if captrure odd and even field crosslly
									  //when lParam=0x40, returm also=0x40, support field extending
									  //when lParam=0x80, returm also=0x80, support magnify (zoom bigger) (c30n)
									  //when lParam=0x100, returm also=0x100,  support x reduce (zoom smaller)
									  //when lParam=0x200, returm also=0x200,  support y reduce (zoom smaller)
#define		CAPTURE_SCRRGBFORMAT	3 //set rgb format code. when return, loword=code, hiword=bitcounts 
#define		CAPTURE_BUFRGBFORMAT	4 //set rgb format code. when return, loword=code, hiword=bitcounts 
#define		CAPTURE_FRMRGBFORMAT	5 //set rgb format code. when return, loword=code, hiword=bitcounts 
#define		CAPTURE_BUFBLOCKSIZE	6 //lParam=MAKELONG(width,height)
									  //if set it 0 (default), the rect set by user will be as block size 
#define		CAPTURE_HARDMIRROR		7 //LOWORD mirror: bit0 x, bit1 y; default for both capture and playback
									  //when lParam==-2, if return -1: not support output mirror indepent from capture; 
									  //Note when lParam's b7=1: mean set capture and playback seperately.  
									  //b4:x, b5:y for output mirror (if support indepent), 
									  //
									  //HIWORD special rotate: b16:17: special angle rotate, b16~17=1: rotate 90 degree left, =2: 90 right, =3 180 degree
									  //when lParam==-3, check if support rotate special angle, return -1, not support								  

#define		CAPTURE_ANGLEROTATE		34 //arbitrary angle rotate.  LOWORD b0~11: angle to ratate in degree (1 represent 1/10 degree)
									   //when lParam= -2, if return -1: mean not support output rotate indepent from capture. 
									   //HIWORD, b31=1 mean set capture and playback seperately.  
									   //if so, b16~27: angle to ratate in degree (1 represent 1/10 degree) 
									   //This set wiil be ignored when HIWORD in CAPTURE_HARDMIRROR is not 0

#define		CAPTURE_VIASHARPEN		8	//LOWORD mean to sample via sharpen filter
#define		CAPTURE_VIAKFILTER		9  //remove noise. LOWORD to use recursion filter; HIWORD take some smooth filter.
										//when lParam==-2, just get capability of LOWORD, if return -1: not support; 
										//when lParam==-3, just get capability of HIWORD, if return -1: not support

#define		CAPTURE_SAMPLEFIELD		10  //0 in field (non-interlaced), 1 in frame (interlaced), (0,1 are basic)
										//2 in field but keep expend row,3 in field but interlaced one frame
										//(2,4 can affect only sampllng field(frame) by field(frame) )
										//in 3 up-dn frame
#define		CAPTURE_HORZPIXELS		11	// set max horz pixel per scan line
#define		CAPTURE_VERTLINES		12	// set max vert lines per frame

#define		CAPTURE_SCROFFSETVID	21 //set screen offset in video's rect just for screen scroll
										//affecting Capture to SCREEN and ConvertRect to or from SCREEN

#define		CAPTURE_ARITHMODE		13 //arithmatic (and model image) mode  LOWORD =0:no, =1:V-M, =2:V+M, =3:V+M
										//=4:Signed V-M, =5:Signed M-V, =6:C-B, =7:B-C
										//HIWORD, 0: just for disp, 1: just for captured, 2: for both 
#define		CAPTURE_TO8BITMODE		14 //the mode of high (eg. 10 bits) converted to 8bit 
										//HIWORD(lParam)=0: linear scale, 
										//HIWORD(lParam)!=0:clip mode, LOWORD(lParam)=offset

#define		CAPTURE_SELRGBDATA		37 //LOWORD: select some rgb channel's data to capture as gray image (cpc61c)
									   //0: default for all, 1:red, 2:green, 3:blue


#define		CAPTURE_SEQCAPWAIT		15 	// bit0 if waiting finished for functions of sequence capturing and playbacking
										//bit1 if waiting finished capture then call callback function 

#define		CAPTURE_TRIGCAPTURE		17 //set triggered capture, LOWORD cap no of fields, LOBYTE of HIWORD delay fields after trigger
									   //HIBYTE of HIWORD is min ex-trig period in milli-Second, special when =0(default value) mean 10 milli-S

#define		CAPTURE_TURNCHANNELS	18 //turn channels when sequence capture
										//b0~6 for turn number (max 127), 
										//b8~23(16) mask 0~15 channles, b7=1 keep this pos
										//b28~31, interval in turn, mean turn one time every interval

#define		CAPTURE_MISCCONTROL		16 //miscellaneous control bits
										//b0: 1: forced okCaptureByBuffer or okGetSeqCapture controled by interrupt
										//b1: 1: take one by one to okCaptureSequence, and okCaptureByBuffer or okGetSeqCapture controled by interrupt
										//	default 0: take last one
									    //b2: 1: one by one just for usb20
									    //b3: 1: capture all 12 bits, 0: capture 10 bits signal for 12 cards like LV20B...
									    //b4: 1: horz count from sync's up-edge; 0: down-edge for some cards like LVDS etc.
									    //b5: 1: vert count from sync's up-edge; 0: down-edge for some cards like LVDS etc.
									    //b6: 1: partial mode for camera, 0: global mode (default)
										//b7: 1: forced to okCaptureTo and okCaptureActive changed to okCaptureByBuffer
										//    note: this bit can be set only when its respond bit in hiword is 1.
										//b8: 0,1 x align adjust for Bayer format 
										//b9: 0,1 y align adjust for Bayer format 
										//b10: 1 enable pass lut/gamma
										//b11:1 take 8 frms as buffer for okCaptureByBuffer, default take 2 frms

//--
#define		CAPTURE_SETLATENCY		24 //set PCI latency, default =0xf8, it is better to set to 0x18 when multi-cards

#define		CAPTURE_SLEEPTIMES		25 //relex times and set max times to wait for all sequence capure functions.
										//LOWORD: a level to release cpu times. 0: disable, 1:system auto (default), >=2 specified level
										//HIWORD: max wait times (in unit 10 milli-second) to wait to next frame. 0:system auto (default). 

//--set some camera params
#define		CAPTURE_EXPOSETIME		20 //(like camera) set exposed time, in unit us (microsecond), -2: get max valid exponsetime 
#define		CAPTURE_DELAYEXPOSE		27 //(for camera) set delay time to expose for camera, in unit us (microsecond) from the Basetime (ususally begin after a ex-trigger)

#define		CAPTURE_FRAMERATE		22 //for area camera, LOWORD is transfer frame rate (be equa less current hardware frm rate set); HIWORD is hardware frm rate of camera, 
									   //and default: HIWORD=0 mean set to max frm rate to support. when lParam=-2, return how many classes of (hardware) frm rate to support, 
									   //lParam=-3, return max (hardware) frame rate to support 
									   //for line scan camera, LOWORD is line rate (Lines/S) and HIBYTE be set to 0x80;

#define		CAPTURE_SETTEMPER		23 //for camera, set camera temperature expected, and to get current real temperature of camera when set -2

#define		CAPTURE_WHITEBALANCE	26 //for camera, set white balance for camera, LOWORD=0: off, =1:onetime, =2:always
											//HIWORD=0: default(auto), =1: daylight, =2:fluorescence, =3 incandesent lamp,...
#define		CAPTURE_SETBINNINGMODE	28 //for camera, LOWORD set binning mode, 0:normal mode ,when BYTE1=0, BYTE0=0,1: 1x1, =2: 2x2, =3: 3x3, ...
									   //when BYTE1!=0, mean BYTE0 is binning number on x-direction, BYTE1 is in y-direction  
 									   //HIWORD is soft binning (sample ratio), default=0 mean take whole iamge, when BYTE3=0, BYTE0=0,1: whole, =2: 1/2x1/2
									   //when BYTE3!=0, mean BYTE2 is sample ratio in x-direction and BYTE3 is in y-direction 

#define		CAPTURE_ENBCAMFUNC		29 //for camera, enable camera's functions, b0: auto-adjust mode, and then BYTE2(when non 0) is specified target gray(8Bits); 
										//b1: gray stretch; b2: histogram equaliize;
										//b3: enable removing isolated spot, and then if BYTE3 (when non 0 ) is specified gray threshold (8Bits)  

#define		CAPTURE_GAMMAFACTOR		30 //for camera, set gamma factorX100 (it is between 0~2 for some camera)

#define		CAPTURE_FLATFIELDING	36 //for camera, frame grabber,  flat fielding (shadow correct), 
										//LOWORD=1: capture a dark field in BUFFER 0, =2; a bright one in BUFFER 1, =3: generat correct model
										//HIWORD=1 enable flat field correct with self-created model, =0x100: disable

//--
#define		CAPTURE_SETCOMPPARAM	31 //LOWORD: qualitiy factor(1~100 for jpg,mjpg), HIWORD: specify compress format(1=JPG, 2=PNG, 3=TIFF, ...)
  
#define		CAPTURE_PACKETINTERVAL	32 //set interval(in 10 vs) between two packets to send in order to control transfer rate (for capture device based net)
										//0: auto-mode, default=0, genarally max is about 6000000(60Seconds). when lParam= -2 return true max value
#define		CAPTURE_PACKETSIZE		33 //set packet size in byte (for capture device based net), min=8, default=1500, max is 2048 
#define		CAPTURE_WAITASKRESEND	35 //wait how many packets to ask re-send after some packet was missing (for capture device based net)
										//0: No ask re-send, others: packets to wait
//max=36


//#define		CAPTURE_OUTHORZPOS		20	// set out horz offset and width on the monitor 
//#define		CAPTURE_OUTVERTPOS		21	// set out vert offset and height on the monitor 
//#define		CAPTURE_OUTHORZPIX		22	// set out horz total pixel



//----defines sub-param lParam for CAPTURE_SAMPLEFIELD
#define		SAMPLE_INFIELD			0 //in field (non-interlaced)
#define		SAMPLE_INFRAME			1 //in frame of interlaced fields
					//the above two (0,1) are basic
#define		SAMPLE_FIELDEXP			2 //in field but expend (keep expend row)
#define		SAMPLE_UPDNFRAME		3 //in frame of up-downed fields 
#define		SAMPLE_FIELDINTER		4 //in field but interlaced to one frame, from even
#define		SAMPLE_INTOPFIELD		5 //in field just top field

//-----defines sub-param lParam for CAPTURE_CLIPMODE
#define		RECT_SCALERECT			0 
#define		RECT_CLIPCENTER			1 
#define		RECT_FIXLEFTTOP			2 
					//in condition video rect great than screen rect:
					//if RECT_SCALERECT video rect will be scaled to match screen rect if it can. else
					//video rect will be adjusted to match screen rect
					//(1: center, centerize video rect position according size of dest, it will affect VIDEO's rect
					//set by okSetTargetRect(hBoard,VIDEO,lprcVideo);
					// 2: left-top fixed, take left-top of current video's rect and same size rect as dest)


//-----sub-function defines for wParam of SetDeviceParam
		//wParam cab be one of the follow
#define		DEVICE_SETDEVMEMORY		0 //set or get all setable parameters to and from device with memory, 
									  //lParam=-1: get device-memory's params as device current , lParam=0: store device's current param to device-memory, 
									  //=1: restore all params to hardware default
					//(for camera, CCU,) this func will be called in func okOpenBoard to get and okCloseBoard to set 
#define		DEVICE_SETIPADDRESS		1 //set or get new ip address of net device. lParam is structure NETCFGPARAM pointer
#define		DEVICE_GETMACADDRESS	2 //only get mac address of net device. lParam is structure NETCFGPARAM pointer

#define		DEVICE_GETACCEPTTHREAD	3 //get handle of accept thread for capturing image from net device

#define		DEVICE_GETPARAMCHANGED	4 //get changed info which group parameters were set and reset the flag after call


//-----sub-function defines for lParam of PutSignalParam
#define		PUTSIGNAL_TRIGGER		1 //LOWORD output 1 trigger signal; HIWORD output a soft-emulation trigger
#define		PUTSIGNAL_VERTSYNC		2 //enbale video vertical sync. output
					//(for GPIO,...)
#define		PUTSIGNAL_SYNCSTART		3 //set start number after begin of reset pulse signal(Vertical). in unit of line pulse number
					//(special for LineScan)
#define		PUTSIGNAL_SYNCCOUNT		4 //set count number after start signal to enbale capture. in line pulse
					//(special for LineScan)

#define		PUTSIGNAL_DELAYSYNC		5 //set delay times to output sync pulse from the Basetime. in unit of us  
					//(for camera, CCU, ...) 
#define		PUTSIGNAL_SYNCWIDTH		6 //set sync pulse's width (in unit of us ) wihich set by PUTSIGNAL_DELAYSYNC. 
					//(for camera, CCU, ...) 	 //	note: it is positive pusle when b31 is 1
#define		PUTSIGNAL_STARTCONSYNC	11 //start sync signals as above. LOWORD is repeat period (in unit of ms) count from the Basetime, 
					//(for CCU, )	//HIWORD (usually is 0) is a extra delay time(in ms) to the Basetime from specified ex-Trigger 

#define		PUTSIGNAL_SETVOLTRANGE	12 //set output voltage range (like ibs for CCU), in unit of 0.1 Voltage, LOWORD=min value, HIWORD max
					//(for CCU,)
#define		PUTSIGNAL_FEEDBACKFACT	13 //set feedback factor, value range 1~16, more big more fast and sensitivity
					//(for CCU,)

#define		PUTSIGNAL_SETFLASHMODE	8 //set flash lamp, LOWORD = 0: default(auto), =1: by extra trigger, =2: single by DSP, =3; continue )
					//(for IC1200, )	//HIWORD: interval of flash, (1~65535) in us 

#define		PUTSIGNAL_EXTRIGDELAY	7 //set delay time (in 10us) to generate a negtive trigger(about 20VS width) after received a ex-trigger 
					//(for gpio )
#define		PUTSIGNAL_SETGPIODIR	9 //set GPIO input or output, 0: input, 1:output (b0~31 for 32 io ports)
					//(for gpio )
#define		PUTSIGNAL_SETGPIOLEV	10 //set GPIO low or hight level, 0: low level, 1: high
					//(for gpio )

#define		PUTSIGNAL_SETLENS		14 //set param of lens, LOBYTE(BYTE0) for aperture, 

#define		PUTSIGNAL_SETOPTFILTER	15 //set param of optical filter, LOBYTE(BYTE0) for light filter,
//max=15



//-----sub-function defines for lParam of okSetConvertParam
#define		CONVERT_RESETALL		0 //reset all to sys default
#define		CONVERT_FIELDEXTEND		1 //field extend				
#define		CONVERT_PALETTE			2 //set (copy) convert palette (just for 8 to 24 or 32), no memorize
										//lParam=0: restore system default, >0: new palette (BYTE) pointer and length=256
#define		CONVERT_HORZEXTEND		3 //horzental extend (integer times)
#define		CONVERT_HORZSTRETCH		4 //horizental stretch (arbitrary number times)
#define		CONVERT_MIRROR			5 //x(=1) and y(=2) mirror (note:just to convert data with BUFFER)
#define		CONVERT_UPRIGHT			6 //up to righ(=1)(rotate right 90 D) or left (=2) (rotate left 90 D
#define		CONVERT_NOSYNCSCREEN	7 //1 no wait for sync of vertical (just when to WND for CaptureByBuffer)
									  //used b13
#define		CONVERT_WINDOWLEVEL		10 //set gray level window's location( middle pos) and its width, mainly for hight gray(>=8bits) to 8bits 
										//Also for its gray of some channel is 8 bits, either src or dst
										// LOWORD=middle, HIWORD=width. HIWORD=0 (default) mean no convert (it (HIWORD) should be 0 usually)
										//this set valid only CONVERT_LEVELMAPLUT as belowed was set to 0
#define		CONVERT_LEVELMAPLUT		11 //set (copy) gray level map LUT. lParam=0 (default): off, >0: set a pointer of LUT in WORD, 
										//which must be WORD pointer anyway. LPWORD[0] is bits of gray level
										//(i.e. it is 8 for GRAY8, GRAY888, RGB888..,it is 16 for GRAY16), and they are true 
										//elements of LUT from LPWORD[1].
										//whose max value in this LUT should be <= max value of dest to convert
										//it must be unset by lParam=0 after finishing convert   
									


//-----defines lParam for wParam=CONVERT_FIELDEXTEND in okSetConvertParam
//--field extend mode
#define		FIELD_JUSTCOPY			0 //just copy row by row 
#define		FIELD_COPYEXTEND		1 //copy one row and expend one row (x2)
#define		FIELD_INTERLEAVE		2 //just copy odd(1.) rows (/2)
#define		FIELD_INTEREXTEND		3 //copy one odd row and expend one row 
#define		FIELD_COPYINTERPOL		4 //copy one odd row and interpolate one row
#define		FIELD_INTERINTERPOL		5 //copy odd row and interpolate even row

#define		FIELD_INTEREVEN			6 //just copy even(2.) rows (/2)
#define		FIELD_INTEREXTEVEN		7 //copy one even row and expend one row 
#define		FIELD_JUSTCOPYODD		8 //just copy odd rows to odd rows  
#define		FIELD_JUSTCOPYEVEN		9 //just copy even rows to even rows  
#define		FIELD_ODDEVENCROSS		10 //copy odd and even cross 

#define		FIELD_COPYROWTOODD		11 //copy row by row and to odd place
#define		FIELD_COPYROWTOEVEN		12 //copy row by row and to even place

									//just for the case without bit converting



//-----sub-function defines for lParam of GetSignalParam
#define		SIGNAL_VIDEOEXIST		1 //0 video  absent, 1 exist
#define		SIGNAL_VIDEOTYPE		2 //0 field, 1 interlaced
#define		SIGNAL_SCANLINES		3 //scan lines per frame
#define		SIGNAL_LINEFREQ			4 //line frequency
#define		SIGNAL_FIELDFREQ		5 //field frequency
#define		SIGNAL_FRAMEFREQ		6 //frame frequency
#define		SIGNAL_EXTTRIGGER		7 //extern trigger status, 1 trigger
#define		SIGNAL_FIELDID			8 //Field ID 0 odd, 1 even
#define		SIGNAL_VIDEOCOLOR		9 //color(1) or B/W(0)
#define		SIGNAL_TRANSFERING		10 //1 transfering video data
#define		SIGNAL_CAPFINISHED		11 //capture finished
#define		SIGNAL_HORZSYNCTIME		12 //time of horz sync header in ns
#define		SIGNAL_VERTSYNCTIME		13 //time of vert sync header in ns
#define		SIGNAL_SYNCPOLARITY		14 //b0: horz sync pol, b1:vert sync. 0:negtive, 1:positive
#define		SIGNAL_HORZVALPOS		15 //horz edge and size of validate area, LOWORD is width, HIWORD is edge 
#define		SIGNAL_VERTVALPOS		16 //vert edge and size of validate area, LOWORD is height, HIWORD is edge 
#define		SIGNAL_VIDEOCHANGED		17 //video signal changed since last call to any items of this func, 1: changed 
#define		SIGNAL_NETCONNECT		18 //status of net(or USB...) connected with capture device, 0: not connected
#define		SIGNAL_FIELDFREQEX		19 //field frequency (number of field per 1000 Seconds)
#define		SIGNAL_FRAMEFREQEX		20 //frame frequency (number of frame per 1000 Seconds)
#define		SIGNAL_CHECKNETRATE		21 //return transfer-rate of net while seconds. LOWORD is data transfer rate, in M Bytes/S.
									   //HIWORD is rate of Frame-Lost, in number/10000 Frames 
#define		SIGNAL_SCANPIXELS		22 //horizental scan pixels per line (only for digital signal)
										

//-----sub-function defines for lEvent of WaitSignalEvent
#define		EVENT_FIELDHEADER		1 //field header
#define		EVENT_FRAMEHEADER		2 //frame header
#define		EVENT_ODDFIELD			3 //odd field come
#define		EVENT_EVENFIELD			4 //even field come
#define		EVENT_EXTTRIGGER		5 //extern trigger come, 
									  //(HIWORD(lEvent) is pole)



//-----defines for several target we can support 
typedef		MLONG 			TARGET; //signed value or  pointor

#define		BUFFER			(TARGET)1	//Buffer(physical) allocated from host memory
#define		VIDEO			(TARGET)0	//Video source input to the board 
#define		SCREEN			(TARGET)-1	//Screen supported by VGA
#define		FRAME			(TARGET)-2	//Frame buffer on the board
#define		GRAPH			(TARGET)-4	//Graph frame on the board 
#define		MONITOR			(TARGET)-3	//Monitor(image out) supported by (D/A) TV standard



#define		SEQFILE			0x5153	//SQ 
#define		BMPFILE			0x4d42	//BM
#define		JPGFILE			0x504A	//JP 
#define		RAWFILE			0x5752	//RW 
#define		JHIFILE			0x484A	//JH 

#define		BLKHEADER		0x4b42	//BK
#define		BMPHEADER		0x4d42	//BM
#define		BUFHEADER		0x4642	//BF


#define		IPADDRESS		0x4A49	//IP(J)
#define		MACADDRESS		0x434d	//MC


//-----defines messages for user 

#define	WM_CLOSEREPLAY			WM_USER+100
		//send this message when close replay dlg

#define	WM_BEGINSEQPROC			WM_USER+101
		//begin seq proc, wParam=hBaord
#define	WM_SEQPROGRESS			WM_USER+102
		//seq in progress, wParam=hBaord, lParam=No. to
#define	WM_ENDSEQPROC			WM_USER+103
		//end seq proc, wParam=hBaord

#define	WM_ASKWAITVERTICAL		WM_USER+104
		//ask to wait next vertical blank

//-----------struct defines----------------------------------------------

//--app user used struct


typedef struct _okdevtype { //added after 28/02/2005, to replace struct _boardtype
	INT32	iBoardTypeCode; //ok image device type code (e.g. 2030(capture card), 201100(camera))
	INT32	iBoardIdentCode; //ok image device identity code (e.g. 2130) , or ip value
	INT32	iBoardRankCode; //ok image device model code 0,1,..

	CHAR	szBoardName[24]; //ok image device name (eg."OK_M20H", "OK_AM1100"...)
	CHAR	szSubDrvName[88]; //ok sub-driver name(eg. "okc30.dll") 
	DWORD	dwSpecCaps; //special capabilities flag. b0: audio, b1:stream encoder
} OKDEVTYPE, *LPOKDEVTYPE; //128 bytes 


typedef struct _boardtype { //discarded
	SHORT	iBoardTypeCode; //ok board type code (e.g. 2030)
	CHAR	szBoardName[18]; //board name (eg."OK_M20H")
	SHORT	iBoardIdentCode; //ok board identity code (e.g. 2130)
	SHORT	iBoardRankCode; //ok board model code 0,1,..
} BOARDTYPE, *LPBOARDTYPE; //24 bytes

//image file block size
typedef struct _blocksize {
	SHORT	iWidth;		//width
	SHORT	iHeight;	//height
	SHORT	iBitCount; //pixel bytes iBitCount
	SHORT	iFormType;	//rgb format type, need to fill when RGB565 or RGB 555
	INT32	lBlockStep; //block stride (step to next image header)
						//need to fill when treat multi block else set 0
}BLOCKSIZE;





//image block info
typedef struct _blockinfo {
	SHORT	iType;	//=BK or SQ, BM
	//struct _blocksize;
	SHORT	iWidth;		//width *
	SHORT	iHeight;	//height *
	SHORT	iBitCount; //pixel bytes iBitCount *
	SHORT	iFormType;	//rgb format type, need to fill when RGB565 or RGB 555
	union {
		struct {
			SHORT	lBlockStep; //block stride (step to next image header)
			SHORT	iHiStep;// HIWORD of block stride
		};
		DWORD	dwBlockStep;
	};
	union {
		struct {
			SHORT	lTotal;	//frame num
			SHORT	iHiTotal;// HIWORD of total
		};
		DWORD	dwTotal;
	};
	SHORT	iInterval; //frame interval
	LPBYTE	lpBits;// image data pointer / file path name
	LPBYTE	lpExtra;// extra data (like as palette, mask) pointer
} BLOCKINFO, *LPBLOCKINFO;

//sequence file info
typedef struct { //file info for seq
	SHORT	iType;	//=SQ or BM
	//struct _blocksize;
	SHORT	iWidth;		//width 
	SHORT	iHeight;	//height 
	SHORT	iBitCount; //pixel bytes iBitCount 
	SHORT	iFormType;	//rgb format type, need to fill when RGB565 or RGB 555
	union {
		struct {
			SHORT	lBlockStep; //block stride (step to next image header)
			SHORT	iHiStep;// HIWORD of block stride
		};
		DWORD	dwBlockStep;
	};
	union {
		struct {
			SHORT	lTotal;	//frame num
			SHORT	iHiTotal;// HIWORD of total
		};
		DWORD	dwTotal;
	};
	SHORT	iInterval; //frame interval
} SEQINFO;


//set raw file parameters
typedef struct _rawfileparam {
	WORD	wRawFlag; //raw file flag,  RAWFILE:0x5752 
	WORD	wPixBits; //bits per pixel
	DWORD	dwWidth; //width 
	DWORD	dwHeight; //height
	DWORD	dwOffset; //file header  
	BYTE	bByteOrder; //=0: little endian低地址存放最低有效字节（LSB），即低位在先，高位在后。 \
						//=1: Big endian (MSB)
	BYTE	reserved2; //res
	WORD	reserved3; //res
} RAWFILEPARAM, *LPRAWFILEPARAM;

//for replay
typedef struct { //file info for seq
	LPBITMAPINFOHEADER	lpbi; //bitmap info
	BYTE	*lpdib; //dib data
	HWND	hwndPlayBox; //1 replaying, 0 quit
	SHORT	iCurrFrame; //current frame in buffer
	SHORT	iReserved; //
} DIBINFO, *LPDIBINFO;



//---set text mode----
typedef struct _settextmode {
	DWORD	dwForeColor; // forecolor, see macro RGB in win
	DWORD	dwBackColor; // backcolor, see macro RGB in win
	DWORD	dwSetMode; // 0:FULLCOPY, 1: FULLXOR, ...  0x100,0x200: 
	WORD	wFrameNo; // place which frame of target
	WORD	wGroupNo; // 
} SETTEXTMODE;

#define		FULLCOPY		0		//copy full text region into target
#define		FULLXOR			1		//xor full text region and target
#define		COPYFONT		2		//just copy fonts strokes to target
#define		XORFONT			3		//just xor fonts strokes and target


//---encode and decode-----------------------------

//---get image size for jpeg------
typedef struct _imagesize {
	DWORD	dwWidth;
	DWORD	dwHeight;
	DWORD	dwBitCount;
	DWORD	dwTotal;
}IMAGESIZE, *LPIMAGESIZE;

//--set jpeg params
typedef struct _jpegparam {
	DWORD	dwSize; // the size of this strcut, must be set
	LPSTR	lpstrName;	//must be NULL if not use
	DWORD	dwQuality; //
	DWORD	dwReserved1;
} JPEGPARAM, *LPJPEGPARAM;



//-----------------------------------------------------
struct _imagesize_info {
	INT32	lHeadSize;      //struct header size
	INT32	lHorzSize;		//width, (default=768 when it as input)
	INT32	lVertSize;		//height, (default=768 when it as input)
	INT32	lBitCount;      //bit count (default=24 when it as input)
	INT32	lTotalFrame;    //total frames 
	INT32	lQuality;       //quality factor (for jpeg, default =75)
	WORD	bHardEncode;    //flag of encode by harware
	WORD	wMakerFlag;
	INT32		res0[5];		//args reserved
}; //12X4

//---used to set and get image info ------
typedef struct _jpg2k_header_info {
	//struct _imagesize_info;
	INT32	lHeadSize;      //struct header size
	INT32	lHorzSize;		//width, (default=768 when it as input)
	INT32	lVertSize;		//height, (default=768 when it as input)
	INT32	lBitCount;      //bit count (default=24 when it as input)
	INT32	lTotalFrame;    //total frames 
	INT32	lQuality;       //quality factor (for jpeg, default =75)
	WORD	bHardEncode;    //flag of encode by harware
	WORD	wMakerFlag;
	DWORD	dwVersion;		//version
	INT32	res0[4];		//args reserved
	//---end commnon--

	void	*lpExtraInfo;    //reserved, to store extra infos
	INT32		res1[17];	  //args reserved
} JPG2KHEADER;  //30x4



//---used to set and get image info ------
typedef struct _mpg2_header_info {
	//struct _imagesize_info;
	INT32	lHeadSize;      //struct header size
	INT32	lHorzSize;		//width, (default=768 when it as input)
	INT32	lVertSize;		//height, (default=768 when it as input)
	INT32	lBitCount;      //bit count (default=24 when it as input)
	INT32	lTotalFrame;    //total frames 
	INT32	lQuality;       //quality factor (for jpeg, default =75)
	WORD	bHardEncode;    //flag of encode by harware
	WORD	wMakerFlag;
	DWORD	dwVersion;		//version
	INT32	res0[4];		//args reserved
	//---end commnon--

	double	dbFrameRate;      //frame rate (default=25, and it is expected rate when it as input)
	double	dbBitRate;        //bits rate

	void	*lpExtraInfo;    //reserved, to store extra infos
	INT32	res1[13];		//args reserved
} MPEG2HEADER; //size=(12+18)*4


//---used to set and get image info ------
typedef struct _mpg4_header_info  
{
	//struct _imagesize_info;
	INT32	lHeadSize;      //struct header size
	INT32	lHorzSize;		//width, (default=768 when it as input)
	INT32	lVertSize;		//height, (default=768 when it as input)
	INT32	lBitCount;      //bit count (default=24 when it as input)
	INT32	lTotalFrame;    //total frames 
	INT32	lQuality;       //quality factor (for jpeg, default =75) (real 31 level)
	WORD	bHardEncode;    //flag of encode by harware
	WORD	wMakerFlag;
	DWORD	dwVersion;		//version
	INT32	res0[4];		//args reserved
	//---end commnon--

	double	dbFrameRate;	//frame rate (default=25, and it is expected rate when it as input)
	double	dbBitRate;		//bits rate, (default=4000000, and it is recommended)
	INT32	lEncodeMode;    //Encode mode, (value range 0~1, 0: double channels, 1:fix code rate as dbBitRate preset.
							//default=0, when it as input) 
	INT32	lInterval;      //Interval between key frame, (range: 1--300, default =100)
	INT32	lBFrameNum;		//B frame mode, (value range:0~2,  default=0. It is compatible as divx5.0
							//=2, little coded data but more encoding times
	INT32	lQualityIdx;	//mpeg4 quality control level index, (value range: 0~6, default=6, mean high level)
							//It will be better quality if value smaller, but it will need longer times

	void	*lpExtraInfo;    //reserved, to store extra infos
	INT32	res1[9];	    //args reserved
} MPEG4HEADER; //size=(12+18)*4





#define		MJPG_SKIP		0x50494B53	//SKIP
#define		MJPG_GINF		0x464E4947	//GINF

//--get infos of current decoded frame ---------
typedef struct _extra_frame_info   //info of current frm
{
	//--Input
	DWORD	dwFlgSkip;	   //if it=="SKIP" mean skip decoded data, just get params

	//--Output of current frm decoded
	DWORD	dwFrmType;	   //type of frm, I,P,B
	DWORD	dwFrmNumber;	//number of frm in origin video stream 
	DWORD	dwFrmRatio;		//current frm ratio //got it at after begin
	DWORD	dwTimeStamp;	//time stamp (in mili S)
	union {
		struct {
			DWORD	Day:5; // day
			DWORD	Mon:4; // month
			DWORD	Year:14; //year
		};
		DWORD	dwDateStamp;	//date stamp  
	};
	DWORD	lQuality;       //quality factor set 
	WORD	wres;		//
	WORD	wMakerTag;		//JH //got it at after begin
	DWORD	dwVersion;		//version //got it at after begin
	DWORD	reserved[5];   //res
} EXTRAFRMINFO, *LPEXTRAFRMINFO; //for lpExtraInfo of mjpg to get info of current frm

//---used to set and get image info ------
typedef struct _mjpg_header_info   
{
	//struct _imagesize_info;
	INT32	lHeadSize;      //struct header size
	INT32	lHorzSize;		//width, (default=768 when it as input)
	INT32	lVertSize;		//height, (default=768 when it as input)
	INT32	lBitCount;      //bit count (default=24 when it as input)
	INT32	lTotalFrame;    //total frames 
	INT32	lQuality;       //quality factor (for jpeg, default =75)
	WORD	bHardEncode;    //flag of encode by harware
	WORD	wMakerFlag;
	DWORD	dwVersion;		//version
	INT32	res0[4];		//args reserved
	//---end commnon--

	double	dbFrameRate;  //frame rate (default=25, and it is expected rate when it as input)

	DWORD	dwGetFrmInfo; //=="GINF" mean to get current frm's infos, and lpExtraInfo points memory to store them
	void	*lpExtraInfo;    //points to memory to store extra infos returned by okDecodeImage,
							//else it must be NULL
	INT32	res1[14];	  //args reserved
} MJPGHEADER;  //size=(12+18)*4



//--get frame type decoded 
typedef struct _mpeg_frame_info    
{
	INT32	lHeadSize;       //struct size
	INT32	lFrameType;		//current frame type
	INT32	lSkip;  		//input
	INT32	reserved1;      //reserved
	INT32	reserved2;      //reserved
	INT32	reserved3;      //reserved
	INT32	reserved4;      //reserved
} MPEGFRAMEINFO;



//-----------------------------------------------------



//---used to set and get net-device's config param  ------
typedef struct _net_cfg_param
{
	WORD	dwType; //not used, just id (IP,MC address.  =IPADDRESS or MACADDRESS)
	BYTE	bRWFlag; // =0:read, 1:write
	BYTE	bReset; // =1 reset immediately after write mac addr, 0: not
	CHAR	szAddress[120]; //in format of string.  ip addr, like "168.196.1.1",  //in 10 decimal 
							//mac addr, like 0x 60 50 40 30 20 10 //in 16 (hexadecimal)
						//both address are in local machine order,  high byte first
	DWORD	dwLowAddress; // in format of 16 decimal, low DWORD responds 0x40302010, string format is "64.48.32.16"
	DWORD	dwHiAddress; // high DWORD, responds 0x 60 50
	DWORD	lRes; //reserved
} NETCFGPARAM, *LPNETCFGPARAM; //note: this structure must be set to zero. 
							// may choose any one format as input address, the format of string has priority



//------ okapi32 functions list -----------

#ifdef __cplusplus
extern "C" {            /* Assume C declarations for C++ */
#endif /* __cplusplus */


//--1. basic routines--------------

//prolog and epilog
HANDLE	WINAPI okOpenBoard(MLONG *iIndex); //okLockBoard		
		//open a Ok series board in specified index(0 based), return 0 if not found any
		//if success, return a handle to control specified board
		//if set index=-1, mean takes default index no. (default is 0 )
		//if user not specified by 'Ok Device Manager' in Control Pannel)
		//this index can be also a specified board type code, or ip address string
		//this function will change iIndex to the true used index, 
		//if index input is -1 or type code 

HANDLE	WINAPI okOpenBoardEx(MLONG *iIndex, HANDLE hParBoard, LPVOID lParam); // 11/04/13  
		//this is expended function of okOpenBoard
		//hParBoard is parent handle of new board to open, which usually be NULL 
		//if use this func open a handle with hParBoard, the handle should be closed before hParBoard be closed
		//lParam now is not used, which should be NULL
		//others are same as okOpenBoard above, 

BOOL	WINAPI okCloseBoard(HANDLE hBoard); //okCloseBoard
		//Save current parameters and then unlock and close Ok board specified handle and 

BOOL	WINAPI okCloseBoardEx(HANDLE hBoard, DWORD	dwSave); //okCloseBoardEx
		//Unlock and close Ok board specified handle
		//dwSave=1: same as okCloseBoard, first save all current parameter to attached device
		//dwSave=0; not to save, just close board


INT32	WINAPI okGetLastError();
		//Get last error msg

INT32	WINAPI okGetDriverVer(LPSTR lpString, INT32 iSize);
		//get version of current ok cards driver
		//lpString return version char string (e.g "5.08"),  iSize is lpString' size


MLONG	WINAPI okGetBufferSize(HANDLE hBoard, void **lpLinear, MWORD *dwSize);
		//get base address and size of pre-allocated buffer,
		//if success return the max. frame num in which can be store according to current set
		//else return false;
LPVOID  WINAPI okGetBufferAddr(HANDLE hBoard, MLONG lNoFrame);
		//get base address of specified frame No. in BUFFER
		//if success return the linear base address 
		//else return false;

LPVOID  WINAPI okGetTargetInfo(HANDLE hBoard, TARGET target, MLONG lNoFrame, SHORT *wid, SHORT *ht, MLONG *stride);
		//get target info include base address, width, height and stride specified frame No. 
		//if success return the linear base address and other infos, else return false;

INT32	WINAPI okGetTypeCode(HANDLE hBoard, LPSTR lpBoardName);
		//return type code and name of specified handle, the memory lpBoardName points must be big enough or be NULL
		//lpBoardName==-3: return code of majmin, =-4: return index number in total;  =-5 return IPCode, =-6 return HI IP 	
		//=-7: return HardVer; =-8: return Devce Type(0:PCI,PCI_E...,1:USB,..), =-10: return if LineScan Device

INT32	WINAPI okGetNetDevNumber(INT32 iTypeCode, LPSTR lpNameString);
		//check and get number of ok net device by typcode or name
		//iTypeCode is type code, lpNameString is name
		//return 0, mean not net device, else return sequence number (based 1)
		//

//set rect and capture 
MLONG	WINAPI okSetTargetRect(HANDLE hBoard, TARGET target, LPRECT lpTgtRect);
		//set target (VIDEO, SCREEN, BUFFER, FRAME, HWND)capture to or from
		//if Rect.right or .bottom) are -1 , they will be filled current value
		//special note 1 for target=BUFFER:
		//if never set CAPTURE_BUFBLOCKSIZE, the block size(W,H) of buffer will be changed 
		//according to size of right x bottom of lpRect, else the size will not changed 
		//specail note 2 for hwnd:
		// left=right mean use rect of SCREEN, left=1 & right=0 mean match real rect of hwwnd, else take its rect set
		//if success return max frames this target can support, else return <=0

INT32	WINAPI okSetToWndRect(HANDLE hBoard, HWND hWnd);
		//set client rect of hwnd as screen rect


BOOL	WINAPI okCaptureSingle(HANDLE hBoard, TARGET Dest, MLONG lStart);
		//capture video source to target which can be BUFFER, SCREEN, FRAME, MONITOR 
		//start(o based).if success return 1, if failed return 0, if not support target -1
		//when this function sent command to grabber, then return immediately not wait to finish.
		//this function same as okCaptureTo(hBoard, Dest, wParam, 1);

BOOL	WINAPI okCaptureActive(HANDLE hBoard, TARGET Dest, MLONG mStart);
		//capture continuous active video to same position in target which can be BUFFER, SCREEN, FRAME, MONITOR 
		//start(o based).if success return 1, if failed return 0, if not support target -1
		//when this function sent continuous command to grabber, then return immediately
		//but note that some card like RGB30. when target is SCREEN, this function is a thread.
		//this function same as okCaptureTo(hBoard, Dest, wParam, 0);

HANDLE	WINAPI okCaptureThread(HANDLE hBoard, TARGET Dest, MLONG mStart, MLONG mNoFrame);
		//capture sequencely video to target which can be BUFFER, SCREEN, FRAME, MONITOR 
		//start(o based). lParam>0: number of frame to capture to,
 		//if lParam > total frames in BUFER, it will loop in rewind mode(i%total)
		//if lParam=-N mean it loop in buffer of N frame infinitely until call okStopCapture, 
		//when -1 mean loop in all buffer.
		//return handle of thread if success,
		// return 0 if failed(eg. format not matched). -1 not support target
		//this call will create a thread to manage to capture sequencely then
		//return immediately not wait to finish. This thread will callback if need
		//this function same as okCaptureTo(hBoard, Dest, wParam, n);
		//but it is not same, when n=1 this function is also a thread and still support callback

MLONG	WINAPI okCaptureToScreen(HANDLE hBoard);
		//Start to capture to screen (video real-time display on screen) and return immediately
		//this is just a special routine of okCaptureTo(hBoard,SCREEN,0,0)

MLONG	WINAPI okCaptureTo(HANDLE hBoard, TARGET Dest, MLONG start, MLONG mParam);
		//capture video source to target which can be BUFFER, SCREEN, FRAME, MONITOR 
		//start(o based), lParam>0: number of frame to capture to, =0: cont. mode,
 		//if lParam > total frames in BUFER, it will loop in rewind mode(i%total)
		//if lParam=-1 mean it loop infinitely until call okStopCapture
		//return max num frame can be stored in the target if success,
		// return 0 if failed(eg. format not matched). -1 not support target
		//this call will return immediately not wait to finish.
		//This function is not recomended to use for new user 


MLONG	WINAPI okPlaybackFrom(HANDLE hBoard, TARGET src, MLONG start, MLONG mParam);
		//playback on monitor from target which can be BUFFER, FRAME
		//start(o based), lParam>0: number of frame to capture to, =0: cont. mode 
		//if lParam > total frames in BUFER, it will loop in rewind mode (i%total)
		//if lParam=-1 mean it loop infinitely until call okStopCapture
		//return max num frame be stored in the target if success, 
		//return 0 if wrong. -1 not support target
		//this call will return immediately not wait to finish.
		//

//get status and stop capture
MLONG	WINAPI okGetCaptureStatus(HANDLE hBoard, BOOL bWait);
		//query capturing status, if bWait==1 then wait to finish capturing(to and from) (=2, just capture to), 
		//else return immediately.
		//return 0 if finished, if cont. mode capturing return target capture to
		//(which include SCREEN -1, FRAME -2, MONITOR -3)
		//if capturing to/from BUFFER or file, return the frame No. in BUFFER (1 based) being capturing 

MLONG	WINAPI okGetLineReady(HANDLE hBoard, MLONG *iCurrFrameNo, MWORD dwRes);
		//get current line number captured already (supported only by LineScan Capture cards)
		//get current line number already captured. 
		//iCurrFrameNo returned is always total frame number(based 1) already captured, 0 mean no frame finished yet. 
		//return line number(based 1) already captured in current frame, 0 mean no line fnished in current frame.
		//dwRes is reserved, should be NULL.

MLONG	WINAPI okStopCapture(HANDLE hBoard);
		//Stop capturing to or playback from SCREEN, BUFFER or other targets 
		//return target just captured to or from. 
		//if capturing to/from BUFFER or file, return the total frame No.(1 based) being capturing 
		//it may be not stoped immediatly for sequence capture after returned

MLONG	WINAPI okStopCaptureEx(HANDLE hBoard, DWORD dwCmdCode, long lParam);
		///It's a extend func of okStopCapture, it's probably used to stop or pause/re-capturing etc.
		///dwCmdCode=0: stop, dwCmdCode=1: pause, dwCmdCode:=2: re-capturing/playbacking
		///dwCmdCode=3, when lParam>0: fast forward, when lParam<0: fast backward,
		///dwCmdCode=4, step to specified (lParam) frame number of whole frames (lParam: from 0 to max);
		///dwCmdCode=5, step to number(lParam) relative to current frame(lParam>0: forward,lParam<0: backward)  
		///when dwCmdCode=0: return target just captured to or from; and  
		//		if capturing to/from BUFFER or file, return the total frame No.(1 based) being capturing 
		//		and it may be not stoped immediatly for sequence capture after returned
		///when dwCmdCode>0: it return 1 when support else return -1


MLONG	WINAPI okGetSeqCapture(HANDLE hBoard, MLONG start, MLONG count);
		//get current frame No. of sequence capturing to buffer
		//start: set buffer No. to use, effecting only count==0
		//count: count No. to catpure, 0: start capture, -1: stop capture, others: continue capture
		//return frame No. finished


//capture by to /from 
HANDLE	WINAPI okCaptureByBuffer(HANDLE hBoard, TARGET dest, MLONG start, MLONG num);
		//capture sequence images to dest by way of two frame buffers (in BUFFER), 
		//the frame size and format is taken as same as current config of BUFFER
		//if dest is file name which can be .seq or .bmp (will generate multi bmp files)
		//dest can be also a user memory pointer or a BLOCKINFO pointer (with user memory pointer)
		//retrun non 0 immediately if success. num should be great than 0

HANDLE	WINAPI okCaptureByBufferEx(HANDLE hBoard, MLONG fileset, TARGET dest, MLONG start, MLONG num);
		// all are same as okCaptureByBuffer except for fileset which is quality when to jpg file

		
HANDLE	WINAPI okPlaybackByBuffer(HANDLE hBoard, TARGET src, MLONG start, MLONG num);
		//playback sequence images on monitor from src by way of two frame buffers (in BUFFER)
		//the size and format of BUFFER will be changed to same as src
		//src can be a file name which may be .seq or .bmp (first orderd bmp files) 
		////src can be also a BLOCKINFO pointer (with infos of user memory pointer,size and format)
		//if src is just user memory pointer, this function will think its block size and format 
		//are same as current config of BUFFER (in this case can not support loop function).
		//retrun true immediately if success 
		//if num is great than the true frame number in src, it will loop back
		//if num=-1 mean it will loop infinitely until call okStopCapture

//set and get params
MLONG	WINAPI okSetVideoParam(HANDLE hBoard, INT32 wParam, MLONG lParam);
		//----set video param sub-function defines
		//set video param and return previous param; 
		//if input lParam=-1, just return previous param 			
		//if not support return -1, if error return -2 

MLONG	WINAPI okSetCaptureParam(HANDLE hBoard, INT32 wParam, MLONG lParam);
		//set capture param and return previous param; 
		//if input lParam=-1, just return previous param 			
		//if not support return -1, if error return -2 

MLONG	WINAPI okSetDeviceParam(HANDLE hBoard, INT32 wParam, MLONG lParam);
		//set (net) device param and return param; 
		//wParam is a item to set, 
		//lParam can be single param to set, or in/out pointer, or structure pointer
		//if not support return -1, if not success return 0, else return true or expected param   


//transfer and convert rect
INT32	WINAPI	okReadPixel(HANDLE hBoard, TARGET src, MLONG start, SHORT x, SHORT y);
		//read value of one pixel specified (x,y) in frame start of src (SCREEN, BUFFER, FRAME...)
		//return is this pixel value, it may be with bits 8,16,24,or 32 depend on the src's format

INT32	WINAPI	okWritePixel(HANDLE hBoard, TARGET tgt, MLONG start, SHORT x, SHORT y, INT32 lValue);
		//write value into specified (x,y) in the frame start of tgt (SCREEN, BUFFER, FRAME...)
		//

MLONG	WINAPI okSetConvertParam(HANDLE hBoard, INT32 wParam, MLONG lParam);
		//set convert param for for function okConvertRect 
		//if not support return -1, if error return -2 

INT32	WINAPI okReadRect(HANDLE hBoard, TARGET src, MLONG start, LPBYTE lpBuf);
		//read data into lpBuf from rect(set previous) in frame start of dst (SCREEN, BUFFER, FRAME)
		//the data in lpBuf stored in way row by row
		//if src not supported return -1, if failed return 0, 
		//return -1 if not support, return 0 failed, 
		//if success return data length read in byte
		//if lpBuf=NULL, just return data length to read

INT32	WINAPI okWriteRect(HANDLE hBoard, TARGET dst, MLONG start, LPBYTE lpBuf);
		//write data in lpBuf to rect(set previous) of dst (SCREEN, BUFFER, FRAME)
		//the data in lpBuf stored in way row by row
		//return -1 if not support, return 0 failed, 
		//if success return data length written in byte

INT32	WINAPI okReadRectEx(HANDLE hBoard, TARGET src, MLONG start, LPBYTE lpBuf, INT32 lParam);
		//read data into lpBuf from rect(set previous) in frame start of dst (SCREEN, BUFFER, FRAME)
		//the data in lpBuf stored in way row by row
		//if src not supported return -1, if failed return 0, 
		//return -1 if not support, return 0 failed, 
		//if success return data length read in byte
		//if lpBuf=NULL, just return data length to read
		//LOWORD(lParam）is form code for bits of lpBuf (e.g.：FORM_GRAY8），if it is 0 mean: as same as src
        //HIWORD(LParam) is the mode of taking channels. mode=0 take all, =1 red, =2 green, =3 blue;

INT32	WINAPI okWriteRectEx(HANDLE hBoard, TARGET dst, MLONG start, LPBYTE lpBuf, INT32 lParam);
		//write data in lpBuf to rect(set previous) of dst (SCREEN, BUFFER, FRAME)
		//the data in lpBuf stored in way row by row
		//return -1 if not support, return 0 failed, 
		//if success return data length written in byte
		//LOWORD(lParam）is form code for bits of lpBuf (e.g.：FORM_GRAY8），if it is 0 mean: as same as dst
        //HIWORD(LParam) is the mode of taking channels. mode=0 take all, =1 red, =2 green, =3 blue;

INT32	WINAPI okTransferRect(HANDLE hBoard, TARGET dest, MLONG iFirst, TARGET src, MLONG iStart, MLONG lNum);
		//transfer source rect to dest rect (here target can be SCREEN, BUFFER, FRAME, also BLOCKINFO point to user memory) 
		//if total in dest or src less than lNum, it will rewind to begin then continue
		//this function transfer in format of src, that means it don't convert pixel bits if dst and src are not same 
		//if src or dst not supported return -1, if failed return 0, 
		//if success return data length of one block image in byte

INT32	WINAPI okConvertRect(HANDLE hBoard, TARGET dst, MLONG first, TARGET src, MLONG start, MLONG lParam);
		//transfer source rect to dest rect (here target can be SCREEN, BUFFER, FRAME, also BLOCKINFO point to user memory) 
		//LOWORD(lParam)=lNum, total num, HIWORD(lParam)=mode, channels to convert
		//mode=0 take all, =1 red, =2 green, =3 blue;
		//if total in dest or src < lNum, it will rewind to begin then continue
		//this function convert to pixel foramt of dst if dst has not same bits format as src 
		//if src or dst not supported return -1, if failed return 0, 
		//if success return image size of one block in pixel
INT32	WINAPI okConvertRectEx(HANDLE hDstBoard, TARGET dst, MLONG first, HANDLE hSrcBoard, TARGET src, MLONG start, MLONG no);
		//same as the above function okConvertRect except with src handle


//get and put signals	
INT32	WINAPI okGetSignalParam(HANDLE hBoard, INT32 wParam);
		//Get specified param of video signal source
		//if not support return -1, if error return -2, else return param

INT32	WINAPI okWaitSignalEvent(HANDLE hBoard, INT32 wParam, MLONG lMilliSecond);
		//Wait specified signal come
		//lMilliSecond is time-out time in milliseconds for to wait
		//if lMilliSecond is zero, the function returns current state immediately
		//if lMilliSecond is INFINITE(-1) wait forever until event come
		//return -1 not support, 0 speicfied signal not come, 1 come

MLONG	WINAPI okPutSignalParam(HANDLE hBoard, INT32 wParam, MLONG lParam);
		//set param to output specified signal to some other device 
		//if not support return -1, if error return -2, 


//treat callback functions
BOOL	WINAPI okSetSeqProcWnd(HANDLE hBoard, HWND hwndMain);
		//set proc hwnd for receive message about sequence capture

BOOL	WINAPI okSetSeqCallback(HANDLE hBoard, 
								BOOL CALLBACK BeginProc(HANDLE hBoard), 
								BOOL CALLBACK SeqProc(HANDLE hBoard, MLONG No), 
								BOOL CALLBACK EndProc(HANDLE hBoard));
		//set callback function for multi-frame capturing function 
		//(which are okCaptureTo,okCaptureByBuffer,okCaptureSequence,okCaptureThread,
		// okPlaybackFrom,okPlaybackByBuffer,okPlaybackSequence)
		//see follow 

MLONG	WINAPI okSetUserData(HANDLE hBoard, MLONG lUserData);
		//set or get user data to or from hBoard. 
		//when lUserData==-1, set nothing, else set value of lUserData to hBoard
		//return lUserData preset


BOOL	CALLBACK BeginProc(HANDLE hBoard); //user defined callback function
		//callback this function before to capture, you should set it whether you use it
BOOL	CALLBACK SeqProc(HANDLE hBoard, MLONG No); //user defined callback function
		// callback this function after finish capturing one frame
		// No is the number(0 based) frame just finished or being playbacked. 
BOOL	CALLBACK EndProc(HANDLE hBoard); //user defined callback function
		// callback this function after end capturing


BOOL	CALLBACK EventProc(HANDLE hBoard, MWORD dwReserved); //user defined callback function for okSetEventCallback

BOOL	WINAPI okSetEventCallback(HANDLE hBoard, INT32 iEvent, BOOL CALLBACK EventProc(HANDLE hBoard, MWORD dwReserved) );
		//set some event callback proc, which will be callback before or after this event

//-----sub-function defines for iEvent of SetEventCallback
#define		EVENT_TOCLOSEDEVICE			1		//callack before close board(device)
#define		EVENT_NETCONNECTCHANGE		2		//callack after net connect status changed



BOOL	WINAPI okSetCloseCallback(HANDLE hBoard, BOOL CALLBACK CloseProc(HANDLE hBoard, MWORD dwReserved) );
		//set callback proc of okCloseBoard, which will be callback before it close board
		//same as EVENT_TOCLOSEDEVICE

BOOL	CALLBACK CloseProc(HANDLE hBoard, MWORD dwReserved); //user defined callback function for okCloseBoard


//save and load files
MLONG	WINAPI okSaveImageFile(HANDLE hBoard, LPSTR szFileName, MLONG first, TARGET target, MLONG start, MLONG num);
		//here target can be BUFFER, SCREEN, FRAME or user buffer pointor
		//1.if ext name=".bmp", "raw", "jpg", thus one file with single frame, then:
		//create num new file and than save one frame in each file begin from start position of target as bmp file 
		//when argument num is great than 1, a: File name will be appended 000 and auto-plus-1 if without digit in
		//szFileName; b: File name will be auto-plus-1 in parts of digits if we find digit in szFileName(like aa1000.bmp);
		//
		//2.if ext name=".seq","avi", thus one file with multi-frames (we call those file as sequences file), then
		//save num frame from (start) in target into (first) frame pos in seq(sequence) file in sequencely.
		//if the file already exist the function will not delete it, that mean old contents in the file will be kept.
		//So if you want create a new seq file with a existed file name you must delete before this call .
		//for sequences file, better we use stream way called "beg,con,end" when we save multi-frames into this one file.
		//to create file just call this func whith szFileName appended ",beg", and to finish file just with szFileName 
		//appended ",end", to save each frame data one by one just call it with szFileName appended ",con". 
		//in this way argument first will be ignored
	
MLONG	WINAPI okLoadImageFile(HANDLE hBoard, LPSTR szFileName, MLONG first, TARGET target, MLONG start, MLONG num);
		//here target can be BUFFER, SCREEN, FRAME or user buffer pointor. here first and start both are based 0
		//1.if ext name=".bmp":
		//load one frame into start position of target from bmp file 
		//
		//2.if ext name=".seq":
		//load no frame into (start) in target from (first) frame pos in seq(sequence) file in sequencely.
		//return total number for sequence format file (like .seq, .avi, ...), total data length for single format file
		//not load image data when num=0 

BOOL	WINAPI okGetCurrImageInfo(HANDLE hBoard, MLONG lpImgFrmInfo, MLONG lSize);
		//used for okLoadImageFile
		//get extra infos of current image which is last loaded by okLoadImageFile 
		//lpImgFrmInfo is structure (see EXTRAFRMINFO) stored extra info, lSize should be size of the structure
		//return 1 if lpImgFrmInfo none 0, else return 0;


//load and save some kind of config files with parameters to set
BOOL	WINAPI okSaveConfigFile(HANDLE hBoard, LPSTR szFileName);
		//save all current parameters(include stream' params after 11/07/11 ver 4.0 )  set to specified config file 
		//(*.okf: full params; *.oks:sample params; *.oko: output params), 

BOOL	WINAPI okLoadConfigFile(HANDLE hBoard, LPSTR szFileName);
		//load some kind of specified config file to set current all paramters
		//

//load and save flat field model 
BOOL	WINAPI okSaveFlatModelFile(HANDLE hApi, LPSTR szFile);
		//save flat model from buf0,and 1 to szFile
		//ext name must be ".fla" 

BOOL	WINAPI okLoadFlatModelFile(HANDLE hApi, LPSTR szFile);
		//load flat model file to buf0,and 1, and then set it to inner model 
		//ext name must be ".fla" 



//--2. special routines supported by some cards--------------

//overlay mask:
INT32	WINAPI okEnableMask(HANDLE hBoard, BOOL bMask);
		//0: disable mask; 1: positive mask, 2: negative mask
		//positive: 0 for win clients visible, 1 video visible
		//negative: 0 for video visible,  1 for win client (graph) visible
		//if bMask=-1 actually not set just get status previous set
		//return last mask status,
 
INT32	WINAPI okSetMaskRect(HANDLE hBoard, LPRECT lpRect, LPBYTE lpMask);
		//Set mask rect(lpRect is relative to lpDstRect in SetScreenRect or
		//SetBufferRect, lpMask is mask code (in byte 0 or 1). one byte for one pixel
		//if lpMask==1, set all rect region in lpRect video visible
		//if lpMask==0, set all rect region in lpRect video unvisible
		//return base linear address of inner mask bit 

//set out LUT:
INT32	WINAPI okFillOutLUT(HANDLE hBoard,  LPVOID bLUT, INT32 start, INT32 num);
		//fill specified playback out LUT. 
		//bLut stored values(which may be 8 or 10, 12 bits data) to fill, (r0,g0,b0, r1,g1,b1 ...)
		//start: offset pos in LUT(based 0), num: num items to fill. if num<0, just to read lut
		//if bLut==NULL, just used to return bits of lut by this hardware. if bLut==-1, just to return lut channles supported
		//if >8 bits, bLut must be used as word not byte 
		//if not support return -1.

//set input LUT:
INT32	WINAPI okFillInputLUT(HANDLE hBoard, LPVOID bLUT, INT32 start, INT32 num);
		//fill specified input LUT. 
		//bLut stored values (which may be 8 or 10, 12 bits data) to fill, (r0,g0,b0, r1,g1,b1 ...)
		//start: offset pos in LUT(based 0), num: num items to fill. if num<0, just to read lut
		//if bLut==NULL, just used to return bits of lut by this hardware. if bLut==-1, just to return lut channles supported
		//if >8 bits, bLut must be used as word not byte 
		//if not support return -1.


BOOL	WINAPI okCaptureSequence(HANDLE hBoard, MLONG lStart, MLONG lNoFrame);
		//capture sequence to BUFFER by way of Interrupt Service Routine 
		//Note: Only M10 series, M20H,M40, M60, M30, M70 and RGB20 support this way 
		//wParam=start(o based). lParam>0: number of frame to capture to,
 		//if lParam > total frames in BUFER, it will loop in rewind mode(i%total)
		//when -1 mean loop in all buffer. infinitely until call okStopCapture, 
		//return max num frame can be stored in the target in this way if success,
		// return 0 if failed(eg. format not matched). -1 not support target
		//this call will start a Interrupt Service Routine to manage to capture sequencely then
		//return immediately not wait to finish. This routine not support callback 

BOOL	WINAPI okPlaybackSequence(HANDLE hBoard, MLONG lStart, MLONG lNoFrame);
		//playback on monitor from BUFFER
		//start(0 based), lNoFrame>0: number of frame to capture to,  
		//if lParam > total frames in BUFER, it will loop in rewind mode (i%total)
		//if lParam=-1 mean it loop infinitely until call okStopCapture
		//return max num frame be stored in the target if success, 
		//return 0 if wrong. -1 not support 
		//this call will start a Interrupt Service Routine to manage to playback sequencely then
		//return immediately not wait to finish. This routine not support callback 
		//




//---------
//
MLONG	WINAPI okSetDriverParam(INT32 lWhich, MLONG lParam); 
		//lWhich defined as the follow
#define		STATICVXD		1
#define		INSTNTDRV		2
#define		ALLOCBUFFER		3
#define		MEMBLOCKSIZE	4
#define		MEMBLOCKCOUNT	5
#define		UNREGISTER		6


//set pre-allocate buffer size in k byte
MLONG	WINAPI okSetAllocBuffer(MLONG mSize); //32/64
		//set the new size to preallocate in  k bytes, 
		//if new size is not same as current, 
		//then the functuion will restart the window system

BOOL	WINAPI okSetStaticVxD(INT32 lMode); //just for win95/98
		//lMode=0: check if static vxd registered.
		//=1: create static vxd register
		//=2: delete static vxd register

BOOL	WINAPI okSetNTDriver(BOOL bCmd);	//just for winNT/2K
		//bCmd=0: check if nt driver installed.
		//=1: install nt driver
		//=2: remove nt driver

BOOL	WINAPI okUnRegister(DWORD dwCmd);
		//uninstall all registered and generated infos

INT32	WINAPI	okGetProgramInfo(INT32 iItem, LPSTR lpString, INT32 iSize);
		//get program info
#define	PROGRAM		1
#define	VERSION		2
#define	PREFIX		3
#define	COMPANY		4
#define	TELFAX		5
#define	WEBEMAIL	6
#define	LANGUAGE	7


INT32	WINAPI okSetLangResource(INT32 langcode); //1252 for English, 936 for Simple Chinese



//--3. multi cards, channels, memories --------------

//multi cards access:
INT32	WINAPI okGetImageDevice(OKDEVTYPE **lpOkDevInfo, MLONG lpParam);
		//Query all Ok series Image devices available in PCI bus, USB, GigE ... return total number
		//lParam is pointer which is input/ouput. Input b0:1, re-check, b1:1, include net devices
		//Output: return flag include net devices from config set by Ok Image Manager
		//return number of board

SHORT	WINAPI okGetSlotBoard(BOARDTYPE **lpOkInfo); // obsolete function
		//Query all Ok boards available in PCI bus, return total number
SHORT	WINAPI okGetBoardIndex(CHAR *szBoardName, SHORT iNo);
		//Get index (start 0) of specified board name string (it can also be typcode string)
		// and order(iNo) in same name (start 0) from current ok image device list. 
		//return -1 if no this specified ok board
INT32	WINAPI okGetBoardName(SHORT lIndex, LPSTR szBoardName);
		//get the board code and name of the specified index 
		//return the type code if success else return 0 if no card



//multi cards capture
MLONG	WINAPI 	okMulCaptureTo(HANDLE *lphBaord,TARGET Dest, MLONG start, MLONG lParam);
		//control multi boards to capture to target simultaneously, lphBaord are pointer of hBoard of multi board
		//it must be end in zero.  
		//other functions are same as okCaptureTo
HANDLE	WINAPI 	okMulCaptureByBuffer(HANDLE *lphBaord,TARGET tgt, MLONG start, MLONG num);
		//control multi boards to capture by buffer simultaneously, lphBaord are pointer of hBoard of multi board  
		//it must be end in zero.  
		//other functions are same as okCaptureByBuffer

//multi channels:
BOOL	WINAPI okLoadInitParam(HANDLE hBoard, SHORT iChannNo);
		//load specified chann(based 1)  (and as current chann.)of initial params
BOOL	WINAPI okSaveInitParam(HANDLE hBoard, SHORT iChannNo);
		//save current init param to specified chann(based 1) (and as current chann.)

//get and lock buffer
MLONG	WINAPI okGetAvailBuffer(void **lpLinear, MWORD *dwSize);
		//Get free meomery buffers pre-allocated . 
		//call it when user hope to access buffer directly or lock for some one board 
MLONG	WINAPI okLockBuffer(HANDLE hBoard, MWORD mwSizeByte, void **lpBasLinear);
		//mode 1: dwSizeByte is size (non-zero) required. In this mode, when in one program 
		//		you can simply specify buffer size you need from the whole BUFFER sequencely, 
		//		and all specified bufferes are seperate,that mean one handle can not
		//		use the other handle. And there are no overlay between them. 
		//mode 2: dwSizeByte is 0. lpBasLinear points a array[2], array[0]=offset, array[1]=size, 
		//		in which all are you specified as your need.
		//		in this mode you can speicify any position and size you need from BUFFER for someone board.
		//		All the buffers specified in this mode can be either seperate or overlay, 
		// return the size of locked buffer in fact
BOOL	WINAPI okUnlockAllBuffer(void);
		//Unlock all buffer for all handle

//Mem Block Buffer appended to BUFFER 
INT32	WINAPI okApplyMemBlock(DWORD dwBlockSize, DWORD dwBlockNo);
		//apply mem block used as buffer appended to BUFFER
		//return the number of blocks allocated actually

BOOL	WINAPI okFreeMemBlock();
		//release appended MemBlock by okApplyMemBlock

INT32	WINAPI okGetMemBlock(HANDLE hBoard, DWORD *dwEachSize,  DWORD *dwBlockNo);
		//get the number of MemBlock and size per block applied by okApplyMemBlock 
		//and return the number can be as buffer as to cureent set size of BUFFER

INT32	WINAPI okLockMemBlock(HANDLE hBoard, DWORD dwBlockNo);
		//lock number of MemBlock to specified handle

BOOL	WINAPI okUnlockMemBlock(void);
		//unlcok all locked MemBlock



//--4. apps utilities-----------------


//apps setup dialog 
BOOL	WINAPI okOpenSetParamDlg( HANDLE hBoard, HWND hParentWnd);
		//dialog to setup video param, default window is modal, it can be a modaless window after call SetWindowLong(hwnd,GWL_USERDATA,1);      
BOOL	WINAPI okOpenSeqCaptureDlg( HANDLE hBoard, HWND hParentWnd);
		//dialog to capture sequence image

LPDIBINFO WINAPI okOpenReplayDlg( HANDLE hBoard, HWND hWnd, TARGET src, MLONG total);
		//open modeless dialog to replay sequence images(in BUFFER, USERBUF or seq file) on SCREEN or MONITOR

HWND	WINAPI okOpenReplayDlgEx( HANDLE hBoard, HWND hWnd, TARGET src, MLONG total, LPBITMAPINFOHEADER lpbi, LPBYTE lpdib);
		//open modeless dialog to replay sequence images(in BUFFER, USERBUF or seq file) on SCREEN or MONITOR


INT32	WINAPI okAutoAdjustBright(HANDLE hBoard, LPRECT lpRect, LPVOID dwRes);
		//Auto adjust image to the best seeing by setting gain, brightness, contrast and saturation if they can be set, 
		//lpRect is AOI in current BUFFER to measure, it must less than rect of BUFFER and great than 8X8
		//Take whole rect of BUFFER if lpRect==NULL
		//dwRes==0, now not used.
		//return mean gray value between 0 ~ 255, if success.

INT32	WINAPI okAutoSetCapRect(HANDLE hBoard, INT32 Width, INT32 Height);
		//auto set all param to adapt source video



BOOL	WINAPI okOpenSetLUTDlg(HANDLE hBoard, HWND hWnd, LPVOID lpLUT);
		//dialog to set LUT of input and output if supported by current card 
		//lpLUT is buffer to store lut value set by this routine, when press OK button
		//


//--image process-----
BOOL	WINAPI okUnifyFields(HANDLE hBoard, TARGET target, MLONG start);
		//unify odd and even two fields which have different intensive brightness 
		//

INT32	WINAPI okDiffusionFilter(HANDLE hBoard, TARGET target, MLONG start, SHORT nLoop);
		//remove noise filter by diffusion method. nLoop is loop times, recommend it be 2 
		//return -1 when not support (not support virtual card)

INT32	WINAPI okGuassFilter(HANDLE hBoard, TARGET target, MLONG start);
		//remove noise filter by guass method 
		//return -1 when not support (not support virtual card)

INT32	WINAPI okSharpFilter(HANDLE hBoard, TARGET target, MLONG start, SHORT iClass);
		//image sharper by edge enhance filter, iClass = 1~4 valid, 1: min. sharper, 4: max sharper  
		//return -1 when not support (not support virtual card)

float	WINAPI okGetFocusMeasure(HANDLE hBoard, TARGET target, MLONG start, LPRECT lpRect, SHORT iMethod, SHORT iPreDegree);
		//lpRect is rect to measure for fucus
		//which method, iMethod=0: deff, =1: robert, =2: 4X Laplacian, =3: 8X laplacian, =4: brenner
		//pre-process degree, iPreDegree=0: no pre-filter, =1: 3x3 cross median, =2: 3x3 squr median, =3: 5x5 squr medain, =4: 7x7 squr median
		//return mean measure value for focusness, more big more focus

INT32	WINAPI okMultiFrmMean(HANDLE hBoard, TARGET target, MLONG start, MLONG number);
		//acummulate number frame from start then take mean and place to start
		//return -1 when not support

INT32	WINAPI okTakeRectMean(HANDLE hBoard, TARGET target, LPRECT lpRect, MLONG start, short iMode);
		//calculate and return gray mean of rect specified
		//if lpRect==NULL it mean whole rect of target
		//return mean gray level value

INT32	WINAPI okEvaluateHistogram(HANDLE hBoard, TARGET target, LPRECT lpRect, MLONG start, LPDWORD lpHist);
		//evaluate histogram of rect specified
		//if lpRect==NULL it mean whole rect of target
		//lpHist points memery allocated by caller and its size must be great than number of gray level for TARGET 
		//for color it store data of b,g,r seperately       
		//if success LOWORD is number of bis(i.e. 8,10,16...), HIWORD is bits number of channels(i.e. 1 or 3) in return value 

INT32	WINAPI okGetLinExtenMaplut(LPDWORD lpHist, short iNumBits, short iNumChann, short iIntensity, LPWORD lpwLUT);
		//get map lut of linear extending according to lpHist input
		//lpHist points to histogram of one image, iNumBits and iNumChannis are bits and color channles of the image
		//iIntensity is intensity (0: no change, 10:recomended value, ) to fullfill extending transfer   
		//lpwLUT is memory allocated by caller, in which the maplut are stored when return sucessful 

INT32	WINAPI okGetHistEquaMaplut(LPDWORD lpHist, short iNumBits, short iNumChann, LPWORD lpwLUT);
		//get map lut of histaogram equalizing according to lpHist input
		//lpHist points to histogram of one image, iNumBits and iNumChannis are bits and color channles of the image
		//lpwLUT is memory allocated by caller, in which the maplut are stored when return sucessful 



//text and graphics
BOOL	WINAPI okSetTextTo(HANDLE hBoard, TARGET target, LPRECT lpRect, LOGFONT *lfLogFont, SETTEXTMODE *textMode, LPSTR lpString, INT32 lLength); 
		//set text into the image of specified target
		//target can be SCREEN, BUFFER, FRAME, also BLOCKINFO
		//lpRect: just use its (left,top) to points start posision
		//lfLogFont: windows font definition, see window's document
		//textMode: specify forecolor, backcolor and set mode
		//lpString, lLength: text string and it's length

INT32	WINAPI okDrawLineTo(HANDLE hBoard, TARGET target, MLONG lStart, POINT ptS, POINT ptE, INT32 iForeColor);
		//draw a straight line into specified target
		//target can be SCREEN, BUFFER, FRAME, also BLOCKINFO
		//ptS, ptE: specify start and end point, which both are close points
		//iForeColor: draw value on to target
		//return the pixel count of line

INT32	WINAPI okDrawEllipsTo(HANDLE hBoard, TARGET target, MLONG lStart, LPRECT lpRect, INT32 iForeColor);
		//draw a ellips into specified target
		//target can be SCREEN, BUFFER, FRAME, also BLOCKINFO
		//lpRect: specify the rect region of expected ellips
		//iForeColor: draw value on to target
		//return the pixel count of ellips

INT32	WINAPI okFillCircleTo(HANDLE hBoard, TARGET target, MLONG lStart, LPRECT lpRect, INT32 lForColor, INT32 iMode);
		//fill inside or outside of circle(or ellipse) area specified by lpRect
		//target can be GRAPH, SCREEN, BUFFER, FRAME, also BLOCKINFO
		//lpRect specify circle area, and circle's diameter take rect's max direction
		//lForColor is the value to fill.  
		//iMode: b0=0: mean to fill value to outside area of circle; =1: to fill inside
		//b1=0; fill circle area; =1: fill ellipse area


HDC		WINAPI okCreateDCBitmap(HANDLE hBoard, TARGET target, HANDLE *hDCBitmap);
		//create a memory DC compatible with windows's GDI, draw graphic and text etc. with GDI functions
		//target can be SCREEN, BUFFER, FRAME, also BLOCKINFO
		//hDCBitmap: return a handle with which we can map the graphics on the memory DC
		//to our target.
		//return the memory DC with which you can use various windows"s GDI functions

BOOL	WINAPI okMapDCBitmapTo(HANDLE hDCBitmap, MLONG lStart);
		//map the graphics (just regions of none zero) of memory DC created by okCreateDCBitmap 
		//into relative regions of specified target
		//hDCBitmap: the handle created by okCreateDCBitmap

BOOL	WINAPI okFreeDCBitmap(HANDLE hDCBitmap);
		//free the allocated resource by okCreateDCBitmap
		//hDCBitmap: the handle created by okCreateDCBitmap


BOOL	WINAPI okWaitVerticalSync(HANDLE hBoard, BOOL bHeader);
		//wait for verticl sync of monitor.
		//bHeader=0, mean wait for begin of vertical, =1, mean wait for end. 

//-----

//-----sub-function defines for wParam of okBeginEncode and okBeginDecode
#define		CODE_JPEG		1			//use JPEGPARAM when to set, and use IMAGESIZE when to get
#define		CODE_MPEG2		2			//use MPEG2HEADER for both 
#define		CODE_MJPG		3			//use MJPGHEADER for both 
#define		CODE_MPEG4		4			//use MPEG4HEADER for both 
#define		CODE_JPG2K		5			//use JPG2KHEADER for both 



//encode and decode
HANDLE	WINAPI okBeginEncode(HANDLE hBoard, DWORD dwCodeWay, MLONG lpImageInfo);
		//start to encode images. wCodeWay is CODE_JPEG or other compress, 
		//lpImageInfo is address of  preset parameters like size to encode
		//return a handle of encoder if sucessful, else  return 0
INT32	WINAPI okEncodeImage(HANDLE hCoder, TARGET src, MLONG start, LPBYTE lpData, MLONG maxlen);
		//encode one frame image . src is source like BUFFER, SCREEN, FRAME and BLOCK.
		//lpData to store coded data,  maxlen is maximum length of lpCodedData
		//return the length coded data
INT32	WINAPI okEndEncode(HANDLE hCoder);
		//end encode and release resources of encoder

HANDLE	WINAPI okBeginDecode(HANDLE hBoard, DWORD dwCodeWay, LPBYTE lpData, MLONG lpImageInfo);
		//start to decode images. wCodeWay is CODE_JPEG or other compress, 
		//lpData is current start pos of coded data, 
		//lpImageInfo will return image info like size, no return when it is NULL
		//return a handle of decoder if sucessful, else  return 0
INT32	WINAPI okDecodeImage(HANDLE hCoder, LPBYTE lpData, MLONG *length, TARGET target, MLONG start);
		//decode coded data to image. 
		//lpData is pointer with coded data 
		//length is input length of coded data, it also output length of real used data 
		//target is aim to decode, like BUFFER, SCREEN, FRAME,BLOCK or memory pointer 
		//start is pos of specified target
		//return TRUE if one image finished, else return 0
INT32	WINAPI okEndDecode(HANDLE hCoder);
		//end decoder and release resources of decoder




//protect
INT32	WINAPI okReadProtCode(HANDLE hBoard, SHORT iIndex);

INT32	WINAPI okWriteProtCode(HANDLE hBoard, SHORT iIndex, INT32 code);

//


//--5. audio section routines--------------

//-----defines wParam in okSetAudioParam
#define		AUDIO_RESETALL			0 //reset all to sys default
#define		AUDIO_SAMPLEFRQ			1 //set sample rate (in unit of samples per second)
#define		AUDIO_SAMPLEBITS		2 //set bits per sample to capture, lParam=-2: just get current signal's bits  
#define		AUDIO_INVOLUME			3 //set audio input gain control
#define		AUDIO_CALLINTERVAL		4 //set min. interval times of callback 
#define		AUDIO_SOURCECHAN		5 //LOWORD select channel number in one signal source, HOWORD select signal source number
#define		AUDIO_SOUNDCHAN			6 //LOWORD: set sound channel to capture, HIWORD: return channels supported with this signal
									  //b0:left(mono) channel,b1:right, b2:middle, b3:left-back, b4:right back, b5:subwoof,...
#define		AUDIO_SIGNALINFO		7 //just get signal infos, b0=1: signal exist,


//prolog and epilog
HANDLE	WINAPI okOpenAudio(HANDLE hBoard, MLONG lParam);
		//open audio device owned by the video capture board 
		//hBoard is handle of image board, lParam reserved argument normally must be set to 0
		//when b0 of lParam =1 mean just check if it support audio
		//return handle of the audio device

BOOL	WINAPI okCloseAudio(HANDLE hAudio);
		//close audio device

//capture and stop
MLONG	WINAPI okCaptureAudio(HANDLE hAudio, TARGET target, FARPROC lpfnUserProc, MLONG lParam);
		//start to capture audio data, target can be BUFFER(audio data buffer) or file name
		//lpfnUserProc is callback function pointer, it must be NULL if not using callback 
		// lParam reserved argument must be set to 0
		//return the maximum times in miliseconds the inner audio data memory can be stored for

BOOL	CALLBACK WriteAudioProc(HANDLE hAudio, LPBYTE lpAudBuf, INT32 length); //32/64
		//this callback function must written as the above protype by user 
		//call this function when there are enough audio data (the length in byte)
		//call this function when capture ended with argument length=0;

BOOL	WINAPI okStopCaptureAudio(HANDLE hAudio);
		//stop capturing audio data
		//return total length of read out by okReadAudioData 

//set and get audio
INT32	WINAPI okSetAudioParam(HANDLE hAudio, INT32 wParam, INT32 lParam);
		//set the parameters to sample audio,  wParam see above defines
		//return the new set value if success
		//if not support return -1, if error return -2 
		//if input lParam=-1, just return previous param 			

INT32	WINAPI okReadAudioData(HANDLE hAudio, LPBYTE lpAudioBuf, INT32 lReadSize); //32/64
		//read audio captured from inner data buffer
		//lpAudioBuf is your memory address to store
		//lReadSize is data length (in byte) you expect to read
		//return the data length (in byte) truelly read 


//--6. stream section routines--------------

#define		NONE			0 //

//-----defines wParam in okSetStreamParam

#define		STREAM_RESETALL			0 //reset all to sys default
#define		STREAM_RESOLUTION		1 //Sample resolution, 0=D1(default),1=2/3D1,2=1/2D1,3=SIF, for c20b
#define		STREAM_BITRATEMODE		2 //Bit Rate mode, =0: CBR(Constant), others: VBR(Variable BR)(default=3000), for c20b  
#define		STREAM_MAXBITRATE		3 //Max Bit Rate, it is BR for CBR, it is max BR for VBR(default=9000) (kb/s), for c20b
#define		STREAM_CALLINTERVAL		4 //callback after interval times (def=120) (ms)

#define		STREAM_RECTSELECT		5 //0(default): take rect same as capture, 1:itself, independent, used in like rgb60c
#define		STREAM_RECTSOURCE		5 //discarded, same as above
#define		STREAM_SOURCERECT		6 //set independent rect of stream source (lParam=&rect), for rgb60c

#define		STREAM_PICKFRMRATE		7 //set rate to pick number of frame per 100 frames, 0(default): auto mode . for rgb60c/e, rgb61c_e
#define		STREAM_QUALITY			8 //set quality factor, for rgb60c 10~100, =100: the best. 0(default) mean 50 
#define		STREAM_MAXFILELEN		9 //Set a size limit of file's length (in MegaByte) for any file on either ntfs or fat32 file system
									 //when great this size set it will create another new file (in format name-1.XXX, etc.).
									 //If no limit size was specified, It will also re-create new file automatically for either AVI file or on FAT32 
									 //when file size is great than 3.5G 



//prolog and epilog
HANDLE	WINAPI okOpenStream(HANDLE hBoard, DWORD lParam);
		//open stream device owned by the video capture board 
		//hBoard is handle of image board, when normaly open lParam must be 0.
		//when b0 of lParam is 1 just check if this board support hardware compressing
		//return handle of the stream device

BOOL	WINAPI okCloseStream(HANDLE hStream);
		//close stream device

//capture and stop
HANDLE	WINAPI okCaptureStream(HANDLE hStream, TARGET target, FARPROC lpfnUserProc, MLONG lMiliSeconds); //32/64
		//start to capture stream data, target can be NONE(0)/BUFFER(stream buffer), or file name
		//lpfnUserProc is callback function pointer, it must be NULL if not using callback 
		//lMiliSeconds=0: forever capturing until stop by okStopCaptureStream; 
		// non 0: specified periods to capture in mili-second
		//success if return non 0 (handle) 

BOOL	CALLBACK WriteStreamProc(HANDLE hStream, LPBYTE lpStreamBuf, INT32 length);
		//this callback function must written as the above prototype by user 
		//call this function when there are enough stream data (the length in byte)
		//lpStreamBuf is input pointer which points memory with stream data provided by system
		//length is size of input stream data this call
		//call this function when capture ended with argument length=0;

BOOL	WINAPI okStopCaptureStream(HANDLE hStream, DWORD dwCmdCode);
		//stop or pause/re-encode capturing stream data
		//dwCmdCode=0: stop, dwCmdCode=1: pause, dwCmdCode:=2: re-encode
		//return total length of read out by okReadStreamData 

//set and get stream
MLONG	WINAPI okSetStreamParam(HANDLE hStream, INT32 wParam, MLONG lParam);
		//set the parameters to sample stream,  wParam see above defines
		//return the new set value if success
		//if not support return -1, if error return -2 
		//if input lParam=-1, just return previous param 			

INT32	WINAPI okReadStreamData(HANDLE hStream, LPBYTE lpStreamBuf, INT32 lReadSize); //32/64
		//read stream captured from inner data buffer
		//lpStreamBuf is your memory address to store
		//lReadSize is data length (in byte) you expect to read
		//return the data length (in byte) truelly read 

MLONG	WINAPI	okGetCurrStream(HANDLE hStream, LPVOID lParam);
		//get infos of current stream being coded by hardware
		//return number of frame coded
 

//--7. serial device embedded in board--------------
INT32	WINAPI okSetSerial(HANDLE hBoard, INT32 wParam, INT32 lParam);
		//set serial device config parameter and return previous parameter; 
		//wParam is item to set, lParam is parameter value
		//if input lParam=-1, just return previous parameter not to set 			
		//if not support return -1, if error (sub-function not support) return -2 

#define		SERIAL_COMMINDEX			0 //current comm index to r/w, default value=0
#define		SERIAL_BAUTRATE				1 //comm baut rate,(like 38400,19200, 9600,4800, 2400,1200...) default=9600
#define		SERIAL_DATABITS				2 //data bits, like 8,9...default=8
#define		SERIAL_STOPBITS				3 //stop bits, 0=1, 1=1.5, 2=2, default=0
#define		SERIAL_PARITY				4 //parity bit, 0:no, 1:odd,2:even, default=0

INT32	WINAPI okReadSerial(HANDLE hBoard, LPVOID lpBuffer, INT32 lSize, MLONG lTimeOut);
		//read data from current serial comm device
		//lpBuffer is buffer to read, lSize is size of buffer 
		//return the size read actually

BOOL	WINAPI okWriteSerial(HANDLE hBoard, LPVOID lpBuffer, INT32 lSize, MLONG lTimeOut);
		//write data to current serial comm device
		//lpBuffer is buffer stored data to write, lSize is data length to write 
		//return non zero if success


//--8. ports io utilities--------------
		
		//this function only used for OK PCI GPIO20
SHORT	WINAPI okGetGPIOPort(SHORT index, WORD *wPortBase);
		//get port base and count of ok PCI GPIO20 
		//index is the number (0 based) of gpio cards,  
		//wPortBase: port base,  return port count if success else return 0

// for Non-PCI IO cards on WinNT/Win2000 
// you must call this function before using follow port io functions 
BOOL	WINAPI okSetPortBase(WORD wPortBase, SHORT iPortCount);
		//preset ports to use by setting port base address and port count 
		//to use some ports (default port=0x300, count=4) must preset by calling this one  
		//and these ports can be used correctly only after system restarted

//note: before call the follow i/o functions you should have called one of 
//those functions: okOpenBoard, okGetGPIOPort, okGetImageDevice
BOOL	WINAPI okOutputByte(WORD wPort, BYTE data);
		//output a byte at specified port 

BOOL	WINAPI okOutputShort(WORD wPort, SHORT data);
		//output a word at specified port 

BOOL	WINAPI okOutputLong(WORD wPort, MLONG data);
		//output a dword at specified port 

//----------- input data at port
BYTE	WINAPI okInputByte(WORD wPort);
		//input a byte at specified port 

SHORT	WINAPI okInputShort(WORD wPort);
		//input a word at specified port 

MLONG	WINAPI okInputLong(WORD wPort);
		//input a dword at specified port 






//--9. others--------------
//#define okGetDriverVer(a, b) okGetProgramInfo(2, a, b)

MWORD	WINAPI okGetAddrForVB(void *);
		//return array address for VB

//-----------get time elapse
DWORD	WINAPI okGetTickCount(void);
		//same as GetTickCount but exactly than it on Win2k

void	WINAPI okSleep(DWORD dwMill);
		//same as Sleep but exactly than it on Win2k


DWORD	WINAPI okGetTickCountMicro(LPDWORD lpHiDWord);
		//same as GetTickCount but exactly in Micro Second
		//if lpHiDWord get high dword  

void	WINAPI okSleepMicro(DWORD dwMicro);
		//Sleep micro seconds

#ifdef __cplusplus
}
#endif    /* __cplusplus */

#endif

//-------------------end-------------------------------------------------------------------

