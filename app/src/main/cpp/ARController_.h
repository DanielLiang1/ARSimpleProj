//
// Created by ys on 2018/8/27.
//

#ifndef VINSI_VINSCONTROLLER_H
#define VINSI_VINSCONTROLLER_H

#include <vector>
#include <AR/ar.h>
#include <AR/arMulti.h>
#include <AR/video.h>

#include <ARWrapper/Image.h>
#include <ARWrapper/VideoSource.h>
#include <ARWrapper/ARMarker.h>
//#include <ARWrapper/ARMarkerSquare.h>
#include <ARWrapper/ARMarkerMulti.h>
#if HAVE_NFT
#  include <AR2/tracking.h>
#  include <KPM/kpm.h>
#  include <ARWrapper/ARMarkerNFT.h>
#endif
#include <AR/gsub_es.h>
#include <AR/param.h>
#include <ARToolKit/include/ARWrapper/AndroidVideoSource.h>
#include "ARMarkerSqu.h"
#include "matrix.h"

namespace es1_controller{
    // 在工程中加入Mu数据，定义数据结构，包括时间戳和三个轴的数据
    typedef struct{
        ARdouble stamp;
        ARdouble gyroCoord[3];
        ARdouble accCoord[3];
    } IMUValues;

    typedef enum {
        ARViewContentModeScaleToFill,
        ARViewContentModeScaleAspectFit,      // contents scaled to fit with fixed aspect. remainder is transparent
        ARViewContentModeScaleAspectFill,     // contents scaled to fill with fixed aspect. some portion of content may be clipped.
        //ARViewContentModeRedraw,              // redraw on bounds change
                ARViewContentModeCenter,              // contents remain same size. positioned adjusted.
        ARViewContentModeTop,
        ARViewContentModeBottom,
        ARViewContentModeLeft,
        ARViewContentModeRight,
        ARViewContentModeTopLeft,
        ARViewContentModeTopRight,
        ARViewContentModeBottomLeft,
        ARViewContentModeBottomRight,
    } ARViewContentMode;

    enum viewPortIndices {
        viewPortIndexLeft = 0,
        viewPortIndexBottom,
        viewPortIndexWidth,
        viewPortIndexHeight
    };

    class ARController {
    private:
        typedef enum {
            NOTHING_INITIALISED,			///< No initialisation yet and no resources allocated.
            BASE_INITIALISED,				///< Marker management initialised, markers can be added.
            WAITING_FOR_VIDEO,				///< Waiting for video source to become ready.
            DETECTION_RUNNING				///< Video running, additional initialisation occurred, marker detection running.
        } ARToolKitState;

        ARToolKitState state;				///< Current state of operation, progress through initialisation
        bool stateWaitingMessageLogged;

        // Image acquisition.
//        AR2VideoParamT *gVid;
//        bool videoInited;                                    ///< true when ready to receive video frames.
//        int videoWidth;                                          ///< Width of the video frame in pixels.
//        int videoHeight;                                         ///< Height of the video frame in pixels.
//        AR_PIXEL_FORMAT gPixFormat;                                  ///< Pixel format from ARToolKit enumeration.
//        ARUint8* gVideoFrame;                                 ///< Buffer containing current video frame.
//        size_t gVideoFrameSize;                                  ///< Size of buffer containing current video frame.
//        bool videoFrameNeedsPixelBufferDataUpload;
//        int gCameraIndex;
//        bool gCameraIsFrontFacing;
        VideoSource *m_videoSource0;          ///< VideoSource providing video frames for tracking
        VideoSource *m_videoSource1;          ///< VideoSource providing video frames for tracking
        pthread_mutex_t m_videoSourceLock;
        bool m_videoSourceIsStereo;
        int m_videoSourceFrameStamp0;
        int m_videoSourceFrameStamp1;

//        renderer
        ARdouble m_projectionMatrix0[16];	///< OpenGL style projection matrix computed from camera parameters
        ARdouble m_projectionMatrix1[16];	///< OpenGL style projection matrix computed from camera parameters
        bool m_projectionMatrixSet;			///< True once the projection matrix has been computed, which requires an open video source


        // Markers.
        int threshold;
        AR_LABELING_THRESH_MODE thresholdMode;
        int imageProcMode;
        int labelingMode;
        ARdouble pattRatio;
        int patternDetectionMode;
        AR_MATRIX_CODE_TYPE matrixCodeType;
        bool debugMode;

        std::vector<ARMarker *> markers;    ///< List of markers.

        bool doMarkerDetection;
        // ARToolKit data.
        ARHandle *m_arHandle0;				///< Structure containing general ARToolKit tracking information
        ARHandle *m_arHandle1;				///< Structure containing general ARToolKit tracking information
        ARPattHandle *m_arPattHandle;			///< Structure containing information about trained patterns
        AR3DHandle *m_ar3DHandle;		    ///< Structure used to compute 3D poses from tracking data
        ARdouble m_transL2R[3][4];
        AR3DStereoHandle *m_ar3DStereoHandle;

#if HAVE_NFT
        bool doNFTMarkerDetection;
    bool m_nftMultiMode;
    bool m_kpmRequired;
    bool m_kpmBusy;
    // NFT data.
    THREAD_HANDLE_T     *trackingThreadHandle;
    AR2HandleT          *m_ar2Handle;
    KpmHandle           *m_kpmHandle;
    AR2SurfaceSetT      *surfaceSet[PAGES_MAX]; // Weak-reference. Strong reference is now in ARMarkerNFT class.
#endif

        int m_error;
        void setError(int error);
        ARMarkerSquare *markersSquare;
        int markersSquareCount;

        VideoSource *vs;
        AndroidVideoSource* avs;

        bool initAVS;


// Tracking.
        ARHandle* arHandle;											///< Structure containing general ARToolKit tracking information
        ARPattHandle* arPattHandle;									///< Structure containing information about trained patterns
        AR3DHandle* ar3DHandle;										///< Structure used to compute 3D poses from tracking data
        int arPattDetectionMode;
//
//        ros::Time imu_timestamp, last_imu_timestamp;
//        ros::Time img_timestamp, last_img_timestamp;

// Drawing.
        int backingWidth;
        int backingHeight;
        GLint viewPort[4];
        ARViewContentMode gContentMode;   //渲染屏幕是否全屏
        bool gARViewLayoutRequired;
        ARParamLT *gCparamLT;                                 ///< Camera paramaters
        ARGL_CONTEXT_SETTINGS_REF gArglSettings;              ///< GL settings for rendering video background
        ARdouble cameraLens[16];
        ARdouble cameraPose[16];
        int cameraPoseValid;
        bool gARViewInited;

// Drawing orientation.
        int gDisplayOrientation; // range [0-3]. 1=landscape.
        int gDisplayWidth;
        int gDisplayHeight;
        int gDisplayDPI; // Android default.

        bool gContentRotate90;
        bool gContentFlipV;
        bool gContentFlipH;

// Network.
        int gInternetState;      // 默认是 -1
        IMUValues arIMUData;

        mat4 mRotationMatrix;

    public:
        float output_position[3];
        float output_rotation[3];
        ARController();

        virtual ~ARController();

        bool shutdown();

        void lockVideoSource();

        void unlockVideoSource();

        bool stopRunning();

        bool initService(jobject instanceOfAndroidContext);

        bool onStart();

        bool onStop();

        bool destroyService();

        bool videoInit(const int videoSourceIndex, int width, int height, int cameraIndex, bool cameraIsFrontFacing);

        bool videoAcceptImage(JNIEnv* env, jobject obj, const int videoSourceIndex, jbyteArray pinArray, jint width, jint height, jint cameraIndex, jboolean cameraIsFrontFacing);

//        void onPause();
//
//        void onResume();

        void setVideoFrame(JNIEnv* env, jobject obj, jbyteArray pinArray);

        void setIMUData(float *wValues,float *aValues);
//
// OpenGL functions.
//
        void SurfaceCreated();

        void SurfaceChanged(int w, int h);

        void displayParametersChanged(int orientation, int width, int height, int dpi);

        void setInternetState(int state);

        void drawFrame();

        void updateRotPosMatrix();


    private:
        void nativeVideoGetCparamCallback(const ARParam *cparam, void *userdata);

// Lays out the AR view. Requires both video and OpenGL to be inited, and must be called on OpenGL thread.
// References globals: gContentMode, backingWidth, backingHeight, videoWidth, videoHeight, .
// Modifies globals: gContentFlipV, gContentFlipH, gContentRotate90, viewPort, gARViewLayoutRequired.
        bool layoutARView(void);

// All tasks which require both video and OpenGL to be inited should be performed here.
// References globals: gCparamLT, gPixFormat
// Modifies globals: gArglSettings
        bool initARView(void);

        void drawCube(float size, float x, float y, float z);

        void drawLine(float size, float x, float y, float z);

        void computePose(float mat[3][4] ,int i);

        void computePose(float mat[16] ,int i);


    };
}


#endif //VINSI_VINSCONTROLLER_H
