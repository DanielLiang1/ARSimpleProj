//
// Created by ys on 2018/8/27.
//
#include "ARWrapper/AndroidVideoSource.h"
#include "ARController.h"
#include <AR/ar.h>
#include <AR/param.h>
#include <AR/matrix.h>
#include <ARToolKit/include/ARWrapper/Error.h>
#include <pthread.h>
#include "jni.h"
#include "Matrix4.h"

#ifndef MIN
#  define MIN(a,b)  ((a) > (b) ? (b) : (a))
#endif

#ifndef MAX
#  define MAX(a,b)  ((a) < (b) ? (b) : (a))
#endif

// Preferences.
static const char *cparaName = "Data/camera_para_RealMax0316.dat";//_REALMAS.dat";		_RealMax0316		///< Camera parameters file optical_param_left camera_para_original
static const char *markerConfigDataFilename = "Data/markers.dat";

const float NEAR_PLANE = 10.0f;                           ///< Near plane distance for projection matrix calculation
const float FAR_PLANE = 5000.0f;                          ///< Far plane distance for projection matrix calculation

#define DEBUG
#define __ANDROID__

// 旋转矩阵转欧拉角
//static void MatrixToEuler(float mat[3][4], float euler[]) {
//
//    if (mat[0][1] > 0.998) { // singularity at north pole
//        euler[0] = atan2(mat[2][0], mat[2][2]);
//        euler[1] = 3.1415926535 / 2;
//        euler[2] = 0;
//        return;
//    }
//    if (mat[0][1] < -0.998) { // singularity at south pole
//        euler[0] = atan2(mat[2][0], mat[2][2]);
//        euler[1] = -3.1415926535 / 2;
//        euler[2] = 0;
//        return;
//    }
//
//    euler[0] = atan2(-mat[0][2], mat[0][0]);
//    euler[1] = asin(mat[0][1]);
//    euler[2] = atan2(-mat[2][1], mat[1][1]);
//
//}

static void LeftMatrixToEuler(float mat[3][4], float euler[]) {
//        R[0][0] = Hx;     R[0][1] = Hy;     R[0][2] = Hz;
//        R[0][3] = Mx;     R[4] = My;     R[5] = Mz;
//        R[6] = Ax;     R[7] = Ay;     R[8] = Az;
//        if (mat[7] > 0.998) { // singularity at north pole
//            euler[6] = atan2(mat[0], mat[2]);
//            euler[3] = 3.1415926535 / 2;
//            euler[0] = 0;
//            return;
//        }
//        if (mat[7] < -0.998) { // singularity at south pole
//            euler[6] = atan2(mat[0], mat[2]);
//            euler[3] = -3.1415926535 / 2;
//            euler[0] = 0;
//            return;
//        }

    euler[0] = atan2(mat[0][1], mat[1][1]);
    euler[1] = asin(-mat[2][1]);
    euler[2] = atan2(-mat[2][0], mat[2][2]);
}

void MatrixToEuler(float mat[], float euler[]) {
    if (mat[3] > 0.998) { // singularity at north pole
        euler[0] = atan2(mat[2], mat[8]);
        euler[1] = 3.1415926535 / 2;
        euler[2] = 0;
        return;
    }
    if (mat[3] < -0.998) { // singularity at south pole
        euler[0] = atan2(mat[2], mat[8]);
        euler[1] = -3.1415926535 / 2;
        euler[2] = 0;
        return;
    }
    euler[0] = atan2(-mat[6], mat[0]);
    euler[1] = asin(mat[3]);
    euler[2] = atan2(-mat[5], mat[4]);

}

namespace es1_controller{

    ARController::ARController(){
        state = NOTHING_INITIALISED,
//        state(NOTHING_INITIALISED),
//        versionString(NULL),
        m_videoSource0 = NULL,
        m_videoSource1 = NULL,
        m_videoSourceIsStereo = false,
        m_videoSourceFrameStamp0 = 0,
        m_videoSourceFrameStamp1 = 0,

        m_projectionMatrixSet = false,
        threshold = AR_DEFAULT_LABELING_THRESH,
        thresholdMode = AR_LABELING_THRESH_MODE_DEFAULT,
        imageProcMode = AR_DEFAULT_IMAGE_PROC_MODE,
        labelingMode = AR_DEFAULT_LABELING_MODE,
        pattRatio = AR_PATT_RATIO,
        patternDetectionMode = AR_DEFAULT_PATTERN_DETECTION_MODE,
        matrixCodeType =AR_MATRIX_CODE_TYPE_DEFAULT,
        debugMode = FALSE,
        doMarkerDetection = false,
        m_arHandle0 = NULL,
        m_arHandle1 =NULL,
        m_arPattHandle = NULL,
        m_ar3DHandle = NULL,
        m_ar3DStereoHandle = NULL,
#if HAVE_NFT
doNFTMarkerDetection(false),
m_nftMultiMode(false),
m_kpmRequired(true),
m_kpmBusy(false),
trackingThreadHandle(NULL),
m_ar2Handle(NULL),
m_kpmHandle(NULL),
#endif
        m_error = ARW_ERROR_NONE,
//                {
//#ifdef __APPLE__
//                //openlog("ARController C++", LOG_CONS, LOG_USER);
//        //syslog(LOG_ERR, "Hello world!\n");
//#endif
//#if HAVE_NFT
//                        for (int i = 0; i < PAGES_MAX; i++) surfaceSet[i] = NULL;
//#endif
//                        pthread_mutex_init(&m_videoSourceLock, NULL),
//                }

        gVid = NULL;
        videoInited = false;                                    ///< true when ready to receive video frames.
        videoWidth = 0;                                          ///< Width of the video frame in pixels.
        videoHeight = 0;                                         ///< Height of the video frame in pixels.
//        gPixFormat;                                  ///< Pixel format from ARToolKit enumeration.
        gVideoFrame = NULL;                                 ///< Buffer containing current video frame.
        gVideoFrameSize = 0;                                  ///< Size of buffer containing current video frame.
        videoFrameNeedsPixelBufferDataUpload = false;
        gCameraIndex = 0;
        gCameraIsFrontFacing = false;

// Markers.
        markersSquare = NULL;
        markersSquareCount = 0;

// Tracking.
//        arHandle;											///< Structure containing general ARToolKit tracking information
//        arPattHandle;									///< Structure containing information about trained patterns
//        ar3DHandle;										///< Structure used to compute 3D poses from tracking data
//        arPattDetectionMode;

// Drawing.
//        backingWidth;
//        backingHeight;
//        viewPort[4];
        gContentMode = ARViewContentModeScaleAspectFill;   //渲染屏幕是否全屏
//        gContentMode = ARViewContentModeScaleAspectFit;
        gARViewLayoutRequired = false;
        gCparamLT = NULL;                                 ///< Camera paramaters
        gArglSettings = NULL;              ///< GL settings for rendering video background
        gARViewInited = false;

// Drawing orientation.
        gDisplayOrientation = 1; // range [0-3]. 1=landscape.
        gDisplayWidth = 0;
        gDisplayHeight = 0;
        gDisplayDPI = 160; // Android default.

        gContentRotate90 = true;
        gContentFlipV = true;
        gContentFlipH = true;

// Network.
        gInternetState = 0;      // 默认是 -1

//        format
        output_position[3]={0};
        output_rotation[3]={0};

        vs = NULL;
        avs = NULL;
        initAVS = false;



    }

    ARController::~ARController() {
        shutdown();
        pthread_mutex_destroy(&m_videoSourceLock);
#ifdef __APPLE__
        //closelog();
#endif
    }

    bool ARController::shutdown()
    {
        logv(AR_LOG_LEVEL_DEBUG, "ARWrapper::ARController::shutdown(): called");
        do {
            switch (state) {
                case DETECTION_RUNNING:
                case WAITING_FOR_VIDEO:
                    logv(AR_LOG_LEVEL_DEBUG, "ARWrapper::ARController::shutdown(): DETECTION_RUNNING or WAITING_FOR_VIDEO, forcing stop.");
                    stopRunning();
                    break;

                case BASE_INITIALISED:
                    if (countMarkers() > 0) {
                        logv(AR_LOG_LEVEL_DEBUG, "ARWrapper::ARController::shutdown(): BASE_INITIALISED, cleaning up markers.");
                        removeAllMarkers();
                    }

                    if (m_arPattHandle) {
                        arPattDeleteHandle(m_arPattHandle);
                        m_arPattHandle = NULL;
                    }

                    state = NOTHING_INITIALISED;
                    // Fall though.
                case NOTHING_INITIALISED:
                    logv(AR_LOG_LEVEL_DEBUG, "ARWrapper::ARController::shutdown(): NOTHING_INITIALISED, complete");
                    break;
            }
        } while (state != NOTHING_INITIALISED);

        logv(AR_LOG_LEVEL_DEBUG, "ARWrapper::ARController::shutdown(): exiting, returning true");
        return true;
    }

    // private
    void ARController::lockVideoSource()
    {
        pthread_mutex_lock(&m_videoSourceLock);
    }

// private
    void ARController::unlockVideoSource()
    {
        pthread_mutex_unlock(&m_videoSourceLock);
    }

    bool ARController::stopRunning()
    {
        logv(AR_LOG_LEVEL_DEBUG, "ARWrapper::ARController::stopRunning(): called");
        if (state != DETECTION_RUNNING && state != WAITING_FOR_VIDEO) {
            logv(AR_LOG_LEVEL_ERROR, "ARController::stopRunning(): Error: Not running.");
            return false;
        }

#if HAVE_NFT
        // Tracking thread is holding a reference to the camera parameters. Closing the
    // video source will dispose of the camera parameters, thus invalidating this reference.
    // So must stop tracking before closing the video source.
    if (trackingThreadHandle) {
        logv(AR_LOG_LEVEL_DEBUG, "ARWrapper::ARController::stopRunning(): calling unloadNFTData()");
        unloadNFTData();
    }
#endif

        lockVideoSource();
        logv(AR_LOG_LEVEL_DEBUG, "ARWrapper::ARController::stopRunning(): called lockVideoSource()");
        if (m_videoSource0) {
            logv(AR_LOG_LEVEL_DEBUG, "ARWrapper::ARController::stopRunning(): if (m_videoSource0) true");
            if (m_videoSource0->isOpen()) {
                logv(AR_LOG_LEVEL_DEBUG, "ARWrapper::ARController::stopRunning(): calling m_videoSource0->close()");
                m_videoSource0->close();
            }
            delete m_videoSource0;
            m_videoSource0 = NULL;
        }

        if (m_videoSource1) {
            logv(AR_LOG_LEVEL_DEBUG, "ARWrapper::ARController::stopRunning(): if (m_videoSource1) true");
            if (m_videoSource1->isOpen()) {
                logv(AR_LOG_LEVEL_DEBUG, "ARWrapper::ARController::stopRunning(): calling m_videoSource1->close()");
                m_videoSource1->close();
            }
            delete m_videoSource1;
            m_videoSource1 = NULL;
        }
        unlockVideoSource();
        logv(AR_LOG_LEVEL_DEBUG, "ARWrapper::ARController::stopRunning(): called unlockVideoSource()");
        m_projectionMatrixSet = false;

#if HAVE_NFT
        // NFT cleanup.
    //logv("Cleaning up ARToolKit NFT handles.");
    if (m_ar2Handle) {
        logv(AR_LOG_LEVEL_DEBUG, "ARWrapper::ARController::stopRunning(): calling ar2DeleteHandle(&m_ar2Handle)");
        ar2DeleteHandle(&m_ar2Handle); // Sets m_ar2Handle to NULL.
    }
    if (m_kpmHandle) {
        logv(AR_LOG_LEVEL_DEBUG, "ARWrapper::ARController::stopRunning(): calling kpmDeleteHandle(&m_kpmHandle)");
        kpmDeleteHandle(&m_kpmHandle); // Sets m_kpmHandle to NULL.
    }
#endif

        //logv("Cleaning up ARToolKit handles.");
        if (m_ar3DHandle) {
            logv(AR_LOG_LEVEL_DEBUG, "ARWrapper::ARController::stopRunning(): calling ar3DDeleteHandle(&m_ar3DHandle)");
            ar3DDeleteHandle(&m_ar3DHandle); // Sets ar3DHandle0 to NULL.
        }
        if (m_ar3DStereoHandle) {
            logv(AR_LOG_LEVEL_DEBUG, "ARWrapper::ARController::stopRunning(): calling ar3DStereoDeleteHandle(&m_ar3DStereoHandle)");
            ar3DStereoDeleteHandle(&m_ar3DStereoHandle); // Sets ar3DStereoHandle to NULL.
        }

        if (m_arHandle0) {
            logv(AR_LOG_LEVEL_DEBUG, "ARWrapper::ARController::stopRunning(): if (m_arHandle0) true");
            arPattDetach(m_arHandle0);
            arDeleteHandle(m_arHandle0);
            m_arHandle0 = NULL;
        }

        if (m_arHandle1) {
            logv(AR_LOG_LEVEL_DEBUG, "ARWrapper::ARController::stopRunning(): if (m_arHandle1) true");
            arPattDetach(m_arHandle1);
            arDeleteHandle(m_arHandle1);
            m_arHandle1 = NULL;
        }

        state = BASE_INITIALISED;

        logv(AR_LOG_LEVEL_DEBUG, "ARWrapper::ARController::stopRunning(): exiting, returning true");
        return true;
    }

    bool ARController::initService(jobject instanceOfAndroidContext){
        ARLOG("init vins service!");
        // Change working directory for the native process, so relative paths can be used for file access.
        arUtilChangeToResourcesDirectory(AR_UTIL_RESOURCES_DIRECTORY_BEHAVIOR_BEST, NULL, instanceOfAndroidContext);
        arPattHandle = arPattCreateHandle();
        if (arPattHandle == NULL) {
            ARLOGe("Error creating pattern handle");
            return false;
        }

        // 测试Eigen库
//    Vector3d EulerAngle;
//    Vector3d avggyr;
//    avggyr = {1,2,3};
//    EulerAngle = 0.5 * avggyr;
//    LOGI("测试数值EulerAngle %f,%f,%f.\n ",EulerAngle(0),EulerAngle(1),EulerAngle(2));

        // 在markerSquare中记录本地Marker的详细信息，MarkerSquare中的 patt_id就是Marker的标识
        newMarkers(markerConfigDataFilename, arPattHandle, &markersSquare, &markersSquareCount, &arPattDetectionMode);
        ARLOG("markersSquareCount %d",markersSquareCount);
        if (!markersSquareCount) {
            ARLOGe("Error loading markers from config. file '%s'.", markerConfigDataFilename);
            arPattDeleteHandle(arPattHandle);
            arPattHandle = NULL;
            return false;
        }
        ARLOG("Marker count = %d\n", markersSquareCount);

        state = BASE_INITIALISED;

        return true;

    }

    bool ARController::onStart() {

        ARLOG("ARController::startRunning(): called, start running");

        // Check for initialization before starting video
        if (state != BASE_INITIALISED) {
            ARLOGe("ARController::startRunning(): Error: not initialized, exiting, returning false");
            return false;
        }

        lockVideoSource();
        m_videoSource0 = VideoSource::newVideoSource();
        unlockVideoSource();
        if (!m_videoSource0) {
            ARLOGe("ARController::startRunning(): Error: no video source, exiting, returning false");
            return false;
        }

//        "-format=NV21"
//        if (!NativeInterface.arwStartRunning("-format=NV21", cameraParaPath, 10.0F, 10000.0F))
//        startRunning(vconf, cparaName, NULL, 0);
        m_videoSource0->configure("-format=NV21", cparaName, NULL, 0);

        if (!m_videoSource0->open()) {
            if (m_videoSource0->getError() == ARW_ERROR_DEVICE_UNAVAILABLE) {
                ARLOGe("ARController::startRunning(): Error: video source unavailable, exiting, returning false");
                setError(ARW_ERROR_DEVICE_UNAVAILABLE);
            } else {
                ARLOGe("ARController::startRunning(): Error: unable to open video source, exiting, returning false");
            }
            lockVideoSource();
            delete m_videoSource0;
            m_videoSource0 = NULL;
            unlockVideoSource();
            return false;
        }

        m_videoSourceIsStereo = false;
        state = WAITING_FOR_VIDEO;
        stateWaitingMessageLogged = false;

        ARLOG("ARController::startRunning(): exiting, returning true");
        return true;
//        ARLOG("nativeStart\n");
//
//        gVid = ar2VideoOpen("");
//        if (!gVid) {
//            ARLOGe("Error: ar2VideoOpen.\n");
//            return (false);
//        }
//
//        return (true);
    }

    bool ARController::onStop() {
        ARLOG("nativeStop\n");

        int i, j;

        // Can't call arglCleanup() here, because nativeStop is not called on rendering thread.

        // Clean up ARToolKit data.
        if (ar3DHandle) ar3DDeleteHandle(&ar3DHandle);
        if (arHandle) {
            arPattDetach(arHandle);
            arDeleteHandle(arHandle);
            arHandle = NULL;
        }
        arParamLTFree(&gCparamLT);

        // OpenGL cleanup -- not done here.

        // Video cleanup.
        if (gVideoFrame) {
            free(gVideoFrame);
            gVideoFrame = NULL;
            gVideoFrameSize = 0;
        }
        ar2VideoClose(gVid);
        gVid = NULL;
        videoInited = false;

        return (true);

    }

    bool ARController::destroyService() {
        ARLOG("nativeDestroy\n");

        if (markersSquare) deleteMarkers(&markersSquare, &markersSquareCount, arPattHandle);
        if (arPattHandle) {
            arPattDeleteHandle(arPattHandle);
            arPattHandle = NULL;
        }

        return (true);
    }

    bool ARController::videoInit(const int videoSourceIndex, int width, int height, int cameraIndex, bool cameraIsFrontFacing) {
        ARLOG("Video Init\n");
        // As of ARToolKit v5.0, NV21 format video frames are handled natively,
        // and no longer require colour conversion to RGBA. A buffer (gVideoFrame)
        // must be set aside to copy the frame from the Java side.
        // If you still require RGBA format information from the video,
        // you can create your own additional buffer, and then unpack the NV21
        // frames into it in nativeVideoFrame() below.
        // Here is where you'd allocate the buffer:
        // ARUint8 *myRGBABuffer = (ARUint8 *)malloc(videoWidth * videoHeight * 4);

        if (videoSourceIndex < 0 || videoSourceIndex > (m_videoSourceIsStereo ? 1 : 0)) return false;

        int ret = true;
        lockVideoSource();

        vs = (videoSourceIndex == 0 ? m_videoSource0 : m_videoSource1);
        if (!vs) {
            ret = false;
        } else {
            avs = (AndroidVideoSource *)vs;

            if (!avs->isRunning()) {
                if (!avs->getVideoReadyAndroid(width, height, cameraIndex, cameraIsFrontFacing)) {
                    ret = false;
                    goto done;
                }
                if (!avs->isRunning()) {
                    // Save ourselves the work of pushing a frame which isn't needed yet.
                    goto done;
                }
            }

            if (avs->getPixelFormat() == AR_PIXEL_FORMAT_NV21) {
//                env->GetByteArrayRegion(pinArray, 0, avs->getFrameSize(), (jbyte *)avs->getFrame());
                avs->acceptImage(NULL);
            }

        }

        done:
        unlockVideoSource();

        return ret;

//        gPixFormat = AR_PIXEL_FORMAT_NV21;
////        gPixFormat = AR_PIXEL_FORMAT_RGB;  // 修改后会出问题
//        ARLOG("RGB ...");
//        gVideoFrameSize = (sizeof(ARUint8)*(w*h + 2*w/2*h/2));
//        gVideoFrame = (ARUint8*) (malloc(gVideoFrameSize));
//        if (!gVideoFrame) {
//            gVideoFrameSize = 0;
//            ARLOGe("Error allocating frame buffer");
//            return false;
//        }
//        videoWidth = w;
//        videoHeight = h;
//
//        gCameraIndex = cameraIndex;
//        gCameraIsFrontFacing = cameraIsFrontFacing;
//        ARLOG("Video camera %d (%s), %dx%d format %s, %d-byte buffer.", gCameraIndex, (gCameraIsFrontFacing ? "front" : "rear"), w, h, arUtilGetPixelFormatName(gPixFormat), gVideoFrameSize);
//
//        ar2VideoSetParami(gVid, AR_VIDEO_PARAM_ANDROID_WIDTH, videoWidth);
//        ar2VideoSetParami(gVid, AR_VIDEO_PARAM_ANDROID_HEIGHT, videoHeight);
//        ar2VideoSetParami(gVid, AR_VIDEO_PARAM_ANDROID_PIXELFORMAT, (int)gPixFormat);
//        ar2VideoSetParami(gVid, AR_VIDEO_PARAM_ANDROID_CAMERA_INDEX, gCameraIndex);
//        ar2VideoSetParami(gVid, AR_VIDEO_PARAM_ANDROID_CAMERA_FACE, gCameraIsFrontFacing);
//        ar2VideoSetParami(gVid, AR_VIDEO_PARAM_ANDROID_INTERNET_STATE, gInternetState);
//
//        nativeVideoGetCparamCallback(NULL,NULL);
//        return true;
    }

//    bool ARController::videoAcceptImage(JNIEnv* env, jobject obj, const int videoSourceIndex, jbyteArray pinArray, jint width, jint height, jint cameraIndex, jboolean cameraIsFrontFacing)
//    {
//        if (videoSourceIndex < 0 || videoSourceIndex > (m_videoSourceIsStereo ? 1 : 0)) return false;
//
//        int ret = true;
//        lockVideoSource();
//
//        VideoSource *vs = (videoSourceIndex == 0 ? m_videoSource0 : m_videoSource1);
//        if (!vs) {
//            ret = false;
//        } else {
//            AndroidVideoSource* avs = (AndroidVideoSource *)vs;
//
//            if (!avs->isRunning()) {
//                if (!avs->getVideoReadyAndroid(width, height, cameraIndex, cameraIsFrontFacing)) {
//                    ret = false;
//                    goto done;
//                }
//                if (!avs->isRunning()) {
//                    // Save ourselves the work of pushing a frame which isn't needed yet.
//                    goto done;
//                }
//            }
//
//            if (!pinArray) { // Sanity check.
//                ret = false;
//                goto done;
//            }
//
//            if (avs->getPixelFormat() == AR_PIXEL_FORMAT_NV21) {
//                env->GetByteArrayRegion(pinArray, 0, avs->getFrameSize(), (jbyte *)avs->getFrame());
//                avs->acceptImage(NULL);
//            } else {
//                if (jbyte* buff = env->GetByteArrayElements(pinArray, NULL)) {
//                    avs->acceptImage((unsigned char *)buff);
//                    env->ReleaseByteArrayElements(pinArray, buff, JNI_ABORT); // JNI_ABORT -> don't copy back changes on the native side to the Java side.
//                }
//            }
//        }
//
//        done:
//        unlockVideoSource();
//
//        return ret;
//
//    }

    void ARController::nativeVideoGetCparamCallback(const ARParam *cparam_p, void *userdata)
   {
        // Load the camera parameters, resize for the window and init.
        ARParam *cparam;
        arMalloc(cparam, ARParam, 1);

       ARLOG("load parameter file %s for camera.\n", cparaName);

       ARLOGe("Unable to automatically determine camera parameters. Using default.\n");
       if (arParamLoad(cparaName, 1, cparam) < 0) {
           ARLOGe("Error: Unable to load parameter file %s for camera.\n", cparaName);
           return;
       }

        if (cparam->xsize != videoWidth || cparam->ysize != videoHeight) {
//            Distortion factor: k1=0.0355135389, k2=-0.0838622078, p1=-0.0034797140, p2=-0.0002992768
//            fx=263.729004, fy=236.675064, x0=157.690140, y0=126.336472, s=0.991185
//            cparam->xsize = 320;
//            cparam->ysize = 240;
//            cparam->dist_factor[0] = 0.0355135389;
//            cparam->dist_factor[1] = -0.0838622078;
//            cparam->dist_factor[2] = -0.0034797140;
//            cparam->dist_factor[3] =-0.0002992768;
//            cparam->dist_factor[4] = 263.729004;
//            cparam->dist_factor[5] = 236.675064;
//            cparam->dist_factor[6] = 157.690140;
//            cparam->dist_factor[7] = 126.336472;
//            cparam->dist_factor[8] =0.991185;
            //            266.07432 0.00000 157.69014 0.00000
//            0.00000 238.77980 126.33647 0.00000
//            0.00000 0.00000 1.00000 0.00000
//
//            cparam->mat[0][0] = 266.07432; cparam->mat[0][1] = 0.00000;
//            cparam->mat[0][2] = 157.69014; cparam->mat[0][3] = 0.00000;
//            cparam->mat[1][0] = 0.00000; cparam->mat[1][1] =  238.77980;
//            cparam->mat[1][2] = 126.33647;cparam->mat[1][3] = 0.00000;
//            cparam->mat[2][0] = 0.00000; cparam->mat[2][1] = 0.00000;
//            cparam->mat[2][2] = 1.00000; cparam->mat[2][3] = 0.00000;
//            SIZE = 1280, 720
//            k1=0.1920062900, k2=-0.8008726835, p1=-0.0007794611, p2=0.0035467513
//            fx=977.631287, fy=977.907593, x0=644.078125, y0=358.304474, s=0.987896
//            cparam->xsize = 1280;
//            cparam->ysize = 720;
//            cparam->dist_factor[0] = 0.1920062900;
//            cparam->dist_factor[1] = -0.8008726835;
//            cparam->dist_factor[2] = -0.0007794611;
//            cparam->dist_factor[3] = 0.0035467513;
//            cparam->dist_factor[4] = 977.631287;
//            cparam->dist_factor[5] = 977.907593;
//            cparam->dist_factor[6] = 644.078125;
//            cparam->dist_factor[7] = 358.304474;
//            cparam->dist_factor[8] = 0.987896;
//
//            cparam->mat[0][0] = 989.60910; cparam->mat[0][1] = 0.00000;
//            cparam->mat[0][2] = 644.07813; cparam->mat[0][3] = 0.00000;
//            cparam->mat[1][0] = 0.00000;  cparam->mat[1][1] =  989.88879;
//            cparam->mat[1][2] = 358.30447;cparam->mat[1][3] = 0.00000;
//            cparam->mat[2][0] = 0.00000; cparam->mat[2][1] = 0.00000;
//            cparam->mat[2][2] = 1.00000; cparam->mat[2][3] = 0.00000;

//            cparam->xsize = 1600;
//            cparam->ysize = 1200;
//            cparam->dist_factor[0] = 0.1456618309;
//            cparam->dist_factor[1] = -0.6506999135;
//            cparam->dist_factor[2] = -0.0024965154;
//            cparam->dist_factor[3] = 0.0002728975;
//            cparam->dist_factor[4] = 1311.555420;
//            cparam->dist_factor[5] = 1313.106079;
//            cparam->dist_factor[6] = 808.808167;
//            cparam->dist_factor[7] = 584.676941;
//            cparam->dist_factor[8] = 0.780306;
//
//            cparam->mat[0][0] = 1680.82195; cparam->mat[0][1] = 0.00000;
//            cparam->mat[0][2] = 808.80817; cparam->mat[0][3] = 0.00000;
//            cparam->mat[1][0] = 0.00000;  cparam->mat[1][1] =  1682.80919;
//            cparam->mat[1][2] = 584.67694;cparam->mat[1][3] = 0.00000;
//            cparam->mat[2][0] = 0.00000; cparam->mat[2][1] = 0.00000;
//            cparam->mat[2][2] = 1.00000; cparam->mat[2][3] = 0.00000;

//            cparam->xsize = 3200;
//            cparam->ysize = 2400;
//            cparam->dist_factor[0] = 0.0889455378;
//            cparam->dist_factor[1] = -0.2885870337;
//            cparam->dist_factor[2] = -0.0031488680;
//            cparam->dist_factor[3] = -0.0006932581;
//            cparam->dist_factor[4] = 2473.736084;
//            cparam->dist_factor[5] = 2475.408203;
//            cparam->dist_factor[6] = 1611.871582;
//            cparam->dist_factor[7] = 1184.756104;
//            cparam->dist_factor[8] = 0.990201;
//
//            cparam->mat[0][0] = 2498.21630; cparam->mat[0][1] = 0.00000;
//            cparam->mat[0][2] = 1611.87158; cparam->mat[0][3] = 0.00000;
//            cparam->mat[1][0] = 0.00000;  cparam->mat[1][1] = 2499.90496;
//            cparam->mat[1][2] = 1184.75610;cparam->mat[1][3] = 0.00000;
//            cparam->mat[2][0] = 0.00000; cparam->mat[2][1] = 0.00000;
//            cparam->mat[2][2] = 1.00000; cparam->mat[2][3] = 0.00000;

                    ARLOG("*** Camera Parameter resized from %d, %d. ***\n", cparam->xsize, cparam->ysize);
            for(int i=0;i<9;i++){
                ARLOG("info.dist_factor[%d]:%f",i,cparam->dist_factor[i]);
            }
            for(int i=0;i<3;i++){
                for(int j=0;j<4;j++){
                    ARLOG("info.mat[%d][%d]:%f",i,j,cparam->mat[i][j]);
                }
            }

//            videoWidth videoHeight 是本地设置的宽和高，预览画面的。
            arParamChangeSize(cparam, videoWidth, videoHeight, cparam);
        }
        ARLOG("*** Camera Parameter ***\n");
        arParamDisp(cparam);

        if ((gCparamLT = arParamLTCreate(cparam, AR_PARAM_LT_DEFAULT_OFFSET)) == NULL) {
            ARLOGe("Error: arParamLTCreate.\n");
            return;
        }
        ARLOG("Camera dist_function_version: %d",cparam->dist_function_version);
        videoInited = true;

        //
        // AR init.
        //

        // Create the OpenGL projection from the calibrated camera parameters.

        arglCameraFrustumRHf(&gCparamLT->param, (ARdouble)NEAR_PLANE, (ARdouble)FAR_PLANE, cameraLens);

        cameraPoseValid = FALSE;

        // Init AR.
        arHandle = arCreateHandle(gCparamLT);
        if (arHandle == NULL) {
            ARLOGe("Error creating AR handle");
            return;
        }
        arPattAttach(arHandle, arPattHandle);

        if (arSetPixelFormat(arHandle, gPixFormat) < 0) {
            ARLOGe("Error setting pixel format");
            return;
        }

        ar3DHandle = ar3DCreateHandle(&gCparamLT->param);
        if (ar3DHandle == NULL) {
            ARLOGe("Error creating 3D handle");
            return;
        }

        // Other ARToolKit setup.
        arSetMarkerExtractionMode(arHandle, AR_USE_TRACKING_HISTORY_V2);
        //arSetMarkerExtractionMode(arHandle, AR_NOUSE_TRACKING_HISTORY);
        //arSetLabelingThreshMode(arHandle, AR_LABELING_THRESH_MODE_MANUAL); // Uncomment to use  manual thresholding.

        // Set the pattern detection mode (template (pictorial) vs. matrix (barcode) based on
        // the marker types as defined in the marker config. file.
        arSetPatternDetectionMode(arHandle, arPattDetectionMode); // Default = AR_TEMPLATE_MATCHING_COLOR
        ARLOG("arPattDetectionMode:%d.\n",arPattDetectionMode);

        // Other application-wide marker options. Once set, these apply to all markers in use in the application.
        // If you are using standard ARToolKit picture (template) markers, leave commented to use the defaults.
        // If you are usign a different marker design (see http://www.artoolworks.com/support/app/marker.php )
        // then uncomment and edit as instructed by the marker design application.
        arSetLabelingMode(arHandle, AR_LABELING_BLACK_REGION); // Default = AR_LABELING_BLACK_REGION
        arSetBorderSize(arHandle, 0.25f); // Default = 0.25f
        arSetMatrixCodeType(arHandle, AR_MATRIX_CODE_3x3); // Default = AR_MATRIX_CODE_3x3

//    arLoadOpticalParam(optical_param_name);
    }

    void ARController::setVideoFrame(JNIEnv* env, jobject obj, jbyteArray pinArray) {

        if (!pinArray || !initAVS) { // Sanity check.
            if (avs->getPixelFormat() == AR_PIXEL_FORMAT_NV21) {
                env->GetByteArrayRegion(pinArray, 0, avs->getFrameSize(), (jbyte *)avs->getFrame());
                avs->acceptImage(NULL);
            } else {
                if (jbyte* buff = env->GetByteArrayElements(pinArray, NULL)) {
                    avs->acceptImage((unsigned char *)buff);
                    env->ReleaseByteArrayElements(pinArray, buff, JNI_ABORT); // JNI_ABORT -> don't copy back changes on the native side to the Java side.
                }
            }
            initAVS = true;
            ARLOG("nativeVideoFrame !VIDEO\n");
            return;
        }


        if (!gARViewInited) {
            ARLOG("nativeVideoFrame !ARVIEW\n");
            return; // Also, we won't track until the ARView has been inited.

        }
//        clock_gettime(CLOCK_MONOTONIC, &tp);
//        uint64_t  sec = (uint64_t)tp.tv_sec;
//        uint64_t nsec = (uint64_t)tp.tv_nsec;// ns 纳秒
//        trans_.stamp = ros::Time(sec, nsec);
//        double diff = trans_.stamp.toSec() - last_img_timestamp.toSec();
//
//        if(diff < IMG_TIMESPAN){
//            return;
//        }


//        last_img_timestamp = trans_.stamp;

        gVideoFrame = gVideoFrame_p;
        videoFrameNeedsPixelBufferDataUpload = true; // Note that buffer needs uploading. (Upload must be done on OpenGL context's thread.)
//        artoolkit marker标求解
        int i, j, k,z;
        ARdouble err;
        // Run marker detection on frame
        arDetectMarker(arHandle, gVideoFrame);

        // Get detected markers
        ARMarkerInfo* markerInfo = arGetMarker(arHandle);
        int markerNum = arGetMarkerNum(arHandle);

        // Update markers.
        for (i = 0; i < markersSquareCount; i++) {
            markersSquare[i].validPrev = markersSquare[i].valid;


            // Check through the marker_info array for highest confidence
            // visible marker matching our preferred pattern.
            k = -1;
            if (markersSquare[i].patt_type == AR_PATTERN_TYPE_TEMPLATE) {
                for (j = 0; j < markerNum; j++) {
                    if (markersSquare[i].patt_id == markerInfo[j].idPatt) {
                        if (k == -1) {
                            if (markerInfo[j].cfPatt >= markersSquare[i].matchingThreshold) k = j; // First marker detected.
                        } else if (markerInfo[j].cfPatt > markerInfo[k].cfPatt) k = j; // Higher confidence marker detected.
                    }
                }
                if (k != -1) {
                    markerInfo[k].id = markerInfo[k].idPatt;
                    markerInfo[k].cf = markerInfo[k].cfPatt;
                    markerInfo[k].dir = markerInfo[k].dirPatt;
                }
            } else {
                for (j = 0; j < markerNum; j++) {
                    if (markersSquare[i].patt_id == markerInfo[j].idMatrix) {
                        if (k == -1) {
                            if (markerInfo[j].cfMatrix >= markersSquare[i].matchingThreshold) k = j; // First marker detected.
                        } else if (markerInfo[j].cfMatrix > markerInfo[k].cfMatrix) k = j; // Higher confidence marker detected.
                    }
                }
                if (k != -1) {
                    markerInfo[k].id = markerInfo[k].idMatrix;
                    markerInfo[k].cf = markerInfo[k].cfMatrix;
                    markerInfo[k].dir = markerInfo[k].dirMatrix;
                }
            }

            if (k != -1) {
                markersSquare[i].valid = TRUE;
//                ARLOG("Marker %d matched pattern %d.\n", k, markerInfo[k].id);

                // Get the transformation between the marker and the real camera into trans.
                if (markersSquare[i].validPrev) {
                    err = arGetTransMatSquareCont(ar3DHandle, &(markerInfo[k]), markersSquare[i].trans, markersSquare[i].marker_width, markersSquare[i].trans);
                } else {
                    err = arGetTransMatSquare(ar3DHandle, &(markerInfo[k]), markersSquare[i].marker_width, markersSquare[i].trans);
                }
//                for(int m=0;m<3;m++){
//                    ARLOG("final Trans %f   %f   %f   [%f]",markersSquare[i].trans[m][0],markersSquare[i].trans[m][1],
//                          markersSquare[i].trans[m][2],markersSquare[i].trans[m][3]);
//
//                }
//                markersSquare[i].trans[0][3] = markersSquare[i].trans[0][3]+20;
//                markersSquare[i].trans[1][3] = markersSquare[i].trans[1][3]+20;

//                for(int m=0;m<3;m++){
//                    ARLOG("final Trans %f   %f   %f   [%f]",markersSquare[i].trans[m][0],markersSquare[i].trans[m][1],
//                          markersSquare[i].trans[m][2],markersSquare[i].trans[m][3]);
//
//                }


            } else {
                markersSquare[i].valid = FALSE;
            }

            if (markersSquare[i].valid) {

                // Filter the pose estimate.
                if (markersSquare[i].ftmi) {
                    if (arFilterTransMat(markersSquare[i].ftmi, markersSquare[i].trans, !markersSquare[i].validPrev) < 0) {
                        ARLOGe("arFilterTransMat error with marker %d.\n", i);
                    }
                }

                if (!markersSquare[i].validPrev) {
                    // Marker has become visible, tell any dependent objects.
                    //ARMarkerAppearedNotification
                }

                // We have a new pose, so set that. 转置
                arglCameraViewRHf(markersSquare[i].trans, markersSquare[i].pose.T, 1.0f /*VIEW_SCALEFACTOR*/);

                computePose(markersSquare[i].trans,i);
                computePose(markersSquare[i].pose.T,i);
//                arglCameraViewRHf(markersSquare[i].trans, markersSquare[i].pose.T, 1.0f /*VIEW_SCALEFACTOR*/);
                // Tell any dependent objects about the update.
                //ARMarkerUpdatedPoseNotification


            } else {

                if (markersSquare[i].validPrev) {
                    // Marker has ceased to be visible, tell any dependent objects.
                    //ARMarkerDisappearedNotification
                }
            }
        }


    }

    void ARController::setIMUData(float *wValues, float *aValues) {
        if (!videoInited || !gARViewInited) {
            ARLOG("nativeVideoFrame !VIDEOgARView\n");
            return; // No point in trying to track until video is inited.
        }

        arIMUData.stamp = wValues[0];
        for(int i=1;i<4;i++){
            arIMUData.gyroCoord[i-1] = wValues[i];
            arIMUData.accCoord[i-1] = aValues[i];
        }

//        clock_gettime(CLOCK_MONOTONIC, &tp);
//        uint64_t  sec = (uint64_t)tp.tv_sec;
//        uint64_t nsec = (uint64_t)tp.tv_nsec;
//
//        imu_timestamp = ros::Time(sec, nsec);
//
//        double diff = imu_timestamp.toSec() - last_imu_timestamp.toSec();
//        if(diff < IMU_TIMESPAN){
//            return;
//        }
//        last_imu_timestamp = imu_timestamp;
//        ARLOG("timestamp imu  sec: %f,diff %f ,fre %f\n", imu_timestamp.toSec(),diff,(1/diff));

//        ARLOG("nativeSetIMUData:quat rev wData: %lld, %f, %f, %f", timestemp, arIMUData.gyroCoord[0], arIMUData.gyroCoord[1], arIMUData.gyroCoord[2]);
//	    ARLOG("nativeSetIMUData:quat rev aData: %lld, %f, %f, %f", timestemp, arIMUData.accCoord[0], arIMUData.accCoord[1], arIMUData.accCoord[2]);

    }

    void ARController::SurfaceCreated() {
        ARLOG("nativeSurfaceCreated\n");
        glClearColor(0.0f, 0.0f, 0.0f, 1.0f);

        glStateCacheFlush(); // Make sure we don't hold outdated OpenGL state.

        if (gArglSettings) {
            arglCleanup(gArglSettings); // Clean up any left-over ARGL data.
            gArglSettings = NULL;
        }

        gARViewInited = false;
    }

    void ARController::SurfaceChanged(int w, int h) {
        backingWidth = w;
        backingHeight = h;
        ARLOG("nativeSurfaceChanged backingWidth=%d, backingHeight=%d\n", w, h);

        // Call through to anyone else who needs to know about window sizing here.

        // In order to do something meaningful with the surface backing size in an AR sense,
        // we also need the content size, which we aren't guaranteed to have yet, so defer
        // the viewPort calculations.
        gARViewLayoutRequired = true;
    }

    void ARController::displayParametersChanged(int orientation, int width, int height,
                                                  int dpi) {
        ARLOG("nativeDisplayParametersChanged orientation=%d, size=%dx%d@%dpi\n", orientation, width, height, dpi);
        gDisplayOrientation = orientation;
//    gDisplayOrientation = 3;
        gDisplayWidth = width;
        gDisplayHeight = height;
        gDisplayDPI = dpi;

        gARViewLayoutRequired = true;
    }

    void ARController::setInternetState(int state) {
        gInternetState = state;
        if (gVid) {
            ar2VideoSetParami(gVid, AR_VIDEO_PARAM_ANDROID_INTERNET_STATE, state);
        }
    }

    bool ARController::layoutARView() {
        if (gDisplayOrientation == 0) {
            gContentRotate90 = true;
            gContentFlipV = false;
            gContentFlipH = gCameraIsFrontFacing;
        } else if (gDisplayOrientation == 1) {
            gContentRotate90 = false;
            gContentFlipV = false;
            gContentFlipH = gCameraIsFrontFacing;
        } else if (gDisplayOrientation == 2) {
            gContentRotate90 = true;
            gContentFlipV = true;
            gContentFlipH = (!gCameraIsFrontFacing);
        } else if (gDisplayOrientation == 3) {
            gContentRotate90 = false;
            gContentFlipV = true;
            gContentFlipH = (!gCameraIsFrontFacing);
        }
        arglSetRotate90(gArglSettings, gContentRotate90);
        arglSetFlipV(gArglSettings, gContentFlipV);
        arglSetFlipH(gArglSettings, gContentFlipH);

        // Calculate viewPort.
        int left, bottom, w, h;
        int contentWidth = videoWidth;
        int contentHeight = videoHeight;

        if (gContentMode == ARViewContentModeScaleToFill) {
            w = backingWidth;
            h = backingHeight;
        } else {
            int contentWidthFinalOrientation = (gContentRotate90 ? contentHeight : contentWidth);
            int contentHeightFinalOrientation = (gContentRotate90 ? contentWidth : contentHeight);
            if (gContentMode == ARViewContentModeScaleAspectFit || gContentMode == ARViewContentModeScaleAspectFill) {
                float scaleRatioWidth, scaleRatioHeight, scaleRatio;
                scaleRatioWidth = (float)backingWidth / (float)contentWidthFinalOrientation;
                scaleRatioHeight = (float)backingHeight / (float)contentHeightFinalOrientation;
                if (gContentMode == ARViewContentModeScaleAspectFill) scaleRatio = MAX(scaleRatioHeight, scaleRatioWidth);
                else scaleRatio = MIN(scaleRatioHeight, scaleRatioWidth);
                w = (int)((float)contentWidthFinalOrientation * scaleRatio);
                h = (int)((float)contentHeightFinalOrientation * scaleRatio);
            } else {
                w = contentWidthFinalOrientation;
                h = contentHeightFinalOrientation;
            }
        }

        if (gContentMode == ARViewContentModeTopLeft
            || gContentMode == ARViewContentModeLeft
            || gContentMode == ARViewContentModeBottomLeft) left = 0;
        else if (gContentMode == ARViewContentModeTopRight
                 || gContentMode == ARViewContentModeRight
                 || gContentMode == ARViewContentModeBottomRight) left = backingWidth - w;
        else left = (backingWidth - w) / 2;

        if (gContentMode == ARViewContentModeBottomLeft
            || gContentMode == ARViewContentModeBottom
            || gContentMode == ARViewContentModeBottomRight) bottom = 0;
        else if (gContentMode == ARViewContentModeTopLeft
                 || gContentMode == ARViewContentModeTop
                 || gContentMode == ARViewContentModeTopRight) bottom = backingHeight - h;
        else bottom = (backingHeight - h) / 2;

        glViewport(left, bottom, w, h);

        viewPort[viewPortIndexLeft] = left;
        viewPort[viewPortIndexBottom] = bottom;
        viewPort[viewPortIndexWidth] = w;
        viewPort[viewPortIndexHeight] = h;

        ARLOG("Viewport={%d, %d, %d, %d}\n", left, bottom, w, h);

        // Call through to anyone else who needs to know about changes in the ARView layout here.
        // --->

        gARViewLayoutRequired = false;

        return (true);
    }


    bool ARController::initARView() {
        ARLOG("Initialising ARView\n");

        if (gARViewInited) return (false);

        ARLOG("Setting up argl.\n");
        if ((gArglSettings = arglSetupForCurrentContext(&gCparamLT->param, gPixFormat)) == NULL) {
            ARLOGe("Unable to setup argl.\n");
            return (false);
        }
        ARLOG("argl setup OK.\n");

        gARViewInited = true;
        ARLOG("ARView initialised.\n");

        return (true);
    }

    void ARController::drawCube(float size, float x, float y, float z) {
        // Colour cube data.
        int i;
        const GLfloat cube_vertices [8][3] = {
                /* +z */ {0.5f, 0.5f, 0.5f}, {0.5f, -0.5f, 0.5f}, {-0.5f, -0.5f, 0.5f}, {-0.5f, 0.5f, 0.5f},
                /* -z */ {0.5f, 0.5f, -0.5f}, {0.5f, -0.5f, -0.5f}, {-0.5f, -0.5f, -0.5f}, {-0.5f, 0.5f, -0.5f} };

        const GLubyte cube_vertex_colors [8][4] = {
                {255, 255, 255, 255}, {255, 255, 0, 255}, {0, 255, 0, 255}, {0, 255, 255, 255},
                {255, 0, 255, 255}, {255, 0, 0, 255}, {0, 0, 0, 255}, {0, 0, 255, 255} };

        const GLushort cube_faces [6][4] = { /* ccw-winding */
                /* +z */ {3, 2, 1, 0}, /* -y */ {2, 3, 7, 6}, /* +y */ {0, 1, 5, 4},
                /* -x */ {3, 0, 4, 7}, /* +x */ {1, 2, 6, 5}, /* -z */ {4, 5, 6, 7} };


        glPushMatrix(); // Save world coordinate system.
        glTranslatef(x, y, z);
        glScalef(size, size, size);
        glStateCacheDisableLighting();
        glStateCacheDisableTex2D();
        glStateCacheDisableBlend();
        glColorPointer(4, GL_UNSIGNED_BYTE, 0, cube_vertex_colors);
        glVertexPointer(3, GL_FLOAT, 0, cube_vertices);
        glStateCacheEnableClientStateVertexArray();
        glEnableClientState(GL_COLOR_ARRAY);
        for (i = 0; i < 6; i++) {
            glDrawElements(GL_TRIANGLE_FAN, 4, GL_UNSIGNED_SHORT, &(cube_faces[i][0]));
        }
        glDisableClientState(GL_COLOR_ARRAY);
        glColor4ub(0, 0, 0, 255);
        for (i = 0; i < 6; i++) {
            glDrawElements(GL_LINE_LOOP, 4, GL_UNSIGNED_SHORT, &(cube_faces[i][0]));
        }
        glPopMatrix();    // Restore world coordinate system.
    }

    void ARController::drawLine(float size, float x, float y, float z) {
        // Colour line data.
        int i;
        const GLfloat line_vertices [6][3] = {
//                          x                       y                   z
                /* +z */ {0.0f, 0.0f, 0.5f}, {0.0f, 0.0f, 0.5f}, {0.0f, 0.0f, 0.5f},
                         {1.5f, 0.0f, 0.5f}, {0.0f, 3.0f, 0.5f}, {0.0f, 0.0f, 0.8f}};

        const GLubyte line_vertex_colors [3][4] = {
//                x  red               y   green          z blue
                {255, 0, 0, 255}, {0, 255, 0, 255}, {0, 0, 255, 255} };

        const GLushort cube_faces [3][2] = { /* ccw-winding */
                /* +z */ {0,3}, /* +x */ {1,4}, /* +y */ {2,5},
//                /* -x */ {3}, /* +x */ {1}, /* -z */ {4}
        };


        glPushMatrix(); // Save world coordinate system.
//        translatef 沿着 x,y,z轴平移的距离，
        glTranslatef(x, y, z);
        glScalef(size, size, size);
        glStateCacheDisableLighting();
        glStateCacheDisableTex2D();
        glStateCacheDisableBlend();
//        定义一个颜色矩阵。4颜色数量只能是4，颜色类型 第三位表示颜色增幅
        glColorPointer(4, GL_UNSIGNED_BYTE, 0, line_vertex_colors);
        glVertexPointer(3, GL_FLOAT, 0, line_vertices);
        glStateCacheEnableClientStateVertexArray();
        glEnableClientState(GL_COLOR_ARRAY);
//        for (i = 0; i < 6; i++) {
//            glDrawElements(GL_TRIANGLE_FAN, 4, GL_UNSIGNED_SHORT, &(cube_faces[i][0]));
//        }
//        glDisableClientState(GL_COLOR_ARRAY);
//        glColor4ub(0, 0, 0, 255);
        for (i = 0; i < 3; i++) {
            glDrawElements(GL_LINE_STRIP , 3, GL_UNSIGNED_SHORT, &(cube_faces[i][0]));
        }
        glPopMatrix();    // Restore world coordinate system.
    }

    void ARController::drawFrame() {
        float width, height;

        if (!videoInited) {
            ARLOG("nativeDrawFrame !VIDEO\n");
            return; // No point in trying to draw until video is inited.
        }
//        ARLOG("nativeDrawFrame\n");

        if (!gARViewInited) {
            if (!initARView()) return;
        }
        if (gARViewLayoutRequired) layoutARView();

        // Upload new video frame if required.
        if (videoFrameNeedsPixelBufferDataUpload) {
            arglPixelBufferDataUploadBiPlanar(gArglSettings, gVideoFrame, gVideoFrame + videoWidth*videoHeight);
            videoFrameNeedsPixelBufferDataUpload = false;
        }

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // Clear the buffers for new frame.

        // Display the current frame
        arglDispImage(gArglSettings);

        // Set up 3D mode.
        glMatrixMode(GL_PROJECTION);
        glLoadMatrixf(cameraLens);
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        glStateCacheEnableDepthTest();

        // Set any initial per-frame GL state you require here.
        // --->

        // Lighting and geometry that moves with the camera should be added here.
        // (I.e. should be specified before camera pose transform.)
        // --->

        // Draw an object on all valid markers.
        for (int i = 0; i < markersSquareCount; i++) {
            if (markersSquare[i].valid) {
                glLoadMatrixf(markersSquare[i].pose.T);
                drawCube(40.0f, 0.0f, 0.0f, 20.0f);
//                drawLine(40.0f, 0.0f,0.0f, 0.0f);

            }
        }

        if (cameraPoseValid) {

            glMultMatrixf(cameraPose);

            // All lighting and geometry to be drawn in world coordinates goes here.
            // --->
        }

        // If you added external OpenGL code above, and that code doesn't use the glStateCache routines,
        // then uncomment the line below.
        //glStateCacheFlush();

        // Set up 2D mode.
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        width = (float)viewPort[viewPortIndexWidth];
        height = (float)viewPort[viewPortIndexHeight];
        glOrthof(0.0f, width, 0.0f, height, -1.0f, 1.0f);
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        glStateCacheDisableDepthTest();

        // Add your own 2D overlays here.
        // --->

        // If you added external OpenGL code above, and that code doesn't use the glStateCache routines,
        // then uncomment the line below.
        //glStateCacheFlush();

#ifdef DEBUG
        // Example of 2D drawing. It just draws a white border line. Change the 0 to 1 to enable.
        const GLfloat square_vertices [4][2] = { {0.5f, 0.5f}, {0.5f, height - 0.5f}, {width - 0.5f, height - 0.5f}, {width - 0.5f, 0.5f} };
        glStateCacheDisableLighting();
        glStateCacheDisableTex2D();
        glVertexPointer(2, GL_FLOAT, 0, square_vertices);
        glStateCacheEnableClientStateVertexArray();
        glColor4ub(255, 255, 255, 255);
        glDrawArrays(GL_LINE_LOOP, 0, 4);
#endif
    }

    void ARController::computePose(float mat[3][4], int i)
    {
        //   //////   求逆函数
        ARMat *tranMat;
        ARMat *tranMatInv;//, *transIn;
        tranMat = arMatrixAlloc(3,3);
//        transIn = arMatrixAlloc(3,3);
        tranMatInv = arMatrixAlloc(3,3);

        for( int j = 0; j < 3; j++ ) {
            for( int k = 0; k < 3; k++ ) {
                tranMat->m[k*3+j] = mat[j][k]; //
            }
//                    ARLOG("num %d:   %f      %f      %f",j,tranMat->m[j*3+0],tranMat->m[j*3+1],tranMat->m[j*3+2]);
        }
//                tranMat.m = markersSquare[i].pose.T;

        tranMatInv = arMatrixAllocInv(tranMat);
//                for( j = 0; j < 3; j++ ) {
//                    ARLOG("invnum %d:   %f      %f      %f",j,tranMatInv->m[j*3+0],tranMatInv->m[j*3+1],tranMatInv->m[j*3+2]);
//                }
//        transIn = arMatrixAllocMul( tranMat, tranMatInv );

//                for( j = 0; j < 3; j++ ) {
//                    ARLOG("num transIn %d:   %f      %f      %f",j,transIn->m[j*3+0],transIn->m[j*3+1],transIn->m[j*3+2]);
//                }

        float leftEuler[3];
        ARdouble position[3]={0};
        position[0] = mat[0][3];
        position[1] = mat[1][3];
        position[2] = mat[2][3];
//                LeftMatrixToEuler(markersSquare[i].trans,leftEuler);

        float rotArray[9];
        for (int m = 0; m < 3; m++) {
            for (int n = 0; n < 3; n++) {
                rotArray[3 * m + n] = mat[m][n];
            }
        }

        MatrixToEuler(rotArray,leftEuler );

//                MatrixToEuler(markersNFT[i].trans,rightEuler);
//            ARLOG("euler L  %f     %f     %f",
//                  leftEular[0] * 180 / PI,leftEular[1] * 180 / PI,leftEular[2] * 180 / PI);
//            ARLOG("euler R  %f     %f     %f",
//                  rightEuler[0] * 180 / PI,rightEuler[1] * 180 / PI,rightEuler[2] * 180 / PI);
        output_rotation[0] = leftEuler[0] * 180 / PI;
        output_rotation[1] = leftEuler[1] * 180 / PI;
        output_rotation[2] = leftEuler[2] * 180 / PI;

        output_position[0] = tranMatInv->m[0] * position[0] + tranMatInv->m[3] * position[1] + tranMatInv->m[6] * position[2];
        output_position[1] = tranMatInv->m[1] * position[0] + tranMatInv->m[4] * position[1] + tranMatInv->m[7] * position[2];
        output_position[2] = tranMatInv->m[2] * position[0] + tranMatInv->m[5] * position[1] + tranMatInv->m[8] * position[2];

//                ARLOG("num position: %f     %f     %f",position[0],position[1],position[2]);
        ARLOG("position: x   %f    y    %f    z     %f",output_position[0],output_position[1],output_position[2]);
        ARLOG("rotation: yaw %f    roll %f    pitch %f",output_rotation[0],output_rotation[1],output_rotation[2]);

        free(tranMat);
        free(tranMatInv);
//        free(transIn);

    }

    void ARController::computePose(float mat[16], int i)
    {
//        mat 是以列存储
        //   //////   求逆函数
        ARMat *tranMat;
        ARMat *tranMatInv;//, *transIn;
        tranMat = arMatrixAlloc(3,3);
//        transIn = arMatrixAlloc(3,3);
        tranMatInv = arMatrixAlloc(3,3);

        tranMat->m[0] = mat[0];
        tranMat->m[1] = mat[1];
        tranMat->m[2] = mat[2];
        tranMat->m[3] = mat[4];
        tranMat->m[4] = mat[5];
        tranMat->m[5] = mat[6];
        tranMat->m[6] = mat[8];
        tranMat->m[7] = mat[9];
        tranMat->m[8] = mat[10];

//        for( int j = 0; j < 3; j++ ) {
//            for( int k = 0; k < 3; k++ ) {
//                tranMat->m[k*3+j] = mat[k*3+j]; //
//            }
////                    ARLOG("num %d:   %f      %f      %f",j,tranMat->m[j*3+0],tranMat->m[j*3+1],tranMat->m[j*3+2]);
//        }
//                tranMat.m = markersSquare[i].pose.T;

        tranMatInv = arMatrixAllocInv(tranMat);
//                for( j = 0; j < 3; j++ ) {
//                    ARLOG("invnum %d:   %f      %f      %f",j,tranMatInv->m[j*3+0],tranMatInv->m[j*3+1],tranMatInv->m[j*3+2]);
//                }
//        transIn = arMatrixAllocMul( tranMat, tranMatInv );

//                for( j = 0; j < 3; j++ ) {
//                    ARLOG("num transIn %d:   %f      %f      %f",j,transIn->m[j*3+0],transIn->m[j*3+1],transIn->m[j*3+2]);
//                }

        float leftEuler[3];
        ARdouble position[3]={0};
        position[0] = mat[12];
        position[1] = mat[13];
        position[2] = mat[14];
//                LeftMatrixToEuler(markersSquare[i].trans,leftEuler);

        float rotArray[9];
        for (int m = 0; m < 3; m++) {
            for (int n = 0; n < 3; n++) {
                rotArray[3 * m + n] = tranMat->m[3 * n + m];
            }
        }

        MatrixToEuler(rotArray,leftEuler );

//                MatrixToEuler(markersNFT[i].trans,rightEuler);
//            ARLOG("euler L  %f     %f     %f",
//                  leftEular[0] * 180 / PI,leftEular[1] * 180 / PI,leftEular[2] * 180 / PI);
//            ARLOG("euler R  %f     %f     %f",
//                  rightEuler[0] * 180 / PI,rightEuler[1] * 180 / PI,rightEuler[2] * 180 / PI);
        output_rotation[0] = leftEuler[0] * 180 / PI;
        output_rotation[1] = leftEuler[1] * 180 / PI;
        output_rotation[2] = leftEuler[2] * 180 / PI;

        output_position[0] = tranMatInv->m[0] * position[0] + tranMatInv->m[3] * position[1] + tranMatInv->m[6] * position[2];
        output_position[1] = tranMatInv->m[1] * position[0] + tranMatInv->m[4] * position[1] + tranMatInv->m[7] * position[2];
        output_position[2] = tranMatInv->m[2] * position[0] + tranMatInv->m[5] * position[1] + tranMatInv->m[8] * position[2];

//                ARLOG("num position: %f     %f     %f",position[0],position[1],position[2]);
        ARLOG("pose position: x   %f    y    %f    z     %f",output_position[0],output_position[1],output_position[2]);
        ARLOG("pose rotation: yaw %f    roll %f    pitch %f",output_rotation[0],output_rotation[1],output_rotation[2]);

        free(tranMat);
        free(tranMatInv);
//        free(transIn);

    }

    void ARController::updateRotPosMatrix(){

        /* ARPose pose = markersSquare[markerValidNum].pose;
         memcpy(mRotationMatrix.value_ptr(), pose.T, sizeof(float)*16);*/


        glmath::Matrix4 mat4;
        // 用 MarkerValidNum标记因为 markersSquare[-1].valid没有定义


        for (int j = 0; j < markersSquareCount; j++){
            if( !markersSquare[j].valid ){
                return;
            }
            for(int i=0;i<16;i++)
            {
                // 默认用左眼
                mat4.m_data[i] = markersSquare[j].pose.T[i];
                //    LOGI("m_data:[%d] %f",i,mat4.m_data[i]);
            }
        }


        //get camera pose
        glmath::Matrix4::invertM(mat4.m_data,mat4.m_data);

        glmath::Vec3 p(0,0,0);
        glmath::Vec3 u(0,1,0);
        glmath::Vec3 d(0,0,-1);
        glmath::Matrix4::multiplyMV(mat4.m_data,p);
        glmath::Matrix4::multiplyMV(mat4.m_data,u);
        glmath::Matrix4::multiplyMV(mat4.m_data,d);
        glmath::Matrix4 newMat4;

        glmath::Vec3 pBegin = p;
        glmath::Vec3 dBegin = d-p;
        glmath::Vec3 uBegin = u-p;

        glmath::Matrix4::setLookAtM(newMat4.m_data,pBegin,dBegin,uBegin);

        float rotArray[9];
//        for (int m = 0; m < 3; m++) {
//            for (int n = 0; n < 3; n++) {
//                rotArray[3 * m + n] = markersSquare[i].trans[m][n];
//            }
//        }

        //更新旋转矩阵

        memcpy(mRotationMatrix.value_ptr(), newMat4.m_data, sizeof(float)*16);

    }

}
 