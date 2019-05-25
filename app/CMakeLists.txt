# For more information about using CMake with Android Studio, read the
# documentation: https://d.android.com/studio/projects/add-native-code.html

# Sets the minimum version of CMake required to build the native library.

cmake_minimum_required(VERSION 3.4.1)

#设置生成的so动态库最后输出的路径 : (/../app/jniLibs/${ANDROID_ABI})
#set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/SDKRelease/libs/jni/${ANDROID_ABI})
message(">>>>>>>>> CMAKE_LIBRARY_OUTPUT_DIRECTORY is " ${CMAKE_LIBRARY_OUTPUT_DIRECTORY})

# -s选项为strip，不strip生成的库文件会很大  但是-s 不能debug
# set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -s")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wno-extern-c-compat -Wno-null-conversion")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DDTR_SDK_MODE -Wno-extern-c-compat -Wno-null-conversion")
message(">>>>>>>>> CMAKE_CXX_FLAGS is " ${CMAKE_CXX_FLAGS})

# Specifies a path to native header files. 头文件路径
INCLUDE_DIRECTORIES(src/main/cpp/)
INCLUDE_DIRECTORIES(${ANDROID_NDK}/sources/android/cpufeatures)
INCLUDE_DIRECTORIES(${ANDROID_NDK}/sources/android/native_app_glue)

# 支持-std=gnu++11
set(CMAKE_VERBOSE_MAKEFILE on)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11")

include_directories(src/main/cpp/ARToolKit/include)
include_directories(src/main/cpp/ARToolKit/include/android)
include_directories(src/main/cpp/ARToolKit/SRC/KPM/FreakMatcher)
include_directories(src/main/cpp/ARToolKit)
include_directories(libs/curl/include)

# Creates and names a library, sets it as either STATIC
# or SHARED, and provides the relative paths to its source code.
# You can define multiple libraries, and CMake builds it for you.
# Gradle automatically packages shared libraries with your APK.


# 创建本地的动态库（也可以是静态的static） src/main/cpp/VinsHandler.cpp
add_library( ARWrapper   SHARED

              src/main/cpp/ARWrapperNativeCarsExample.cpp
              src/main/cpp/ARToolKitWrapperExportedAPI.cpp
              src/main/cpp/ARController.cpp
              src/main/cpp/ARMarkerSquare.cpp

                  src/main/cpp/ARToolKit/SRC/AR/ar3DCreateHandle.c
                  src/main/cpp/ARToolKit/SRC/AR/ar3DUtil.c
                  src/main/cpp/ARToolKit/SRC/AR/arCreateHandle.c
                  src/main/cpp/ARToolKit/SRC/AR/arDetectMarker.c
                  src/main/cpp/ARToolKit/SRC/AR/arDetectMarker2.c
                  src/main/cpp/ARToolKit/SRC/AR/arFilterTransMat.c
                  src/main/cpp/ARToolKit/SRC/AR/arGetLine.c
                  src/main/cpp/ARToolKit/SRC/AR/arGetMarkerInfo.c
                  src/main/cpp/ARToolKit/SRC/AR/arGetTransMat.c
                  src/main/cpp/ARToolKit/SRC/AR/arGetTransMatStereo.c
                  src/main/cpp/ARToolKit/SRC/AR/arImageProc.c
                  src/main/cpp/ARToolKit/SRC/AR/arLabeling.c
                  src/main/cpp/ARToolKit/SRC/AR/arLabelingSub/arLabelingSubDBI3C.c
                  src/main/cpp/ARToolKit/SRC/AR/arLabelingSub/arLabelingSubDBI3C565.c
                  src/main/cpp/ARToolKit/SRC/AR/arLabelingSub/arLabelingSubDBI3CA.c
                  src/main/cpp/ARToolKit/SRC/AR/arLabelingSub/arLabelingSubDBI3CA4444.c
                  src/main/cpp/ARToolKit/SRC/AR/arLabelingSub/arLabelingSubDBI3CA5551.c
                  src/main/cpp/ARToolKit/SRC/AR/arLabelingSub/arLabelingSubDBIA3C.c
                  src/main/cpp/ARToolKit/SRC/AR/arLabelingSub/arLabelingSubDBIC.c
                  src/main/cpp/ARToolKit/SRC/AR/arLabelingSub/arLabelingSubDBICY.c
                  src/main/cpp/ARToolKit/SRC/AR/arLabelingSub/arLabelingSubDBIYC.c
                  src/main/cpp/ARToolKit/SRC/AR/arLabelingSub/arLabelingSubDBR3C.c
                  src/main/cpp/ARToolKit/SRC/AR/arLabelingSub/arLabelingSubDBR3C565.c
                  src/main/cpp/ARToolKit/SRC/AR/arLabelingSub/arLabelingSubDBR3CA.c
                  src/main/cpp/ARToolKit/SRC/AR/arLabelingSub/arLabelingSubDBR3CA4444.c
                  src/main/cpp/ARToolKit/SRC/AR/arLabelingSub/arLabelingSubDBR3CA5551.c
                  src/main/cpp/ARToolKit/SRC/AR/arLabelingSub/arLabelingSubDBRA3C.c
                  src/main/cpp/ARToolKit/SRC/AR/arLabelingSub/arLabelingSubDBRC.c
                  src/main/cpp/ARToolKit/SRC/AR/arLabelingSub/arLabelingSubDBRCY.c
                  src/main/cpp/ARToolKit/SRC/AR/arLabelingSub/arLabelingSubDBRYC.c
                  src/main/cpp/ARToolKit/SRC/AR/arLabelingSub/arLabelingSubDBZ.c
                  src/main/cpp/ARToolKit/SRC/AR/arLabelingSub/arLabelingSubDWI3C.c
                  src/main/cpp/ARToolKit/SRC/AR/arLabelingSub/arLabelingSubDWI3C565.c
                  src/main/cpp/ARToolKit/SRC/AR/arLabelingSub/arLabelingSubDWI3CA.c
                  src/main/cpp/ARToolKit/SRC/AR/arLabelingSub/arLabelingSubDWI3CA4444.c
                  src/main/cpp/ARToolKit/SRC/AR/arLabelingSub/arLabelingSubDWI3CA5551.c
                  src/main/cpp/ARToolKit/SRC/AR/arLabelingSub/arLabelingSubDWIA3C.c
                  src/main/cpp/ARToolKit/SRC/AR/arLabelingSub/arLabelingSubDWIC.c
                  src/main/cpp/ARToolKit/SRC/AR/arLabelingSub/arLabelingSubDWICY.c
                  src/main/cpp/ARToolKit/SRC/AR/arLabelingSub/arLabelingSubDWIYC.c
                  src/main/cpp/ARToolKit/SRC/AR/arLabelingSub/arLabelingSubDWR3C.c
                  src/main/cpp/ARToolKit/SRC/AR/arLabelingSub/arLabelingSubDWR3C565.c
                  src/main/cpp/ARToolKit/SRC/AR/arLabelingSub/arLabelingSubDWR3CA.c
                  src/main/cpp/ARToolKit/SRC/AR/arLabelingSub/arLabelingSubDWR3CA4444.c
                  src/main/cpp/ARToolKit/SRC/AR/arLabelingSub/arLabelingSubDWR3CA5551.c
                  src/main/cpp/ARToolKit/SRC/AR/arLabelingSub/arLabelingSubDWRA3C.c
                  src/main/cpp/ARToolKit/SRC/AR/arLabelingSub/arLabelingSubDWRC.c
                  src/main/cpp/ARToolKit/SRC/AR/arLabelingSub/arLabelingSubDWRCY.c
                  src/main/cpp/ARToolKit/SRC/AR/arLabelingSub/arLabelingSubDWRYC.c
                  src/main/cpp/ARToolKit/SRC/AR/arLabelingSub/arLabelingSubDWZ.c
                  src/main/cpp/ARToolKit/SRC/AR/arLabelingSub/arLabelingSubEBI3C.c
                  src/main/cpp/ARToolKit/SRC/AR/arLabelingSub/arLabelingSubEBI3C565.c
                  src/main/cpp/ARToolKit/SRC/AR/arLabelingSub/arLabelingSubEBI3CA.c
                  src/main/cpp/ARToolKit/SRC/AR/arLabelingSub/arLabelingSubEBI3CA4444.c
                  src/main/cpp/ARToolKit/SRC/AR/arLabelingSub/arLabelingSubEBI3CA5551.c
                  src/main/cpp/ARToolKit/SRC/AR/arLabelingSub/arLabelingSubEBIA3C.c
                  src/main/cpp/ARToolKit/SRC/AR/arLabelingSub/arLabelingSubEBIC.c
                  src/main/cpp/ARToolKit/SRC/AR/arLabelingSub/arLabelingSubEBICY.c
                  src/main/cpp/ARToolKit/SRC/AR/arLabelingSub/arLabelingSubEBIYC.c
                  src/main/cpp/ARToolKit/SRC/AR/arLabelingSub/arLabelingSubEBR3C.c
                  src/main/cpp/ARToolKit/SRC/AR/arLabelingSub/arLabelingSubEBR3C565.c
                  src/main/cpp/ARToolKit/SRC/AR/arLabelingSub/arLabelingSubEBR3CA.c
                  src/main/cpp/ARToolKit/SRC/AR/arLabelingSub/arLabelingSubEBR3CA4444.c
                  src/main/cpp/ARToolKit/SRC/AR/arLabelingSub/arLabelingSubEBR3CA5551.c
                  src/main/cpp/ARToolKit/SRC/AR/arLabelingSub/arLabelingSubEBRA3C.c
                  src/main/cpp/ARToolKit/SRC/AR/arLabelingSub/arLabelingSubEBRC.c
                  src/main/cpp/ARToolKit/SRC/AR/arLabelingSub/arLabelingSubEBRCY.c
                  src/main/cpp/ARToolKit/SRC/AR/arLabelingSub/arLabelingSubEBRYC.c
                  src/main/cpp/ARToolKit/SRC/AR/arLabelingSub/arLabelingSubEBZ.c
                  src/main/cpp/ARToolKit/SRC/AR/arLabelingSub/arLabelingSubEWI3C.c
                  src/main/cpp/ARToolKit/SRC/AR/arLabelingSub/arLabelingSubEWI3C565.c
                  src/main/cpp/ARToolKit/SRC/AR/arLabelingSub/arLabelingSubEWI3CA.c
                  src/main/cpp/ARToolKit/SRC/AR/arLabelingSub/arLabelingSubEWI3CA4444.c
                  src/main/cpp/ARToolKit/SRC/AR/arLabelingSub/arLabelingSubEWI3CA5551.c
                  src/main/cpp/ARToolKit/SRC/AR/arLabelingSub/arLabelingSubEWIA3C.c
                  src/main/cpp/ARToolKit/SRC/AR/arLabelingSub/arLabelingSubEWIC.c
                  src/main/cpp/ARToolKit/SRC/AR/arLabelingSub/arLabelingSubEWICY.c
                  src/main/cpp/ARToolKit/SRC/AR/arLabelingSub/arLabelingSubEWIYC.c
                  src/main/cpp/ARToolKit/SRC/AR/arLabelingSub/arLabelingSubEWR3C.c
                  src/main/cpp/ARToolKit/SRC/AR/arLabelingSub/arLabelingSubEWR3C565.c
                  src/main/cpp/ARToolKit/SRC/AR/arLabelingSub/arLabelingSubEWR3CA.c
                  src/main/cpp/ARToolKit/SRC/AR/arLabelingSub/arLabelingSubEWR3CA4444.c
                  src/main/cpp/ARToolKit/SRC/AR/arLabelingSub/arLabelingSubEWR3CA5551.c
                  src/main/cpp/ARToolKit/SRC/AR/arLabelingSub/arLabelingSubEWRA3C.c
                  src/main/cpp/ARToolKit/SRC/AR/arLabelingSub/arLabelingSubEWRC.c
                  src/main/cpp/ARToolKit/SRC/AR/arLabelingSub/arLabelingSubEWRCY.c
                  src/main/cpp/ARToolKit/SRC/AR/arLabelingSub/arLabelingSubEWRYC.c
                  src/main/cpp/ARToolKit/SRC/AR/arLabelingSub/arLabelingSubEWZ.c
                  src/main/cpp/ARToolKit/SRC/AR/arPattAttach.c
                  src/main/cpp/ARToolKit/SRC/AR/arPattCreateHandle.c
                  src/main/cpp/ARToolKit/SRC/AR/arPattGetID.c
                  src/main/cpp/ARToolKit/SRC/AR/arPattLoad.c
                  src/main/cpp/ARToolKit/SRC/AR/arPattSave.c
                  src/main/cpp/ARToolKit/SRC/AR/arUtil.c
                  src/main/cpp/ARToolKit/SRC/AR/mAlloc.c
                  src/main/cpp/ARToolKit/SRC/AR/mAllocDup.c
                  src/main/cpp/ARToolKit/SRC/AR/mAllocInv.c
                  src/main/cpp/ARToolKit/SRC/AR/mAllocMul.c
                  src/main/cpp/ARToolKit/SRC/AR/mAllocTrans.c
                  src/main/cpp/ARToolKit/SRC/AR/mAllocUnit.c
                  src/main/cpp/ARToolKit/SRC/AR/mDet.c
                  src/main/cpp/ARToolKit/SRC/AR/mDisp.c
                  src/main/cpp/ARToolKit/SRC/AR/mDup.c
                  src/main/cpp/ARToolKit/SRC/AR/mFree.c
                  src/main/cpp/ARToolKit/SRC/AR/mInv.c
                  src/main/cpp/ARToolKit/SRC/AR/mMul.c
                  src/main/cpp/ARToolKit/SRC/AR/mPCA.c
                  src/main/cpp/ARToolKit/SRC/AR/mSelfInv.c
                  src/main/cpp/ARToolKit/SRC/AR/mTrans.c
                  src/main/cpp/ARToolKit/SRC/AR/mUnit.c
                  src/main/cpp/ARToolKit/SRC/AR/paramChangeSize.c
                  src/main/cpp/ARToolKit/SRC/AR/paramClear.c
                  src/main/cpp/ARToolKit/SRC/AR/paramDecomp.c
                  src/main/cpp/ARToolKit/SRC/AR/paramDisp.c
                  src/main/cpp/ARToolKit/SRC/AR/paramDistortion.c
                  src/main/cpp/ARToolKit/SRC/AR/paramFile.c
                  src/main/cpp/ARToolKit/SRC/AR/paramGetPerspective.c
                  src/main/cpp/ARToolKit/SRC/AR/paramLT.c
                  src/main/cpp/ARToolKit/SRC/AR/vAlloc.c
                  src/main/cpp/ARToolKit/SRC/AR/vDisp.c
                  src/main/cpp/ARToolKit/SRC/AR/vFree.c
                  src/main/cpp/ARToolKit/SRC/AR/vHouse.c
                  src/main/cpp/ARToolKit/SRC/AR/vInnerP.c
                  src/main/cpp/ARToolKit/SRC/AR/vTridiag.c
                  src/main/cpp/ARToolKit/SRC/AR2/coord.c
                  src/main/cpp/ARToolKit/SRC/AR2/featureMap.c
                  src/main/cpp/ARToolKit/SRC/AR2/featureSet.c
                  src/main/cpp/ARToolKit/SRC/AR2/handle.c
                  src/main/cpp/ARToolKit/SRC/AR2/imageSet.c
                  src/main/cpp/ARToolKit/SRC/AR2/jpeg.c
                  src/main/cpp/ARToolKit/SRC/AR2/marker.c
                  src/main/cpp/ARToolKit/SRC/AR2/matching.c
                  src/main/cpp/ARToolKit/SRC/AR2/matching2.c
                  src/main/cpp/ARToolKit/SRC/AR2/searchPoint.c
                  src/main/cpp/ARToolKit/SRC/AR2/selectTemplate.c
                  src/main/cpp/ARToolKit/SRC/AR2/surface.c
                  src/main/cpp/ARToolKit/SRC/AR2/template.c
                  src/main/cpp/ARToolKit/SRC/AR2/tracking.c
                  src/main/cpp/ARToolKit/SRC/AR2/tracking2d.c
                  src/main/cpp/ARToolKit/SRC/AR2/util.c
                  src/main/cpp/ARToolKit/SRC/ARICP/icpCalibStereo.c
                  src/main/cpp/ARToolKit/SRC/ARICP/icpCore.c
                  src/main/cpp/ARToolKit/SRC/ARICP/icpHandle.c
                  src/main/cpp/ARToolKit/SRC/ARICP/icpPoint.c
                  src/main/cpp/ARToolKit/SRC/ARICP/icpPointRobust.c
                  src/main/cpp/ARToolKit/SRC/ARICP/icpStereoHandle.c
                  src/main/cpp/ARToolKit/SRC/ARICP/icpStereoPoint.c
                  src/main/cpp/ARToolKit/SRC/ARICP/icpStereoPointRobust.c
                  src/main/cpp/ARToolKit/SRC/ARICP/icpUtil.c
                  src/main/cpp/ARToolKit/SRC/ARMulti/arMultiFreeConfig.c
                  src/main/cpp/ARToolKit/SRC/ARMulti/arMultiGetTransMat.c
                  src/main/cpp/ARToolKit/SRC/ARMulti/arMultiGetTransMatStereo.c
                  src/main/cpp/ARToolKit/SRC/ARMulti/arMultiReadConfigFile.c
                  src/main/cpp/ARToolKit/SRC/Eden/ALFramework.cpp
                  src/main/cpp/ARToolKit/SRC/Eden/CWaves.cpp
                  src/main/cpp/ARToolKit/SRC/Eden/EdenError.c
                  src/main/cpp/ARToolKit/SRC/Eden/EdenGLFont.c
                  src/main/cpp/ARToolKit/SRC/Eden/EdenMath.c
                  src/main/cpp/ARToolKit/SRC/Eden/EdenMessage.c
                  src/main/cpp/ARToolKit/SRC/Eden/EdenSound.c
                  src/main/cpp/ARToolKit/SRC/Eden/EdenSurfaces.c
                  src/main/cpp/ARToolKit/SRC/Eden/EdenTime.c
                  src/main/cpp/ARToolKit/SRC/Eden/EdenUtil.c
                  src/main/cpp/ARToolKit/SRC/Eden/glm.c
                  src/main/cpp/ARToolKit/SRC/Eden/gluttext/glut_8x13.c
                  src/main/cpp/ARToolKit/SRC/Eden/gluttext/glut_9x15.c
                  src/main/cpp/ARToolKit/SRC/Eden/gluttext/glut_bitmap.c
                  src/main/cpp/ARToolKit/SRC/Eden/gluttext/glut_bwidth.c
                  src/main/cpp/ARToolKit/SRC/Eden/gluttext/glut_hel10.c
                  src/main/cpp/ARToolKit/SRC/Eden/gluttext/glut_hel12.c
                  src/main/cpp/ARToolKit/SRC/Eden/gluttext/glut_hel18.c
                  src/main/cpp/ARToolKit/SRC/Eden/gluttext/glut_mroman.c
                  src/main/cpp/ARToolKit/SRC/Eden/gluttext/glut_roman.c
                  src/main/cpp/ARToolKit/SRC/Eden/gluttext/glut_stroke.c
                  src/main/cpp/ARToolKit/SRC/Eden/gluttext/glut_swidth.c
                  src/main/cpp/ARToolKit/SRC/Eden/gluttext/glut_tr10.c
                  src/main/cpp/ARToolKit/SRC/Eden/gluttext/glut_tr24.c
                  src/main/cpp/ARToolKit/SRC/Eden/readtex.c
                  src/main/cpp/ARToolKit/SRC/Util/profile.c
                  src/main/cpp/ARToolKit/SRC/Util/thread_sub.c
                  src/main/cpp/ARToolKit/SRC/Util/thread_sub_winrt.cpp
                  src/main/cpp/ARToolKit/SRC/Video/video.c
                  src/main/cpp/ARToolKit/SRC/Video/video2.c
                  src/main/cpp/ARToolKit/SRC/Video/videoAspectRatio.c
                  src/main/cpp/ARToolKit/SRC/Video/videoSaveImage.c
                  src/main/cpp/ARToolKit/SRC/VideoAndroid/cparamSearch.c
                  src/main/cpp/ARToolKit/SRC/VideoAndroid/nxjson.c
                  src/main/cpp/ARToolKit/SRC/VideoAndroid/sqlite3.c
                  src/main/cpp/ARToolKit/SRC/VideoAndroid/videoAndroid.c
                  src/main/cpp/ARToolKit/SRC/VideoDummy/videoDummy.c
                  src/main/cpp/ARToolKit/SRC/VideoImage/videoImage.c
                  src/main/cpp/ARToolKit/SRC/Gl/glStateCache.c
                  src/main/cpp/ARToolKit/SRC/Gl/gsub_es.c
                  src/main/cpp/ARToolKit/SRC/Gl/gsub_mtx.c
                  src/main/cpp/ARToolKit/SRC/ARosg/arosg.cpp
                  src/main/cpp/ARToolKit/SRC/KPM/FreakMatcher/detectors/gaussian_scale_space_pyramid.cpp
                  src/main/cpp/ARToolKit/SRC/KPM/FreakMatcher/detectors/gradients.cpp
                  src/main/cpp/ARToolKit/SRC/KPM/FreakMatcher/detectors/harris.cpp
                  src/main/cpp/ARToolKit/SRC/KPM/FreakMatcher/detectors/orientation_assignment.cpp
                  src/main/cpp/ARToolKit/SRC/KPM/FreakMatcher/detectors/pyramid.cpp
                  src/main/cpp/ARToolKit/SRC/KPM/FreakMatcher/detectors/DoG_scale_invariant_detector.cpp
                  src/main/cpp/ARToolKit/SRC/KPM/FreakMatcher/facade/visual_database_facade.cpp
                  src/main/cpp/ARToolKit/SRC/KPM/FreakMatcher/framework/date_time.cpp
                  src/main/cpp/ARToolKit/SRC/KPM/FreakMatcher/framework/image.cpp
                  src/main/cpp/ARToolKit/SRC/KPM/FreakMatcher/framework/logger.cpp
                  src/main/cpp/ARToolKit/SRC/KPM/FreakMatcher/framework/timers.cpp
                  src/main/cpp/ARToolKit/SRC/KPM/FreakMatcher/unsupported/test/mpreal/dlmalloc.c
                  src/main/cpp/ARToolKit/SRC/KPM/FreakMatcher/matchers/feature_store_io.cpp
                  src/main/cpp/ARToolKit/SRC/KPM/FreakMatcher/matchers/freak.cpp
                  src/main/cpp/ARToolKit/SRC/KPM/FreakMatcher/matchers/hough_similarity_voting.cpp
                  src/main/cpp/ARToolKit/SRC/KPM/FreakMatcher/math/math_io.cpp
                  src/main/cpp/ARToolKit/SRC/KPM/kpmRefDataSet.cpp
                  src/main/cpp/ARToolKit/SRC/KPM/kpmMatching.cpp
                  src/main/cpp/ARToolKit/SRC/KPM/kpmFopen.c
                  src/main/cpp/ARToolKit/SRC/KPM/kpmHandle.cpp
                  src/main/cpp/ARToolKit/SRC/KPM/kpmResult.cpp
                  src/main/cpp/ARToolKit/SRC/KPM/kpmUtil.cpp
                  src/main/cpp/ARToolKit/SRC/ARWrapper/AndroidFeatures.cpp
                  src/main/cpp/ARToolKit/SRC/ARWrapper/ARPattern.cpp
                  src/main/cpp/ARToolKit/SRC/ARWrapper/ARMarker.cpp
                  src/main/cpp/ARToolKit/SRC/ARWrapper/ARMarkerMulti.cpp
                  src/main/cpp/ARToolKit/SRC/ARWrapper/ColorConversion.cpp
                  src/main/cpp/ARToolKit/SRC/ARWrapper/ARToolKitVideoSource.cpp
                  src/main/cpp/ARToolKit/SRC/ARWrapper/AndroidVideoSource.cpp
                  src/main/cpp/ARToolKit/SRC/ARWrapper/VideoSource.cpp

             )

# 预编译库 静态库
add_library( cpu-features
             STATIC
             ${ANDROID_NDK}/sources/android/cpufeatures/cpu-features.c)
add_library( app-glue
             STATIC
             ${ANDROID_NDK}/sources/android/native_app_glue/android_native_app_glue.c)

add_library(libc++_shared SHARED IMPORTED)
set_target_properties(libc++_shared PROPERTIES IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/src/main/jniLibs/${ANDROID_ABI}/libc++_shared.so)

add_library(lib-curl STATIC IMPORTED)
set_target_properties(lib-curl PROPERTIES IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/src/main/jniLibs/${ANDROID_ABI}/libcurl.a)

add_library(lib-jpeg STATIC IMPORTED)
set_target_properties(lib-jpeg PROPERTIES IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/src/main/jniLibs/${ANDROID_ABI}/libjpeg.a)

# OSG库 动态方式加载，静态库
add_library(osgdb_osg SHARED IMPORTED)
set_target_properties(osgdb_osg PROPERTIES IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/src/main/jniLibs/${ANDROID_ABI}/static/libosgdb_osg.a)

add_library(osgdb_ive SHARED IMPORTED)
set_target_properties(osgdb_ive PROPERTIES IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/src/main/jniLibs/${ANDROID_ABI}/static/libosgdb_ive.a)

add_library(osgdb_jpeg SHARED IMPORTED)
set_target_properties(osgdb_jpeg PROPERTIES IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/src/main/jniLibs/${ANDROID_ABI}/static/libosgdb_jpeg.a)

add_library(jpeg SHARED IMPORTED)
set_target_properties(jpeg PROPERTIES IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/src/main/jniLibs/${ANDROID_ABI}/static/libjpeg.a)

add_library(osgdb_gif SHARED IMPORTED)
set_target_properties(osgdb_gif PROPERTIES IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/src/main/jniLibs/${ANDROID_ABI}/static/libosgdb_gif.a)

add_library(gif SHARED IMPORTED)
set_target_properties(gif PROPERTIES IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/src/main/jniLibs/${ANDROID_ABI}/static/libgif.a)

add_library(osgdb_tiff SHARED IMPORTED)
set_target_properties(osgdb_tiff PROPERTIES IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/src/main/jniLibs/${ANDROID_ABI}/static/libosgdb_tiff.a)

add_library(tiff SHARED IMPORTED)
set_target_properties(tiff PROPERTIES IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/src/main/jniLibs/${ANDROID_ABI}/static/libtiff.a)

add_library(osgdb_bmp SHARED IMPORTED)
set_target_properties(osgdb_bmp PROPERTIES IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/src/main/jniLibs/${ANDROID_ABI}/static/libosgdb_bmp.a)

add_library(osgdb_png SHARED IMPORTED)
set_target_properties(osgdb_png PROPERTIES IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/src/main/jniLibs/${ANDROID_ABI}/static/libosgdb_png.a)

add_library(png SHARED IMPORTED)
set_target_properties(png PROPERTIES IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/src/main/jniLibs/${ANDROID_ABI}/static/libpng.a)

add_library(osgdb_tga SHARED IMPORTED)
set_target_properties(osgdb_tga PROPERTIES IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/src/main/jniLibs/${ANDROID_ABI}/static/libosgdb_tga.a)

add_library(osgdb_freetype SHARED IMPORTED)
set_target_properties(osgdb_freetype PROPERTIES IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/src/main/jniLibs/${ANDROID_ABI}/static/libosgdb_freetype.a)

add_library(ft2 SHARED IMPORTED)
set_target_properties(ft2 PROPERTIES IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/src/main/jniLibs/${ANDROID_ABI}/static/libft2.a)

add_library(osgAnimation SHARED IMPORTED)
set_target_properties(osgAnimation PROPERTIES IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/src/main/jniLibs/${ANDROID_ABI}/static/libosgAnimation.a)

add_library(osgFX SHARED IMPORTED)
set_target_properties(osgFX PROPERTIES IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/src/main/jniLibs/${ANDROID_ABI}/static/libosgFX.a)

add_library(osgParticle SHARED IMPORTED)
set_target_properties(osgParticle PROPERTIES IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/src/main/jniLibs/${ANDROID_ABI}/static/libosgParticle.a)

add_library(osgPresentation SHARED IMPORTED)
set_target_properties(osgPresentation PROPERTIES IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/src/main/jniLibs/${ANDROID_ABI}/static/libosgPresentation.a)

add_library(osgShadow SHARED IMPORTED)
set_target_properties(osgShadow PROPERTIES IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/src/main/jniLibs/${ANDROID_ABI}/static/libosgShadow.a)

add_library(osgSim SHARED IMPORTED)
set_target_properties(osgSim PROPERTIES IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/src/main/jniLibs/${ANDROID_ABI}/static/libosgSim.a)

add_library(osgTerrain SHARED IMPORTED)
set_target_properties(osgTerrain PROPERTIES IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/src/main/jniLibs/${ANDROID_ABI}/static/libosgTerrain.a)

add_library(osgText SHARED IMPORTED)
set_target_properties(osgText PROPERTIES IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/src/main/jniLibs/${ANDROID_ABI}/static/libosgText.a)

add_library(osgVolume SHARED IMPORTED)
set_target_properties(osgVolume PROPERTIES IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/src/main/jniLibs/${ANDROID_ABI}/static/libosgVolume.a)

add_library(osgWidget SHARED IMPORTED)
set_target_properties(osgWidget PROPERTIES IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/src/main/jniLibs/${ANDROID_ABI}/static/libosgWidget.a)

add_library(osgViewer SHARED IMPORTED)
set_target_properties(osgViewer PROPERTIES IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/src/main/jniLibs/${ANDROID_ABI}/static/libosgViewer.a)

add_library(osgGA SHARED IMPORTED)
set_target_properties(osgGA PROPERTIES IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/src/main/jniLibs/${ANDROID_ABI}/static/libosgGA.a)

add_library(osgDB SHARED IMPORTED)
set_target_properties(osgDB PROPERTIES IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/src/main/jniLibs/${ANDROID_ABI}/static/libosgDB.a)

add_library(osgUtil SHARED IMPORTED)
set_target_properties(osgUtil PROPERTIES IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/src/main/jniLibs/${ANDROID_ABI}/static/libosgUtil.a)

add_library(osgdb_deprecated_osg SHARED IMPORTED)
set_target_properties(osgdb_deprecated_osg PROPERTIES IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/src/main/jniLibs/${ANDROID_ABI}/static/libosgdb_deprecated_osg.a)

add_library(osgdb_deprecated_osganimation SHARED IMPORTED)
set_target_properties(osgdb_deprecated_osganimation PROPERTIES IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/src/main/jniLibs/${ANDROID_ABI}/static/libosgdb_deprecated_osganimation.a)

add_library(osgdb_deprecated_osgfx SHARED IMPORTED)
set_target_properties(osgdb_deprecated_osgfx PROPERTIES IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/src/main/jniLibs/${ANDROID_ABI}/static/libosgdb_deprecated_osgfx.a)

add_library(osgdb_deprecated_osgparticle SHARED IMPORTED)
set_target_properties(osgdb_deprecated_osgparticle PROPERTIES IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/src/main/jniLibs/${ANDROID_ABI}/static/libosgdb_deprecated_osgparticle.a)

add_library(osgdb_deprecated_osgshadow SHARED IMPORTED)
set_target_properties(osgdb_deprecated_osgshadow PROPERTIES IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/src/main/jniLibs/${ANDROID_ABI}/static/libosgdb_deprecated_osgshadow.a)

add_library(osgdb_deprecated_osgsim SHARED IMPORTED)
set_target_properties(osgdb_deprecated_osgsim PROPERTIES IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/src/main/jniLibs/${ANDROID_ABI}/static/libosgdb_deprecated_osgsim.a)

add_library(osgdb_deprecated_osgterrain SHARED IMPORTED)
set_target_properties(osgdb_deprecated_osgterrain PROPERTIES IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/src/main/jniLibs/${ANDROID_ABI}/static/libosgdb_deprecated_osgterrain.a)

add_library(osgdb_deprecated_osgtext SHARED IMPORTED)
set_target_properties(osgdb_deprecated_osgtext PROPERTIES IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/src/main/jniLibs/${ANDROID_ABI}/static/libosgdb_deprecated_osgtext.a)

add_library(osgdb_deprecated_osgviewer SHARED IMPORTED)
set_target_properties(osgdb_deprecated_osgviewer PROPERTIES IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/src/main/jniLibs/${ANDROID_ABI}/static/libosgdb_deprecated_osgviewer.a)

add_library(osgdb_deprecated_osgvolume SHARED IMPORTED)
set_target_properties(osgdb_deprecated_osgvolume PROPERTIES IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/src/main/jniLibs/${ANDROID_ABI}/static/libosgdb_deprecated_osgvolume.a)

add_library(osgdb_deprecated_osgwidget SHARED IMPORTED)
set_target_properties(osgdb_deprecated_osgwidget PROPERTIES IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/src/main/jniLibs/${ANDROID_ABI}/static/libosgdb_deprecated_osgwidget.a)

add_library(osg SHARED IMPORTED)
set_target_properties(osg PROPERTIES IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/src/main/jniLibs/${ANDROID_ABI}/static/libosg.a)

add_library(OpenThreads SHARED IMPORTED)
set_target_properties(OpenThreads PROPERTIES IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/src/main/jniLibs/${ANDROID_ABI}/static/libOpenThreads.a)

add_library(util SHARED IMPORTED)
set_target_properties(util PROPERTIES IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/src/main/jniLibs/${ANDROID_ABI}/static/libutil.a)

# curl库
add_library(so_libcurl SHARED IMPORTED)
set_target_properties(so_libcurl PROPERTIES IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/src/main/jniLibs/${ANDROID_ABI}/libcurl.a)

# 动态方式加载
add_library(so_camera22 SHARED IMPORTED)
set_target_properties(so_camera22 PROPERTIES IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/src/main/jniLibs/${ANDROID_ABI}/libnative_camera_r2.2.0.so)

add_library(so_camera23 SHARED IMPORTED)
set_target_properties(so_camera23 PROPERTIES IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/src/main/jniLibs/${ANDROID_ABI}/libnative_camera_r2.3.3.so)

add_library(so_camera30 SHARED IMPORTED)
set_target_properties(so_camera30 PROPERTIES IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/src/main/jniLibs/${ANDROID_ABI}/libnative_camera_r3.0.1.so)

add_library(so_camera40 SHARED IMPORTED)
set_target_properties(so_camera40 PROPERTIES IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/src/main/jniLibs/${ANDROID_ABI}/libnative_camera_r4.0.0.so)

add_library(so_camera43 SHARED IMPORTED)
set_target_properties(so_camera43 PROPERTIES IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/src/main/jniLibs/${ANDROID_ABI}/libnative_camera_r4.0.3.so)

add_library(so_camera41 SHARED IMPORTED)
set_target_properties(so_camera41 PROPERTIES IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/src/main/jniLibs/${ANDROID_ABI}/libnative_camera_r4.1.1.so)

#C++日志
find_library( log-lib log )

target_link_libraries( # Specifies the target library.  ${OpenCV_LIBS}/so_lib_opencv_shared/so_libARNativeOSGNative
                       ARWrapper

                        libc++_shared
                        lib-curl
                        log
                        m
                        z
                        android
                        app-glue
                        cpu-features
                        -lGLESv2
                        -lGLESv1_CM
                        osgdb_osg osgdb_ive osgdb_jpeg jpeg osgdb_gif gif osgdb_tiff tiff osgdb_bmp osgdb_png png osgdb_tga
                        osgdb_freetype ft2 osgAnimation osgFX osgParticle osgPresentation osgShadow osgSim osgTerrain osgText
                        osgVolume osgWidget osgViewer osgGA osgDB osgUtil osgdb_deprecated_osg osgdb_deprecated_osganimation
                        osgdb_deprecated_osgfx  osgdb_deprecated_osgparticle osgdb_deprecated_osgshadow osgdb_deprecated_osgsim
                        osgdb_deprecated_osgterrain osgdb_deprecated_osgtext osgdb_deprecated_osgviewer osgdb_deprecated_osgvolume
                        osgdb_deprecated_osgwidget osg OpenThreads util
                        #so_camera22 so_camera23 so_camera30 so_camera40 so_camera41 so_camera43

                        )
