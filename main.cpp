// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include <k4a/k4a.h>
#include <math.h>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <map>
#include <k4arecord/playback.h>
#include <cstdlib>
#include <vector>
#include <experimental/filesystem>
#include <apriltag.h>
#include <apriltag_pose.h>
#include <omp.h>
//#include "kinfu.hpp"
//#include "bilateral_filter.h"

#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

namespace fs = std::experimental::filesystem;

typedef std::pair<int,int> point_pair;

#define INVALID INT32_MIN

typedef struct
{
    const char* filename;
    k4a_playback_t handle;
    k4a_record_configuration_t record_config;
    k4a_capture_t capture;
    k4a_calibration_t k4a_calibration;
    k4a_transformation_t k4a_transformation;
} recording_t;

typedef struct{
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;

} opencv_camera_info;


typedef struct{
    cv::Mat descriptor, image;
    std::vector<cv::KeyPoint> kp;
} feature_info;

typedef struct _pinhole_t
{
    float px;
    float py;
    float fx;
    float fy;

    int width;
    int height;
} pinhole_t;

typedef struct _coordinate_t
{
    int x;
    int y;
    float weight[4];
} coordinate_t;

typedef struct{
    k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    k4a_image_t k4a_color_image = NULL;
    k4a_image_t depth_image = NULL;
    k4a_image_t xy_table = NULL;
    k4a_image_t point_cloud = NULL;
    cv::Mat color_image_cv;
    cv::Mat undistorted_depth_image;
    cv::Mat depth_in_color;
    cv::Mat depth_in_color_v2;
    cv::Mat undistorted_color_image;
    opencv_camera_info camInfo_cv;
    int point_count;
    int size_;
} frame;
    

typedef enum
{
    INTERPOLATION_NEARESTNEIGHBOR, /**< Nearest neighbor interpolation */
    INTERPOLATION_BILINEAR,        /**< Bilinear interpolation */
    INTERPOLATION_BILINEAR_DEPTH   /**< Bilinear interpolation with invalidation when neighbor contain invalid
                                                 data with value 0 */
} interpolation_t;

static void init_opencv_params(k4a_calibration_t& calibration, frame& cFrame){

   
    auto calib = calibration.color_camera_calibration;
    cFrame.camInfo_cv.cameraMatrix = cv::Mat::zeros(3,3, CV_32FC1); // init empty camera matrix
    cFrame.camInfo_cv.distCoeffs = cv::Mat::zeros(1,8, CV_32FC1); // init empty distortion coefficients vector

    // set the camera matrix
    cFrame.camInfo_cv.cameraMatrix.ptr<float>(0)[0] = calib.intrinsics.parameters.param.fx; // set fx
    cFrame.camInfo_cv.cameraMatrix.ptr<float>(1)[1] = calib.intrinsics.parameters.param.fy; // set fy
    cFrame.camInfo_cv.cameraMatrix.ptr<float>(0)[2] = calib.intrinsics.parameters.param.cx; // set cx
    cFrame.camInfo_cv.cameraMatrix.ptr<float>(1)[2] = calib.intrinsics.parameters.param.cy; // set cy
    cFrame.camInfo_cv.cameraMatrix.ptr<float>(2)[2] = 1; // 

    // set the undistortion parameters (k1, k2 , p1, p2, k3, k4, k5, k6)
    cFrame.camInfo_cv.distCoeffs.ptr<float>(0)[0] = calib.intrinsics.parameters.param.k1;
    cFrame.camInfo_cv.distCoeffs.ptr<float>(0)[1] = calib.intrinsics.parameters.param.k2;
    cFrame.camInfo_cv.distCoeffs.ptr<float>(0)[2] = calib.intrinsics.parameters.param.p1;
    cFrame.camInfo_cv.distCoeffs.ptr<float>(0)[3] = calib.intrinsics.parameters.param.p2;
    cFrame.camInfo_cv.distCoeffs.ptr<float>(0)[4] = calib.intrinsics.parameters.param.k3;
    cFrame.camInfo_cv.distCoeffs.ptr<float>(0)[5] = calib.intrinsics.parameters.param.k4;
    cFrame.camInfo_cv.distCoeffs.ptr<float>(0)[6] = calib.intrinsics.parameters.param.k5;
    cFrame.camInfo_cv.distCoeffs.ptr<float>(0)[7] = calib.intrinsics.parameters.param.k6;

}

static void k4a_color_to_opencv(frame& cFrame){
    int width = k4a_image_get_width_pixels(cFrame.k4a_color_image);
    int height = k4a_image_get_height_pixels(cFrame.k4a_color_image);
    int stride = k4a_image_get_stride_bytes(cFrame.k4a_color_image);
    const uint8_t* color_image_data = reinterpret_cast<const uint8_t*>(k4a_image_get_buffer(cFrame.k4a_color_image));
    const size_t color_size = k4a_image_get_size(cFrame.k4a_color_image);
    
    //std::vector<uint8_t> ColorImage;
    //ColorImage.resize(color_size);
    //memcpy(ColorImage.data(),color_image_data,color_size);
    //cv::Mat image(height,width, CV_8UC4, (void*)ColorImage.data(),cv::Mat::AUTO_STEP);

    cv::Mat image(height,width, CV_8UC4, (void*)color_image_data,cv::Mat::AUTO_STEP);
    cFrame.color_image_cv = image.clone();
}

static void ocv_undistort_color_image(frame& cFrame){
    cv::undistort(cFrame.color_image_cv, cFrame.undistorted_color_image, cFrame.camInfo_cv.cameraMatrix, cFrame.camInfo_cv.distCoeffs);
}

static void write_calibration(k4a_calibration_t& calibration, const std::string& filename){
    std::ofstream ofs;
    ofs.open(filename);
    if(ofs){
        //cout << "Color camera:" << endl;
        auto calib = calibration.color_camera_calibration;
        ofs << calib.resolution_width << "\n";
        ofs << calib.resolution_height << "\n";
        ofs << calib.intrinsics.parameters.param.fx << "\n"; 
        ofs << calib.intrinsics.parameters.param.fy << "\n";
        ofs << calib.intrinsics.parameters.param.cx << "\n";
        ofs << calib.intrinsics.parameters.param.cy << "\n"; 
    }
    ofs.close();
}

static void print_calibration(k4a_calibration_t& calibration)
    {
        {
            cout << "Depth camera:" << endl;
            auto calib = calibration.depth_camera_calibration;

            cout << "resolution width: " << calib.resolution_width << endl;
            cout << "resolution height: " << calib.resolution_height << endl;
            cout << "principal point x: " << calib.intrinsics.parameters.param.cx << endl;
            cout << "principal point y: " << calib.intrinsics.parameters.param.cy << endl;
            cout << "focal length x: " << calib.intrinsics.parameters.param.fx << endl;
            cout << "focal length y: " << calib.intrinsics.parameters.param.fy << endl;
            cout << "radial distortion coefficients:" << endl;
            cout << "k1: " << calib.intrinsics.parameters.param.k1 << endl;
            cout << "k2: " << calib.intrinsics.parameters.param.k2 << endl;
            cout << "k3: " << calib.intrinsics.parameters.param.k3 << endl;
            cout << "k4: " << calib.intrinsics.parameters.param.k4 << endl;
            cout << "k5: " << calib.intrinsics.parameters.param.k5 << endl;
            cout << "k6: " << calib.intrinsics.parameters.param.k6 << endl;
            cout << "center of distortion in Z=1 plane, x: " << calib.intrinsics.parameters.param.codx << endl;
            cout << "center of distortion in Z=1 plane, y: " << calib.intrinsics.parameters.param.cody << endl;
            cout << "tangential distortion coefficient x: " << calib.intrinsics.parameters.param.p1 << endl;
            cout << "tangential distortion coefficient y: " << calib.intrinsics.parameters.param.p2 << endl;
            cout << "metric radius: " << calib.intrinsics.parameters.param.metric_radius << endl;
        }

        {
            cout << "Color camera:" << endl;
            auto calib = calibration.color_camera_calibration;

            cout << "resolution width: " << calib.resolution_width << endl;
            cout << "resolution height: " << calib.resolution_height << endl;
            cout << "principal point x: " << calib.intrinsics.parameters.param.cx << endl;
            cout << "principal point y: " << calib.intrinsics.parameters.param.cy << endl;
            cout << "focal length x: " << calib.intrinsics.parameters.param.fx << endl;
            cout << "focal length y: " << calib.intrinsics.parameters.param.fy << endl;
            cout << "radial distortion coefficients:" << endl;
            cout << "k1: " << calib.intrinsics.parameters.param.k1 << endl;
            cout << "k2: " << calib.intrinsics.parameters.param.k2 << endl;
            cout << "k3: " << calib.intrinsics.parameters.param.k3 << endl;
            cout << "k4: " << calib.intrinsics.parameters.param.k4 << endl;
            cout << "k5: " << calib.intrinsics.parameters.param.k5 << endl;
            cout << "k6: " << calib.intrinsics.parameters.param.k6 << endl;
            cout << "center of distortion in Z=1 plane, x: " << calib.intrinsics.parameters.param.codx << endl;
            cout << "center of distortion in Z=1 plane, y: " << calib.intrinsics.parameters.param.cody << endl;
            cout << "tangential distortion coefficient x: " << calib.intrinsics.parameters.param.p1 << endl;
            cout << "tangential distortion coefficient y: " << calib.intrinsics.parameters.param.p2 << endl;
            cout << "metric radius: " << calib.intrinsics.parameters.param.metric_radius << endl;
        }

        auto extrinsics = calibration.extrinsics[K4A_CALIBRATION_TYPE_DEPTH][K4A_CALIBRATION_TYPE_COLOR];
        cout << "depth2color translation: (" << extrinsics.translation[0] << "," << extrinsics.translation[1] << "," << extrinsics.translation[2] << ")" << endl;
        cout << "depth2color rotation: |" << extrinsics.rotation[0] << "," << extrinsics.rotation[1] << "," << extrinsics.rotation[2] << "|" << endl;
        cout << "                      |" << extrinsics.rotation[3] << "," << extrinsics.rotation[4] << "," << extrinsics.rotation[5] << "|" << endl;
        cout << "                      |" << extrinsics.rotation[6] << "," << extrinsics.rotation[7] << "," << extrinsics.rotation[8] << "|" << endl;

    }





static frame process_capture(recording_t *file){
    //std::cout << "Processing the capture!!!" << std::endl;
    frame curr_frame;
    curr_frame.depth_image =  k4a_capture_get_depth_image(file->capture);
    //std::cout << "Got depth!!!" << std::endl;
    
    bool flag = true;
    while(flag){
        k4a_image_t temp_image = k4a_capture_get_color_image(file->capture);
        if( 0 == k4a_image_get_width_pixels(temp_image) ){
            k4a_stream_result_t stream_result = k4a_playback_get_next_capture(file->handle, &file->capture);
            if (stream_result == K4A_STREAM_RESULT_EOF)
            {
                printf("ERROR: Recording file is empty: %s\n", file->filename);
                //throw std::runtime_error("Recording file is empty || EOF");
            }
            else if (stream_result == K4A_STREAM_RESULT_FAILED)
            {
                printf("ERROR: Failed to read first capture from file: %s\n", file->filename);
                //throw std::runtime_error("Recording file is empty || FAILED");
              
            }
        }

        else{
            //std::cout << "Finally got a valid Image" << std::endl;
            flag = false;
            break;
        }
    }
    curr_frame.k4a_color_image =  k4a_capture_get_color_image(file->capture);
   
    init_opencv_params(file->k4a_calibration, curr_frame);
    //std::cout << "Created single frame" << std::endl;
    return curr_frame;
}



static void print_cmd(int argc, char* argv[]){

    for(int i=0; i< argc; i++){
        std::cout << "argv[" << i << "] = " << argv[i] << std::endl;
    }
}


static cv::Mat depth_to_opencv(const k4a_image_t &im)
{
    return cv::Mat(k4a_image_get_height_pixels(im),
                   k4a_image_get_width_pixels(im),
                   CV_16U,
                   (void *)k4a_image_get_buffer(im),
                   static_cast<size_t>(k4a_image_get_stride_bytes(im)));
}

static void validate_input_files(char* argv[], int argc, std::vector<std::string>& filenames, std::vector<int>& camID, std::vector<std::string>& rawNames)
{
    
    // this is to ensure that all the supplied files terminate with a '.mkv' extension
    for(int i{1}; i < argc; i++){
        
        std::string tmp = argv[i];
        size_t pos = tmp.rfind('.');
        size_t lastindex = tmp.find_last_of("."); 
        std::string rawname = tmp.substr(0, lastindex);

        if (pos == std::string::npos){
            std::cout << "'" << tmp << "' is not a valid file, ignoring..." << std::endl;
            continue;
        }
        
        std::string ext = tmp.substr(pos+1);
        if (ext == "mkv"){
            filenames.emplace_back(argv[i]);
            //std::cout << rawname[rawname.size()-2] << " -- " << rawname[rawname.size()-1] << std::endl; 
            std::string fileID = std::string(1,rawname[rawname.size()-2]) + std::string(1,rawname[rawname.size()-1]);
	    //std::cout << "eaw name: " << rawname << std::endl; 
	    rawNames.emplace_back(rawname);
	    //std::cout << fileID << std::endl;
            camID.emplace_back(stoi(fileID));
           // std::cout << "file ID: " << fileID << std::endl;
        }
        else{
            std::cout << "'" << tmp << "' is not a valid file, ignoring..." << std::endl;
            continue;
        }
        
    }

}

template<typename T>
void print_vector(const std::vector<T>& vector,const std::string& delim)
{
    for(const auto& item : vector){
        std::cout << item << delim;
    }
    std::cout << std::endl;
}



void k4a_undistort_depth_image(frame& cFrame, k4a_calibration_t calibration, k4a_transformation_t transformation);
void k4a_get_depth_in_color_frame(frame& cFrame, k4a_transformation_t transformation, k4a_image_t undistorted_depth );
void remap(const k4a_image_t src, const k4a_image_t lut, k4a_image_t dst, interpolation_t type);
void k4a_get_depth_in_color_frame(frame& cFrame, k4a_transformation_t transformation);

int main(int argc, char *argv[])
{

    int returnCode = 1;
    k4a_device_t device = NULL;
    const int32_t TIMEOUT_IN_MS = 1000;
    k4a_capture_t capture = NULL;
    k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    k4a_image_t depth_image = NULL;
   
    k4a_result_t result = K4A_RESULT_SUCCEEDED;

   

    if (argc < 2)
    {
        std::cout << "Usage: " << argv[0] << "rec1.mkv rec2.mkv...." << std::endl;
        returnCode = 2;
        return -1;
    }
    std::vector<std::string> filenames, output_filenames;
    std::vector<int>camIDs;
    std::vector<char>stars(100,'=');

    std::vector<std::string> rawFnames;

    validate_input_files(argv, argc,filenames, camIDs, rawFnames);
    //print_vector(stars, "");
    //std::cout << "Writing undistorted RGB images for : \n";
    //print_vector(filenames,"\n");
    //print_vector(stars,"");
    size_t file_count = static_cast<size_t>(argc-1); // -2 because of output filename and record flag
    //std::cout << "file count = " << file_count << std::endl;
    
    bool verbose = false;
    //return 0;
    if(1) { // read from MKV

        recording_t *files = reinterpret_cast<recording_t*>(malloc(sizeof(recording_t) * file_count));
        if(files == NULL){
            printf("Failed to allocate memory for playback : %zu bytes \n", sizeof(recording_t)*file_count);
            return -1;
        }

        memset(files, 0, sizeof(recording_t)*file_count);
        //std::cout << "file count = " << file_count << std::endl;
        // Open each recording file and validate they were recorded in master/subordinate mode.
        #pragma omp parallel for
        for (size_t i = 0; i < file_count; i++) // read mkv files into memory {recording_t struct - files}
        {

            //printf("%-32s  %12s  %12s  %12s\n", "Source file", "COLOR", "DEPTH", "IR");
            //printf("============================================================================================\n");

            files[i].filename = argv[i + 1];
            //std::cout << "Processing file : " << files[i].filename << std::endl;


            result = k4a_playback_open(files[i].filename, &files[i].handle);
            if (result != K4A_RESULT_SUCCEEDED)
            {
                printf("Failed to open file: %s\n", files[i].filename);
                continue;
               // break; -- No breaks because of openmp
            }
           // std::cout << "Opened recording successfully" << std::endl;

            result = k4a_playback_get_record_configuration(files[i].handle, &files[i].record_config);
            if (result != K4A_RESULT_SUCCEEDED)
            {
                printf("Failed to get record configuration for file: %s\n", files[i].filename);
                continue;

                // break; 
            }
            result = k4a_playback_get_calibration(files[i].handle, &files[i].k4a_calibration);
            if (result != K4A_RESULT_SUCCEEDED)
            {
                printf("Failed to get record configuration for file: %s\n", files[i].filename);
                continue;

                //break;
            }

        
            files[i].k4a_transformation = k4a_transformation_create(&files[i].k4a_calibration);
            result = k4a_playback_set_color_conversion(files[i].handle, K4A_IMAGE_FORMAT_COLOR_BGRA32);
            //print_calibration(files[i].k4a_calibration);
            //return -1;
            k4a_stream_result_t stream_result;
            
            //stream_result = k4a_playback_get_next_capture(playback,&capture); // old line

           
	   
	    result = k4a_playback_seek_timestamp(files[i].handle, 0, K4A_PLAYBACK_SEEK_END);
	    if(result != K4A_RESULT_SUCCEEDED)
	    {
		    std::cout << "Failed to seek to the end of the recording!" << std::endl;
            continue;

		    //break;
	    }
            stream_result = k4a_playback_get_previous_capture(files[i].handle, &files[i].capture); //  get last frame
            
            
            if (stream_result == K4A_STREAM_RESULT_EOF)
            {
                // End of file reached
                continue;

               // break;
            }
 
            
	    if (K4A_STREAM_RESULT_SUCCEEDED == stream_result)
            {
                recording_t* file = &files[i];
            
                frame cFrame = process_capture(file);
                write_calibration(file->k4a_calibration, rawFnames[i]+ "_intrinsics.txt");
                
                k4a_color_to_opencv(cFrame);
                ocv_undistort_color_image(cFrame);
                //k4a_undistort_depth_image(cFrame, file->k4a_calibration,file->k4a_transformation);
                k4a_get_depth_in_color_frame(cFrame, file->k4a_transformation);
                //bilateral_filter<<,>>();
                if(camIDs[i] == 14 || camIDs[i] == 17){ // rotate cameras 14 and 17
                    //print_vector(stars, "");
                   // cv::rotate(cFrame.undistorted_color_image,cFrame.undistorted_color_image, cv::ROTATE_180);
                    //cv::rotate(cFrame.color_image_cv,cFrame.color_image_cv, cv::ROTATE_180);
                    //print_vector(stars, "");
                }
                cv::imwrite(rawFnames[i]+ ".jpg", cFrame.undistorted_color_image);
                //cv::imwrite(rawFnames[i]+ "_depth.png", cFrame.depth_in_color); //  this depth is first undistorted then transformed in color space
                cv::imwrite(rawFnames[i]+ "_depth.png", cFrame.depth_in_color_v2); // Transformation occurs internally (CORRECT)
            
                
                k4a_image_release(cFrame.depth_image);
                k4a_image_release(cFrame.k4a_color_image);
                k4a_capture_release(file->capture);
            }
        }


            // close the files
            for (size_t i = 0; i < file_count; i++)
            {
                if (files[i].handle != NULL)
                {
                    k4a_playback_close(files[i].handle);
                    files[i].handle = NULL;
                }
            }
            free(files);
        }
        return 0;
        
}


void k4a_get_depth_in_color_frame(frame& cFrame, k4a_transformation_t transformation){

   
    int width = k4a_image_get_width_pixels(cFrame.k4a_color_image);
    int height = k4a_image_get_height_pixels(cFrame.k4a_color_image);

    k4a_image_t transformed_depth_image = NULL;
    if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16,
                                                 width,
                                                 height,
                                                 width*(int)sizeof(uint16_t),
                                                 &transformed_depth_image))
    {
        std::cout << "Failed to create transformed depth image" << std::endl;
    }

    if (K4A_RESULT_SUCCEEDED != k4a_transformation_depth_image_to_color_camera(transformation, cFrame.depth_image, transformed_depth_image))
    {
        std::cout << "Failed to compute transformed depth image" << std::endl;
    }
    //  undistorted depth image in color camera coordinate space
    cFrame.depth_in_color_v2 = depth_to_opencv(transformed_depth_image); 
}
