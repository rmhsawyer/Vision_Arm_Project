#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <ransac_geometry.h>
#include <vector>
#include <stdio.h>
#include <iostream>

using namespace std;
using namespace cv;

typedef struct intrinsic_param{
    int width;
    int height;
    int fx;
    int fy;
    int ppx;
    int ppy;
    void Set(int width_, int height_, int fx_, int fy_, int ppx_, int ppy_)
    {
        width = width_;
        height = height_;
        fx = fx_;
        fy = fy_;
        ppx = ppx_;
        ppy = ppy_;
    }
}intrinsic_param;


// Capture Example demonstrates how to
// capture depth and color video streams and render them to the screen
void convertDepthToColor(cv::Mat& depth, cv::Mat& color)
{
    const uint8_t nearColor[] = { 255, 0, 0 }, farColor[] = { 20, 40, 255 };
    auto rgbImage = color.ptr<uint8_t>();
    auto depthImage = depth.ptr<uint16_t>();
    auto width = color.cols;
    auto height = color.rows;
    // Produce a cumulative histogram of depth values
    int histogram[256 * 256] = { 1 };
    for (int i = 0; i < width * height; ++i)
    {
        if (auto d = depthImage[i]) ++histogram[d];
    }
    for (int i = 1; i < 256 * 256; i++)
    {
        histogram[i] += histogram[i - 1];
    }
    // Remap the cumulative histogram to the range 0..256
    for (int i = 1; i < 256 * 256; i++)
    {
        histogram[i] = (histogram[i] << 8) / histogram[256 * 256 - 1];
    }
    // Produce RGB image by using the histogram to interpolate between two colors
    auto rgb = rgbImage;
    for (int i = 0; i < width * height; i++)
    {
        if (uint16_t d = depthImage[i]) // For valid depth values (depth > 0)
        {
            auto t = histogram[d]; // Use the histogram entry (in the range of 0..256) to interpolate between nearColor and farColor
            *rgb++ = ((256 - t) * nearColor[0] + t * farColor[0]) >> 8;
            *rgb++ = ((256 - t) * nearColor[1] + t * farColor[1]) >> 8;
            *rgb++ = ((256 - t) * nearColor[2] + t * farColor[2]) >> 8;
        }
        else // Use black pixels for invalid values (depth == 0)
        {
            *rgb++ = 0;
            *rgb++ = 0;
            *rgb++ = 0;
        }                //show color image converted from depth
    }//end for
}
//
inline void _SPTransformFromZImageToZCamera(float one_divide_fx, float one_divide_fy, float u0, float v0, float vin1, float vin2, float vin3, float &vout1, float &vout2, float &vout3)
{
    vout1 = vin3*(vin1 - u0)*one_divide_fx;
    vout2 = vin3*(vin2 - v0)*one_divide_fy;
    vout3 = vin3;
}

void GetPointCloud(unsigned short *psDepth, int width, int height, intrinsic_param intrin, float *pointcloud)
{
    float one_divide_fx = 1.0f / intrin.fx;
    float one_divide_fy = 1.0f / intrin.fy;
    float v1, v2, v3;
    float _v1, _v2, _v3;

    for (size_t y = 0; y < height; y++)
    {
        for (size_t x = 0; x < width; x++)
        {
            int i = y * width + x;
            float d = (float)psDepth[i];

            if (0)//d < 1000.0f || d > 10000.0f)
            {
                pointcloud[3 * i] = 0.0f;
                pointcloud[3 * i + 1] = 0.0f;
                pointcloud[3 * i + 2] = 0.0f;
            }
            else
            {
                _SPTransformFromZImageToZCamera(one_divide_fx, one_divide_fy, intrin.ppx, intrin.ppy,
                    x, y, d, v1, v2, v3);
                pointcloud[3 * i] = 0.0001*v1;
                pointcloud[3 * i + 1] = 0.0001*v2;
                pointcloud[3 * i + 2] = 0.0001*v3;
            }
        }
    }
}

float _PI_ = 3.1415927;
void FilterPointCloudWithNormal(float *pointcloudIn, float *pointcloudOut, int &n, int width, int height)
{
    n = 0;
    for (size_t y = 0; y < height; y += 1)
    {
        for (size_t x = 0; x < width ; x += 1)
        {
            int ii = y * width + x;
            float d = pointcloudIn[3*ii + 2];
            
            if (d > 0.3 && d < 1.2) // too near or too far
            {
               // depthColor.data[3*ii + 1] = 255;
    
                pointcloudOut[3 * n] = pointcloudIn[3 * ii];
                pointcloudOut[3 * n + 1] = pointcloudIn[3 * ii + 1];
                pointcloudOut[3 * n + 2] = pointcloudIn[3 * ii + 2];
                n++;
            }
        }
    }
}

void Erosion(Mat src, Mat dst, int N)
{
    Mat element = getStructuringElement(MORPH_ELLIPSE, Size( 2*N + 1, 2*N + 1 ), Point(N, N));
    erode( src, dst, element );
}

void Dilate(Mat src, Mat dst, int N)
{
    Mat element = getStructuringElement(MORPH_ELLIPSE, Size( 2*N + 1, 2*N + 1 ), Point(N, N));
    dilate( src, dst, element );
}


int main()
{
    return 0;
}


// Init global data information
float RedCoordinate[3];
float GreenCoordinate[3];
float BlueCoordinate[3];
float Intrinsic[6];
int NumOfPoints[3];
float planeCenter[3];

vector<vector<float>> Red;
vector<vector<float>> Green;
vector<vector<float>> Blue;
vector<float> angle0;
vector<float> angle1;

vector<float> planeCenterLeft;
vector<float> planeCenterRight;
vector<float> planeAngleLeft;
vector<float> planeAngleRight;  

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// adjustable parameters for box area detection:

// if the white rectangle is flashing when the box is put near the screen center area, upper the area upper bound;
double areaUpperBound = 20000.0; // Range: > 15000, reference value: 20000

// if the white rectangle is flashing when the box is put near the screen edge area, lower the area upper bound;
double areaLowerBound = 5000.0;// Range: < 8000, reference value: 5000

// When iPhone is in the box, if the white rectangle disappears, lower the ratio limit;
// if fake area is too easy to be recognized as our box area, upper the ratio limit;
double ratioLimit = 0.75; // Range: 0.6 ~ 0.8, reference value: 0.75
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//
extern "C"
{
    void Run()
    {
        int width = 640;
        int height = 480;

        rs2::pipeline p;
        rs2::config cfg;
        cfg.enable_stream(RS2_STREAM_DEPTH, width, height, RS2_FORMAT_Z16, 30);
        cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);

        rs2::align align_to(RS2_STREAM_DEPTH);

        rs2::pipeline_profile selection = p.start(cfg);
        auto depth_stream = selection.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
        auto intrin_ = depth_stream.get_intrinsics();
        intrinsic_param intrin;
        intrin.Set(intrin_.width, intrin_.height, intrin_.fx, intrin_.fy, intrin_.ppx, intrin_.ppy);

        //Get intrinsic information
        Intrinsic[0] = intrin_.width;
        Intrinsic[1] = intrin_.height;
        Intrinsic[2] = intrin_.fx;
        Intrinsic[3] = intrin_.fy;
        Intrinsic[4] = intrin_.ppx;
        Intrinsic[5] = intrin_.ppy;


        float *pointcloud = new float[width*height*3];
        float *pointcloudOut = new float[width*height*3];

        rs2::align align(RS2_STREAM_DEPTH);

        // Init data for contour detection
        cv::Mat depthBinary = cv::Mat(height, width, CV_8UC1);
        cv::Mat depthBinary0 = cv::Mat(height, width, CV_8UC1);
        cv::Mat depthBinary1 = cv::Mat(height, width, CV_8UC1);
        cv::Mat depthBinary2 = cv::Mat(height, width, CV_8UC1);
        vector<cv:: Mat> color_data;
        vector<vector<cv::Point>> contours;
        vector<vector<cv::Point>> contours0;
        vector<vector<cv::Point>> contours1;
        vector<cv::Vec4i> hierarchy;
        vector<cv::Vec4i> hierarchy0;
		vector<cv::Vec4i> hierarchy1;

        while (true)
        {              
            // Block program until frames arrive
            rs2::frameset frame = p.wait_for_frames(); 
            rs2::frameset frames = align.process(frame);   
            // Try to get a frame of a depth image


            rs2::frame color_frame1 = frame.get_color_frame();
            cv::Mat color_image1(cv::Size(640, 480), CV_8UC3, (void*)color_frame1.get_data(), cv::Mat::AUTO_STEP);
            rs2::frame color_frame = frames.get_color_frame();
            cv::Mat color_image(cv::Size(640, 480), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
           
            // Get the depth frame's dimensions
            rs2::depth_frame depth = frames.get_depth_frame(); 
            cv::Mat depth_image = cv::Mat(height, width, CV_16UC1);
            cv::Mat depthColor = cv::Mat(depth_image.rows,depth_image.cols, CV_8UC3);
            depth_image.data = (unsigned char*)depth.get_data();

            // Covert Depth information to color display 
            convertDepthToColor(depth_image, depthColor);

            // point cloud processing
            GetPointCloud((unsigned short*)depth_image.data, width, height, intrin, pointcloud);
            int npoint;
            FilterPointCloudWithNormal(pointcloud, pointcloudOut, npoint, width, height);

            // Use hsv_image for color detection 
            cv::Mat hsv_image;
            cv::cvtColor(color_image,hsv_image, cv::COLOR_BGR2HSV);
            Mat1b mask1, mask2,mask3;
            // inRange(hsv_image, Scalar(0, 70, 50), Scalar(10, 255, 255), mask1);
            inRange(hsv_image, Scalar(170, 70, 50), Scalar(180, 255, 255), mask2);
            Mat1b redmask =  mask2;
            inRange(hsv_image, Scalar(0, 10, 20), Scalar(10, 255, 255), mask3);
            Mat1b pinkmask = mask3;


            // Find ground plane
            float _g[3] = {0, 0, 1};
            int min_best_total = 5;
            float sigma = 0.1*0.1*0.015;
            int error_code;
            bool bPlaneSuccess = false;
            float A, B, C;            
            bPlaneSuccess = ransac_estimation_plane(pointcloudOut, npoint, _g, 0.5, sigma, 4, 150, min_best_total, 2.0, CV_PI/2.0f, A, B, C, error_code);

            // Find objects of different color
            for (size_t y = 0; y < height; y++)
            {
                int _i = y*width;
                for (size_t x = 0; x < width; x++)
                {
                    int i = _i + x;
                    int iii = 3*i;
                    float d = (float)pointcloud[3*i+2];

                    depthBinary.data[i] = 0;
                    depthBinary0.data[i] = 0;
                    depthBinary1.data[i] = 0;
                    depthBinary2.data[i] = 0;

                    if (pointcloud[iii] == 0.0f && pointcloud[iii+1] == 0.0f && pointcloud[iii+2] == 0.0f) // too near or too far
                    {
                        continue;
                    }

                    float fThickness = 0.02f;
          
                    float dis = distance_to_plane_signed(pointcloud[iii], pointcloud[iii+1], pointcloud[iii+2], A, B, C);
                    if (fabs(dis) < fThickness)
                    {
                        depthColor.data[iii + 1] = 255;
                        depthBinary.data[i] = 255;
                    }
                    if (dis < -fThickness)
                    {
                        depthColor.data[iii + 0] = 0;
                        depthColor.data[iii + 1] = 0;
                        depthColor.data[iii + 2] = 0;
                    }
                    else
                    {
                        // Red color
                        if (redmask[y][x])
                        {
                            depthBinary0.data[i] = 255;
                        }
                        else{
                            depthBinary0.data[i] = 0;
                        }
                        // Pink color 
                        if (pinkmask[y][x])
                        {
                            depthBinary1.data[i] = 255;
                        }
                        else{
                            depthBinary1.data[i] = 0;
                        }                   
                    }
                }
            }

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            
            Erosion(depthBinary, depthBinary, 5);
            
            // find contours of the planes
            cv::findContours(depthBinary, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );
            int idx = 0;
            double maxArea = -1.0f;
            int maxid = -1;
            if (hierarchy.size() > 0)
            {
                for(; idx >= 0; idx = hierarchy[idx][0])
                {
                    double area = cv::contourArea(contours[idx]);

                    if (area > maxArea)
                    {
                      maxArea = area;
                      maxid = idx;
                    }
                }
            }
 
            // find planes
            vector<vector<Point> > contours_poly( contours.size() );
            // Rect boundRect;
            RotatedRect rectPoint;

            if (maxid != -1)
            {                       
                planeCenterLeft.clear();
                planeCenterRight.clear();
                planeAngleLeft.clear();
                planeAngleRight.clear(); 
                // drawContours(depthColor, contours, 0, Scalar(255, 255, 0), 2, 8 );
                for (int idx = 0; idx < contours.size(); idx++)
                {   
                    double area = cv::contourArea(contours[idx]);
                    int father = hierarchy[idx][3];
                    int son = hierarchy[idx][2];
                    double fatherArea;
                    double sonArea;
                    
                    approxPolyDP( Mat(contours[idx]), contours_poly[idx], 5, true );                  
                    if (father != -1)
                    {
                        fatherArea = cv::contourArea(contours[father]);
                    }
                    if (son != -1)
                    {
                        sonArea = cv::contourArea(contours[son]);
                    } 

                    if (father == -1)
                    {
                        if ( area > areaLowerBound && area < areaUpperBound )
                        {
                            // only draw the contours of plane with enough area
                            rectPoint = minAreaRect( Mat(contours[idx]) );
                            float planeAngle = rectPoint.angle;
                            if (rectPoint.size.width < rectPoint.size.height)
                                planeAngle = planeAngle - 90;
                            Point2f fourPoint2f[4];
                            rectPoint.points( fourPoint2f );
                            vector<Point2f> points;
                            points.push_back(fourPoint2f[0]);
                            points.push_back(fourPoint2f[1]);
                            points.push_back(fourPoint2f[2]);
                            points.push_back(fourPoint2f[3]);
                            double rectArea = cv::contourArea(points);
                            double ratio = area / rectArea;  
                            if (ratio > ratioLimit)
                            {
                                line(depthColor, fourPoint2f[0], fourPoint2f[1], Scalar(255, 255, 255), 3 );
                                line(depthColor, fourPoint2f[1], fourPoint2f[2], Scalar(255, 255, 255), 3 );
                                line(depthColor, fourPoint2f[2], fourPoint2f[3], Scalar(255, 255, 255), 3 );
                                line(depthColor, fourPoint2f[0], fourPoint2f[3], Scalar(255, 255, 255), 3 );

                                // take the center point as target point
                                int x = (fourPoint2f[0].x + fourPoint2f[2].x)/2;
                                int y = (fourPoint2f[0].y + fourPoint2f[2].y)/2;
                                planeCenter[0] = pointcloud[3*(y*width + x)]*1000;
                                planeCenter[1] = pointcloud[3*(y*width + x)+1]*1000;
                                planeCenter[2] = pointcloud[3*(y*width + x)+2]*100;
                                if (x > 320)
                                {   
                                    planeCenterRight.push_back(planeCenter[0]);
                                    planeCenterRight.push_back(planeCenter[1]);
                                    planeCenterRight.push_back(planeCenter[2]);
                                    circle(depthColor, Point(x,y), 2, Scalar(0, 255, 0));
                                    planeAngleRight.push_back(planeAngle);
                                }
                                else
                                {
                                    planeCenterLeft.push_back(planeCenter[0]);
                                    planeCenterLeft.push_back(planeCenter[1]);
                                    planeCenterLeft.push_back(planeCenter[2]);                              
                                    circle(depthColor, Point(x,y), 2, Scalar(0, 0, 255));
                                    planeAngleLeft.push_back(planeAngle);
                                }         
                            }
                        }
                    }

                    else if (son != -1 && father != -1)
                    {
                        if ( sonArea > areaLowerBound && sonArea < areaUpperBound )
                        {
                            // only draw the contours of plane with enough area
                            rectPoint = minAreaRect( Mat(contours[son]) );
                            float planeAngle = rectPoint.angle;
                            if (rectPoint.size.width < rectPoint.size.height)
                                planeAngle = planeAngle - 90;                           
                            Point2f fourPoint2f[4];
                            rectPoint.points( fourPoint2f );
                            vector<Point2f> points;
                            points.push_back(fourPoint2f[0]);
                            points.push_back(fourPoint2f[1]);
                            points.push_back(fourPoint2f[2]);
                            points.push_back(fourPoint2f[3]);
                            double rectArea = cv::contourArea(points);
                            double ratio = sonArea / rectArea;  
                            if (ratio > ratioLimit)
                            {
                                line(depthColor, fourPoint2f[0], fourPoint2f[1], Scalar(255, 255, 255), 3 );
                                line(depthColor, fourPoint2f[1], fourPoint2f[2], Scalar(255, 255, 255), 3 );
                                line(depthColor, fourPoint2f[2], fourPoint2f[3], Scalar(255, 255, 255), 3 );
                                line(depthColor, fourPoint2f[0], fourPoint2f[3], Scalar(255, 255, 255), 3 );

                                // take the center point as target point
                                int x = (fourPoint2f[0].x + fourPoint2f[2].x)/2;
                                int y = (fourPoint2f[0].y + fourPoint2f[2].y)/2;
                                planeCenter[0] = pointcloud[3*(y*width + x)]*1000;
                                planeCenter[1] = pointcloud[3*(y*width + x)+1]*1000;
                                planeCenter[2] = pointcloud[3*(y*width + x)+2]*100;
                                if (x > 320)
                                {   
                                    planeCenterRight.push_back(planeCenter[0]);
                                    planeCenterRight.push_back(planeCenter[1]);
                                    planeCenterRight.push_back(planeCenter[2]);
                                    circle(depthColor, Point(x,y), 2, Scalar(0, 255, 0));
                                    planeAngleRight.push_back(planeAngle);
                                }
                                else
                                {
                                    planeCenterLeft.push_back(planeCenter[0]);
                                    planeCenterLeft.push_back(planeCenter[1]);
                                    planeCenterLeft.push_back(planeCenter[2]);                              
                                    circle(depthColor, Point(x,y), 2, Scalar(0, 0, 255));
                                    planeAngleLeft.push_back(planeAngle);
                                }    
                            }
                        }
                    }
                }
            }

            else
            {
                planeCenter[0] = 0.0f;
                planeCenter[1] = 0.0f;
                planeCenter[2] = -1000.0f;
                planeCenterLeft.push_back(0.0f);
                planeCenterLeft.push_back(0.0f);
                planeCenterLeft.push_back(-1000.0f);
                planeCenterRight.push_back(0.0f);
                planeCenterRight.push_back(0.0f);
                planeCenterRight.push_back(-1000.0f);
                planeAngleLeft.push_back(0.0f);
                planeAngleRight.push_back(0.0f);
            }

            cv::namedWindow("colorPlane");
            cv::imshow("colorPlane", depthBinary);
            cvWaitKey(1);

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

            // Collect color binary data
            color_data.push_back(depthBinary0);
            color_data.push_back(depthBinary1);
            color_data.push_back(depthBinary2);

            // Find color 1 object contour
            cv::findContours(color_data[0], contours0, hierarchy0, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );
            int idx0 = 0;
            double maxArea0 = -1.0f;
            int maxid0 = -1;
            if (hierarchy0.size() > 0)
            {
                for(; idx0 >= 0; idx0 = hierarchy0[idx0][0])
                {
                    double area0 = cv::contourArea(contours0[idx0]);

                    if (area0 > maxArea0)
                    {
                      maxArea0 = area0;
                      maxid0 = idx0;
                    }
                }
        	}

            // Find color 2 object contour
            cv::findContours(color_data[1], contours1, hierarchy1, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );
            int idx1 = 0;
            double maxArea1 = -1.0f;
            int maxid1 = -1;
            if (hierarchy1.size() > 0)
            {
                for(; idx1 >= 0; idx1 = hierarchy1[idx1][0])
                {
                     double area1 = cv::contourArea(contours1[idx1]);

                    if (area1 > maxArea1)
                    {
                      maxArea1 = area1;
                      maxid1 = idx1;
                    }
                }
        	}
 
            // Init data for coordiantes
            vector<vector<Point> > contours_poly0( contours0.size() );
            Rect boundRect0;
            RotatedRect rectPoint0; 
            Point2f fourPoint0[4];
            
            vector<vector<Point> > contours_poly1( contours1.size() );
            Rect boundRect1;
            RotatedRect rectPoint1; 
            Point2f fourPoint1[4];

            // Find the coordinates of color 1 object and draw rectangles on them
            if (maxid0 != -1 )
            {

            	Red.clear();
                angle0.clear();
            	vector<float> temp;
            	for( maxid0 = 0; maxid0 < contours0.size(); maxid0++ )
            	{	
            		
	            	approxPolyDP( Mat(contours0[maxid0]), contours_poly0[maxid0], 3, true );
	            	double area0  = cv::contourArea( contours_poly0[maxid0]);
                    float radius0;
                    rectPoint0 = minAreaRect( Mat(contours_poly0[maxid0]));

	            	
                    if (area0 > 2000)
	            	{

		            	boundRect0 = boundingRect( Mat(contours_poly0[maxid0]) );
                        rectPoint0.points( fourPoint0 );
                        line(depthColor, fourPoint0[0], fourPoint0[1], Scalar(0, 0, 255), 3 );
                        line(depthColor, fourPoint0[1], fourPoint0[2], Scalar(0, 0, 255), 3 );
                        line(depthColor, fourPoint0[2], fourPoint0[3], Scalar(0, 0, 255), 3 );
                        line(depthColor, fourPoint0[0], fourPoint0[3], Scalar(0, 0, 255), 3 );

		                int x = (boundRect0.tl().x + boundRect0.br().x)/2;
		                int y = (boundRect0.tl().y + boundRect0.br().y)/2;

                        radius0 = rectPoint0.angle;
                        if (rectPoint0.size.width < rectPoint0.size.height)
                            radius0 = radius0 - 90;
 
		                RedCoordinate[0] = pointcloud[3*(y*width + x)]*1000;
		                RedCoordinate[1] = pointcloud[3*(y*width + x)+1]*1000;
		                RedCoordinate[2] = pointcloud[3*(y*width + x)+2]*100;

		                for(int i = 0; i<3;i++){
		                	temp.push_back(RedCoordinate[i]);
		                }
                        angle0.push_back(radius0);
		                Red.push_back(temp);
                        temp.clear();

	            	}
            	}
            }
            else
            {
                GreenCoordinate[0] = 0;
                GreenCoordinate[1] = 0;
                GreenCoordinate[2] = -1000;
            }

            // Find the coordinates of color 2 object and draw rectangles on them
            if ( maxid1 != -1)
            {

            	Green.clear();
                angle1.clear();
            	vector<float> temp1;

            	for( maxid1 = 0; maxid1 < contours1.size(); maxid1++ )
            	{	

	            	approxPolyDP( Mat(contours1[maxid1]), contours_poly1[maxid1], 3, true );
	            	double area1  = cv::contourArea( contours_poly1[maxid1]);
                    rectPoint1 = minAreaRect( Mat(contours_poly1[maxid1]) );
                    float  radius1;

	                if(area1 > 2000)
	                {
	              		boundRect1 = boundingRect( Mat(contours_poly1[maxid1]) );
                        rectPoint1.points(fourPoint1 );
                        line(depthColor, fourPoint1[0], fourPoint1[1], Scalar(203, 192, 255), 3 );
                        line(depthColor, fourPoint1[1], fourPoint1[2], Scalar(203, 192, 255), 3 );
                        line(depthColor, fourPoint1[2], fourPoint1[3], Scalar(203, 192, 255), 3 );
                        line(depthColor, fourPoint1[0], fourPoint1[3], Scalar(203, 192, 255), 3 );

		                int x = (boundRect1.tl().x + boundRect1.br().x)/2;
		                int y = (boundRect1.tl().y + boundRect1.br().y)/2;

                        radius1 = rectPoint1.angle;
                        if (rectPoint1.size.width < rectPoint1.size.height)
                            radius1 = radius1 - 90;

		                GreenCoordinate[0] = pointcloud[3*(y*width + x)]*1000;
		                GreenCoordinate[1] = pointcloud[3*(y*width + x)+1]*1000;
		                GreenCoordinate[2] = pointcloud[3*(y*width + x)+2]*100;
		               	for(int i = 0; i<3;i++){
		                	temp1.push_back(GreenCoordinate[i]);
		                }

		                Green.push_back(temp1);
                        temp1.clear();
                        angle1.push_back(radius1);

	            	}
            	}
            }
            else
            {
                GreenCoordinate[0] = 0;
                GreenCoordinate[1] = 0;
                GreenCoordinate[2] = -1000;
            }

            cv::namedWindow("colorRed");
            cv::imshow("colorRed", color_data[0]);

            cv::namedWindow("colorPink");
            cv::imshow("colorPink", color_data[1]);

            cv::namedWindow("depth2");
            cv::imshow("depth2", depthColor);

            cv::namedWindow("color");
            cv::imshow("color", color_image1);
            cvWaitKey(1);

        }
    }

    // Function not use
    float* GetCentralPoint()
    {

        return GreenCoordinate;
    }

    // Get camera intrinsics info
    float* GetIntrinsics()
    {

        return Intrinsic;
    }

    // Get number of total objects
    int* GetNumOfPoints()
    {

        NumOfPoints[0] = Red.size();
        NumOfPoints[1] = Green.size();
        NumOfPoints[2] = 0;
        return NumOfPoints;
    }

    // Get color 1 object angles
    float* GetRedAngles()
    {   
        int NumofRed = Red.size();
        float* RedAngles = new float[NumofRed];

        for(int i= 0; i<NumofRed; i++){
            {
                RedAngles[i] = angle0[i]; 
            }
        }

        return RedAngles;
    }

    // Get color 2 object angles
    float* GetGreenAngles()
    {   
        int NumofGreen = Green.size();
        float* GreenAngles = new float[NumofGreen];

        for(int i= 0; i<NumofGreen; i++){
            {
                GreenAngles[i] = angle1[i]; 
            }
        }

        return GreenAngles;
    }


    // Get color 1 object coordinates
    float* GetRedCentralPoint()
    {	
    	int NumofRed = Red.size();
    	float* RedCoordinates = new float[NumofRed];

    	for(int i= 0; i<NumofRed; i++){
    		for(int j = 0; j< Red[i].size(); j++){
    			RedCoordinates[3*i+j] = Red[i][j]; 
    		}
    	}

        return RedCoordinates;
    }

    // Get color 2 object coordinates
    float* GetGreenCentralPoint()
    {
    	int NumofGreen = Green.size();
    	float* GreenCoordinates = new float[NumofGreen];

    	for(int i= 0; i<NumofGreen; i++){
    		for(int j = 0; j< Green[i].size(); j++){
    			GreenCoordinates[3*i+j] = Green[i][j]; 
    		}
    	}

        return GreenCoordinates;
    }

    float* GetLeftPlaneCentralPoint()
    {
        float *centersLeft = new float[planeCenterLeft.size()];
        if (!planeCenterLeft.empty())
        {
            memcpy(centersLeft, &planeCenterLeft[0], planeCenterLeft.size()*sizeof(float));
        }        
        return centersLeft;
    }
    float* GetRightPlaneCentralPoint()
    {
        float *centersRight = new float[planeCenterRight.size()];
        if (!planeCenterRight.empty())
        {
            memcpy(centersRight, &planeCenterRight[0], planeCenterRight.size()*sizeof(float));
        }
        return centersRight;
    }
    float* GetLeftPlaneAngle()
    {
        float *anglesLeft = new float[planeAngleLeft.size()];
        if (!planeAngleLeft.empty())
        {
            memcpy(anglesLeft, &planeAngleLeft[0], planeAngleLeft.size()*sizeof(float));
        }  
        return anglesLeft;
    }
    float* GetRightPlaneAngle()
    {
        float *anglesRight = new float[planeAngleRight.size()];
        if (!planeAngleRight.empty())
        {
            memcpy(anglesRight, &planeAngleRight[0], planeAngleRight.size()*sizeof(float));
        }  
        return anglesRight;
    }    
}