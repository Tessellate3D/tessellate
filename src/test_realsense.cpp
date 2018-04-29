#include <librealsense2/rs.hpp>
#include <iostream>
#include <opencv2/opencv.hpp>

using namespace cv;


int main() {

	// Create a Pipeline - this serves as a top-level API for streaming and processing frames
	rs2::pipeline p;

	rs2::config cfg;
	cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
	cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
	// Configure and start the pipeline
	p.start(cfg);

	namedWindow("Depth Image", WINDOW_AUTOSIZE);
	namedWindow("Color Image", WINDOW_AUTOSIZE);


	while (true)
	{
    		// Block program until frames arrive
    		rs2::frameset frames = p.wait_for_frames(); 
    
    		// Try to get a frame of a depth image
    		rs2::depth_frame depthf = frames.get_depth_frame(); 
    		rs2::frame colorf = frames.get_color_frame(); 

    		// Get the depth frame's dimensions
		float width = depthf.get_width();
	    	float height = depthf.get_height();
    
    		// Query the distance from the camera to the object in the center of the image
    		float dist_to_center = depthf.get_distance(width / 2, height / 2);
    
    		// Print the distance 
    		std::cout << "The camera is facing an object " << dist_to_center << " meters away \r";

		Mat depth(Size(640,480), CV_8UC1, (void*)depthf.get_data(), Mat::AUTO_STEP);
		Mat color(Size(640,480), CV_8UC3, (void*)colorf.get_data(), Mat::AUTO_STEP);
		

		imshow("Depth Image", depth);
		imshow("Color Image", color);

		waitKey(1);
	}
}
