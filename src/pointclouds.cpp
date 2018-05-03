#include <iostream>
#include <string>
#include <vector>

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include "example.hpp"          // Include short list of convenience functions for rendering

#include <zmq.hpp>
#include <thread>

#include <algorithm>            // std::min, std::max

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

// Helper functions
void register_glfw_callbacks(window& app, glfw_state& app_state);

using pcl_ptr = pcl::PointCloud<pcl::PointXYZ>::Ptr;
using namespace std;

struct PC_sample{
    pcl_ptr pc;
    float theta; //radians
    float r; //meters
    float z; //meters
};

//GLOBAL STUFF
rs2::points global_rs_points;
std::vector<PC_sample> all_samples;

pcl_ptr get_pc()
{
    rs2::points points = global_rs_points;
    pcl_ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    auto sp = points.get_profile().as<rs2::video_stream_profile>();
    cloud->width = sp.width();
    cloud->height = sp.height();
    cloud->is_dense = false;
    cloud->points.resize(points.size());
    auto ptr = points.get_vertices();
    for (auto& p : cloud->points)
    {
        p.x = ptr->x;
        p.y = ptr->y;
        p.z = ptr->z;
        ptr++;
    } 
    return cloud;
}

pcl_ptr transform_pc(PC_sample sample) {
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translation() << sample.r * cos(sample.theta), sample.r * sin(sample.theta), -sample.z;
    transform.rotate(Eigen::AngleAxisf(sample.theta, Eigen::Vector3f::UnitZ()));

    pcl_ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::transformPointCloud(*sample.pc, *transformed_cloud, transform);

    return transformed_cloud;
}

//main method for saving pc
int run_pc_collection(std::vector<PC_sample> samples, int num_samples, std::string pc_filename){
    
    //create one big pc
    pcl_ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    //apply transforms, merge
    for (int i = 0; i < num_samples; i++)
    {
        pcl_ptr transformed = transform_pc(samples[i]);
	*cloud += *transformed; //merges
    }

    //save everything
    pcl::io::savePCDFileASCII(pc_filename, *cloud);
    std::cout << "Saved " << cloud->points.size() << " data points to " << pc_filename << std::endl;
    return 1;
}

std::vector<string> split(const string &s, char delim) {
    stringstream ss(s);
    string item;
    std::vector<string> tokens;
    while (getline(ss, item, delim)) {
        tokens.push_back(item);
    }
    return tokens;
}

int run_op(int * stop){
    
    //  Prepare our context and socket
    zmq::context_t context (1);
    zmq::socket_t socket (context, ZMQ_REP);
    socket.bind ("tcp://127.0.0.1:5555");

    while (*stop != 1) {
        zmq::message_t request;
        socket.recv(&request);
        string req = (char *)request.data();
        std::cout << "Backend Received Request: " << req << std::endl;
        if (req.compare("STOP") == 0) {
            *stop = 1;
            std::cout << "-- STOPPING BY REQUEST --" << std::endl;
            break;
        } 
        else if (req.compare("COMPUTE") == 0) {
            run_pc_collection(all_samples, all_samples.size(), "pointcloud.pcd");
	    all_samples.clear();
	}
	else {
	    //assume 3 element string, cylindrical coordinates: "radius,theta,height"
	    std::vector<string> splits = split(req, ',');
	    double r = std::stod(splits[0]);
	    double theta = std::stod(splits[1]);
	    double z = std::stod(splits[2]);

	    //gets pointcloud at this current timestep
	    pcl_ptr sample = get_pc();
	    PC_sample pcs;

            pcs.pc = sample;
            pcs.r = r;
            pcs.theta = theta;
            pcs.z = z;

	    all_samples.push_back(pcs);
        }
        zmq::message_t reply (7);
        memcpy (reply.data (), "success", 7);
        socket.send (reply);
    }
}

int main(int argc, char * argv[]) try
{
    // Create a simple OpenGL window for rendering:
    window app(1280, 720, "RealSense Pointcloud Example");
    // Construct an object to manage view state
    glfw_state app_state;
    // register callbacks to allow manipulation of the pointcloud
    register_glfw_callbacks(app, app_state);

    int stop = 0;

    std::thread pc_collection(run_op, &stop);

    std::cout << "Successfully beginning PC Collection" << std::endl;

    // Declare pointcloud object, for calculating pointclouds and texture mappings
    rs2::pointcloud pc;
    // We want the points object to be persistent so we can display the last cloud when a frame drops
    rs2::points points;
 
    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    // Start streaming with default recommended configuration
    pipe.start();

    while (app and stop != 1) // Application still alive?
    {
        // Wait for the next set of frames from the camera
        auto frames = pipe.wait_for_frames();

        auto depth = frames.get_depth_frame();

        // Generate the pointcloud and texture mappings
        points = pc.calculate(depth);
        global_rs_points = points;
        
        auto color = frames.get_color_frame();

        // Tell pointcloud object to map to this color frame
        pc.map_to(color);

        // Upload the color frame to OpenGL
        app_state.tex.upload(color);

        // Draw the pointcloud
        draw_pointcloud(app.width(), app.height(), app_state, points);
    }

    std::cout << "Main Thread Terminating." << std::endl;
    stop = 0;
    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception & e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
