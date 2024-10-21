#include <iostream>
#include <getopt.h>
#include <sys/ioctl.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Time.h>
#include <std_msgs/Header.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

using namespace std;

int get_term_width( void ) {
  struct winsize ws;
  if ( ioctl( STDIN_FILENO , TIOCGWINSZ, &ws ) != 0 &&
       ioctl( STDOUT_FILENO, TIOCGWINSZ, &ws ) != 0 &&
       ioctl( STDERR_FILENO, TIOCGWINSZ, &ws ) != 0 ) {
    fprintf( stderr,
      "ioctl() failed (%d): %s\n", errno, strerror( errno )
    );
    return 40;
  }
  return ws.ws_col;
}

int main(int argc, char **argv)
{
    // get the terminal width for prog bar size
    int barWidth = get_term_width() - 10;

    ros::init(argc, argv, "BagFromVideo");

    string input_file;
    string output_bag;
    string node_name = "/camera/image_raw";
    double freq = 30.0;

    // get user options
    const struct option long_options[] = {
        {"input", required_argument, 0, 'i'},
        {"output", required_argument, 0, 'o'},
        {"node", optional_argument, 0, 'n'},
        {"frequency", optional_argument, 0, 'f'},
        {0, 0, 0, 0}
    };

    int option_index = 0;
    int opt;
    while ((opt = getopt_long(argc, argv, "i:o:f:n:", long_options, &option_index)) != -1) {
        switch (opt) {
            case 'i':
                input_file = optarg;
                break;
            case 'o':
                output_bag = optarg;
                break;
            case 'n':
                node_name = optarg;
                break;
            case 'f':
                freq = atof(optarg);
                break;
            default:
                cerr << "Usage: BagFromVideo --input <video_file> --output <bag_file> -f <frequency> [--node <topic>]" << endl;
                return 1;
        }
    }
    if (input_file.empty() || output_bag.empty()) {
        cerr << "Error: Input video file and output bag file are required." << endl;
        return 1;
    }
    cv::VideoCapture cap(input_file);
    if(!cap.isOpened())
    {
        cerr << "Error: Couldn't open the video file." << endl;
        return 0;
    }

    cout << "Video file selected: " << input_file << endl; 
    cout << "Node name is " << node_name << endl;

    // set ros vars
    ros::start();
    rosbag::Bag bag_out(output_bag, rosbag::bagmode::Write);
    ros::Time t = ros::Time::now();
    cv::Mat frame, rgb_frame;

    const float T = 1.0f / freq;

    ros::Duration d(T);
    
    // start reading and writing frames
    int frame_count = 0;
    int total_frames = cap.get(cv::CAP_PROP_FRAME_COUNT);
    while(cap.read(frame)) {
        if (!ros::ok())
            break;
        cv::cvtColor(frame, rgb_frame, cv::COLOR_BGR2RGB);
        cv_bridge::CvImage cvImage;
        cvImage.image = rgb_frame;
        cvImage.encoding = sensor_msgs::image_encodings::RGB8;
        cvImage.header.stamp = t;

        bag_out.write(node_name, ros::Time(t), cvImage.toImageMsg());

        t += d;
        float prog = static_cast<float>(frame_count) / total_frames;
        uint8_t pos = prog * barWidth;
        cout << "[";
        for (int i = 0; i < barWidth; ++i) {
            if (i <= pos) cout << "|";
            else cout << " ";
        }
        cout << "] " << int(prog * 100) << "%\r";
        cout.flush();
        frame_count++;
    }
    cout << "\r\nDone! Written to " << output_bag << endl;

    bag_out.close();
    ros::shutdown();

    return 0;
}
