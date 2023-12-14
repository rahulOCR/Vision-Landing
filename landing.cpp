#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>
#include <math.h>
#include "include/mavlink/v2.0/common/mavlink.h"
#include <cstring>
#include <unistd.h>
#include <arpa/inet.h>

int SHOW_VIDEO = 0;

#define TCP_PORT 1330   // Default MAVProxy port

#define USE_UDP 1       // 1 for UDP; 0 for TCP

int id_to_find  = 31;

int y_sum = 0;
int x_sum = 0;
double x_avg;
double y_avg;
    



int horizontal_res = 1280;
int vertical_res = 720;

double horizontal_fov = 62.2 * (M_PI / 180);  // Pi cam V1: 53.5 V2: 62.2
double vertical_fov = 48.8 * (M_PI / 180);    // Pi cam V1: 41.41 V2: 48.8


// Detect markers in the frame
std::vector<int> markerIds;
std::vector<std::vector<cv::Point2f>> markerCorners;

cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);




/*
Find current position of target in angle
*/
void Find_Position(cv::Mat& frame, double& x_ang, double& y_ang) {
    
    cv::aruco::detectMarkers(frame, dictionary, markerCorners, markerIds);

    // Draw the detected markers on the frame
    cv::aruco::drawDetectedMarkers(frame, markerCorners, markerIds);

        
        if (markerIds.size() > 0 && markerIds[0] == id_to_find)
        {
            x_sum = 0;
            y_sum = 0;
            
            for (const cv::Point2f& corner : markerCorners[0]) 
            {
                // std::cout <<"corner :" << corner << std::endl;
                
                x_sum += corner.x;
                y_sum += corner.y;
            }
                

            x_avg = x_sum * .25;
            y_avg = y_sum * .25;

            x_ang = (x_avg - horizontal_res * .5) * (horizontal_fov / horizontal_res);
            y_ang = (y_avg - vertical_res * .5) * (vertical_fov / vertical_res);
            
            
           
        }
        else
        {
            x_ang = 0;
            y_ang = 0;

        }
        

}



int main(int argc, char *argv[])
{
    // cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 6.498112210160318227e+02, 0.000000000000000000e+00, 3.033898824453654015e+02,
    //                                                     0.000000000000000000e+00, 6.491710307598696090e+02, 2.242803248887676659e+02,
    //                                                     0.000000000000000000e+00, 0.000000000000000000e+00, 1.000000000000000000e+00);

    // cv::Mat distCoeffs = (cv::Mat_<double>(1, 5) << -8.639972154214454678e-03, 8.761494385760753012e-01, -1.025338394352215236e-02,
    //                                                 -1.655004726785345992e-02, -2.063772018914907136e+00);
   
    if (argc < 5) {
        std::cerr << "Usage: " << argv[0] << " <Sensor-ID> <UDP PORT NUMBER OF FCU> <TARGET ID> <SHOW VIDEO {1/0}" << std::endl;
        return 1;  // Exit with an error code
    }
    
    int device_id = std::atoi(argv[1]);
    int udp_port = std::atoi(argv[2]);
    id_to_find = std::atoi(argv[3]);
    SHOW_VIDEO = std::atoi(argv[4]);

    std::ofstream outputFile("log.txt");

    if (!outputFile.is_open()) {
        std::cerr << "Error opening file!" << std::endl;
        return 1;  // Exit with an error code
    }



    cv::VideoCapture cap(device_id);

    // Check if the camera opened successfully
    if (!cap.isOpened()) {
        std::cerr << "Error opening camera!" << std::endl;
        return -1;
    }

    horizontal_res = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH));
    vertical_res = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_HEIGHT));

    // Print the resolution
    outputFile << "Camera resolution: " << horizontal_res << " x " << vertical_res << std::endl;


    cv::Mat frame;
    double angle_x, angle_y;
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        // Create a UDP socket
#if USE_UDP == 1

    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
        perror("Error creating socket");
        return -1;
    }

    // Set up the destination address for UDP communication
    struct sockaddr_in destAddr;
    memset(&destAddr, 0, sizeof(destAddr));
    destAddr.sin_family = AF_INET;
    destAddr.sin_port = htons(udp_port);
    inet_pton(AF_INET, "127.0.0.1", &(destAddr.sin_addr));

#else
    Create a TCP socket
    int sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) {
        perror("Error creating socket");
        return -1;
    }

    Set up the destination address for TCP communication
    struct sockaddr_in destAddr;
    memset(&destAddr, 0, sizeof(destAddr));
    destAddr.sin_family = AF_INET;
    destAddr.sin_port = htons(TCP_PORT);
    inet_pton(AF_INET, "127.0.0.1", &(destAddr.sin_addr));

    // Connect to the server (assumed to be MAVProxy in this case)
    if (connect(sockfd, (struct sockaddr*)&destAddr, sizeof(destAddr)) < 0) {
        perror("Error connecting to server");
        close(sockfd);
        return -1;
    }

#endif

    mavlink_message_t msg;
    float array[] = {1,0,0,0};
    
    mavlink_landing_target_t land_msg;
    land_msg.type = 2;
    land_msg.position_valid = 1;
    land_msg.q[0] = 1;
    land_msg.q[1] = 0;
    land_msg.q[2] = 0;
    land_msg.q[3] = 0;
    

    // Serialize the message
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    


    while (true)

    {
        // Capture a frame from the camera
        
        cap >> frame;

        Find_Position(frame, angle_x, angle_y);

        // utilized in FCU
        outputFile << "x_ang = " << angle_x << "y_ang = " << angle_y<<std::endl;

        land_msg.angle_x = (float)angle_x;
        land_msg.angle_y = (float)angle_y;


        mavlink_msg_landing_target_encode(255, 1, &msg, &land_msg);
        uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);

#if USE_UDP == 1

    // Send the message over UDP
    ssize_t bytesSent = sendto(sockfd, buffer, len, 0, (struct sockaddr*)&destAddr, sizeof(destAddr));
    if (bytesSent < 0) {
        perror("Error sending message");
    } else {
        outputFile << "Message sent successfully" << std::endl;
    }



#else
    ssize_t bytesSent = send(sockfd, buffer, len, 0);
    if (bytesSent < 0) {
        perror("Error sending message");
    } else {
        std::cout << "Message sent successfully" << std::endl;
    }


#endif

    if (SHOW_VIDEO == 1)      
        // Display the result
        cv::imshow("Detected Markers", frame);



        // Break the loop if the 'ESC' key is pressed
        if (cv::waitKey(1) == 27) {
            break;
        }
    

    }


    // Release the camera
    cap.release();

    // Close socket
    close(sockfd);


    return 0;
}
