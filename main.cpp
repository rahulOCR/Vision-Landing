#include "include/mavlink/v2.0/common/mavlink.h"
#include <iostream>
#include <cstring>
#include <unistd.h>
#include <arpa/inet.h>

#define UDP_PORT 14553

#define TCP_PORT 1330   // Default MAVProxy port

#define USE_UDP 1       // 1 for UDP; 0 for TCP

int main() {
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
    destAddr.sin_port = htons(UDP_PORT);
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

    // Initialize the MAVLink system
    // mavlink_system_t mavlinkSys = {1, 1, MAV_TYPE_GCS};

    // Create a message to arm the vehicle
    mavlink_message_t msg;
    
    mavlink_auth_takeoff_t auth_takeoff;
    auth_takeoff.status = 0;
    mavlink_msg_auth_takeoff_encode(225, 1, &msg, &auth_takeoff);

    // Serialize the message
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);


#if USE_UDP == 1

    // Send the message over UDP
    ssize_t bytesSent = sendto(sockfd, buffer, len, 0, (struct sockaddr*)&destAddr, sizeof(destAddr));
    if (bytesSent < 0) {
        perror("Error sending message");
    } else {
        std::cout << "Message sent successfully" << std::endl;
    }


    

    // Close the UDP socket
    close(sockfd);


#else
    ssize_t bytesSent = send(sockfd, buffer, len, 0);
    if (bytesSent < 0) {
        perror("Error sending message");
    } else {
        std::cout << "Message sent successfully" << std::endl;
    }

    // Close the TCP socket
    close(sockfd);

#endif

    return 0;
}
