// ros headers
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
using std::placeholders::_1;

//socket headers
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <netdb.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <arpa/inet.h>

//constants for tcp client
#define PORT "3490" // the port client will be connecting to 
#define MAXDATASIZE 100 // max number of bytes we can get at once 

//constants for ros
#define topic "/camera_angles"
#define sendaddr "0.0.0.0"
#define pan_index 0
#define tilt_index 1

//conversion functions
inline int angleto12bit(float x) {
    return floor((x*4095)/180);
} 

//client struct
struct client {
  int sockfd;
  char buf[MAXDATASIZE];
  struct addrinfo hints,*servinfo,*p;
  char s[INET_ADDRSTRLEN];
} client_t;

//function initialisation
void *get_in_addr(struct sockaddr *);
int client_setup(struct client *, char *);



