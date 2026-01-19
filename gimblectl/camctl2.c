// header files 
//-------------------------------------------------------------------------------
//camera interface headers
#include <stdio.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <errno.h>
#include <linux/videodev2.h>
#include <linux/v4l2-controls.h>
#include <unistd.h>
#include <math.h>

//tcp server headers
//-------------------------------------------------------------------------------
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <sys/wait.h>
#include <signal.h>
#include <time.h>

//camera control constants 
//-------------------------------------------------------------------------------
#define tilt_op 0x009a0909
#define pan_op 0x009a0908
#define tilt_speed_op 0x009a0921
#define pan_speed_op 0x009a0920
#define tilt_max 324000
#define tilt_step 3600
#define pan_max 468000
#define pan_step 3600
#define tilt_max_n 90
#define pan_max_n 135
#define x_index 5
#define y_index 15
#define offset 4
//server constants constants
//-------------------------------------------------------------------------------
#define PORT "3490" 
#define BACKLOG 10
#define MAXBUFLEN 100
const char* obsbot_name = "OBSBOT Tiny 2 Lite: OBSBOT Tiny"; //name of the device we wanna find 

//server struct definition
//-------------------------------------------------------------------------------
typedef struct server {
    int sockfd;

    struct addrinfo hints, *servinfo, *p;

    struct sockaddr_storage their_addr;

    struct sigaction sa;

    char s[INET_ADDRSTRLEN];
}   server_t;


//get address function 
//-------------------------------------------------------------------------------
void *get_in_addr(struct sockaddr *sa)
{
    if (sa->sa_family == AF_INET) {
        return &(((struct sockaddr_in*)sa)->sin_addr);
    }

    return &(((struct sockaddr_in6*)sa)->sin6_addr);
}

//signal child handler
//-------------------------------------------------------------------------------
void sigchld_handler(int s)
{
    (void)s; // quiet unused variable warning

    // waitpid() might overwrite errno, so we save and restore it:
    int saved_errno = errno;
    printf("sigchild called");

    while(waitpid(-1, NULL, WNOHANG) > 0);

    errno = saved_errno;
}

//tcp configuration function
//-------------------------------------------------------------------------------


int tcpconfig(server_t * serv) {
    int yes =1; // flag variable
    int rv =0; // return value for get address information

    memset(&serv->hints,0, sizeof serv->hints);

    //configure addrinfo hints
    serv->hints.ai_family = AF_INET;
    serv->hints.ai_socktype = SOCK_STREAM;
    serv->hints.ai_flags = AI_PASSIVE;

    if ((rv = getaddrinfo(NULL, PORT, &serv->hints, &serv->servinfo)) != 0) {
        fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(rv));
        return 1;
    }

    // loop through all the results and bind to the first we can
    for(serv->p = serv->servinfo; serv->p != NULL; serv->p = serv->p->ai_next) {
        if ((serv->sockfd = socket(serv->p->ai_family, serv->p->ai_socktype,
                serv->p->ai_protocol)) == -1) {
            perror("server: socket");
            continue;
        }

        if (setsockopt(serv->sockfd, SOL_SOCKET, SO_REUSEADDR, &yes,
                sizeof(int)) == -1) {
            perror("setsockopt");
            return 1;
        }

        if (bind(serv->sockfd, serv->p->ai_addr, serv->p->ai_addrlen) == -1) {
            close(serv->sockfd);
            perror("server: bind");
            continue;
        }

        break;
    }
}

//camera initialisation function
//-------------------------------------------------------------------------------
/**
 * idea is to just pass a command struct and just have this function interface with ioctl
 */
int camctl(int camfd,struct v4l2_ext_controls * controller , struct v4l2_ext_control* ctl, int count) {
    controller->count = count;
    controller->controls = ctl;
    controller->which = V4L2_CTRL_CLASS_CAMERA;
    if(ioctl(camfd,VIDIOC_S_EXT_CTRLS,controller) < 0) {
        perror("ioctl : camctl");
        return -1;
    }
    return 0;
}

//getting x values from recieved tcp data
//-------------------------------------------------------------------------------
int getX(char * buf) {
    char ss[4];
    strncpy(ss,buf+x_index,4);
    return atoi(ss);
}

//getting y values from recieved tcp data
//-------------------------------------------------------------------------------
int getY(char * buf) {
    char ss[4];
    strncpy(ss,buf+y_index,4);
    return atoi(ss);
}

//detecting and finding cam descriptor
//-------------------------------------------------------------------------------
int get_device(char* videofile) {
    char device_path[20];
    int camfd;
    struct v4l2_capability cap;
    for(int i=0; i< 64; i++) {
        snprintf(device_path,sizeof(device_path),"/dev/video%d",i);
        if((camfd = open(device_path,O_RDONLY | O_NONBLOCK)) == -1) continue;
        if(ioctl(camfd,VIDIOC_QUERYCAP,&cap) == 0) {
            if(!strncmp(cap.card,obsbot_name,sizeof(obsbot_name))) {
                if(cap.device_caps & V4L2_CAP_VIDEO_CAPTURE) {
                    strncpy(videofile,device_path,sizeof(device_path));
                    return 0;
                }
            }
        } 
    }
    return -1;
}

//the main function
//-------------------------------------------------------------------------------
int main(void) {
    int cam = 0; // camera file descriptor (for now at /dev/video2)
    server_t serv; //server struct 
    socklen_t sin_size; //size of the recieved address
    int new_fd; //recieved connection file descriptor
    char buf[MAXBUFLEN]; //recieve buffer
    int x =0, y=0; //tilt and pan coordinates
    int numbytes =0; //number of bytes recieved
    struct timeval tv; //timer for non blocking delays
    long long tim = 0, curr_time = 0; //current time in microseconds

    struct v4l2_ext_controls controller; // extended control struct
    struct v4l2_ext_control tilt_control,pan_control;
    //struct v4l2_ext_control  tilt_speed_control, pan_speed_control; // use when you want to add speed controller

    tilt_control.id = tilt_op;
    pan_control.id = pan_op;
    // pan_speed_control.id = pan_speed_op; //unused
    // tilt_speed_control.id = tilt_speed_op; //unused

    char videofile[20] = {0};
    if(get_device(videofile) == -1) perror("get_device");
    printf("Obsbot found at %s \n",videofile);
    cam = open(videofile,O_RDWR);
    //-------------------------------------------------------------------------------
    //initialisation routine to see if cam is working and setting speed
    //-------------------------------------------------------------------------------
    printf("entering initialisation and test \n");
    tilt_control.value = 60*tilt_step;
    pan_control.value = 50*pan_step;
    struct v4l2_ext_control commands[2] = {tilt_control,pan_control};
    if(camctl(cam,&controller,commands,2) != 0) perror("camctl");
    sleep(2);
    tilt_control.value = 0;
    pan_control.value = 0;
    commands[0] = tilt_control;
    commands[1] = pan_control;
    if(camctl(cam,&controller,commands,2) !=0 ) perror("camctl");

    //-------------------------------------------------------------------------------
    //end of initialisation block
    //-------------------------------------------------------------------------------


    if(tcpconfig(&serv) < 0) {
        perror("tcpconfig");
    }

    freeaddrinfo(serv.servinfo);
    if(listen(serv.sockfd,BACKLOG) == -1) {
        perror("listen");
        exit(1);
    }

    printf("waiting for connections \n");

    while(1) { 
        sin_size = sizeof serv.their_addr;
        new_fd = accept(serv.sockfd, (struct sockaddr *)&serv.their_addr,
            &sin_size);
        if (new_fd == -1) {
            perror("accept");
            continue;
        }

        inet_ntop(serv.their_addr.ss_family,
            get_in_addr((struct sockaddr *)&serv.their_addr),
            serv.s, sizeof serv.s);
        printf("server: got connection from %s\n", serv.s);

        if (!fork()) { // this is the child process
            close(serv.sockfd); // child doesn't need the listener
            //-------------------------------------------------------------------------------
            //all code logic to be inserted from this point
            //-------------------------------------------------------------------------------
            gettimeofday(&tv,NULL);
            tim= (long long)tv.tv_sec*1000000 + tv.tv_usec;
            while((numbytes = recv(new_fd,buf,MAXBUFLEN-1, 0)) > 0) {
                gettimeofday(&tv,NULL);
                curr_time = (long long)tv.tv_sec*1000000 + tv.tv_usec;
                if(curr_time - tim > 100000) {
                    tim = curr_time;
                    //printf("%ld \n",timer);
                    //printf("x = %d, y = %d",x,y);
                    
                    // camctl(cam,&controller,commands,2);
                    x = getX(buf);
                    y = getY(buf);
                    if(!(!x && !y)) {
                        pan_control.value = floor(-(135.0/4096.0)*(x-1918))*pan_step;
                        tilt_control.value = floor((90.0/4096.0)*(y-1918))*tilt_step;
                        commands[0] = pan_control;
                        commands[1] = tilt_control;
                        if(camctl(cam,&controller,commands,2) != 0) perror("camctl (accept)");
                        printf("y = %d \n",y);
                    }
                }

            }
            //-------------------------------------------------------------------------------
            //end of ioctl logic
            //-------------------------------------------------------------------------------
            
            exit(0);
        }
        close(new_fd);  // parent doesn't need this
    }
    return 0;
}


