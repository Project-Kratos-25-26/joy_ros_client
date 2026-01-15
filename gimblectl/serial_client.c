// C library headers
#include <stdio.h>
#include <string.h>

// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()


// http headers
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

#define PORT "3490" // the port client will be connecting to 

#define MAXDATASIZE 100 // max number of bytes we can get at once 

//http headers end

// for my mental sanity i will use a struct for socket
struct client {
  int sockfd;
  char buf[MAXDATASIZE];
  struct addrinfo hints,*servinfo,*p;
  char s[INET_ADDRSTRLEN];
} client_t;



int serial_config(int serial_port, struct termios *tty) {
    if(tcgetattr(serial_port, tty) != 0) {
        printf("%d\n",errno);
        perror("get attribute error");
      return 1;
    }
     // Control Modes (c_cflag)
          tty->c_cflag &= ~PARENB;        // Clear parity bit, disabling parity
          tty->c_cflag &= ~CSTOPB;        // Clear stop field, only one stop bit used
          tty->c_cflag &= ~CSIZE;         // Clear all bits that set the data size
          tty->c_cflag |= CS8;            // 8 bits per byte
          //tty->c_cflag &= ~CRTSCTS;       // Disable RTS/CTS hardware flow control
          tty->c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines

          // Local Modes (c_lflag)
          tty->c_lflag &= ~ICANON;        // Disable canonical mode (use raw mode)
          tty->c_lflag &= ~ECHO;          // Disable echo
          tty->c_lflag &= ~ECHOE;         // Disable erasure
          tty->c_lflag &= ~ECHONL;        // Disable new-line echo
          tty->c_lflag &= ~ISIG;          // Disable interpretation of INTR, QUIT and SUSP

          // Input Modes (c_iflag)
          tty->c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
          tty->c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL); // Disable special handling of received bytes

          // Output Modes (c_oflag)
          tty->c_oflag &= ~OPOST;         // Prevent special interpretation of output bytes
          tty->c_oflag &= ~ONLCR;         // Prevent conversion of newline to carriage return/line feed

            tty->c_cc[VTIME] = 255;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
            tty->c_cc[VMIN] = 0;

            cfsetispeed(tty, B115200);
            cfsetospeed(tty, B115200);

            if (tcsetattr(serial_port, TCSANOW, tty) != 0) {
                  perror("save tty settings error");
                return 1;
            }

          return 0;
}
void *get_in_addr(struct sockaddr *sa) {
    if (sa->sa_family == AF_INET) {
        return &(((struct sockaddr_in*)sa)->sin_addr);
    }
    return &(((struct sockaddr_in6*)sa)->sin6_addr);
}
int client_setup(struct client *cli, char *hostname) {
    int rv;

    memset(&cli->hints, 0, sizeof cli->hints);
    cli->hints.ai_family = AF_UNSPEC;
    cli->hints.ai_socktype = SOCK_STREAM;

    if ((rv = getaddrinfo(hostname, PORT, &cli->hints, &cli->servinfo)) != 0) {
        fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(rv));
        return 1;
    }

    // Loop through all the results and connect to the first we can
    for(cli->p = cli->servinfo; cli->p != NULL; cli->p = cli->p->ai_next) {
        if ((cli->sockfd = socket(cli->p->ai_family, cli->p->ai_socktype,
                cli->p->ai_protocol)) == -1) {
            perror("client: socket");
            continue;
        }

        if (connect(cli->sockfd, cli->p->ai_addr, cli->p->ai_addrlen) == -1) {
            close(cli->sockfd);
            perror("client: connect");
            continue;
        }

        break; // If we get here, we must have connected successfully
    }

    if (cli->p == NULL) {
        fprintf(stderr, "client: failed to connect\n");
        freeaddrinfo(cli->servinfo);
        return 2;
    }

    // Convert the IP to a string for display purposes
    inet_ntop(cli->p->ai_family, get_in_addr((struct sockaddr *)cli->p->ai_addr),
              cli->s, sizeof cli->s);
    
    printf("client: connected to %s\n", cli->s);

    freeaddrinfo(cli->servinfo); // All done with this structure
    return 0;
}


int main(void) {
    int serial_port = open("/dev/ttyUSB0",O_RDWR);
    struct termios tty;
    char buf[256];
    struct client cli;
    char *hostname = "127.0.0.1";
    
    if(serial_config(serial_port,&tty)!=0) {
        perror("serial config error");
    }

    if (client_setup(&cli, hostname) != 0) {
        exit(1);
    }

    int numbytes;
    memset(&buf,'\0',sizeof buf);
    while(1) {
          if((numbytes = read(serial_port,&buf,sizeof buf)) < 0) {
            perror("read");
          }
          if(send(cli.sockfd,buf,sizeof buf,0) != 0) {
            perror("send");
          }
          printf("Read %i bytes. Received message: %s", numbytes, buf);
          memset(buf, 0, sizeof buf);
  }

    return 0;
}