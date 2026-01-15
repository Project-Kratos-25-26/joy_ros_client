#include "main.hpp"

void *get_in_addr(struct sockaddr *sa) {
    if (sa->sa_family == AF_INET) {
        return &(((struct sockaddr_in*)sa)->sin_addr);
    }
    return &(((struct sockaddr_in6*)sa)->sin6_addr);
}

int client_setup(struct client *cli, const char *hostname) {
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


class Subscriber : public rclcpp::Node
{
  public:
    Subscriber()
    : Node("minimal_subscriber")
    {
      subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      topic, 10, std::bind(&Subscriber::topic_callback, this, _1));
      if(client_setup(&cli,hostname) < 0) perror("client setup");
    }

  private:
    char buf[100];
    void topic_callback(const std_msgs::msg::Float32MultiArray & msg) const
    {
      float x = msg.data[pan_index];
      float y = msg.data[tilt_index];
      sprintf((char *)buf,"{x : %04d, y : %04d}",angleto12bit(x),angleto12bit(y));
      if(send(cli.sockfd,buf,sizeof buf,0) < 0) perror("send");
      memset((char *)buf,0,sizeof buf);
    }
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_;
    struct client cli;
    const char* hostname = sendaddr;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Subscriber>());
  rclcpp::shutdown();
  return 0;
}
