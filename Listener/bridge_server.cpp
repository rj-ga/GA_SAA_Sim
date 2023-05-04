/*
@author: Dhruv Parikh
@organisation: General Aeronautics
@Description: Bridge server for getting data from gazebo to send to python using sockets
References: Help taken from GeekforGeeks website for socket programming. 
*/

#include <unistd.h>
#include <stdio.h>
#include <sys/socket.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <string.h>

#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>
#include <unistd.h>

#include<typeinfo>
#include <iostream>
#include<string>


#define PORT 8080


float ranges[64];

void cb(ConstLaserScanStampedPtr &_msg)
{

  float rang;
  for(int i=0;i<=63;i++){
    rang = _msg->scan().ranges(i);  

    if(rang<38 && rang>1)
    {

      ranges[i] = rang;
    }  
    else{ranges[i] = 40.0;}
  }
}


int main( int _argc, char **_argv)
{

    int server_fd, new_socket, valread;
    struct sockaddr_in address;
    int opt = 1;
    int addrlen = sizeof(address);

      


    // Creating socket file descriptor
    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0)
    {
        perror("socket failed");
        exit(EXIT_FAILURE);
    }
       
    // Forcefully attaching socket to the port 8080
    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT,
                                                  &opt, sizeof(opt)))
    {
        perror("setsockopt");
        exit(EXIT_FAILURE);
    }
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons( PORT );
       
    // Forcefully attaching socket to the port 8080
    if (bind(server_fd, (struct sockaddr *)&address, 
                                 sizeof(address))<0)
    {
        perror("bind failed");
        exit(EXIT_FAILURE);
    }
    if (listen(server_fd, 3) < 0)
    {
        perror("listen");
        exit(EXIT_FAILURE);
    }
    if ((new_socket = accept(server_fd, (struct sockaddr *)&address, 
                       (socklen_t*)&addrlen))<0)
    {
        perror("accept");
        exit(EXIT_FAILURE);
    }

      // Load gazebo
  gazebo::client::setup(_argc, _argv);

  // Create our node for communication
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();

  // Listen to Gazebo world_stats topic
  gazebo::transport::SubscriberPtr sub = node->Subscribe("~/iris/hokuyo_link/laser/scan", cb);

  // Busy wait loop...replace with your own code as needed.
  std::string conv_rang, conv_i;
  while (true){
 
    for(int i=0;i<64;i++){
        conv_rang = std::to_string(ranges[i])+"\n";
        conv_i = std::to_string(i) + "\n";
        send(new_socket , conv_rang.c_str() , 4 , 0 );
    }
    sleep(0.7);
    send(new_socket , "new " , 4 , 0 );
    
    gazebo::common::Time::MSleep(20);//Changed the sleep from 10 to 20 to test

  }

  // Make sure to shut everything down.
  gazebo::client::shutdown();

    return 0;
}
