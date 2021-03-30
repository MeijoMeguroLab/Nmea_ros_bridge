/*
 * nmea_tcp.cpp
 *
 * Author Kashimoto
 * Ver 1.00 2021/03/19
 */
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "ros/ros.h"
#include "nmea_msgs/Sentence.h"

#define BUFFER_SAFE 2000

struct sockaddr_in dstAddr;
static ros::Publisher pub;
static bool isConnected = false;

static void packet_receive_rate(int fd, std::string frame_id, double rate)
{
  nmea_msgs::Sentence nmea_msg;
  nmea_msg.header.frame_id = frame_id.c_str();
  ros::Rate loop_rate(rate);

  int rem;
  int ret;
  char buffer[2048];
  char* w_buffer = buffer;
  char* buffer_end = &buffer[sizeof(buffer)];

  while (ros::ok())
  {
    errno = 0;
    ret = read(fd, w_buffer, buffer_end - w_buffer - 1);
    if (ret > 0)
    {
      if (strnlen(w_buffer, ret) != ret)
      {
        w_buffer = buffer;
        continue;
      }
      w_buffer += ret;
    }
    else if (ret == 0)
    {
      pub.shutdown();
      close(fd);
      return;
    }
    else
    {
      if (errno == EAGAIN)
      {
        continue;
      }
      else
      {
        ros::shutdown();
      }
    }

    *w_buffer = '\0';
    char* r_buffer = buffer;

    ros::Time now = ros::Time::now();
    while (1)
    {
      char* sentence = strchr(r_buffer, '$');
      if (sentence == NULL)
      {
        break;
      }
      char* end_sentence = strchr(sentence, '\r');
      if (end_sentence == NULL)
      {
        break;
      }
      *end_sentence = '\0';

      nmea_msg.header.stamp = now;
      nmea_msg.sentence = sentence;
      pub.publish(nmea_msg);
      r_buffer = end_sentence + 1;
    }

    rem = w_buffer - r_buffer;
    if (rem > BUFFER_SAFE)
    {
      ROS_WARN("Buffer over.Init Buffer.");
      rem = 0;
    }
    memmove(buffer, r_buffer, rem);
    w_buffer = buffer + rem;

    ros::spinOnce();
    loop_rate.sleep();
  }
  close(fd);
}

static void packet_receive_no_rate(int fd, std::string frame_id)
{
  nmea_msgs::Sentence nmea_msg;
  nmea_msg.header.frame_id = frame_id.c_str();

  int rem;
  int ret;
  char buffer[2048];
  char* w_buffer = buffer;
  char* buffer_end = &buffer[sizeof(buffer)];

  while (ros::ok())
  {
    errno = 0;
    ret = read(fd, w_buffer, buffer_end - w_buffer - 1);
    if (ret > 0)
    {
      if (strnlen(w_buffer, ret) != ret)
      {
        w_buffer = buffer;
        continue;
      }
      w_buffer += ret;
    }
    else if (ret == 0)
    {
      pub.shutdown();
      close(fd);
      return;
    }
    else
    {
      if (errno == EAGAIN)
      {
        continue;
      }
      else
      {
        ros::shutdown();
      }
    }

    *w_buffer = '\0';
    char* r_buffer = buffer;

    ros::Time now = ros::Time::now();
    while (1)
    {
      char* sentence = strchr(r_buffer, '$');
      if (sentence == NULL)
      {
        break;
      }
      char* end_sentence = strchr(sentence, '\r');
      if (end_sentence == NULL)
      {
        break;
      }
      *end_sentence = '\0';

      nmea_msg.header.stamp = now;
      nmea_msg.sentence = sentence;
      pub.publish(nmea_msg);
      r_buffer = end_sentence + 1;
    }

    rem = w_buffer - r_buffer;
    if (rem > BUFFER_SAFE)
    {
      ROS_WARN("Buffer over.Init Buffer.");
      rem = 0;
    }
    memmove(buffer, r_buffer, rem);
    w_buffer = buffer + rem;

    ros::spinOnce();

  }
  close(fd);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "nmea_tcp");
  ros::NodeHandle node_handle_;

  int port,sock;
  std::string address, nmea_topic, frame_id;
  int result,val;
  double rate;

  // Read parameters
  node_handle_.param("/nmea_udp/address", address, std::string("127.0.0.1"));
  node_handle_.param("/nmea_udp/port", port, 28003);
  node_handle_.param("/nmea_udp/nmea_topic", nmea_topic, std::string("nmea_sentence"));
  node_handle_.param("/nmea_udp/frame_id", frame_id, std::string("sentence"));
  node_handle_.param("/nmea_udp/rate", rate, 0.0);

  pub = node_handle_.advertise<nmea_msgs::Sentence>(nmea_topic, 10);

  ROS_INFO("IP: %s", address.c_str());
  ROS_INFO("PORT: %d", port);
  ROS_INFO("RATE: %.1lf", rate);

  sock = socket(AF_INET, SOCK_DGRAM, 0);

  memset(&dstAddr, 0, sizeof(dstAddr));
  dstAddr.sin_family = AF_INET;
  dstAddr.sin_addr.s_addr = INADDR_ANY;
  dstAddr.sin_port = htons(port);

  ros::Rate loop_rate(1.0);
  while (ros::ok())
  {
    //result = connect(sock, (struct sockaddr *) &dstAddr, sizeof(dstAddr));
    result = bind(sock, (struct sockaddr *)&dstAddr, sizeof(dstAddr));
    if( result < 0 )
    {
      /* non-connect */
      ROS_INFO("CONNECT TRY...");
    }
    else
    {
      /* connect */
      isConnected = true;
      break;
    }
    ros::spinOnce();
    loop_rate.sleep();
  }

  if ((isConnected == true)&&(ros::ok()))
  {
    if( rate != 0.0 )
    {
      /* non-block *//* ros::rate() */
      val = 1;
      ioctl(sock, FIONBIO, &val);
      packet_receive_rate(sock, frame_id, rate);
    }
    else
    {
      /* block */
      packet_receive_no_rate(sock, frame_id);
    }

  }

  return 0;
}
