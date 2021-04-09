/*
 * nmea_serial.cpp
 *
 * Author Kashimoto
 * Ver 1.00 2021/04/09
 */
#include <fcntl.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <linux/serial.h>

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
    if (ret >= 0)
    {
      if (strnlen(w_buffer, ret) != ret)
      {
        w_buffer = buffer;
        continue;
      }
      w_buffer += ret;
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
    if (ret >= 0)
    {
      if (strnlen(w_buffer, ret) != ret)
      {
        w_buffer = buffer;
        continue;
      }
      w_buffer += ret;
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

int open_serial(const char *device_name, int baud){
  speed_t baudrate;

  switch (baud)
  {
  case 9600:
    baudrate = B9600;
    break;
  case 19200:
    baudrate = B19200;
    break;
  case 38400:
    baudrate = B38400;
    break;
  case 57600:
    baudrate = B57600;
    break;
  case 115200:
    baudrate = B115200;
    break;
  default:
    ROS_WARN("Serial BundRate Error!");
    return -1;
  }


  struct termios tio;
  int fd =open(device_name, O_RDWR | O_NOCTTY | O_NDELAY);
  if( -1 == fd ){
    perror("open");
    return fd;
  }
  struct termios serial;

  //Serial Configuration
  serial.c_iflag = 0;
  serial.c_oflag = 0;
  serial.c_lflag = 0;
  serial.c_cflag = 0;
  serial.c_cc[VMIN] = 0;
  serial.c_cc[VTIME] = 0;

  serial.c_cflag |= baudrate;
  serial.c_cflag |= CS8;
  // 1 stop bit default
  // no parity
  tcsetattr(fd, TCSANOW, &serial);

  struct serial_struct serial_setting;
  ioctl(fd, TIOCGSERIAL, &serial_setting);
  serial_setting.flags |= ASYNC_LOW_LATENCY;
  ioctl(fd, TIOCSSERIAL, &serial_setting);

  return fd;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "nmea_serial");
  ros::NodeHandle node_handle_;

  int baud,sock;
  std::string port, nmea_topic, frame_id;
  int val;
  double rate;

  // Read parameters
  node_handle_.param("/nmea_serial/port", port, std::string("/dev/ttyUSB0"));
  node_handle_.param("/nmea_serial/baud", baud, 115200);
  node_handle_.param("/nmea_serial/nmea_topic", nmea_topic, std::string("nmea_sentence"));
  node_handle_.param("/nmea_serial/frame_id", frame_id, std::string("sentence"));
  node_handle_.param("/nmea_serial/rate", rate, 0.0);

  pub = node_handle_.advertise<nmea_msgs::Sentence>(nmea_topic, 1);

  ROS_INFO("PORT: %s", port.c_str());
  ROS_INFO("BUND: %d", baud);
  ROS_INFO("RATE: %.1lf", rate);

//  ros::Rate loop_rate(1.0);

  sock = open_serial(port.c_str(), baud);
  if( sock < 0 )
  {
    /* non-connect */
    ROS_WARN("Serial Open Failed! %s\n Check Device File and Check Permission (R/W)", port.c_str());
    isConnected = false;
  }
  else
  {
    /* connect */
    ROS_INFO("Conneact.");
    isConnected = true;
  }

  if ((isConnected == true)&&(ros::ok()))
  {
    if( rate != 0.0 )
    {
      /* non-block *//* ros::rate() */
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
