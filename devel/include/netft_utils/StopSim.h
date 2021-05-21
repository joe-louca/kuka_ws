// Generated by gencpp from file netft_utils/StopSim.msg
// DO NOT EDIT!


#ifndef NETFT_UTILS_MESSAGE_STOPSIM_H
#define NETFT_UTILS_MESSAGE_STOPSIM_H

#include <ros/service_traits.h>


#include <netft_utils/StopSimRequest.h>
#include <netft_utils/StopSimResponse.h>


namespace netft_utils
{

struct StopSim
{

typedef StopSimRequest Request;
typedef StopSimResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct StopSim
} // namespace netft_utils


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::netft_utils::StopSim > {
  static const char* value()
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const ::netft_utils::StopSim&) { return value(); }
};

template<>
struct DataType< ::netft_utils::StopSim > {
  static const char* value()
  {
    return "netft_utils/StopSim";
  }

  static const char* value(const ::netft_utils::StopSim&) { return value(); }
};


// service_traits::MD5Sum< ::netft_utils::StopSimRequest> should match
// service_traits::MD5Sum< ::netft_utils::StopSim >
template<>
struct MD5Sum< ::netft_utils::StopSimRequest>
{
  static const char* value()
  {
    return MD5Sum< ::netft_utils::StopSim >::value();
  }
  static const char* value(const ::netft_utils::StopSimRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::netft_utils::StopSimRequest> should match
// service_traits::DataType< ::netft_utils::StopSim >
template<>
struct DataType< ::netft_utils::StopSimRequest>
{
  static const char* value()
  {
    return DataType< ::netft_utils::StopSim >::value();
  }
  static const char* value(const ::netft_utils::StopSimRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::netft_utils::StopSimResponse> should match
// service_traits::MD5Sum< ::netft_utils::StopSim >
template<>
struct MD5Sum< ::netft_utils::StopSimResponse>
{
  static const char* value()
  {
    return MD5Sum< ::netft_utils::StopSim >::value();
  }
  static const char* value(const ::netft_utils::StopSimResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::netft_utils::StopSimResponse> should match
// service_traits::DataType< ::netft_utils::StopSim >
template<>
struct DataType< ::netft_utils::StopSimResponse>
{
  static const char* value()
  {
    return DataType< ::netft_utils::StopSim >::value();
  }
  static const char* value(const ::netft_utils::StopSimResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // NETFT_UTILS_MESSAGE_STOPSIM_H
