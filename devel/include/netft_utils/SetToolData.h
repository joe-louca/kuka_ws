// Generated by gencpp from file netft_utils/SetToolData.msg
// DO NOT EDIT!


#ifndef NETFT_UTILS_MESSAGE_SETTOOLDATA_H
#define NETFT_UTILS_MESSAGE_SETTOOLDATA_H

#include <ros/service_traits.h>


#include <netft_utils/SetToolDataRequest.h>
#include <netft_utils/SetToolDataResponse.h>


namespace netft_utils
{

struct SetToolData
{

typedef SetToolDataRequest Request;
typedef SetToolDataResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct SetToolData
} // namespace netft_utils


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::netft_utils::SetToolData > {
  static const char* value()
  {
    return "d732082be6860b41d1ad5c3342683b63";
  }

  static const char* value(const ::netft_utils::SetToolData&) { return value(); }
};

template<>
struct DataType< ::netft_utils::SetToolData > {
  static const char* value()
  {
    return "netft_utils/SetToolData";
  }

  static const char* value(const ::netft_utils::SetToolData&) { return value(); }
};


// service_traits::MD5Sum< ::netft_utils::SetToolDataRequest> should match
// service_traits::MD5Sum< ::netft_utils::SetToolData >
template<>
struct MD5Sum< ::netft_utils::SetToolDataRequest>
{
  static const char* value()
  {
    return MD5Sum< ::netft_utils::SetToolData >::value();
  }
  static const char* value(const ::netft_utils::SetToolDataRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::netft_utils::SetToolDataRequest> should match
// service_traits::DataType< ::netft_utils::SetToolData >
template<>
struct DataType< ::netft_utils::SetToolDataRequest>
{
  static const char* value()
  {
    return DataType< ::netft_utils::SetToolData >::value();
  }
  static const char* value(const ::netft_utils::SetToolDataRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::netft_utils::SetToolDataResponse> should match
// service_traits::MD5Sum< ::netft_utils::SetToolData >
template<>
struct MD5Sum< ::netft_utils::SetToolDataResponse>
{
  static const char* value()
  {
    return MD5Sum< ::netft_utils::SetToolData >::value();
  }
  static const char* value(const ::netft_utils::SetToolDataResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::netft_utils::SetToolDataResponse> should match
// service_traits::DataType< ::netft_utils::SetToolData >
template<>
struct DataType< ::netft_utils::SetToolDataResponse>
{
  static const char* value()
  {
    return DataType< ::netft_utils::SetToolData >::value();
  }
  static const char* value(const ::netft_utils::SetToolDataResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // NETFT_UTILS_MESSAGE_SETTOOLDATA_H
