// Generated by gencpp from file rft_sensor_serial/rft_operationRequest.msg
// DO NOT EDIT!


#ifndef RFT_SENSOR_SERIAL_MESSAGE_RFT_OPERATIONREQUEST_H
#define RFT_SENSOR_SERIAL_MESSAGE_RFT_OPERATIONREQUEST_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace rft_sensor_serial
{
template <class ContainerAllocator>
struct rft_operationRequest_
{
  typedef rft_operationRequest_<ContainerAllocator> Type;

  rft_operationRequest_()
    : opType(0)
    , param1(0)
    , param2(0)
    , param3(0)  {
    }
  rft_operationRequest_(const ContainerAllocator& _alloc)
    : opType(0)
    , param1(0)
    , param2(0)
    , param3(0)  {
  (void)_alloc;
    }



   typedef uint8_t _opType_type;
  _opType_type opType;

   typedef uint8_t _param1_type;
  _param1_type param1;

   typedef uint8_t _param2_type;
  _param2_type param2;

   typedef uint8_t _param3_type;
  _param3_type param3;





  typedef boost::shared_ptr< ::rft_sensor_serial::rft_operationRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::rft_sensor_serial::rft_operationRequest_<ContainerAllocator> const> ConstPtr;

}; // struct rft_operationRequest_

typedef ::rft_sensor_serial::rft_operationRequest_<std::allocator<void> > rft_operationRequest;

typedef boost::shared_ptr< ::rft_sensor_serial::rft_operationRequest > rft_operationRequestPtr;
typedef boost::shared_ptr< ::rft_sensor_serial::rft_operationRequest const> rft_operationRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::rft_sensor_serial::rft_operationRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::rft_sensor_serial::rft_operationRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::rft_sensor_serial::rft_operationRequest_<ContainerAllocator1> & lhs, const ::rft_sensor_serial::rft_operationRequest_<ContainerAllocator2> & rhs)
{
  return lhs.opType == rhs.opType &&
    lhs.param1 == rhs.param1 &&
    lhs.param2 == rhs.param2 &&
    lhs.param3 == rhs.param3;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::rft_sensor_serial::rft_operationRequest_<ContainerAllocator1> & lhs, const ::rft_sensor_serial::rft_operationRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace rft_sensor_serial

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::rft_sensor_serial::rft_operationRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::rft_sensor_serial::rft_operationRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::rft_sensor_serial::rft_operationRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::rft_sensor_serial::rft_operationRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::rft_sensor_serial::rft_operationRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::rft_sensor_serial::rft_operationRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::rft_sensor_serial::rft_operationRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "76ad4c31d463fda0980c1dc1fd97aa8b";
  }

  static const char* value(const ::rft_sensor_serial::rft_operationRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x76ad4c31d463fda0ULL;
  static const uint64_t static_value2 = 0x980c1dc1fd97aa8bULL;
};

template<class ContainerAllocator>
struct DataType< ::rft_sensor_serial::rft_operationRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "rft_sensor_serial/rft_operationRequest";
  }

  static const char* value(const ::rft_sensor_serial::rft_operationRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::rft_sensor_serial::rft_operationRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint8 opType\n"
"uint8 param1\n"
"uint8 param2\n"
"uint8 param3\n"
;
  }

  static const char* value(const ::rft_sensor_serial::rft_operationRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::rft_sensor_serial::rft_operationRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.opType);
      stream.next(m.param1);
      stream.next(m.param2);
      stream.next(m.param3);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct rft_operationRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::rft_sensor_serial::rft_operationRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::rft_sensor_serial::rft_operationRequest_<ContainerAllocator>& v)
  {
    s << indent << "opType: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.opType);
    s << indent << "param1: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.param1);
    s << indent << "param2: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.param2);
    s << indent << "param3: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.param3);
  }
};

} // namespace message_operations
} // namespace ros

#endif // RFT_SENSOR_SERIAL_MESSAGE_RFT_OPERATIONREQUEST_H
