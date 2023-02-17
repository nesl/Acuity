// Generated by gencpp from file hark_msgs/HarkSourceVal.msg
// DO NOT EDIT!


#ifndef HARK_MSGS_MESSAGE_HARKSOURCEVAL_H
#define HARK_MSGS_MESSAGE_HARKSOURCEVAL_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace hark_msgs
{
template <class ContainerAllocator>
struct HarkSourceVal_
{
  typedef HarkSourceVal_<ContainerAllocator> Type;

  HarkSourceVal_()
    : id(0)
    , power(0.0)
    , x(0.0)
    , y(0.0)
    , z(0.0)
    , azimuth(0.0)
    , elevation(0.0)  {
    }
  HarkSourceVal_(const ContainerAllocator& _alloc)
    : id(0)
    , power(0.0)
    , x(0.0)
    , y(0.0)
    , z(0.0)
    , azimuth(0.0)
    , elevation(0.0)  {
  (void)_alloc;
    }



   typedef int32_t _id_type;
  _id_type id;

   typedef float _power_type;
  _power_type power;

   typedef float _x_type;
  _x_type x;

   typedef float _y_type;
  _y_type y;

   typedef float _z_type;
  _z_type z;

   typedef float _azimuth_type;
  _azimuth_type azimuth;

   typedef float _elevation_type;
  _elevation_type elevation;





  typedef boost::shared_ptr< ::hark_msgs::HarkSourceVal_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::hark_msgs::HarkSourceVal_<ContainerAllocator> const> ConstPtr;

}; // struct HarkSourceVal_

typedef ::hark_msgs::HarkSourceVal_<std::allocator<void> > HarkSourceVal;

typedef boost::shared_ptr< ::hark_msgs::HarkSourceVal > HarkSourceValPtr;
typedef boost::shared_ptr< ::hark_msgs::HarkSourceVal const> HarkSourceValConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::hark_msgs::HarkSourceVal_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::hark_msgs::HarkSourceVal_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::hark_msgs::HarkSourceVal_<ContainerAllocator1> & lhs, const ::hark_msgs::HarkSourceVal_<ContainerAllocator2> & rhs)
{
  return lhs.id == rhs.id &&
    lhs.power == rhs.power &&
    lhs.x == rhs.x &&
    lhs.y == rhs.y &&
    lhs.z == rhs.z &&
    lhs.azimuth == rhs.azimuth &&
    lhs.elevation == rhs.elevation;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::hark_msgs::HarkSourceVal_<ContainerAllocator1> & lhs, const ::hark_msgs::HarkSourceVal_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace hark_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::hark_msgs::HarkSourceVal_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::hark_msgs::HarkSourceVal_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::hark_msgs::HarkSourceVal_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::hark_msgs::HarkSourceVal_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hark_msgs::HarkSourceVal_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hark_msgs::HarkSourceVal_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::hark_msgs::HarkSourceVal_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ee0b7cc36255925b0a96b74055ee462f";
  }

  static const char* value(const ::hark_msgs::HarkSourceVal_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xee0b7cc36255925bULL;
  static const uint64_t static_value2 = 0x0a96b74055ee462fULL;
};

template<class ContainerAllocator>
struct DataType< ::hark_msgs::HarkSourceVal_<ContainerAllocator> >
{
  static const char* value()
  {
    return "hark_msgs/HarkSourceVal";
  }

  static const char* value(const ::hark_msgs::HarkSourceVal_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::hark_msgs::HarkSourceVal_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int32 id\n"
"float32 power\n"
"float32 x\n"
"float32 y\n"
"float32 z\n"
"float32 azimuth\n"
"float32 elevation\n"
;
  }

  static const char* value(const ::hark_msgs::HarkSourceVal_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::hark_msgs::HarkSourceVal_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.id);
      stream.next(m.power);
      stream.next(m.x);
      stream.next(m.y);
      stream.next(m.z);
      stream.next(m.azimuth);
      stream.next(m.elevation);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct HarkSourceVal_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::hark_msgs::HarkSourceVal_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::hark_msgs::HarkSourceVal_<ContainerAllocator>& v)
  {
    s << indent << "id: ";
    Printer<int32_t>::stream(s, indent + "  ", v.id);
    s << indent << "power: ";
    Printer<float>::stream(s, indent + "  ", v.power);
    s << indent << "x: ";
    Printer<float>::stream(s, indent + "  ", v.x);
    s << indent << "y: ";
    Printer<float>::stream(s, indent + "  ", v.y);
    s << indent << "z: ";
    Printer<float>::stream(s, indent + "  ", v.z);
    s << indent << "azimuth: ";
    Printer<float>::stream(s, indent + "  ", v.azimuth);
    s << indent << "elevation: ";
    Printer<float>::stream(s, indent + "  ", v.elevation);
  }
};

} // namespace message_operations
} // namespace ros

#endif // HARK_MSGS_MESSAGE_HARKSOURCEVAL_H
