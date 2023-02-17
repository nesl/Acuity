// Generated by gencpp from file NESLMessages/NeslCoord.msg
// DO NOT EDIT!


#ifndef NESLMESSAGES_MESSAGE_NESLCOORD_H
#define NESLMESSAGES_MESSAGE_NESLCOORD_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace NESLMessages
{
template <class ContainerAllocator>
struct NeslCoord_
{
  typedef NeslCoord_<ContainerAllocator> Type;

  NeslCoord_()
    : x(0.0)
    , y(0.0)
    , z(0.0)  {
    }
  NeslCoord_(const ContainerAllocator& _alloc)
    : x(0.0)
    , y(0.0)
    , z(0.0)  {
  (void)_alloc;
    }



   typedef double _x_type;
  _x_type x;

   typedef double _y_type;
  _y_type y;

   typedef double _z_type;
  _z_type z;





  typedef boost::shared_ptr< ::NESLMessages::NeslCoord_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::NESLMessages::NeslCoord_<ContainerAllocator> const> ConstPtr;

}; // struct NeslCoord_

typedef ::NESLMessages::NeslCoord_<std::allocator<void> > NeslCoord;

typedef boost::shared_ptr< ::NESLMessages::NeslCoord > NeslCoordPtr;
typedef boost::shared_ptr< ::NESLMessages::NeslCoord const> NeslCoordConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::NESLMessages::NeslCoord_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::NESLMessages::NeslCoord_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::NESLMessages::NeslCoord_<ContainerAllocator1> & lhs, const ::NESLMessages::NeslCoord_<ContainerAllocator2> & rhs)
{
  return lhs.x == rhs.x &&
    lhs.y == rhs.y &&
    lhs.z == rhs.z;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::NESLMessages::NeslCoord_<ContainerAllocator1> & lhs, const ::NESLMessages::NeslCoord_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace NESLMessages

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::NESLMessages::NeslCoord_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::NESLMessages::NeslCoord_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::NESLMessages::NeslCoord_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::NESLMessages::NeslCoord_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::NESLMessages::NeslCoord_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::NESLMessages::NeslCoord_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::NESLMessages::NeslCoord_<ContainerAllocator> >
{
  static const char* value()
  {
    return "4a842b65f413084dc2b10fb484ea7f17";
  }

  static const char* value(const ::NESLMessages::NeslCoord_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x4a842b65f413084dULL;
  static const uint64_t static_value2 = 0xc2b10fb484ea7f17ULL;
};

template<class ContainerAllocator>
struct DataType< ::NESLMessages::NeslCoord_<ContainerAllocator> >
{
  static const char* value()
  {
    return "NESLMessages/NeslCoord";
  }

  static const char* value(const ::NESLMessages::NeslCoord_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::NESLMessages::NeslCoord_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float64 x\n"
"float64 y\n"
"float64 z\n"
;
  }

  static const char* value(const ::NESLMessages::NeslCoord_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::NESLMessages::NeslCoord_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.x);
      stream.next(m.y);
      stream.next(m.z);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct NeslCoord_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::NESLMessages::NeslCoord_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::NESLMessages::NeslCoord_<ContainerAllocator>& v)
  {
    s << indent << "x: ";
    Printer<double>::stream(s, indent + "  ", v.x);
    s << indent << "y: ";
    Printer<double>::stream(s, indent + "  ", v.y);
    s << indent << "z: ";
    Printer<double>::stream(s, indent + "  ", v.z);
  }
};

} // namespace message_operations
} // namespace ros

#endif // NESLMESSAGES_MESSAGE_NESLCOORD_H