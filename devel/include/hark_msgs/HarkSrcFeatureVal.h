// Generated by gencpp from file hark_msgs/HarkSrcFeatureVal.msg
// DO NOT EDIT!


#ifndef HARK_MSGS_MESSAGE_HARKSRCFEATUREVAL_H
#define HARK_MSGS_MESSAGE_HARKSRCFEATUREVAL_H


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
struct HarkSrcFeatureVal_
{
  typedef HarkSrcFeatureVal_<ContainerAllocator> Type;

  HarkSrcFeatureVal_()
    : id(0)
    , power(0.0)
    , x(0.0)
    , y(0.0)
    , z(0.0)
    , azimuth(0.0)
    , elevation(0.0)
    , length(0)
    , data_bytes(0)
    , featuredata()  {
    }
  HarkSrcFeatureVal_(const ContainerAllocator& _alloc)
    : id(0)
    , power(0.0)
    , x(0.0)
    , y(0.0)
    , z(0.0)
    , azimuth(0.0)
    , elevation(0.0)
    , length(0)
    , data_bytes(0)
    , featuredata(_alloc)  {
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

   typedef int32_t _length_type;
  _length_type length;

   typedef int32_t _data_bytes_type;
  _data_bytes_type data_bytes;

   typedef std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> _featuredata_type;
  _featuredata_type featuredata;





  typedef boost::shared_ptr< ::hark_msgs::HarkSrcFeatureVal_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::hark_msgs::HarkSrcFeatureVal_<ContainerAllocator> const> ConstPtr;

}; // struct HarkSrcFeatureVal_

typedef ::hark_msgs::HarkSrcFeatureVal_<std::allocator<void> > HarkSrcFeatureVal;

typedef boost::shared_ptr< ::hark_msgs::HarkSrcFeatureVal > HarkSrcFeatureValPtr;
typedef boost::shared_ptr< ::hark_msgs::HarkSrcFeatureVal const> HarkSrcFeatureValConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::hark_msgs::HarkSrcFeatureVal_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::hark_msgs::HarkSrcFeatureVal_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::hark_msgs::HarkSrcFeatureVal_<ContainerAllocator1> & lhs, const ::hark_msgs::HarkSrcFeatureVal_<ContainerAllocator2> & rhs)
{
  return lhs.id == rhs.id &&
    lhs.power == rhs.power &&
    lhs.x == rhs.x &&
    lhs.y == rhs.y &&
    lhs.z == rhs.z &&
    lhs.azimuth == rhs.azimuth &&
    lhs.elevation == rhs.elevation &&
    lhs.length == rhs.length &&
    lhs.data_bytes == rhs.data_bytes &&
    lhs.featuredata == rhs.featuredata;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::hark_msgs::HarkSrcFeatureVal_<ContainerAllocator1> & lhs, const ::hark_msgs::HarkSrcFeatureVal_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace hark_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::hark_msgs::HarkSrcFeatureVal_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::hark_msgs::HarkSrcFeatureVal_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::hark_msgs::HarkSrcFeatureVal_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::hark_msgs::HarkSrcFeatureVal_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hark_msgs::HarkSrcFeatureVal_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hark_msgs::HarkSrcFeatureVal_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::hark_msgs::HarkSrcFeatureVal_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d113c763364c0f932a790285d83f9656";
  }

  static const char* value(const ::hark_msgs::HarkSrcFeatureVal_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd113c763364c0f93ULL;
  static const uint64_t static_value2 = 0x2a790285d83f9656ULL;
};

template<class ContainerAllocator>
struct DataType< ::hark_msgs::HarkSrcFeatureVal_<ContainerAllocator> >
{
  static const char* value()
  {
    return "hark_msgs/HarkSrcFeatureVal";
  }

  static const char* value(const ::hark_msgs::HarkSrcFeatureVal_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::hark_msgs::HarkSrcFeatureVal_<ContainerAllocator> >
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
"int32 length\n"
"int32 data_bytes\n"
"float32[] featuredata\n"
;
  }

  static const char* value(const ::hark_msgs::HarkSrcFeatureVal_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::hark_msgs::HarkSrcFeatureVal_<ContainerAllocator> >
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
      stream.next(m.length);
      stream.next(m.data_bytes);
      stream.next(m.featuredata);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct HarkSrcFeatureVal_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::hark_msgs::HarkSrcFeatureVal_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::hark_msgs::HarkSrcFeatureVal_<ContainerAllocator>& v)
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
    s << indent << "length: ";
    Printer<int32_t>::stream(s, indent + "  ", v.length);
    s << indent << "data_bytes: ";
    Printer<int32_t>::stream(s, indent + "  ", v.data_bytes);
    s << indent << "featuredata[]" << std::endl;
    for (size_t i = 0; i < v.featuredata.size(); ++i)
    {
      s << indent << "  featuredata[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.featuredata[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // HARK_MSGS_MESSAGE_HARKSRCFEATUREVAL_H
