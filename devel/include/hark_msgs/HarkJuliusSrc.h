// Generated by gencpp from file hark_msgs/HarkJuliusSrc.msg
// DO NOT EDIT!


#ifndef HARK_MSGS_MESSAGE_HARKJULIUSSRC_H
#define HARK_MSGS_MESSAGE_HARKJULIUSSRC_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <hark_msgs/HarkJuliusSrcVal.h>

namespace hark_msgs
{
template <class ContainerAllocator>
struct HarkJuliusSrc_
{
  typedef HarkJuliusSrc_<ContainerAllocator> Type;

  HarkJuliusSrc_()
    : id(0)
    , azimuth(0.0)
    , elevation(0.0)
    , sec(0)
    , usec(0)
    , frames(0)
    , msec(0)
    , status(0)
    , src()  {
    }
  HarkJuliusSrc_(const ContainerAllocator& _alloc)
    : id(0)
    , azimuth(0.0)
    , elevation(0.0)
    , sec(0)
    , usec(0)
    , frames(0)
    , msec(0)
    , status(0)
    , src(_alloc)  {
  (void)_alloc;
    }



   typedef int32_t _id_type;
  _id_type id;

   typedef float _azimuth_type;
  _azimuth_type azimuth;

   typedef float _elevation_type;
  _elevation_type elevation;

   typedef int32_t _sec_type;
  _sec_type sec;

   typedef int32_t _usec_type;
  _usec_type usec;

   typedef int32_t _frames_type;
  _frames_type frames;

   typedef int32_t _msec_type;
  _msec_type msec;

   typedef int8_t _status_type;
  _status_type status;

   typedef std::vector< ::hark_msgs::HarkJuliusSrcVal_<ContainerAllocator> , typename std::allocator_traits<ContainerAllocator>::template rebind_alloc< ::hark_msgs::HarkJuliusSrcVal_<ContainerAllocator> >> _src_type;
  _src_type src;



// reducing the odds to have name collisions with Windows.h 
#if defined(_WIN32) && defined(TERMINATED)
  #undef TERMINATED
#endif
#if defined(_WIN32) && defined(STARTRECOG)
  #undef STARTRECOG
#endif
#if defined(_WIN32) && defined(INPUTPARAM)
  #undef INPUTPARAM
#endif
#if defined(_WIN32) && defined(SOURCEINFO)
  #undef SOURCEINFO
#endif
#if defined(_WIN32) && defined(RECOGOUT)
  #undef RECOGOUT
#endif
#if defined(_WIN32) && defined(RECOGFAIL)
  #undef RECOGFAIL
#endif

  enum {
    TERMINATED = 0,
    STARTRECOG = 1,
    INPUTPARAM = 2,
    SOURCEINFO = 3,
    RECOGOUT = 4,
    RECOGFAIL = 5,
  };


  typedef boost::shared_ptr< ::hark_msgs::HarkJuliusSrc_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::hark_msgs::HarkJuliusSrc_<ContainerAllocator> const> ConstPtr;

}; // struct HarkJuliusSrc_

typedef ::hark_msgs::HarkJuliusSrc_<std::allocator<void> > HarkJuliusSrc;

typedef boost::shared_ptr< ::hark_msgs::HarkJuliusSrc > HarkJuliusSrcPtr;
typedef boost::shared_ptr< ::hark_msgs::HarkJuliusSrc const> HarkJuliusSrcConstPtr;

// constants requiring out of line definition

   

   

   

   

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::hark_msgs::HarkJuliusSrc_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::hark_msgs::HarkJuliusSrc_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::hark_msgs::HarkJuliusSrc_<ContainerAllocator1> & lhs, const ::hark_msgs::HarkJuliusSrc_<ContainerAllocator2> & rhs)
{
  return lhs.id == rhs.id &&
    lhs.azimuth == rhs.azimuth &&
    lhs.elevation == rhs.elevation &&
    lhs.sec == rhs.sec &&
    lhs.usec == rhs.usec &&
    lhs.frames == rhs.frames &&
    lhs.msec == rhs.msec &&
    lhs.status == rhs.status &&
    lhs.src == rhs.src;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::hark_msgs::HarkJuliusSrc_<ContainerAllocator1> & lhs, const ::hark_msgs::HarkJuliusSrc_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace hark_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::hark_msgs::HarkJuliusSrc_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::hark_msgs::HarkJuliusSrc_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::hark_msgs::HarkJuliusSrc_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::hark_msgs::HarkJuliusSrc_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hark_msgs::HarkJuliusSrc_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hark_msgs::HarkJuliusSrc_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::hark_msgs::HarkJuliusSrc_<ContainerAllocator> >
{
  static const char* value()
  {
    return "1aff8127ead3951895e7fd2d0cb19ac9";
  }

  static const char* value(const ::hark_msgs::HarkJuliusSrc_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x1aff8127ead39518ULL;
  static const uint64_t static_value2 = 0x95e7fd2d0cb19ac9ULL;
};

template<class ContainerAllocator>
struct DataType< ::hark_msgs::HarkJuliusSrc_<ContainerAllocator> >
{
  static const char* value()
  {
    return "hark_msgs/HarkJuliusSrc";
  }

  static const char* value(const ::hark_msgs::HarkJuliusSrc_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::hark_msgs::HarkJuliusSrc_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int32 id\n"
"float32 azimuth\n"
"float32 elevation\n"
"int32 sec\n"
"int32 usec\n"
"int32 frames\n"
"int32 msec\n"
"int8 status            # status of recognition process defined below\n"
"HarkJuliusSrcVal[] src # WHYPO results for each source\n"
"\n"
"int8 TERMINATED=0\n"
"int8 STARTRECOG=1\n"
"int8 INPUTPARAM=2\n"
"int8 SOURCEINFO=3\n"
"int8 RECOGOUT=4\n"
"int8 RECOGFAIL=5\n"
"\n"
"================================================================================\n"
"MSG: hark_msgs/HarkJuliusSrcVal\n"
"string word\n"
"int32 classid\n"
"string phone\n"
"float32 cm\n"
;
  }

  static const char* value(const ::hark_msgs::HarkJuliusSrc_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::hark_msgs::HarkJuliusSrc_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.id);
      stream.next(m.azimuth);
      stream.next(m.elevation);
      stream.next(m.sec);
      stream.next(m.usec);
      stream.next(m.frames);
      stream.next(m.msec);
      stream.next(m.status);
      stream.next(m.src);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct HarkJuliusSrc_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::hark_msgs::HarkJuliusSrc_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::hark_msgs::HarkJuliusSrc_<ContainerAllocator>& v)
  {
    s << indent << "id: ";
    Printer<int32_t>::stream(s, indent + "  ", v.id);
    s << indent << "azimuth: ";
    Printer<float>::stream(s, indent + "  ", v.azimuth);
    s << indent << "elevation: ";
    Printer<float>::stream(s, indent + "  ", v.elevation);
    s << indent << "sec: ";
    Printer<int32_t>::stream(s, indent + "  ", v.sec);
    s << indent << "usec: ";
    Printer<int32_t>::stream(s, indent + "  ", v.usec);
    s << indent << "frames: ";
    Printer<int32_t>::stream(s, indent + "  ", v.frames);
    s << indent << "msec: ";
    Printer<int32_t>::stream(s, indent + "  ", v.msec);
    s << indent << "status: ";
    Printer<int8_t>::stream(s, indent + "  ", v.status);
    s << indent << "src[]" << std::endl;
    for (size_t i = 0; i < v.src.size(); ++i)
    {
      s << indent << "  src[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::hark_msgs::HarkJuliusSrcVal_<ContainerAllocator> >::stream(s, indent + "    ", v.src[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // HARK_MSGS_MESSAGE_HARKJULIUSSRC_H
