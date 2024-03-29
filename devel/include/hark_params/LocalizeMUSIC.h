// Generated by gencpp from file hark_params/LocalizeMUSIC.msg
// DO NOT EDIT!


#ifndef HARK_PARAMS_MESSAGE_LOCALIZEMUSIC_H
#define HARK_PARAMS_MESSAGE_LOCALIZEMUSIC_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace hark_params
{
template <class ContainerAllocator>
struct LocalizeMUSIC_
{
  typedef LocalizeMUSIC_<ContainerAllocator> Type;

  LocalizeMUSIC_()
    : num_source(0)
    , min_deg(0)
    , max_deg(0)
    , lower_bound_frequency(0)
    , upper_bound_frequency(0)  {
    }
  LocalizeMUSIC_(const ContainerAllocator& _alloc)
    : num_source(0)
    , min_deg(0)
    , max_deg(0)
    , lower_bound_frequency(0)
    , upper_bound_frequency(0)  {
  (void)_alloc;
    }



   typedef int32_t _num_source_type;
  _num_source_type num_source;

   typedef int32_t _min_deg_type;
  _min_deg_type min_deg;

   typedef int32_t _max_deg_type;
  _max_deg_type max_deg;

   typedef int32_t _lower_bound_frequency_type;
  _lower_bound_frequency_type lower_bound_frequency;

   typedef int32_t _upper_bound_frequency_type;
  _upper_bound_frequency_type upper_bound_frequency;





  typedef boost::shared_ptr< ::hark_params::LocalizeMUSIC_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::hark_params::LocalizeMUSIC_<ContainerAllocator> const> ConstPtr;

}; // struct LocalizeMUSIC_

typedef ::hark_params::LocalizeMUSIC_<std::allocator<void> > LocalizeMUSIC;

typedef boost::shared_ptr< ::hark_params::LocalizeMUSIC > LocalizeMUSICPtr;
typedef boost::shared_ptr< ::hark_params::LocalizeMUSIC const> LocalizeMUSICConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::hark_params::LocalizeMUSIC_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::hark_params::LocalizeMUSIC_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::hark_params::LocalizeMUSIC_<ContainerAllocator1> & lhs, const ::hark_params::LocalizeMUSIC_<ContainerAllocator2> & rhs)
{
  return lhs.num_source == rhs.num_source &&
    lhs.min_deg == rhs.min_deg &&
    lhs.max_deg == rhs.max_deg &&
    lhs.lower_bound_frequency == rhs.lower_bound_frequency &&
    lhs.upper_bound_frequency == rhs.upper_bound_frequency;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::hark_params::LocalizeMUSIC_<ContainerAllocator1> & lhs, const ::hark_params::LocalizeMUSIC_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace hark_params

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::hark_params::LocalizeMUSIC_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::hark_params::LocalizeMUSIC_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::hark_params::LocalizeMUSIC_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::hark_params::LocalizeMUSIC_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hark_params::LocalizeMUSIC_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hark_params::LocalizeMUSIC_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::hark_params::LocalizeMUSIC_<ContainerAllocator> >
{
  static const char* value()
  {
    return "e153c85ee682045538315f87c1a9f5f0";
  }

  static const char* value(const ::hark_params::LocalizeMUSIC_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xe153c85ee6820455ULL;
  static const uint64_t static_value2 = 0x38315f87c1a9f5f0ULL;
};

template<class ContainerAllocator>
struct DataType< ::hark_params::LocalizeMUSIC_<ContainerAllocator> >
{
  static const char* value()
  {
    return "hark_params/LocalizeMUSIC";
  }

  static const char* value(const ::hark_params::LocalizeMUSIC_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::hark_params::LocalizeMUSIC_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int32 num_source\n"
"int32 min_deg\n"
"int32 max_deg\n"
"int32 lower_bound_frequency\n"
"int32 upper_bound_frequency\n"
;
  }

  static const char* value(const ::hark_params::LocalizeMUSIC_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::hark_params::LocalizeMUSIC_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.num_source);
      stream.next(m.min_deg);
      stream.next(m.max_deg);
      stream.next(m.lower_bound_frequency);
      stream.next(m.upper_bound_frequency);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct LocalizeMUSIC_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::hark_params::LocalizeMUSIC_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::hark_params::LocalizeMUSIC_<ContainerAllocator>& v)
  {
    s << indent << "num_source: ";
    Printer<int32_t>::stream(s, indent + "  ", v.num_source);
    s << indent << "min_deg: ";
    Printer<int32_t>::stream(s, indent + "  ", v.min_deg);
    s << indent << "max_deg: ";
    Printer<int32_t>::stream(s, indent + "  ", v.max_deg);
    s << indent << "lower_bound_frequency: ";
    Printer<int32_t>::stream(s, indent + "  ", v.lower_bound_frequency);
    s << indent << "upper_bound_frequency: ";
    Printer<int32_t>::stream(s, indent + "  ", v.upper_bound_frequency);
  }
};

} // namespace message_operations
} // namespace ros

#endif // HARK_PARAMS_MESSAGE_LOCALIZEMUSIC_H
