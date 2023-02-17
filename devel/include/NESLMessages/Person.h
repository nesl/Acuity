// Generated by gencpp from file NESLMessages/Person.msg
// DO NOT EDIT!


#ifndef NESLMESSAGES_MESSAGE_PERSON_H
#define NESLMESSAGES_MESSAGE_PERSON_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <NESLMessages/NeslCoord.h>

namespace NESLMessages
{
template <class ContainerAllocator>
struct Person_
{
  typedef Person_<ContainerAllocator> Type;

  Person_()
    : colorArr()
    , personID(0)
    , personCoord()
    , accountedFor(false)
    , talking(false)
    , bbx(0.0)
    , bby(0.0)
    , bbz(0.0)  {
    }
  Person_(const ContainerAllocator& _alloc)
    : colorArr(_alloc)
    , personID(0)
    , personCoord(_alloc)
    , accountedFor(false)
    , talking(false)
    , bbx(0.0)
    , bby(0.0)
    , bbz(0.0)  {
  (void)_alloc;
    }



   typedef std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> _colorArr_type;
  _colorArr_type colorArr;

   typedef int32_t _personID_type;
  _personID_type personID;

   typedef  ::NESLMessages::NeslCoord_<ContainerAllocator>  _personCoord_type;
  _personCoord_type personCoord;

   typedef uint8_t _accountedFor_type;
  _accountedFor_type accountedFor;

   typedef uint8_t _talking_type;
  _talking_type talking;

   typedef float _bbx_type;
  _bbx_type bbx;

   typedef float _bby_type;
  _bby_type bby;

   typedef float _bbz_type;
  _bbz_type bbz;





  typedef boost::shared_ptr< ::NESLMessages::Person_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::NESLMessages::Person_<ContainerAllocator> const> ConstPtr;

}; // struct Person_

typedef ::NESLMessages::Person_<std::allocator<void> > Person;

typedef boost::shared_ptr< ::NESLMessages::Person > PersonPtr;
typedef boost::shared_ptr< ::NESLMessages::Person const> PersonConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::NESLMessages::Person_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::NESLMessages::Person_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::NESLMessages::Person_<ContainerAllocator1> & lhs, const ::NESLMessages::Person_<ContainerAllocator2> & rhs)
{
  return lhs.colorArr == rhs.colorArr &&
    lhs.personID == rhs.personID &&
    lhs.personCoord == rhs.personCoord &&
    lhs.accountedFor == rhs.accountedFor &&
    lhs.talking == rhs.talking &&
    lhs.bbx == rhs.bbx &&
    lhs.bby == rhs.bby &&
    lhs.bbz == rhs.bbz;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::NESLMessages::Person_<ContainerAllocator1> & lhs, const ::NESLMessages::Person_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace NESLMessages

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::NESLMessages::Person_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::NESLMessages::Person_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::NESLMessages::Person_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::NESLMessages::Person_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::NESLMessages::Person_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::NESLMessages::Person_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::NESLMessages::Person_<ContainerAllocator> >
{
  static const char* value()
  {
    return "07061084615db4a1b1a7a2a5b8b45019";
  }

  static const char* value(const ::NESLMessages::Person_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x07061084615db4a1ULL;
  static const uint64_t static_value2 = 0xb1a7a2a5b8b45019ULL;
};

template<class ContainerAllocator>
struct DataType< ::NESLMessages::Person_<ContainerAllocator> >
{
  static const char* value()
  {
    return "NESLMessages/Person";
  }

  static const char* value(const ::NESLMessages::Person_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::NESLMessages::Person_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float64[] colorArr\n"
"int32 personID\n"
"NeslCoord personCoord\n"
"bool accountedFor\n"
"bool talking\n"
"float32 bbx\n"
"float32 bby\n"
"float32 bbz\n"
"\n"
"================================================================================\n"
"MSG: NESLMessages/NeslCoord\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
;
  }

  static const char* value(const ::NESLMessages::Person_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::NESLMessages::Person_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.colorArr);
      stream.next(m.personID);
      stream.next(m.personCoord);
      stream.next(m.accountedFor);
      stream.next(m.talking);
      stream.next(m.bbx);
      stream.next(m.bby);
      stream.next(m.bbz);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Person_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::NESLMessages::Person_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::NESLMessages::Person_<ContainerAllocator>& v)
  {
    s << indent << "colorArr[]" << std::endl;
    for (size_t i = 0; i < v.colorArr.size(); ++i)
    {
      s << indent << "  colorArr[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.colorArr[i]);
    }
    s << indent << "personID: ";
    Printer<int32_t>::stream(s, indent + "  ", v.personID);
    s << indent << "personCoord: ";
    s << std::endl;
    Printer< ::NESLMessages::NeslCoord_<ContainerAllocator> >::stream(s, indent + "  ", v.personCoord);
    s << indent << "accountedFor: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.accountedFor);
    s << indent << "talking: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.talking);
    s << indent << "bbx: ";
    Printer<float>::stream(s, indent + "  ", v.bbx);
    s << indent << "bby: ";
    Printer<float>::stream(s, indent + "  ", v.bby);
    s << indent << "bbz: ";
    Printer<float>::stream(s, indent + "  ", v.bbz);
  }
};

} // namespace message_operations
} // namespace ros

#endif // NESLMESSAGES_MESSAGE_PERSON_H