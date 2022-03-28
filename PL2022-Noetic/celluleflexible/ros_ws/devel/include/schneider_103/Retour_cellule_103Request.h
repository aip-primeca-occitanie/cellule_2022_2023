// Generated by gencpp from file schneider_103/Retour_cellule_103Request.msg
// DO NOT EDIT!


#ifndef SCHNEIDER_103_MESSAGE_RETOUR_CELLULE_103REQUEST_H
#define SCHNEIDER_103_MESSAGE_RETOUR_CELLULE_103REQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace schneider_103
{
template <class ContainerAllocator>
struct Retour_cellule_103Request_
{
  typedef Retour_cellule_103Request_<ContainerAllocator> Type;

  Retour_cellule_103Request_()
    : memoire(0)  {
    }
  Retour_cellule_103Request_(const ContainerAllocator& _alloc)
    : memoire(0)  {
  (void)_alloc;
    }



   typedef int32_t _memoire_type;
  _memoire_type memoire;





  typedef boost::shared_ptr< ::schneider_103::Retour_cellule_103Request_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::schneider_103::Retour_cellule_103Request_<ContainerAllocator> const> ConstPtr;

}; // struct Retour_cellule_103Request_

typedef ::schneider_103::Retour_cellule_103Request_<std::allocator<void> > Retour_cellule_103Request;

typedef boost::shared_ptr< ::schneider_103::Retour_cellule_103Request > Retour_cellule_103RequestPtr;
typedef boost::shared_ptr< ::schneider_103::Retour_cellule_103Request const> Retour_cellule_103RequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::schneider_103::Retour_cellule_103Request_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::schneider_103::Retour_cellule_103Request_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::schneider_103::Retour_cellule_103Request_<ContainerAllocator1> & lhs, const ::schneider_103::Retour_cellule_103Request_<ContainerAllocator2> & rhs)
{
  return lhs.memoire == rhs.memoire;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::schneider_103::Retour_cellule_103Request_<ContainerAllocator1> & lhs, const ::schneider_103::Retour_cellule_103Request_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace schneider_103

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::schneider_103::Retour_cellule_103Request_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::schneider_103::Retour_cellule_103Request_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::schneider_103::Retour_cellule_103Request_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::schneider_103::Retour_cellule_103Request_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::schneider_103::Retour_cellule_103Request_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::schneider_103::Retour_cellule_103Request_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::schneider_103::Retour_cellule_103Request_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d38585a9ace9d44e8aeed18cb0d33881";
  }

  static const char* value(const ::schneider_103::Retour_cellule_103Request_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd38585a9ace9d44eULL;
  static const uint64_t static_value2 = 0x8aeed18cb0d33881ULL;
};

template<class ContainerAllocator>
struct DataType< ::schneider_103::Retour_cellule_103Request_<ContainerAllocator> >
{
  static const char* value()
  {
    return "schneider_103/Retour_cellule_103Request";
  }

  static const char* value(const ::schneider_103::Retour_cellule_103Request_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::schneider_103::Retour_cellule_103Request_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int32 memoire\n"
;
  }

  static const char* value(const ::schneider_103::Retour_cellule_103Request_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::schneider_103::Retour_cellule_103Request_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.memoire);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Retour_cellule_103Request_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::schneider_103::Retour_cellule_103Request_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::schneider_103::Retour_cellule_103Request_<ContainerAllocator>& v)
  {
    s << indent << "memoire: ";
    Printer<int32_t>::stream(s, indent + "  ", v.memoire);
  }
};

} // namespace message_operations
} // namespace ros

#endif // SCHNEIDER_103_MESSAGE_RETOUR_CELLULE_103REQUEST_H
