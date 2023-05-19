// Generated by gencpp from file rover_control/corner.msg
// DO NOT EDIT!


#ifndef ROVER_CONTROL_MESSAGE_CORNER_H
#define ROVER_CONTROL_MESSAGE_CORNER_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace rover_control
{
template <class ContainerAllocator>
struct corner_
{
  typedef corner_<ContainerAllocator> Type;

  corner_()
    : wheel_pos_actual(0.0)
    , wheel_pos_desired(0.0)
    , wheel_vel_actual(0.0)
    , wheel_vel_desired(0.0)
    , steer_pos_actual(0.0)
    , steer_pos_desired(0.0)  {
    }
  corner_(const ContainerAllocator& _alloc)
    : wheel_pos_actual(0.0)
    , wheel_pos_desired(0.0)
    , wheel_vel_actual(0.0)
    , wheel_vel_desired(0.0)
    , steer_pos_actual(0.0)
    , steer_pos_desired(0.0)  {
  (void)_alloc;
    }



   typedef double _wheel_pos_actual_type;
  _wheel_pos_actual_type wheel_pos_actual;

   typedef double _wheel_pos_desired_type;
  _wheel_pos_desired_type wheel_pos_desired;

   typedef double _wheel_vel_actual_type;
  _wheel_vel_actual_type wheel_vel_actual;

   typedef double _wheel_vel_desired_type;
  _wheel_vel_desired_type wheel_vel_desired;

   typedef double _steer_pos_actual_type;
  _steer_pos_actual_type steer_pos_actual;

   typedef double _steer_pos_desired_type;
  _steer_pos_desired_type steer_pos_desired;





  typedef boost::shared_ptr< ::rover_control::corner_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::rover_control::corner_<ContainerAllocator> const> ConstPtr;

}; // struct corner_

typedef ::rover_control::corner_<std::allocator<void> > corner;

typedef boost::shared_ptr< ::rover_control::corner > cornerPtr;
typedef boost::shared_ptr< ::rover_control::corner const> cornerConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::rover_control::corner_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::rover_control::corner_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::rover_control::corner_<ContainerAllocator1> & lhs, const ::rover_control::corner_<ContainerAllocator2> & rhs)
{
  return lhs.wheel_pos_actual == rhs.wheel_pos_actual &&
    lhs.wheel_pos_desired == rhs.wheel_pos_desired &&
    lhs.wheel_vel_actual == rhs.wheel_vel_actual &&
    lhs.wheel_vel_desired == rhs.wheel_vel_desired &&
    lhs.steer_pos_actual == rhs.steer_pos_actual &&
    lhs.steer_pos_desired == rhs.steer_pos_desired;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::rover_control::corner_<ContainerAllocator1> & lhs, const ::rover_control::corner_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace rover_control

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::rover_control::corner_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::rover_control::corner_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::rover_control::corner_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::rover_control::corner_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::rover_control::corner_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::rover_control::corner_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::rover_control::corner_<ContainerAllocator> >
{
  static const char* value()
  {
    return "516e2e5c8624f5f3853fb88a0ab5fb10";
  }

  static const char* value(const ::rover_control::corner_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x516e2e5c8624f5f3ULL;
  static const uint64_t static_value2 = 0x853fb88a0ab5fb10ULL;
};

template<class ContainerAllocator>
struct DataType< ::rover_control::corner_<ContainerAllocator> >
{
  static const char* value()
  {
    return "rover_control/corner";
  }

  static const char* value(const ::rover_control::corner_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::rover_control::corner_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float64 wheel_pos_actual\n"
"float64 wheel_pos_desired\n"
"float64 wheel_vel_actual\n"
"float64 wheel_vel_desired\n"
"float64 steer_pos_actual\n"
"float64 steer_pos_desired\n"
;
  }

  static const char* value(const ::rover_control::corner_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::rover_control::corner_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.wheel_pos_actual);
      stream.next(m.wheel_pos_desired);
      stream.next(m.wheel_vel_actual);
      stream.next(m.wheel_vel_desired);
      stream.next(m.steer_pos_actual);
      stream.next(m.steer_pos_desired);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct corner_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::rover_control::corner_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::rover_control::corner_<ContainerAllocator>& v)
  {
    s << indent << "wheel_pos_actual: ";
    Printer<double>::stream(s, indent + "  ", v.wheel_pos_actual);
    s << indent << "wheel_pos_desired: ";
    Printer<double>::stream(s, indent + "  ", v.wheel_pos_desired);
    s << indent << "wheel_vel_actual: ";
    Printer<double>::stream(s, indent + "  ", v.wheel_vel_actual);
    s << indent << "wheel_vel_desired: ";
    Printer<double>::stream(s, indent + "  ", v.wheel_vel_desired);
    s << indent << "steer_pos_actual: ";
    Printer<double>::stream(s, indent + "  ", v.steer_pos_actual);
    s << indent << "steer_pos_desired: ";
    Printer<double>::stream(s, indent + "  ", v.steer_pos_desired);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ROVER_CONTROL_MESSAGE_CORNER_H