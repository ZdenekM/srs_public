/* Auto-generated by genmsg_cpp for file /home/emisario/git/care-o-bot/srs/srs_grasping/msg/GraspGoal.msg */
#ifndef SRS_GRASPING_MESSAGE_GRASPGOAL_H
#define SRS_GRASPING_MESSAGE_GRASPGOAL_H
#include <string>
#include <vector>
#include <map>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/time.h"

#include "ros/macros.h"

#include "ros/assert.h"


namespace srs_grasping
{
template <class ContainerAllocator>
struct GraspGoal_ {
  typedef GraspGoal_<ContainerAllocator> Type;

  GraspGoal_()
  : object_id(0.0)
  , pose_id()
  {
  }

  GraspGoal_(const ContainerAllocator& _alloc)
  : object_id(0.0)
  , pose_id(_alloc)
  {
  }

  typedef float _object_id_type;
  float object_id;

  typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _pose_id_type;
  std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  pose_id;


private:
  static const char* __s_getDataType_() { return "srs_grasping/GraspGoal"; }
public:
  ROS_DEPRECATED static const std::string __s_getDataType() { return __s_getDataType_(); }

  ROS_DEPRECATED const std::string __getDataType() const { return __s_getDataType_(); }

private:
  static const char* __s_getMD5Sum_() { return "d3953c2a3b42a0ed371295e68b1f4e27"; }
public:
  ROS_DEPRECATED static const std::string __s_getMD5Sum() { return __s_getMD5Sum_(); }

  ROS_DEPRECATED const std::string __getMD5Sum() const { return __s_getMD5Sum_(); }

private:
  static const char* __s_getMessageDefinition_() { return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
float32 object_id\n\
string pose_id\n\
\n\
"; }
public:
  ROS_DEPRECATED static const std::string __s_getMessageDefinition() { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED const std::string __getMessageDefinition() const { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED virtual uint8_t *serialize(uint8_t *write_ptr, uint32_t seq) const
  {
    ros::serialization::OStream stream(write_ptr, 1000000000);
    ros::serialization::serialize(stream, object_id);
    ros::serialization::serialize(stream, pose_id);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint8_t *deserialize(uint8_t *read_ptr)
  {
    ros::serialization::IStream stream(read_ptr, 1000000000);
    ros::serialization::deserialize(stream, object_id);
    ros::serialization::deserialize(stream, pose_id);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint32_t serializationLength() const
  {
    uint32_t size = 0;
    size += ros::serialization::serializationLength(object_id);
    size += ros::serialization::serializationLength(pose_id);
    return size;
  }

  typedef boost::shared_ptr< ::srs_grasping::GraspGoal_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::srs_grasping::GraspGoal_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct GraspGoal
typedef  ::srs_grasping::GraspGoal_<std::allocator<void> > GraspGoal;

typedef boost::shared_ptr< ::srs_grasping::GraspGoal> GraspGoalPtr;
typedef boost::shared_ptr< ::srs_grasping::GraspGoal const> GraspGoalConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::srs_grasping::GraspGoal_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::srs_grasping::GraspGoal_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace srs_grasping

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::srs_grasping::GraspGoal_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::srs_grasping::GraspGoal_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::srs_grasping::GraspGoal_<ContainerAllocator> > {
  static const char* value() 
  {
    return "d3953c2a3b42a0ed371295e68b1f4e27";
  }

  static const char* value(const  ::srs_grasping::GraspGoal_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xd3953c2a3b42a0edULL;
  static const uint64_t static_value2 = 0x371295e68b1f4e27ULL;
};

template<class ContainerAllocator>
struct DataType< ::srs_grasping::GraspGoal_<ContainerAllocator> > {
  static const char* value() 
  {
    return "srs_grasping/GraspGoal";
  }

  static const char* value(const  ::srs_grasping::GraspGoal_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::srs_grasping::GraspGoal_<ContainerAllocator> > {
  static const char* value() 
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
float32 object_id\n\
string pose_id\n\
\n\
";
  }

  static const char* value(const  ::srs_grasping::GraspGoal_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::srs_grasping::GraspGoal_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.object_id);
    stream.next(m.pose_id);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct GraspGoal_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::srs_grasping::GraspGoal_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::srs_grasping::GraspGoal_<ContainerAllocator> & v) 
  {
    s << indent << "object_id: ";
    Printer<float>::stream(s, indent + "  ", v.object_id);
    s << indent << "pose_id: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.pose_id);
  }
};


} // namespace message_operations
} // namespace ros

#endif // SRS_GRASPING_MESSAGE_GRASPGOAL_H

