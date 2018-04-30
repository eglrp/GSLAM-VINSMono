#pragma once
#include "Header.h"

namespace sensor_msgs
{
template <class ContainerAllocator>
struct Image_
{
  typedef Image_<ContainerAllocator> Type;

  Image_()
    : header()
    , height(0)
    , width(0)
    , encoding()
    , is_bigendian(0)
    , step(0)
    , data()  {
    }
  Image_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , height(0)
    , width(0)
    , encoding(_alloc)
    , is_bigendian(0)
    , step(0)
    , data(_alloc)  {
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef uint32_t _height_type;
  _height_type height;

   typedef uint32_t _width_type;
  _width_type width;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _encoding_type;
  _encoding_type encoding;

   typedef uint8_t _is_bigendian_type;
  _is_bigendian_type is_bigendian;

   typedef uint32_t _step_type;
  _step_type step;

   typedef std::vector<uint8_t, typename ContainerAllocator::template rebind<uint8_t>::other >  _data_type;
  _data_type data;




  typedef boost::shared_ptr< ::sensor_msgs::Image_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::sensor_msgs::Image_<ContainerAllocator> const> ConstPtr;

}; // struct Image_

typedef ::sensor_msgs::Image_<std::allocator<void> > Image;

typedef boost::shared_ptr< ::sensor_msgs::Image > ImagePtr;
typedef boost::shared_ptr< ::sensor_msgs::Image const> ImageConstPtr;

// constants requiring out of line definition


} // namespace sensor_msgs

