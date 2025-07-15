// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_msg:msg/Age.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSG__MSG__DETAIL__AGE__BUILDER_HPP_
#define CUSTOM_MSG__MSG__DETAIL__AGE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom_msg/msg/detail/age__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom_msg
{

namespace msg
{

namespace builder
{

class Init_Age_day
{
public:
  explicit Init_Age_day(::custom_msg::msg::Age & msg)
  : msg_(msg)
  {}
  ::custom_msg::msg::Age day(::custom_msg::msg::Age::_day_type arg)
  {
    msg_.day = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_msg::msg::Age msg_;
};

class Init_Age_month
{
public:
  explicit Init_Age_month(::custom_msg::msg::Age & msg)
  : msg_(msg)
  {}
  Init_Age_day month(::custom_msg::msg::Age::_month_type arg)
  {
    msg_.month = std::move(arg);
    return Init_Age_day(msg_);
  }

private:
  ::custom_msg::msg::Age msg_;
};

class Init_Age_year
{
public:
  Init_Age_year()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Age_month year(::custom_msg::msg::Age::_year_type arg)
  {
    msg_.year = std::move(arg);
    return Init_Age_month(msg_);
  }

private:
  ::custom_msg::msg::Age msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_msg::msg::Age>()
{
  return custom_msg::msg::builder::Init_Age_year();
}

}  // namespace custom_msg

#endif  // CUSTOM_MSG__MSG__DETAIL__AGE__BUILDER_HPP_
