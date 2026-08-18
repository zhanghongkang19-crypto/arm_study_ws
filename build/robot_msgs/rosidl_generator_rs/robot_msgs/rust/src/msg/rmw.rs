#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};


#[link(name = "robot_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__robot_msgs__msg__ArmPositionCmd() -> *const std::ffi::c_void;
}

#[link(name = "robot_msgs__rosidl_generator_c")]
extern "C" {
    fn robot_msgs__msg__ArmPositionCmd__init(msg: *mut ArmPositionCmd) -> bool;
    fn robot_msgs__msg__ArmPositionCmd__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ArmPositionCmd>, size: usize) -> bool;
    fn robot_msgs__msg__ArmPositionCmd__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ArmPositionCmd>);
    fn robot_msgs__msg__ArmPositionCmd__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ArmPositionCmd>, out_seq: *mut rosidl_runtime_rs::Sequence<ArmPositionCmd>) -> bool;
}

// Corresponds to robot_msgs__msg__ArmPositionCmd
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ArmPositionCmd {

    // This member is not documented.
    #[allow(missing_docs)]
    pub position: rosidl_runtime_rs::Sequence<f64>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub velocity: rosidl_runtime_rs::Sequence<f64>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub effort: rosidl_runtime_rs::Sequence<f64>,

}



impl Default for ArmPositionCmd {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !robot_msgs__msg__ArmPositionCmd__init(&mut msg as *mut _) {
        panic!("Call to robot_msgs__msg__ArmPositionCmd__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ArmPositionCmd {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { robot_msgs__msg__ArmPositionCmd__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { robot_msgs__msg__ArmPositionCmd__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { robot_msgs__msg__ArmPositionCmd__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ArmPositionCmd {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ArmPositionCmd where Self: Sized {
  const TYPE_NAME: &'static str = "robot_msgs/msg/ArmPositionCmd";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__robot_msgs__msg__ArmPositionCmd() }
  }
}


