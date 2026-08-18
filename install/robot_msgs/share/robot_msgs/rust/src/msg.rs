#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



// Corresponds to robot_msgs__msg__ArmPositionCmd

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ArmPositionCmd {

    // This member is not documented.
    #[allow(missing_docs)]
    pub position: Vec<f64>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub velocity: Vec<f64>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub effort: Vec<f64>,

}



impl Default for ArmPositionCmd {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::ArmPositionCmd::default())
  }
}

impl rosidl_runtime_rs::Message for ArmPositionCmd {
  type RmwMsg = super::msg::rmw::ArmPositionCmd;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        position: msg.position.into(),
        velocity: msg.velocity.into(),
        effort: msg.effort.into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        position: msg.position.as_slice().into(),
        velocity: msg.velocity.as_slice().into(),
        effort: msg.effort.as_slice().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      position: msg.position
          .into_iter()
          .collect(),
      velocity: msg.velocity
          .into_iter()
          .collect(),
      effort: msg.effort
          .into_iter()
          .collect(),
    }
  }
}


