import React, { Component } from "react"
// import ROSLIB from "roslib"
import "./ButtonContainer.css"

class ButtonContainer extends Component {
  render() {
    return (
      <div className={"ButtonContainer"}>
        <button
          className={"Button"}
          onClick={() => this.props.setAppState("idle")}
        >
          Dismiss Notification
        </button>
        {/* <button
          className={"Button"}
          onClick={() => this.props.setAppState("point")}
        >
          Point to Location
        </button> */}
        <button
          className={"Button"}
          onClick={() => this.props.setAppState("guide")}
        >
          Take Me There
        </button>
      </div>
    )
  }
}

export default ButtonContainer
