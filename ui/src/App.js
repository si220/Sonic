import React, { Component } from "react"
import ROSLIB from "roslib"
import "./App.css"

import { PiDogDuotone } from "react-icons/pi"

import { ButtonContainer, Title, SoundInfoContainer } from "./containers"

export class App extends Component {
  constructor(props) {
    super(props)

    this.state = {
      appState: "idle",
      soundDetected: false,
      soundInfo: {
        sound_type: "Alarm",
        volume: 4,
        angle: 270,
      },
      dismissTime: Date.now(),
    }

    this.updateStateFromButton = this.updateStateFromButton.bind(this)
    this.ros = new ROSLIB.Ros()
  }

  changeSonicState(newState) {
    var stateTopic = new ROSLIB.Topic({
      ros: this.ros,
      name: "/state",
      messageType: "std_msgs/String",
    })

    var msg = new ROSLIB.Message({ data: newState })
    stateTopic.publish(msg)
  }

  updateStateFromButton(newState) {
    this.changeSonicState(newState)
    this.setState({ appState: newState })

    if (newState === "idle") {
      this.setState({
        soundDetected: false,
        dismissTime: Date.now() /*+ 1000 * 60 * 5*/,
      })
    }
  }

  handleSoundDetection(message) {
    let correctedJsonString = message.data.replace(/'/g, '"')
    console.log(correctedJsonString)
    var sound = JSON.parse(correctedJsonString)
    var sameSound =
      sound.sound_type === this.state.soundInfo.sound_type &&
      Date.now() < this.state.dismissTime
        ? true
        : false
    console.log("Received message: " + message.data)

    if (!this.state.soundDetected) {
      if (!sameSound) {
        this.setState({
          appState: "sound detected",
          soundDetected: true,
          soundInfo: sound,
        })
        this.changeSonicState("find user")
      }
    }
  }

  testButton() {
    // Simulate a JSON string that might be received from a real ROS topic
    var simulatedJsonString =
      '{ "sound_type" : "alarm", "volume" : 400, "angle" : 90 }'

    // Create a test message object that mimics the structure of real ROS messages
    var testMessage = {
      data: simulatedJsonString,
    }

    // Directly use the simulated message as argument to handleSoundDetection
    this.handleSoundDetection(testMessage)
  }

  componentDidMount() {
    this.ros.on("connection", function () {
      console.log("Connected to websocket server.")
    })

    this.ros.on("error", function (error) {
      console.log("Error connecting to websocket server: ", error)
    })

    this.ros.on("close", function () {
      console.log("Connection to websocket server closed.")
    })

    //146.169.239.60
    this.ros.connect("ws://146.169.234.217:9090")

    var listener = new ROSLIB.Topic({
      ros: this.ros,
      name: "/sound",
      messageType: "std_msgs/String",
    })

    listener.subscribe((message) => this.handleSoundDetection(message))
    this.changeSonicState("idle")
  }

  componentWillUnmount() {
    this.ros.close()
  }

  render() {
    var text = ""

    var bg = this.state.soundInfo.volume / 1000 < 0.5 ? "amber" : "red"
    switch (this.state.appState) {
      case "sound detected":
        text = "SOUND DETECTED"
        break
      case "point":
        text = "Pointing to the Sound Source"
        break
      case "guide":
        text = "Guiding You to Sound Source"
        break
      case "idle":
        text = "Listening for Sounds"
        bg = "idle"
        break
      default:
    }
    return (
      <div className="App" id={bg}>
        <Title text={text} />
        <div className="box">
          <div className="MainDisplay">
            {this.state.appState !== "idle" ? (
              <SoundInfoContainer sound_info={this.state.soundInfo} />
            ) : (
              <PiDogDuotone size={"20em"} />
            )}
          </div>
        </div>
        {this.state.appState !== "idle" && (
          <ButtonContainer setAppState={this.updateStateFromButton} />
        )}
        <button onClick={() => this.testButton()}>detect sound (test)</button>
      </div>
    )
  }
}

export default App
