import React from "react"
import "./SoundInfoContainer.css"
import { IoVolumeHighSharp, IoWater } from "react-icons/io5"
import { FaExclamation, FaBaby } from "react-icons/fa"
import { RiAlarmWarningLine } from "react-icons/ri"
import { MdOutlineDoorbell } from "react-icons/md"
import { CiWarning } from "react-icons/ci"

const SoundInfoContainer = (props) => {
  var icon
  var sz = "10em"
  switch (props.sound_info.sound_type) {
    case "Alarm":
      icon = <RiAlarmWarningLine size={sz} />
      break
    case "Doorbell":
      icon = <MdOutlineDoorbell size={sz} />
      break
    case "Water":
      icon = <IoWater size={sz} />
      break
    case "Baby":
      icon = <FaBaby size={sz} />
      break
    default:
      icon = <CiWarning size={sz} />
  }
  return (
    <div className="sound-info-container">
      <h1 id="sound-type-text">
        Sound Detected: {props.sound_info.sound_type}
      </h1>
      {icon}
      <div className="progress-container">
        <h1>Volume: </h1>
        <progress value={1 - props.sound_info.volume / 10} />
        <IoVolumeHighSharp size={"2em"} />
      </div>
      {/* <div className="progress-container">
        <h1>Urgency: </h1>
        <progress value={1 - props.sound_urgency} />
        <FaExclamation size={"2em"} />
      </div> */}
    </div>
  )
}

export default SoundInfoContainer
