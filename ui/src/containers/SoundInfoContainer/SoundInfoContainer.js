import React from "react"
import "./SoundInfoContainer.css"
import { IoVolumeHighSharp, IoWater } from "react-icons/io5"
import { FaExclamation, FaBaby, FaArrowUp } from "react-icons/fa"
import { RiAlarmWarningLine } from "react-icons/ri"
import { MdOutlineDoorbell } from "react-icons/md"
import { CiWarning } from "react-icons/ci"
import { FiPhoneCall } from "react-icons/fi"

const SoundInfoContainer = (props) => {
  var icon, text
  var sz = "10em"
  var angle = 360 - props.sound_info.angle + "deg"
  switch (props.sound_info.sound_type) {
    case "alarm":
      text = "Alarm"
      icon = <RiAlarmWarningLine size={sz} />
      break
    case "door_bell":
      text = "Door Bell "
      icon = <MdOutlineDoorbell size={sz} />
      break
    case "phone_ring":
      text = "Phone"
      icon = <FiPhoneCall size={sz} />
      break
    case "water":
      icon = <IoWater size={sz} />
      break
    case "baby":
      icon = <FaBaby size={sz} />
      break
    default:
      icon = <CiWarning size={sz} />
  }
  return (
    <div className="sound-info-container">
      <h1 id="sound-type-text">Sound Detected: {text}</h1>
      {icon}
      <div className="progress-container">
        <h1>Volume: </h1>
        <progress value={1 - props.sound_info.volume / 1000} />
        <IoVolumeHighSharp size={"2em"} />
      </div>
      <div className="progress-container">
        <h1>Location:</h1>
        <FaArrowUp
          size={"5em"}
          style={{ transform: "rotate(" + angle + ")" }}
        />
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
