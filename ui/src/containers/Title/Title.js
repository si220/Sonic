import React from "react"
import "./Title.css"

function Title(props) {
  return (
    <div className="Title">
      <div className="Title-Text">
        <h1>{props.text}</h1>
      </div>
    </div>
  )
}

export default Title
