import React from "react";

class TestWidget extends React.Component {
  state = {
    ip: "123.3.33.2:3000",
    numb: 0,
  };

  constructor(props) {
    super(props);
  }

  //
  // Provide custom renderMethod()
  //
  render = () => {
    return (
      <div
        onClick={() => {
          console.log("click!");
          this.setState({ ip: "0.0.0.0:3000" });
        }}
      >
        hello world! {this.state.ip}
        {this.props.text}
      </div>
    );
  };
}

export default TestWidget;
