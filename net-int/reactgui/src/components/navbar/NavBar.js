import React from "react";
import "./Navbar.css";
import { NavbarItems } from "./NavbarItems";
import { uses } from "react";

class NavBar extends React.Component {
  state = { buttonClicks: null };

  constructor(props) {
    super(props);
    this.state.buttonClicks = this.addItems();
  }

  addItems = () => {
    let items = new Map();
    items.set("menuIcon", false);
    let idx;
    for (idx in NavbarItems) {
      let item = NavbarItems[idx];
      items.set(item.title, false);
    }
    return items;
  };

  handleButtonClick = (title) => {
    let newVal = !this.state.buttonClicks.get(title);
    this.setState({ buttonClicks: this.state.buttonClicks.set(title, newVal) });
    if (title !== "Cameras") {
      this.props.addWidget(title);
    }
    // IF you want one click to add and the other to close
    // if (newVal){
    //     this.props.addWidget(title);
    // } else{
    // 	this.props.removeWidget(title);
    // }
  };

  NavItem = (item) => {
    if (item.dropdown && this.state.buttonClicks.get(item.title)) {
      return (
        <div className="dropDown">
          <a
            className={"dropDownButton"}
            onClick={() => {
              this.handleButtonClick(item.title);
            }}
          >
            {item.title}
          </a>
          {item.dropdownElements.map((dropDownElem) => {
            return (
              <a
                className={dropDownElem.cName}
                onClick={() => {
                  this.handleButtonClick(dropDownElem.title);
                }}
              >
                {dropDownElem.title}
              </a>
            );
          })}
        </div>
      );
    } else {
      return (
        <a
          className={item.cName + this.state.buttonClicks.get(item.title)}
          onClick={() => {
            this.handleButtonClick(item.title);
          }}
        >
          {item.title}
        </a>
      );
    }
  };

  render() {
    return (
      <nav className="NavbarItems">
        <h1 className="navbar-logo">
          <a href="http://uwrov.org/">UWROV</a>
        </h1>
        <div
          className="menu-icon"
          onClick={() => {
            this.handleButtonClick("menuIcon");
          }}
        >
          <i
            className={
              this.state.buttonClicks.get("menuIcon")
                ? "fas fa-times"
                : "fas fa-bars"
            }
          ></i>
        </div>
        <ul
          className={
            this.state.buttonClicks.get("menuIcon")
              ? "nav-menu active"
              : "nav-menu"
          }
        >
          {NavbarItems.map((item, index) => {
            return <li key={index}>{this.NavItem(item)}</li>;
          })}
        </ul>
      </nav>
    );
  }
}

export default NavBar;
