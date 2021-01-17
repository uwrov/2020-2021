import React from 'react';
import "./Navbar.css";
import {NavbarItems} from "./NavbarItems";

class NavBar extends React.Component {
	state = { clicked: false }

    handleClick = () => {
        this.setState({ clicked: !this.state.clicked })
    }
    
	render() {
		return(
			<nav className="NavbarItems">
				<h1 className = "navbar-logo">UWROV</h1>
				<div className="menu-icon" onClick={this.handleClick}>
                	<i className={this.state.clicked ? 'fas fa-times' : 'fas fa-bars'}></i>
                </div>
		        <ul className={this.state.clicked ? 'nav-menu active' : 'nav-menu'}>
		        	{NavbarItems.map((item, index) => {
		        		return (
                            <li key={index}>
                                <a className={item.cName}>
                                {item.title}
                                </a>
                            </li>
                        )
		        	})}
		        </ul>
	        </nav>
		);
	}
}

export default NavBar;
