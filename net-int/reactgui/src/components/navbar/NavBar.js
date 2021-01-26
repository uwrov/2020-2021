import React from 'react';
import "./Navbar.css";
import {NavbarItems} from "./NavbarItems";

class NavBar extends React.Component {
	
	state = { buttonClicks: null}
	
	constructor(props){
		super(props);
		this.state.buttonClicks= this.addItems();
	}
	
	addItems = () => {
       	let items = new Map();
       	items.set('menuIcon', false);
       	let idx;       	
       	for(idx in NavbarItems){
       		let item = NavbarItems[idx];
       		items.set(item.title, false);
       	}
       	return items;
    }
    
    handleButtonClick = (title) => {
        this.setState({ buttonClicks: this.state.buttonClicks.set(title, !this.state.buttonClicks.get(title))});
    	console.log(this.state.buttonClicks);    	
    }
    
    
	render() {
		
		return(
			<nav className="NavbarItems">
				<h1 className = "navbar-logo">UWROV</h1>
				<div className="menu-icon" onClick={() => {this.handleButtonClick('menuIcon')}}>
                	<i className={this.state.buttonClicks.get('menuIcon') ? 'fas fa-times' : 'fas fa-bars'}></i>
                </div>
		        <ul className={this.state.buttonClicks.get('menuIcon') ? 'nav-menu active' : 'nav-menu'}>
		        	{NavbarItems.map((item, index) => {
		        		return (
                            <li key={index}>
                                <a className={item.cName} onClick = {() => {this.handleButtonClick(item.title)}}>
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