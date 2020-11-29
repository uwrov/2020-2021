import React from 'react';
import ReactDOM from 'react-dom';
import './index.css';
import Console from './Console.js';
//import App from './App';
import GUI from "./components/GUI.js";


//ReactDOM.render(<App />, document.getElementById('root'));

//let button = new Button();
ReactDOM.render(<GUI />, document.getElementById('root'));
ReactDOM.render(<Console />, document.getElementById('console'));
//ReactDOM.render(button.render(), document.getElementById('root'));
