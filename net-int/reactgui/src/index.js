import React from 'react';
import ReactDOM from 'react-dom';
//import './components/index.css';
import Console from './components/console/Console.js';
//import App from './App';
import GUI from "./oldcomponents/components/GUI.js";
import Controller from "./components/controller/Controller.js";
import Camera from "./components/camera/Camera.js";


//ReactDOM.render(<App />, document.getElementById('root'));

//let button = new Button();
//ReactDOM.render(<GUI />, document.getElementById('root'));
//ReactDOM.render(<Console />, document.getElementById('root'));
//ReactDOM.render(<Controller />, document.getElementById('root'));
//ReactDOM.render(button.render(), document.getElementById('root'));
ReactDOM.render(<Camera />, document.getElementById('root'));