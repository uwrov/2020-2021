export const NavbarItems = [
	{
		title: 'Settings',
		dropdown: false,
		defaultWidget: '',
		cName: 'navButton'
	},
	{
		title: 'Cameras',
		dropdown: true,
		dropdownElements:[
			{
				title: 'IP Camera',
				cName: 'dropDownButton'
			},
			{
				title: 'ROS Camera',
				cName: 'dropDownButton'
			}
		],
		defaultWidget: '',
		cName: 'navButton'
	},
	{
		title: 'Controller',
		dropdown: false,
		defaultWidget: '',
		cName: 'navButton'
	},
	{
		title: 'Script Runner',
		dropdown: false,
		defaultWidget: '',
		cName: 'navButton'
	},
	{
		title: 'Console',
		dropdown: false,
		defaultWidget: '',
		cName: 'navButton'
	}
]