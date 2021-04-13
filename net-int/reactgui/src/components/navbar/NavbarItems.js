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
				cName: 'navButton'
			},
			{
				title: 'ROS Camera',
				cName: 'navButton'
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
		title: 'Console',
		dropdown: false,
		defaultWidget: '',
		cName: 'navButton'
	}
]