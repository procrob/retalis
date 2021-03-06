/****
    Retalis Language for Information Processing and Management in Autonomous Robot Software
    
    http://wiki.ros.org/retalis

    Copyright (C) 2014 __Pouyan Ziafati__ pziafati@gmail.com __ 
    University of Luxembourg, Supported by the Fonds National de la Recherche, Luxembourg	
     
    Retalis Integrates the ELE language of the complex event processing system ETALIS ( http://code.google.com/p/etalis ) 	


    Retalis is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Retalis is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.                   

    You should have received a copy of the GNU General Public License
    along with Retalis.  If not, see <http://www.gnu.org/licenses/>.	
****/




*** Installation:

Step 0: it is assumed that the "retalis" package has been downloaded to the "catkin_ws/src" folder

Step 1: install OpenGL Mathematics (GLM)  
	1- sudo apt-get install libglm-dev

Step 2: install SWI-PL as a shared library
	1- Download pl source code (6.2.2. for me) from http://www.swi-prolog.org/download/stable
	2- Extract the tarball and edit the build.templ file as follows: 
		Set PREFIX=/usr
		Set SUDO="sudo"
		Disable some library that might cause problem by uncommenting 
			export DISABLE_PKGS="ssl  xpce zlib"
		Uncomment the following
			EXTRACFG+=" --enable-shared"

	3- Run build.templ. (as root)
	4- Make sure libswipl.so has been created. In my system, it is found at
		/usr/lib/swipl-6.2.2/lib/i686-linux/libswipl.so


Step 3: install Retalis as follows
	1- In the CMakeLists.txt file, change "/usr/lib/swipl-6.6.6/include" and "/usr/lib/swipl-6.6.6/lib/x86_64-linux" to refer to the include and lib folders of your SWI-PL installtion, respectively
	2- catkin_make (in your catkin workspace)
	3- roscd retalis
	4- In the install.py file, change '/home/robolab/catkin_ws' and '/usr/lib/swipl-6.6.6/lib/x86_64-linux' to refer to the catkin workspace and the lib folder of your SWI-PL installation, respectively
	4- Run the install.py file as root (you might first need to make the install.py file executable by: sudo chmod +x install.py)










