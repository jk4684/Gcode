# Driving a robot using G-code
*Project was created as a part of a seminar of robotics and measurements. 
Seminar is part of the Robotics masters program, on Faculty of electrical engineering - Ljubljana 2020/21*
 
## Description
Project presents user interface, where translator runs and simulated robot with virtual controller. 
Goal of translator is to convert G-code commands into RAPID commands and send them to controller.
They store on a buffer and are executed when a signal is set.

Project was initially intended to be implemented on KUKA IRB1600, but was then only done in ABB's virtual environment 
(RobotStudio) because of the COVID-19 pandemic. That is the reason that robots accuracy wasn't measured and that 
correction field (for making robot absolutely accurate) couldn't be implemented. 

## Installation instructions (Windows)
- Install Python: https://www.python.org/downloads/release/python-382/
- Install Git: https://git-scm.com/
- Clone the repository in a directory of your choice: *git clone git@github.com/jk4684/gkoda_seminar.git*
- Install the python virtual environment manager: *pip install virtualenv virtualenvwrapper-win*
- Setup the project virtual environment:
    - Create the project virtual environment: *mkvirtualenv Gcode*
    - Make sure you are located in the root project directory
    - Set the virtual environment project directory: *setprojectdir .*
    - Install all project-required modules: *pip install -r requirements.txt*
- Setup virtual environment for a robot (RobotStudio):
    - Create a Solution with Station and Virtual Controller in RobotStudio
    - Go to controller tab and under 'Virtual Controller' click on 'Change Options'
    - Then in 'Categories' tab select 'Communication' and check 'PC Interface'
    - Use SERVER.mod to setup server.

## Instructions
1. In RobotStudio run simulation.
2. Then run python script. User interface opens.
3. Connect with 'localhost' and port 5500.
4. Upload your G-code (.ngc) file.
5. Send to Robot.
6. You can also configure tool geometry and world object position and orientation.
7. in folder GcodeFiles are some simple .ngc files for testing.
