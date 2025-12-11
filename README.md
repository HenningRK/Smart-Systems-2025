--------------------------------------------------------- Smart Systems 2025 ------------------------------------------------------------------------------------
-----Introduction-----
AI/GUI code performed through QT-programming language. Made for at university project at USN. Made for commucation between AI --> Raspberry pi --> Automatic car. 
The AI is programmed in a way that it can describe enviorments through camera, solve mazes(through BFS), explain mazes for directions in coding language and normal conversation with added memory.

-----Chatwindow.cpp-----
Inside here is the main code for all of the AI. On the mainwindow this would be the "Connect to AI".
Everything from restriction to the drawing tool and every function in which i mentioned above.

-----Chatwindow.h-----
This is just a header-file for the AI.
Private slots and variables which is used in the "Chatwindow.cpp"-code.

-----Dashboard.cpp-----
This file contains the code for capturing data from the car. (Needs changes so it can import the Raspberry Pi data).
The code only functions manually and will get data as long as its directly connected to the car. 
In mainwindow this is refered to the "Sensor Data" button.

-----Dashboard.h-----
Private slots and variables for the "Dasboard.cpp"-code

-----Dashboard.ui-----
This is the window where data is extracted and to where i can manually change the view.
Right now it only contains two labels which takes in sensor 1 and sensor 2.

-----Main.cpp-----
Main.cpp only function is to make sure that the code runs.
Has no special funcitons that affects the sytem. Only there so that the program can run in general.

-----Mainwindow.cpp-----
This is the Mainwindow or the main GUI.
Contains two buttons "connect to AI" and "Sensor data". 
Connects the buttons and makes sure that we get a new window when each button is pressed which let us interact with the different code.

-----Mainwindow.h-----
Private slots and private variables to "Mainwindow.cpp".
