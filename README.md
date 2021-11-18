# Smart Clock

Project using Node MCU, STM NUCLEO-L476RG boards, for the hardware, and ReactJS and Firebase for the softwares.
It consists in creating a smart clock where you can add your tasks with day and hour, and when the time of your task
come, the screen of the clock will show the task description and the buzzer conected with the NUCLEO will starts
to beep until you click the button, when you do the buzzer stops beeping and the screen of the clock will show the
hours again. 

The comunication is done this way: 
- Front-end -> Firebase
- Firebase -> NodeMCU (and the other way too)
- NodeMCU -> Nucleo

## Running project
- Connect the NUCLEO and the NodeMCU first
- Run the .INO code in NodeMCU
- Run the STM project in NUCLEO
- Start the front-end project
- Connect a buzzer in the D7 port
