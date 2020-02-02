# jetson
jetson nano and quadcopter
Problem:  Quadcopter gets armed, gets GPS location but I am not able to spin motors or lift it even one meter, I have tested udacidrone, dronekit-python, mavproxy.py, ardupilot in all cases it is not able to takeoff. 

Definitely I am doing something wrong but after days of work I am still not able to figure out what should be changed and what I have to modify. I need help from experts in this area. On web I have not seen implementation of Jetson nano with quadcopter however NVIDIA’s redtail github (https://github.com/NVIDIA-AI-IOT/redtail)  supports jetsonTX1 and TX2.  There is some initiative for jetson nano (https://github.com/mtbsteve/redtail) however its not ready to implement and test.
I have tested this F450 assembly using FS-i6 transmitter and it fly’s , so  there is no issue with quadcopter , it is something related to connection between jetson-nano and pixhawk . 
 
I NEED HELP AND GUIDANCE FOR jetson-nano and pixhawk implementation on Quadcopter (please do not point me towards available solution for TX1, TX2, XAVIER or Pi4, I am exclusively trying to implement with nano). I have used the same jetson-nano for jetbots and door camera etc. so i am sure there is no problem or issue with jetson-nano 

I think 

Companion computer: Jetson-nano 
Quadcopter: F450 Frame + PXI PX4 Flight Control + 920KV Motor + GPS + FS-i6 Transmitter      (https://www.amazon.com/gp/product/B01HEQQDNK/ref=ppx_yo_dt_b_asin_title_o06_s00?ie=UTF8&psc=1)  . 

Details in the word document:
