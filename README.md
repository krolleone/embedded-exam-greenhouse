# Automated greenhouse
Automated greenhouse, Embedded Systems examination

My answer to the examination in Embedded Systems @ Høyskolen Kristiania.
This repo consists of the final product code, Fritzing diagram of my solution, list of components used and a description of the task and system.

I was tasked with creating a system to keep track of sensordata over time for a greenhouse that grows bananas and pineapples in Norway in 2035.
Requirements:
 - Use an ESP32 to connect to a cloud service and display sensordata over time
 - Limited to sensors and components given throughout the course

The system I created, measures temperature inside, outside and at the top of the plant, humidity inside and outside, the level of light at the top of the plant, and soil humidity.
Based on the sensordata, I added components to regulate temperature, open/close windows, change distance between secondary light-sources and top of the plant and a grow lamp that can alternate between optimal light-conditions based on the plants life stage, as well as turn on/off based on current light levels.
I used Blynk as cloud service.

*NOTE*: This is a prototype proof-of-concept, and is in no way capable of being used as an actual solution to a growhouse. I learned a lot during the process of creating this system, and in a given real scenario, I would do a lot of things differently.
