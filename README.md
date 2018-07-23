# MSPio
A Python implementation of MultiWii Serial Protocol used to configure and control drones

### Setup
You must have `pyserial` installed. You can get it through pip, like: `pip install pyserial`

You should have `matplotlib`and `numpy` installed. You can get those through pip, like `pip install matplotlib numpy`

### How-to...

#### Run an example?

To check your gyros a sample has been included. You must have `matplotlib` and `numpy` installed (see Setup section)

To run it, 

`python3 MSPplotting.py` 
will open a serial connection to your `/dev/ttyUSB0`. I will be adding some cli arguments to make it easier. 

![mspimu](https://user-images.githubusercontent.com/23175380/43070607-9801f564-8e70-11e8-95bf-dcb0b9819fe3.png)


#### Check my motors and comms?

To check your motors, and comms with the board and its devices, just run 

`python3 MSPio.py` 

and the board will arm and run for a few seconds. The system will retrieve some feedback for you, in order to check some parameters are right. 

**IMPORTANT:**
```diff
THE BOARD WILL ARM, AND THEREFORE, THE MOTORS MAY MOVE. 
MAKE SURE YOU HAVE NO PROPS INSTALLED WHEN RUNNING THIS TEST.
```
#### Use the library?

Just clone or download this project. You can import the library as usually do. I will be adding it to `pip`... 
