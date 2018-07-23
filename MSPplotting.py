import matplotlib.animation as animation
import matplotlib.pyplot as plt
import threading
import numpy as np
from MultiWiiProtocol import MSPio

signal = {'x': [], 'y': [], 'z': []}
fig, ax = plt.subplots()
lines = [ax.plot([], [])[0] for _ in signal.keys()]

timelim = ()


def stream():
	att = ser.read_attitude()
	signal['x'].append(att['x'])
	signal['y'].append(att['y'])
	signal['z'].append(att['heading'])

	if ser.is_open():
		threading.Timer(10/1000., stream).start()


def update(i):
	global timelim
	ax.relim()
	ax.autoscale_view()

	if ax.get_ylim() != timelim:
		timelim = ax.get_ylim()
		fig.canvas.draw()

	for k, line in zip(signal.keys(), lines):
		_, ly = line.get_data()
		ly = np.append(ly, signal[k])
		_xdata = np.arange(ly.size)
		line.set_data(_xdata, ly)
		signal[k] = []

	return lines,


if __name__ == '__main__':
	ser = MSPio(serial_port='/dev/tty.usbmodem14411')
	if ser.is_open():
		ani = animation.FuncAnimation(fig, update, interval=20)
		stream()
		plt.show()
