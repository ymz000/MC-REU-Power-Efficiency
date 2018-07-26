import socket
import sys
import re
import matplotlib.pyplot as plt
import matplotlib.animation as animation

MSGLEN = 40
data_point = []
X_LABEL = 'Time / s'
Y_LABEL = 'Average Power / W'

def myreceive():
    chunks = []
    bytes_recd = 0
    while bytes_recd < MSGLEN:
        chunk = sock.recv(min(MSGLEN - bytes_recd, 2048))
        if chunk == b'':
            raise RuntimeError("socket connection broken")
        chunks.append(chunk)
        bytes_recd = bytes_recd + len(chunk)
    return b''.join(chunks)

def data_proc():
    message = myreceive()
    msg = message.decode('utf-8')
    m = re.search("(?<=A: )\d+", msg)
    amp = int(m.group(0))
    m = re.search("(?<=V: )\d+", msg)
    volt = int(m.group(0))
    amp = amp * 3300 / 4096 / 2500 / 524288
    amp = amp * 1.06083 - 0.0004
    volt = volt * 3.3 * 6 / 4096 / 524288
    volt = 1.1685 * volt - 0.084
    print("%.4f A, %.4f V" % (amp, volt))
    pwr = volt * amp
    data_point.append(pwr)

def init():
    line.set_data([], [])
    return line,

def animate(i):
    data_proc()
    x = []
    y = []
    max = 0
    if (i <= (len(data_point) - 40)) :
        for x_pos in range(i, i + 40) :
            x.append(x_pos)
            y.append(data_point[x_pos])
            if data_point[x_pos] > max :
                max = data_point[x_pos]
    ax.clear()
    ax.plot(x, y)
    ax.set_ylim(0, max * 1.1)
    #ax.axis([x_pos, x_pos+40, 0, max])
    ax.grid(True)
    ax.set_xlabel(X_LABEL)
    ax.set_ylabel(Y_LABEL)
    return line,

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

server_address = ('192.168.4.1', 8080)

print('Connecting to %s port %s' % server_address)

sock.connect(server_address)

message = 'Connected'.encode('utf-8')
print('Sending \"%s\"' % message)
sock.sendall(message)

fig = plt.figure()
ax = plt.axes(xlim=(0, 0.6), ylim=(0, 100))
line, = ax.plot([], [], lw=2)

for i in range(1, 40):
    data_point.append(0)

anim = animation.FuncAnimation(fig, animate, interval=100)

plt.show()

sock.close()