# # from pynput import mouse
# #
# # # Function to handle mouse click events
# # def on_click(x, y, button, pressed):
# #     if pressed:
# #         print(f"Mouse clicked at ({x}, {y}) with {button}")
# #     else:
# #         print(f"Mouse released at ({x}, {y}) with {button}")
# #
# # # Create a listener for mouse events
# # with mouse.Listener(on_click=on_click) as listener:
# #     listener.join()
# # #
# # from matplotlib import pyplot as plt
# #
# # class LineBuilder:
# #     def __init__(self, line):
# #         self.line = line
# #         self.xs = list(line.get_xdata())
# #         self.ys = list(line.get_ydata())
# #         self.cid = line.figure.canvas.mpl_connect('button_press_event', self)
# #
# #     def __call__(self, event):
# #         print('click', event)
# #         if event.inaxes!=self.line.axes: return
# #         self.xs.append(event.xdata)
# #         self.ys.append(event.ydata)
# #         self.line.set_data(self.xs, self.ys)
# #         self.line.figure.canvas.draw()
# #
# # fig = plt.figure()
# # ax = fig.add_subplot(111)
# # ax.set_title('click to build line segments')
# # line, = ax.plot([0], [0])  # empty line
# # linebuilder = LineBuilder(line)
# #
# # plt.show()
#
#
#
# import matplotlib.pyplot as plt
#
#
# fig, ax = plt.subplots()
#
# ax.set_xlim(0, 1920-1)
# ax.set_ylim(0, 1080-1)
#
# x,y = [0], [0]
# # create empty plot
# points, = ax.plot([], [], 'o')
#
# # cache the background
# background = fig.canvas.copy_from_bbox(ax.bbox)
#
# # def on_move(event):
# #     # append event's data to lists
# #     x.append(event.xdata)
# #     y.append(event.ydata)
# #     # update plot's data
# #     points.set_data(x,y)
# #     # restore background
# #     fig.canvas.restore_region(background)
# #     # redraw just the points
# #     ax.draw_artist(points)
# #     # fill in the axes rectangle
# #     fig.canvas.blit(ax.bbox)
#
#
# def onclick(event):
#     print('%s click: button=%d, x=%d, y=%d, xdata=%f, ydata=%f' %
#           ('double' if event.dblclick else 'single', event.button,
#            event.x, event.y, event.xdata, event.ydata))
#
# plt.show()
# while(True):
#     cid = fig.canvas.mpl_connect('button_press_event', onclick)
#     print("ss")
#
#
#
#
#
# # fig.canvas.mpl_connect("motion_notify_event", on_move)
from matplotlib import pyplot as plt

class LineBuilder:
    def __init__(self, line):
        self.line = line
        self.xs = list(line.get_xdata())
        self.ys = list(line.get_ydata())
        self.cid = line.figure.canvas.mpl_connect('button_press_event', self)

    def __call__(self, event):
        print('click', event)
        if event.inaxes!=self.line.axes: return
        self.xs.append(event.xdata)
        self.ys.append(event.ydata)
        self.line.set_data(self.xs, self.ys)
        self.line.figure.canvas.draw()

fig, ax = plt.subplots()
ax.set_title('click to build line segments')
line, = ax.plot([0], [0])  # empty line
linebuilder = LineBuilder(line)

plt.show()