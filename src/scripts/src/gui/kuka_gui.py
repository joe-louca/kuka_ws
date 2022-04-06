import tkinter as tk
from tkinter import ttk
import rospy
from std_msgs.msg import Float32MultiArray
from math import pi

class CustomScale(ttk.Scale):
    def __init__(self, master=None, **kw):
        kw.setdefault("orient", "horizontal")
        self.variable = kw.pop('variable', tk.DoubleVar(master))
        ttk.Scale.__init__(self, master, variable=self.variable, **kw)
        self._style_name = '{}.custom.{}.TScale'.format(self, kw['orient'].capitalize()) # unique style name to handle the text
        self['style'] = self._style_name
        self.variable.trace_add('write', self._update_text)
        self._update_text()

    def _update_text(self, *args):
        style.configure(self._style_name, text="{:.1f}".format(self.variable.get()))


def jpos_callback(msg):
    global JPOS
    JPOS = msg.data



trough = b'\x89PNG\r\n\x1a\n\x00\x00\x00\rIHDR\x00\x00\x00\x02\x00\x00\x00\x02\x08\x06\x00\x00\x00r\xb6\r$\x00\x00\x00\tpHYs\x00\x00\x0e\xc3\x00\x00\x0e\xc3\x01\xc7o\xa8d\x00\x00\x00\x19tEXtSoftware\x00www.inkscape.org\x9b\xee<\x1a\x00\x00\x00\x15IDAT\x08\x99c\\\xb5j\xd5\x7f\x06\x06\x06\x06&\x06(\x00\x00.\x08\x03\x01\xa5\\\x04^\x00\x00\x00\x00IEND\xaeB`\x82'
slider = b'\x89PNG\r\n\x1a\n\x00\x00\x00\rIHDR\x00\x00\x000\x00\x00\x000\x08\x06\x00\x00\x00W\x02\xf9\x87\x00\x00\x00\x04sBIT\x08\x08\x08\x08|\x08d\x88\x00\x00\x00\tpHYs\x00\x00\x1d\x87\x00\x00\x1d\x87\x01\x8f\xe5\xf1e\x00\x00\x00\x19tEXtSoftware\x00www.inkscape.org\x9b\xee<\x1a\x00\x00\x05\xa4IDATh\x81\xd5\x9a\xcfo\x1bE\x14\xc7?3\xcez\x13\xb7k7\xb1\x9c4\x07\xdaT \xe8\x11*\x0e\xe5\x04A\xaa\x80\x1cP\x13\xe2\xcd\x85\x1eB\xab\x1e\x81\x0b\xe2ohOpD\xa8p(\x95\xaa\x8d\xa0\xad\x04F\xfc\x90\xc2\xad\x1c\xa0\xaa8\x15\xd1\xaa\xa4\x97\xd6?b\xe2\x9f\xb1\xb3\xf6>\x0e\xf6\x86%q\x9a8mj\xf7s\xb2gv\xc7\xdf7\x9e\x9d7\xef\xbdU<\x06\x16\x17\x17\x07\xd2\xe9\xf4\xcb\xa1P\xe8\xb8\xe7yG\x95R\xcf\x01\x07\x80X\xfb\x92\x02\xb0""\xb7\xb5\xd6\xb7\x94R\xd7\xe3\xf1\xf8\xef\x93\x93\x93\x8dG\xfdm\xf5(\xa2s\xb9\xdc\x9b\xc0)\x11y\x0b\xb0\xba\x1c\xa2(")\xa5\xd4W\x89D\xe2\x87\xdd\x1a\xd3\xb5\x01\x8e\xe3\x0c\x01\xa7\x81\x8f\x80C~\xbba\x18\x0c\r\ra\x9a&\x86a\x10\n\x85\xd0Z\x03\xe0y\x1e\xcdf\x13\xd7u\xa9\xd7\xeb\xd4j5\xd6\xd6\xd6\x82\xc3\xde\x13\x91\xf3\xd5j\xf5\xc2\xfc\xfc|m\xcf\x0cXXX\x98\x11\x91O\x80g|\xd1\xd1h\x14\xcb\xb2\x18\x18\x18\xe8f(\x1a\x8d\x06\xa5R\x89b\xb1\x88\xeb\xba~\xf3\x92R\xea\x83d2ym\xa7\xe3\xec\xc8\x80K\x97.\r\x1b\x86q\x01\x98\x060M\x93\x91\x91\x11\xf6\xed\xdb\xd7\x95\xe8N\x88\x08\xd5j\x95|>O\xbd^\xf7\x9b\xbf6\x0c\xe3\xcc\xf4\xf4\xf4\xcav\xf7ok\xc0\xe5\xcb\x97\x8fi\xad\xbf\x01\x0ek\xad\x89\xc7\xe3D\xa3Q\x94\xda\xf5\xe3\xb3%\x85B\x81\xe5\xe5e<\xcf\x03\xb8\x0b\xcc\xd8\xb6}\xf3a\xf7<T\xc5\xc2\xc2\xc2k"r\r\x88\x9a\xa6\xc9\xf8\xf8x\xd7K\xa5[\x1a\x8d\x06\x0f\x1e<\xa0V\xab\x01\x94\x95R\xef$\x93\xc9\x1f\xb7\xba~K\x03\x1c\xc7y\x1dH\x01\xa6eY\x8c\x8e\x8e\xee\xc9\xacwBD\xc8d2\x94J%\x80\x9a\x88L\xcd\xcd\xcd-v\xba\xb6\xa3\xa2\xf6\xb2\xf9\x05\xb0b\xb1\x18\x89Db\xef\xd4n\x81\x88\x90\xcb\xe5(\x14\n\x00E\xe0\xd5N\xcbi\x93\x01W\xae\\9\xe0\xba\xee\r\xe0\xc8\x93\x9e\xf9Nd2\x19\x8a\xc5"\xc0=\xe0%\xdb\xb6\xf3\xc1~\xbd\xf1\x06\xd7u\xbf\x00\x8e\x98\xa6\xd9s\xf1\x00\x89D\x82\xc1\xc1Ah\xf9\x9c\xcf6\xf6\xff\xcf\x00\xc7q\xa6\x81i\xad5\xe3\xe3\xe3=\x17\x0f\xa0\x94bll\xccw\x8a\xb3\x8e\xe3\xbc\x1d\xec_7\xa0\xeda?\x01\x88\xc7\xe3{\xbe\xdbt\x83a\x18\xc4\xe3q\xff\xeb\xa7\xa9T\xca\xf4\xbf\x04\xff\x81\xd3\xc0!\xd34\x89F\xa3OR\xdf\x8e\x88\xc5b\x98\xa6\t0Q.\x97\xdf\xf3\xdb5\xb4\x0ef\xb4\xce6\x0c\x0f\x0f\xf7\xc5\xd2\xe9\xc4\xc8\xc8\x88\xff\xf1\xe3\xb6\xe6\x96\x01\xd9l\xf6\r\xe0\x90a\x18\xec\xdf\xbf\xbfG\xf2\xb6\'\x12\x89\x10\x0e\x87\x01\x0e\xe7r\xb9\x13\xf0\xdf\x12:\x05\xf4\xe5\xd2\t\xa2\x94\nN\xf0)\x00\xb5\xb8\xb88\x90\xcdf\x97\x81\xe8\xc4\xc4D_=\xbc\x9dp]\x97\xa5\xa5%h\x05Iq\x9dN\xa7_\x06\xa2\x86a\xf4\xbdxh\xedH\x86a\x00\xc4<\xcf;\xa6C\xa1\xd0q\x80\xa1\xa1\xa1\xde*\xeb\x82H$\x02\x80R\xea\x15\xedy\xdeQ\xc0\xdf\xa2\x9e\n\xda\xff\x00\xc0\x0b\xba\x1d\x80\x07\x1b\xfb\x9e\xf6N\x84R\xeay\r\x0c\x03\x84B\xa1^j\xea\x8a\x80\xd6\x03\x1a\xd8\x0f\xf4\xad\xf3\xeaD@\xab\xb5\xe94\xfa\xb4\xa1\x812\xb4\x02\x88\xa7\x85\x80\xd6\x92\x06\xfe\x01h6\x9b=\x13\xd4-\x01\xad\xffh\x11\xb9\r\x04s3}\x8f\xafUD\xfe\xd2Z\xeb[@0\'\xd3\xf7\x04\xb4\xfe\xa9\x9b\xcd\xe6\xaf\x00\xab\xab\xab\xbdS\xd4%\xbeV\x11\xb9\xae\xc7\xc6\xc6~\x03\n\xae\xeb\xd2h<r\xb2x\xcfi4\x1a\xfe\x12Z\xd1Z\xdf\xd0\x93\x93\x93\r\x11\xf9\x1e\xf0\xf30}M@c\xca\xb6\xed\xa6\xef\x07.\x02~\xfa\xa2o\x11\x91\xa0\x01\x17\xa1\x1d\xd0\x8c\x8e\x8e\xfe\x08\xdcs]\x97J\xa5\xd2#y\xdbS\xa9T\xfc\xb4\xfc\xdf\x89D\xe2gh\x1b\xd0^F\xe7\x01\xf2\xf9\xfc\xd6#\xf4\x18_\x9b\x88\x9c\xf3\x0b"\xebG\x89j\xb5z\x01X\xaa\xd7\xeb~:\xaf\xafXYY\xf1g\xff\xaeeY_\xfa\xed\xeb\x06\xcc\xcf\xcf\xd7D\xe4C\x80\xe5\xe5\xe5\xberl\x8dFc}\xf6\x95R\xefOMM\xad;\x82\xff\x1d\xe6\xe6\xe6\xe6\xae\x02_{\x9eG:\x9d\xee\x8b\xf3\x91\x88p\xff\xfe}<\xcfC)\xe5$\x93\xc9o\x83\xfd\x9bN\xa3\x86a\x9c\x01\xee\xd6j\xb5\xbe0"\x93\xc9\xf8\x9e\xf7\x8e\x88\x9c\xdd\xd8\xbf\xc9\x80vYg\x06(\x96\xcber\xb9\\O\x8c\x10\x11\xb2\xd9\xac\xbfm\x16\xb4\xd63\xb6moz8;\xc6\x03\xb6m\xdf\x14\x91\x93@\xadP(\x90\xc9d\x9e\xa8\x11"B:\x9d\xf67\x93\x9aR\xea\xe4\xec\xec\xec\x1f\x9d\xae\xddI\x89\xe9*\x103M\x93\x83\x07\x0f\xeey\xec\xbc\xb1\xc4D\xabN\xf6\xd3V\xd7o\x1bG:\x8e\xf3"\xf0\rp\xc4/\xf2\xc5b\xb1\xedn\xdb\x15\x1b\x8a|w\xb4\xd63[\xcd\xbc\xcf\x8e\x02\xe1v\xd5\xe6s`\x16\xfe+\xb3F"\x91G\x8e\xa5E\x84J\xa5B>\x9f_/~+\xa5\x1c\x119\xdbi\xcdo\xa4\xab_o\x17\x17>\x05&\xa0\x95\xde\xb0,kW\x85n\xd7u)\x97\xcb\x94J\xa5`\xd5\xfe\xaeR\xea\xfd\x8d[\xe5\xc3\xe8z\xfaR\xa9\x94\xd9\xce\xcf\x7f\x0c\x1c\xf6\xdb\xc3\xe10\x83\x83\x83\x84\xc3a\xc2\xe1\xf0\xfa\xab\x06"\x82\x88\xac\x1f\x83\xd7\xd6\xd6X]]\xdd\xe8(\xff\x16\x91s\x96e}\x19tR{b\x80O\xfbe\x8f\x13"\xf2.0E\xeb\xed\x94nX\x11\x91\xef\x94R_\x01?\xd9\xb6\xbd\xab\xa0\xfc\xb1$\x83\x1c\xc7\ty\x9ewL)\xf5\x8aR\xea(\xf0,\x10\x07\xfc|}\x11X\xa6\xe5\x8cn\x89\xc8u\xad\xf5\x8d\xdd\x8a\x0e\xf2/#\xf8\x81 \xf2;_\x08\x00\x00\x00\x00IEND\xaeB`\x82'

root = tk.Tk()
root.geometry("1000x800")
root.title('Joint Positions')
root.resizable(0, 6)

## Create custom slider
# create images used for the theme
img_trough = tk.PhotoImage(master=root, data=trough)
img_slider = tk.PhotoImage(master=root, data=slider)
style = ttk.Style(root)
# create scale elements
style.element_create('custom.Scale.trough', 'image', img_trough)
style.element_create('custom.Scale.slider', 'image', img_slider)
# create custom layout
style.layout('custom.Horizontal.TScale',
             [('custom.Scale.trough', {'sticky': 'ew'}),
              ('custom.Scale.slider',
               {'side': 'left', 'sticky': '',
                'children': [('custom.Horizontal.Scale.label', {'sticky': ''})]
                })])

# configure the grid
root.columnconfigure(0, weight=1)
root.columnconfigure(1, weight=3)

tk.Label(text="Joint 1",foreground="gray",background="peach puff",width=7,height=3).grid(row=0,column=0,sticky="nsew",pady = 1)
scl_1 = CustomScale(root, from_=-175, to=175)
scl_1.grid(row=0,column=1,sticky='ew',pady = 1,padx = 10)

tk.Label(text="Joint 2",foreground="gray",background="PaleTurquoise1",width=7,height=3).grid(row=1,column=0,sticky="nsew",pady = 1)
scl_2 = CustomScale(root, from_=-120, to=120)
scl_2.grid(row=1,column=1,sticky='ew',pady = 1,padx = 125)

tk.Label(text="Joint 3",foreground="gray",background="peach puff",width=7,height=3).grid(row=2,column=0,sticky="nsew",pady = 1)
scl_3 = CustomScale(root, from_=-175, to=175)
scl_3.grid(row=2,column=1,sticky='ew',pady = 1,padx = 10)

tk.Label(text="Joint 4",foreground="gray",background="PaleTurquoise1",width=7,height=3).grid(row=3,column=0,sticky="nsew",pady = 1)
scl_4 = CustomScale(root, from_=-120, to=120)
scl_4.grid(row=3,column=1,sticky='ew',pady = 1,padx = 125)

tk.Label(text="Joint 5",foreground="gray",background="peach puff",width=7,height=3).grid(row=4,column=0,sticky="nsew",pady = 1)
scl_5 = CustomScale(root, from_=-175, to=175)
scl_5.grid(row=4,column=1,sticky='ew',pady = 1,padx = 10)

tk.Label(text="Joint 6",foreground="gray",background="PaleTurquoise1",width=7,height=3).grid(row=5,column=0,sticky="nsew",pady = 1)
scl_6 = CustomScale(root, from_=-120, to=120)
scl_6.grid(row=5,column=1,sticky='ew',pady = 1,padx = 125)

tk.Label(text="Joint 7",foreground="gray",background="peach puff",width=7,height=3).grid(row=6,column=0,sticky="nsew",pady = 1)
scl_7 = CustomScale(root, from_=-175, to=175)
scl_7.grid(row=6,column=1,sticky='ew',pady = 1,padx = 10)


tk.Label(text=" ",foreground="black",background="snow",width=7,height=3).grid(row=7,column=0,sticky="nsew")
tk.Label(text=" ",foreground="black",background="snow",width=7,height=3).grid(row=7,column=1,sticky="nsew")


tk.Label(text="Scale Force Feedback (%)",foreground="black",background="coral1",width=7,height=3).grid(row=8,column=0,sticky="nsew",pady = 1)
scl_ft = tk.Scale(master=root, from_=0, to=100, orient=tk.HORIZONTAL)
scl_ft.grid(row=8,column=1,sticky="nsew")
scl_ft.set(50)

tk.Label(text="Scale Speed (%)",foreground="black",background="burlywood1",width=7,height=3).grid(row=9,column=0,sticky="nsew",pady= 1)
scl_spd = tk.Scale(master=root, from_=0, to=100, orient=tk.HORIZONTAL)
scl_spd.grid(row=9,column=1,sticky="nsew")
scl_spd.set(50)

#tk.Label(text="Latency (ms)",foreground="black",background="coral1",width=7,height=3).grid(row=10,column=0,sticky="nsew",pady = 1)
#scl_lat = tk.Scale(master=root, from_=0, to=2600, orient=tk.HORIZONTAL, resolution=50)
#scl_lat.grid(row=10,column=1,sticky="nsew")


# Set up a variable for JPOS
global JPOS
JPOS = [0, 0, 0, 0, 0, 0, 0]

rospy.init_node('gui', anonymous=True)
sub = rospy.Subscriber("/delayed_jpos_cmd", Float32MultiArray, jpos_callback, queue_size=1)    

rate_hz = rospy.get_param('rate_hz')
r = rospy.Rate(rate_hz)



while not rospy.is_shutdown():
    ft_user_scale = scl_ft.get()
    rospy.set_param('ft_user_scale', ft_user_scale)

    ws_user_scale = scl_spd.get()
    rospy.set_param('ws_user_scale', ws_user_scale)

    #latency = scl_lat.get()
    #latency = latency/2000.0
    #rospy.set_param('latency', latency)

    scl_1.set(JPOS[0]*180/pi)
    scl_2.set(JPOS[1]*180/pi)
    scl_3.set(JPOS[2]*180/pi)
    scl_4.set(JPOS[3]*180/pi)
    scl_5.set(JPOS[4]*180/pi)
    scl_6.set(JPOS[5]*180/pi)
    scl_7.set(JPOS[6]*180/pi)

    """if rospy.has_param('jpos_cmd'):
        j1 = rospy.get_param('jpos_cmd/j1')
        j2 = rospy.get_param('jpos_cmd/j2')
        j3 = rospy.get_param('jpos_cmd/j3')
        j4 = rospy.get_param('jpos_cmd/j4')
        j5 = rospy.get_param('jpos_cmd/j5') # This is the error
        j6 = rospy.get_param('jpos_cmd/j6')
        j7 = rospy.get_param('jpos_cmd/j7')
        scl_1.set(j1)
        scl_2.set(j2)
        scl_3.set(j3)
        scl_4.set(j4)
        scl_5.set(j5)
        scl_6.set(j6)
        scl_7.set(j7)
    """
    root.update()
    r.sleep()

       
