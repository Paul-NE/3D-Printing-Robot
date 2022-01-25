from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

import tkinter as tk
from tkinter import Frame,Label,Entry,Button
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import animation

from source.kinematics import Transform, Vector as Vec
from source.kinematics import Quaternion as Quat
from source.kinematics import Transform as Transf

class Window(Frame):
    def __init__(self, master = None):
        Frame.__init__(self, master)
        self.master = master
        self.init_window()

    def Clear(self):
        self.q_speed = np.array([60.]*6)
        self.q_start = np.array([.0]*6)
        for i in range(6):
            self.textSpeed_q[i].delete(0,tk.END)
            self.textSpeed_q[i].insert(0,str(self.q_speed[i]))
        for i in range(6):
            self.textStart_q[i].delete(0,tk.END)
            self.textStart_q[i].insert(0,str(self.q_start[i]))
        self.Plot()

    def Plot(self):
        try:
            for i in range(6):
                self.q_speed[i] = float(self.textSpeed_q[i].get())
        except:
            print('Error on speed setting section')
        try:
            for i in range(6):
                self.q_start[i] = float(self.textStart_q[i].get()) / 180 * np.pi 
        except:
            print('Error on start setting section')
            
    def Save(self):
        try:
            self.ani.save('myAnimation.gif', writer='imagemagick', fps=50)
            tk.messagebox.showinfo(title='Saving', message='Immage has been saved successfully')
        except:
            tk.messagebox.showinfo(title='Saving', message='Something\'s gone wrong!')

    def init_window(self):
        def mk_puma(q, l, ):
            base = Transf.identity()
            shoulder = base + Transf(Vec(0, 0, l[0]), Quat.from_angle_axis(q[0], Vec(0, 0, 1)) * Quat.from_angle_axis(q[1], Vec(1, 0, 0)))
            elbow = shoulder + Transf(Vec(0, 0, l[1]), Quat.from_angle_axis(q[2], Vec(1, 0, 0)) * Quat.from_angle_axis(q[3], Vec(0, 0, 1)))
            wrist = elbow + Transf(Vec(0, 0, l[2]), Quat.from_angle_axis(q[4], Vec(1, 0, 0)) * Quat.from_angle_axis(q[5], Vec(0, 0, 1)))
            flange = wrist + Transf(Vec(0, 0, l[3]), Quat(0, 0 , 0, 1))
            return [base, shoulder, elbow, wrist, flange]

        def mk_axis(transform, size=1.5):
            origin = transform.translation
            x = transform + Vec(size, 0, 0)
            y = transform + Vec(0, size, 0)
            z = transform + Vec(0, 0, size)
            r = ([origin.x, x.x], [origin.y, x.y], [origin.z, x.z])
            g = ([origin.x, y.x], [origin.y, y.y], [origin.z, y.z])
            b = ([origin.x, z.x], [origin.y, z.y], [origin.z, z.z])
            return (r, g, b)

        def animate(t, l, edges, fl_axises, joints, shadow, traec):
            a = np.array([t]*6)
            a *= self.q_speed * np.pi / 180
            a += self.q_start
            vert = mk_puma(a, l)
            for i in range(5):
                joints[i].set_data_3d(vert[i].translation.x, vert[i].translation.y, vert[i].translation.z)
            for i in range(4):
                shadow[i].set_data_3d((vert[i].translation.x, vert[i+1].translation.x), (vert[i].translation.y, vert[i+1].translation.y), (0, 0))
            for i in range(4):
                edges[i].set_data_3d((vert[i].translation.x, vert[i+1].translation.x), (vert[i].translation.y, vert[i+1].translation.y), (vert[i].translation.z, vert[i+1].translation.z))
            axi = mk_axis(vert[4])
            fl_axises['r'].set_data_3d(*axi[0])
            fl_axises['g'].set_data_3d(*axi[1])
            fl_axises['b'].set_data_3d(*axi[2])

            if t:
                tDist = np.arange(0, t, self.tStep)
                transfDist = [None]*len(tDist)
                for i in range(len(tDist)):
                    temp = mk_puma((a - self.q_start)*tDist[i]/t + self.q_start, l)[4]
                    transfDist[i] = (temp.translation.x, temp.translation.y, temp.translation.z)
                transfDist = np.array(transfDist)
                transfDist = transfDist.transpose()
                traec.set_data_3d(*transfDist)

        self.master.title("Python robotics playground")
        self.pack(fill='both', expand=1)     

        self.q_speed = np.array([60.]*6)
        self.q_start = np.array([.0]*6)

#Create the controls, note use of grid
        self.labelSpeed_q = Label(self,text="Speed q (degrees/s)",width=20)
        self.labelSpeed_q.grid(row=0,column=2, columnspan=3)

        self.textSpeed_q = [None]*6
        for i in range(6):
            self.textSpeed_q[i] = Entry(self,width=12)
            self.textSpeed_q[i].insert(0, str(self.q_speed[i]))
            self.textSpeed_q[i].grid(row=1,column=i+2)
        
        self.labelStart_q = Label(self,text="Start q (degrees)",width=12)
        self.labelStart_q.grid(row=3,column=2, columnspan=3)

        self.textStart_q = [None]*6
        for i in range(6):
            self.textStart_q[i] = Entry(self,width=12)
            self.textStart_q[i].insert(0,'0')
            self.textStart_q[i].grid(row=4,column=i+2)

        #self.scalSimTime = tk.Scale(self, orient=tk.HORIZONTAL, length=300, from_=1, to=10, tickinterval=10, resolution=1)
        #self.scalSimTime.grid(row=5,column=2, columnspan=6)

        tk.Label(self,text="Техасская резня роботом").grid(column=0, row=0, columnspan=2)

        self.fig = plt.Figure(figsize=(5, 5))
        self.ax = self.fig.add_subplot(projection='3d')

        l = np.array([2.5, 2.5, 2.5, 2.5])
        vert = mk_puma([0]*6, l)
        
        edges = [self.ax.plot((vert[i].translation.x, vert[i+1].translation.x), (vert[i].translation.y, vert[i+1].translation.y), (vert[i].translation.z, vert[i+1].translation.z), color='#000000', linewidth=2)[0] for i in range(4)]
        fl_coords = mk_axis(vert[4])
        fl_axises = {'r': (self.ax.plot(*fl_coords[0], color='#ff0000'))[0], 
                     'g': self.ax.plot(*fl_coords[1], color='#00ff00')[0], 
                     'b': self.ax.plot(*fl_coords[2], color='#0000ff')[0]}
        shadow = [self.ax.plot((vert[i].translation.x, vert[i+1].translation.x), (vert[i].translation.y, vert[i+1].translation.y), color='#aaaaaa', linewidth=2)[0] for i in range(4)]
        joints = [self.ax.plot(vert[i].translation.x, vert[i].translation.y, vert[i].translation.z, 'ro', markersize=3)[0] for i in range(5)]
        traec, = self.ax.plot((),(),(), color='#aaaaaa')

        self.ax.set_xlim3d([-7.0, 7.0])
        self.ax.set_xlabel('Ось X')

        self.ax.set_ylim3d([-7.0, 7.0])
        self.ax.set_ylabel('Ось Y')

        self.ax.set_zlim3d([0.0, 14.0])
        self.ax.set_zlabel('Ось Z')

        self.canvas = FigureCanvasTkAgg(self.fig, master=self)
        self.canvas.get_tk_widget().grid(column=0,row=1, rowspan=10, columnspan=2)

        self.buttonPlot = Button(self,text="Plot", command=lambda: self.Plot(), width=12)
        self.buttonPlot.grid(row=11,column=0)
        self.buttonClear = Button(self,text="Save",command=self.Save,width=12)
        self.buttonClear.grid(row=11,column=1)
        self.buttonClear = Button(self,text="Clear",command=self.Clear,width=12)
        self.buttonClear.grid(row=11,column=2)

        self.tStep = 0.017
        self.tStart = 0
        self.tEnd = 1
        self.frms = np.arange(self.tStart, self.tEnd, self.tStep)
        self.ani = animation.FuncAnimation(self.fig, animate, frames=self.frms, fargs=(l, edges, fl_axises, joints, shadow, traec), interval=1, blit=False)



root = tk.Tk()
root.geometry("1200x700")
app = Window(root)
tk.mainloop()
