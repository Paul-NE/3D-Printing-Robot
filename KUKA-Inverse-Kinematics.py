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

import source.graphics as graphics

class Window(Frame):
    def __init__(self, master = None):
        Frame.__init__(self, master)
        self.master = master
        self.init_window()

    def Clear(self):
        pass
        self.Plot()

    def Plot(self):
        try:
            xyz = [0]*3
            for i in range(3):
                xyz[i] = float(self.startPose_E[i].get())
                
            self.s.translation.x = xyz[0]
            self.s.translation.y = xyz[1]
            self.s.translation.z = xyz[2]
        except:
            print('Error on speed setting section')
        #self.s.translation = 
        #self.e.translation = 
            
    def Save(self):
        try:
            self.ani.save('myAnimation.gif', writer='imagemagick', fps=50)
            tk.messagebox.showinfo(title='Saving', message='Immage has been saved successfully')
        except:
            tk.messagebox.showinfo(title='Saving', message='Something\'s gone wrong!')

    def init_window(self):
        def irb_chain(q, l, ):
            base = Transform.identity()
            column = base + Transform(
                Vec(0, 0, l[0]),
                Quat.from_angle_axis(q[0], Vec(0, 0, 1))
            )
            shoulder = column + Transform(
                Vec(l[1], 0, 0),
                Quat.from_angle_axis(q[1], Vec(0, -1, 0))
            )
            elbow = shoulder + Transform(
                Vec(0, 0, l[2]),
                Quat.from_angle_axis(q[2], Vec(0, 1, 0))
            )
            wrist = elbow + Transform(
                Vec(l[3], 0, 0),
                Quat.from_angle_axis(q[3], Vec(1, 0, 0)) *
                Quat.from_angle_axis(q[4], Vec(0, 1, 0))
            )
            flange = wrist + Transform(
                Vec(l[4], 0, 0),
                Quat.from_angle_axis(q[5], Vec(1, 0, 0)) *
                Quat.from_angle_axis(np.pi / 2, Vec(0, 1, 0))
            )
            return [
                base,
                column,
                shoulder,
                elbow,
                wrist,
                flange
            ]
        
        def wrap_from_to(value, s, e):
            r = e - s
            return value - (r * np.floor((value - s) / r))

        def irb_ik(target, l, i=[1, 1, 1]):
            wrist = target + Vec(0, 0, -l[4]) +  Vec(0, 0, -l[0])
            projection = Vec(wrist.x, wrist.y, 0)
            q0 = Vec(0, 1, 0).angle_to(projection, Vec(0, 0, 1)) - np.pi / 2 * i[0] + np.pi
            d = ((projection.magnitude() - i[0] * l[1]) ** 2 + wrist.z ** 2) ** 0.5
            q2 = -i[1] * np.arccos(
                (l[2] **  2 + l[3] ** 2 - d ** 2) /\
                (2 * l[2] * l[3])
            ) + np.pi / 2
            triangle_angle = np.arcsin(
                l[3] * i[0] * np.sin(q2 - np.pi / 2) / d
            )
            lift_angle = np.arctan2(
                wrist.z,
                (projection.magnitude() - i[0] * l[1])
            )
            q1 = -i[0] * (np.pi / 2 + triangle_angle - lift_angle)
            ori = Quat.from_angle_axis(q0, Vec(0, 0, 1)) *\
                Quat.from_angle_axis(q1, Vec(0, -1, 0)) *\
                Quat.from_angle_axis(q2, Vec(0, 1, 0))
            ez = ori * Vec(1, 0, 0)
            ey = ori * Vec(0, 1, 0)
            tz = target.rotation * Vec(0, 0, 1)
            ty = target.rotation * Vec(0, 1, 0)
            wy = ez.cross(tz)
            q3 = ey.angle_to(wy, ez) + np.pi / 2 - np.pi / 2 * i[2]
            q4 = ez.angle_to(tz, wy) * i[2]
            q5 = wy.angle_to(ty, tz) + np.pi / 2 -np.pi / 2 * i[2]
            return (
                wrap_from_to(q0, -np.pi, np.pi),
                wrap_from_to(q1, -np.pi, np.pi),
                wrap_from_to(q2, -np.pi, np.pi),
                wrap_from_to(q3, -np.pi, np.pi),
                wrap_from_to(q4, -np.pi, np.pi),
                wrap_from_to(q5, -np.pi, np.pi)
            )

        def irb_ik_lim(target, l, i=[1, 1, 1]):
            solution = irb_ik(target, l, i)
            for index in range(len(solution)):
                if solution[index] < np.deg2rad(irb_lim[index][0]) or\
                    solution[index] > np.deg2rad(irb_lim[index][1]) or\
                    np.isnan(solution[index]):
                    return None
            return solution

        def lin(start, end, t, total):
            return Transform.lerp(
                start,
                end,
                t / total
            )

        def animate(t):
            origin = self.s.translation
            size = 100
            x = self.s + Vec(size, 0, 0)
            y = self.s + Vec(0, size, 0)
            z = self.s + Vec(0, 0, size)
            rs.set_data_3d([origin.x, x.x], [origin.y, x.y], [origin.z, x.z])
            gs.set_data_3d([origin.x, y.x], [origin.y, y.y], [origin.z, y.z])
            bs.set_data_3d([origin.x, z.x], [origin.y, z.y], [origin.z, z.z])

            trs = lin(self.s, self.e, t, total)
            q = irb_ik_lim(
                trs,
                irb_l,
                irb_i
            )
            if q != None:
                chain = irb_chain(q, irb_l)
                (x, y, z) = graphics.chain_to_points(chain)
                lines.set_data_3d(x, y, z)

        #A bunch of constants
        irb_l = [352.0, 70.0, 350.0, 380.0, 65.0]

        irb_lim = [
            (-180, 180),
            (-90, 110),
            (-230, 50),
            (-200, 200),
            (-115, 115),
            (-400, 400)
        ]

        self.s = Transform(
            Vec(200, 400, 600),
            Quat.from_angle_axis(np.pi / 2, Vec(-1, 0, 0))
        )
        self.e = Transform(
            Vec(0, -300, 800),
            Quat.from_angle_axis(np.pi / 2, Vec(0, 1, 0))
        )
        irb_i = [1, 1, -1]

        self.master.title("Python robotics playground")
        self.pack(fill='both', expand=1)     
        tk.Label(self,text="Техасская резня роботом").grid(column=0, row=0, columnspan=2)
        self.fig = plt.Figure(figsize=(5, 5))
        self.ax = self.fig.add_subplot(projection='3d')

#Поля для заполнения
        self.startPose_L = Label(self,text="Start pose x y z alpha betta gamma",width=20)
        self.startPose_L.grid(row=0,column=2, columnspan=3)

        self.startPose_E = [None]*6
        for i in range(6):
            self.startPose_E[i] = Entry(self,width=12)
            self.startPose_E[i].insert(0, str(0))
            self.startPose_E[i].grid(row=1,column=i+2)

        self.ax.set_xlim3d([-1000.0, 1000.0])
        self.ax.set_xlabel('Ось X')

        self.ax.set_ylim3d([-1000.0, 1000.0])
        self.ax.set_ylabel('Ось Y')

        self.ax.set_zlim3d([0.0, 1000.0])
        self.ax.set_zlabel('Ось Z')

        (x, y, z) = graphics.chain_to_points(
            irb_chain([0, 0, 0, 0, 0, 0], irb_l)
        )

        lines, = self.ax.plot(x, y, z, color="#000000")
        rs, gs, bs = graphics.axis(self.ax, self.s, 100)
        graphics.axis(self.ax, self.e, 100)
        total = 100

        self.canvas = FigureCanvasTkAgg(self.fig, master=self)
        self.canvas.get_tk_widget().grid(column=0,row=1, rowspan=10, columnspan=2)

        self.buttonPlot = Button(self,text="Plot", command=lambda: self.Plot(), width=12)
        self.buttonPlot.grid(row=11,column=0)
        self.buttonClear = Button(self,text="Save",command=self.Save,width=12)
        self.buttonClear.grid(row=11,column=1)
        self.buttonClear = Button(self,text="Clear",command=self.Clear,width=12)
        self.buttonClear.grid(row=11,column=2)

        animate(0)
        fps = 30
        self.ani = animation.FuncAnimation(self.fig, animate, frames=total, fargs=(None), interval=1000/fps, blit=False)


if __name__ == "__main__":
    root = tk.Tk()
    root.geometry("1200x700")
    app = Window(root)
    tk.mainloop()
