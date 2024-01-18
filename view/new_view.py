import math
import time
import types
import numpy as np
from view.ui.ui import Ui_MainWindow, QtWidgets
import pyqtgraph as pg
import pyqtgraph.opengl as gl
import threading
from submodule.QLogic.src.QLogic import Quaternion


class ViewQVisualiser(Ui_MainWindow):
    def __init__(self, main_window):
        super(ViewQVisualiser, self).__init__()
        self.main_window = main_window
        self.main_window.setFixedSize(964, 643)

        # Змінюємо метод closeEvent для вікна
        self.main_window.closeEvent = types.MethodType(self.__close_event, self.main_window)

        self.setupUi(main_window)

        # Шести-осьова анімація обертання об'єкта в просторі
        self.q_ = Quaternion()

        self.vector_len = 0.1
        self.text_vector_distance = 0.12

        background = (30, 30, 10)

        gx = gl.GLGridItem()
        gx.setSize(x=0.3, y=0.3, z=0.3)
        gx.setSpacing(x=0.01, y=0.01, z=0.01)
        gx.rotate(90, 0, 1, 0)
        gx.translate(-0.15, 0, 0)
        self.six_dof_animation.addItem(gx)

        gy = gl.GLGridItem()
        gy.setSize(x=0.3, y=0.3, z=0.3)
        gy.setSpacing(x=0.01, y=0.01, z=0.01)
        gy.rotate(90, 1, 0, 0)
        gy.translate(0, -0.15, 0)
        self.six_dof_animation.addItem(gy)

        gz = gl.GLGridItem()
        gz.setSize(x=0.3, y=0.3, z=0.3)
        gz.setSpacing(x=0.01, y=0.01, z=0.01)
        gz.translate(0, 0, -0.15)
        self.six_dof_animation.addItem(gz)
        self.zero_point = np.array([0, 0, 0])  # specify the (x, y, z) values of the first point in a tuple

        self.x_point = np.array([0, self.vector_len, 0])  # specify the (x, y, z) values of the second point in a tuple
        self.y_point = np.array([0, 0, self.vector_len])
        self.z_point = np.array([self.vector_len, 0, 0])
        self.rot_vector_point = np.zeros(3)

        self.x_text_point = np.array([0, self.text_vector_distance, 0])
        self.y_text_point = np.array([0, 0, self.text_vector_distance])
        self.z_text_point = np.array([self.text_vector_distance, 0, 0])

        self.x_axis = gl.GLLinePlotItem(pos=np.array([self.zero_point, self.x_point]), width=3)
        self.y_axis = gl.GLLinePlotItem(pos=np.array([self.zero_point, self.y_point]), width=3)
        self.z_axis = gl.GLLinePlotItem(pos=np.array([self.zero_point, self.z_point]), width=3)
        self.rot_vector_axis = gl.GLLinePlotItem(pos=np.array([self.zero_point, self.rot_vector_point]), width=3)

        self.x_axis.setData(color=(1, 0, 0, 1))
        self.y_axis.setData(color=(1, 1, 0, 1))
        self.z_axis.setData(color=(0, 0, 1, 1))
        self.rot_vector_axis.setData(color=(1, 1, 1, 0.5))

        self.six_dof_animation.addItem(self.x_axis)
        self.six_dof_animation.addItem(self.y_axis)
        self.six_dof_animation.addItem(self.z_axis)
        self.six_dof_animation.addItem(self.rot_vector_axis)

        self.text_item_x = gl.GLTextItem(pos=self.x_text_point, text='X', color=(255, 0, 0, 200))
        self.six_dof_animation.addItem(self.text_item_x)

        self.text_item_y = gl.GLTextItem(pos=self.y_text_point, text='Y', color=(255, 255, 0, 200))
        self.six_dof_animation.addItem(self.text_item_y)

        self.text_item_z = gl.GLTextItem(pos=self.z_text_point, text='Z', color=(0, 0, 255, 200))
        self.six_dof_animation.addItem(self.text_item_z)

        self.six_dof_animation.setCameraPosition(distance=0.5)
        self.six_dof_animation.setBackgroundColor(background)

        self.connector()


    def __close_event(self, window, event):
        """
        Callback for close of windows event
        """
        event.accept()

    def connector(self):
        """
        Initialization events with callbacks
        """
        # self.angle.valueChanged.connect(self.callback_angle_vector)
        # self.vx.valueChanged.connect(self.callback_angle_vector)
        # self.vy.valueChanged.connect(self.callback_angle_vector)
        # self.vz.valueChanged.connect(self.callback_angle_vector)

        self.qw.valueChanged.connect(self.callback_q)
        self.qx.valueChanged.connect(self.callback_q)
        self.qy.valueChanged.connect(self.callback_q)
        self.qz.valueChanged.connect(self.callback_q)

        # self.roll.valueChanged.connect(self.callback_euler)
        # self.pitch.valueChanged.connect(self.callback_euler)
        # self.yaw.valueChanged.connect(self.callback_euler)

        # self.bt_play_animation.clicked.connect(self.animation_callback)

    def callback_q(self):
        """
        Callback which callings when changing quaternion double spin boxes
        """
        self.q_.set_using_q(np.array([
            self.qw.value(), self.qx.value(), self.qy.value(), self.qz.value()
        ]))

        # self.q_to_dcm()
        # self.q_to_euler()
        # self.q_to_angle_vector()
        #
        # self.update_euler()
        # self.update_angle_vector()
        self.update()

    def update(self):
        """
        Update normalized quaternion, norm of quaternion, direct cosine matrix and 3D visualisation
        """
        self.teznor.setText(str(self.q_.length))
        vector_val = self.q_.rotation_vector

        self.qwn.setText(str(self.q_.w))
        self.qxn.setText(str(self.q_.x))
        self.qyn.setText(str(self.q_.y))
        self.qzn.setText(str(self.q_.z))
        
        dcm = self.q_.dcm_for_qt
        self.Xz.setText(str(dcm[0][1]))
        self.Xx.setText(str(dcm[1][1]))
        self.Xy.setText(str(dcm[2][1]))

        self.Yz.setText(str(dcm[0][2]))
        self.Yx.setText(str(dcm[1][2]))
        self.Yy.setText(str(dcm[2][2]))

        self.Zz.setText(str(dcm[0][0]))
        self.Zx.setText(str(dcm[1][0]))
        self.Zy.setText(str(dcm[2][0]))

        # Set X vector
        self.x_axis.setData(pos=np.array([np.zeros(3), self.vector_len * dcm[:, 1].copy()]))
        self.text_item_x.setData(pos=self.text_vector_distance * dcm[:, 1].copy())

        # Set Y vector
        self.y_axis.setData(pos=np.array([np.zeros(3), self.vector_len * dcm[:, 2].copy()]))
        self.text_item_y.setData(pos=self.text_vector_distance * dcm[:, 2].copy())

        # Set Z vector
        self.z_axis.setData(pos=np.array([np.zeros(3), self.vector_len * dcm[:, 0].copy()]))
        self.text_item_z.setData(pos=self.text_vector_distance * dcm[:, 0].copy())

        vector = np.array([vector_val[3], vector_val[1], vector_val[2]])
        self.rot_vector_axis.setData(pos=np.array([np.zeros(3), self.vector_len * vector]))