import math
import types
import numpy as np
from view.ui.ui import Ui_MainWindow, QtWidgets
import pyqtgraph as pg
import pyqtgraph.opengl as gl


class ViewDcmTester(Ui_MainWindow):
    def __init__(self, main_window):
        super(ViewDcmTester, self).__init__()
        self.main_window = main_window
        self.main_window.setFixedSize(964, 643)

        # Змінюємо метод closeEvent для вікна
        self.main_window.closeEvent = types.MethodType(self.close_event, self.main_window)

        self.setupUi(main_window)

        # Шести-осьова анімація обертання об'єкта в просторі
        self.vector_len = 0.1
        self.text_vector_distance = 0.12
        self.dcm = np.eye(3, dtype=np.float64)
        self.euler = np.zeros(3, dtype=np.float64)
        self.angle_val = 0
        self.vector_val = np.zeros(3, dtype=np.float64)
        self.q = np.array([1, 0, 0, 0], dtype=np.float64)
        self.q_norm = np.array([1, 0, 0, 0], dtype=np.float64)
        self.norm_val = 0

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

        self.x_axis = gl.GLLinePlotItem(pos=np.array([self.zero_point, self.x_point]), width=6)
        self.y_axis = gl.GLLinePlotItem(pos=np.array([self.zero_point, self.y_point]), width=6)
        self.z_axis = gl.GLLinePlotItem(pos=np.array([self.zero_point, self.z_point]), width=6)
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

    def connector(self):
        """
        Initialization events with callbacks
        """
        self.angle.valueChanged.connect(self.callback_angle_vector)
        self.vx.valueChanged.connect(self.callback_angle_vector)
        self.vy.valueChanged.connect(self.callback_angle_vector)
        self.vz.valueChanged.connect(self.callback_angle_vector)

        self.qw.valueChanged.connect(self.callback_q)
        self.qx.valueChanged.connect(self.callback_q)
        self.qy.valueChanged.connect(self.callback_q)
        self.qz.valueChanged.connect(self.callback_q)

        self.roll.valueChanged.connect(self.callback_euler)
        self.pitch.valueChanged.connect(self.callback_euler)
        self.yaw.valueChanged.connect(self.callback_euler)

    def close_event(self, window, event):
        """
        Callback for close of windows event
        """
        reply = QtWidgets.QMessageBox.question(self.main_window, "Attention!",
                                               "Do you want to close window?\n\n")
        if reply == QtWidgets.QMessageBox.Yes:
            event.accept()
        else:
            event.ignore()

    # ========== MATH ==========
    def q_to_dcm(self):
        """
        Calculating DCM using quaternion
        """
        w, x, y, z = self.q_norm

        mXx = 1.0 - 2.0 * (y ** 2 + z ** 2)
        mXy = 2.0 * (x * y + w * z)
        mXz = 2.0 * (x * z - y * w)

        mYx = 2.0 * x * y - 2 * z * w
        mYy = 1.0 - 2.0 * x ** 2 - 2 * z ** 2
        mYz = 2.0 * (y * z + x * w)

        mZx = 2.0 * (x * z + y * w)
        mZy = 2.0 * (y * z - x * w)
        mZz = 1.0 - 2.0 * (x ** 2 + y ** 2)

        dcm = np.array([
            [mZz, mXz, mYz],
            [mZx, mXx, mYx],
            [mZy, mXy, mYy]
        ])

        self.dcm = np.around(dcm, decimals=4)

    def q_from_euler(self, roll, pitch, yaw):
        """
        Calculating quaternion using Euler angles
        """
        c1 = np.cos(yaw * 0.5)
        c2 = np.cos(pitch * 0.5)
        c3 = np.cos(roll * 0.5)
        s1 = np.sin(yaw * 0.5)
        s2 = np.sin(pitch * 0.5)
        s3 = np.sin(roll * 0.5)

        w = c1 * c2 * c3 - s1 * s2 * s3
        x = s1 * s2 * c3 + c1 * c2 * s3
        y = s1 * c2 * c3 + c1 * s2 * s3
        z = c1 * s2 * c3 - s1 * c2 * s3

        self.q = np.around(np.array([w, x, y, z]), decimals=4)

    def q_to_euler(self):
        """
        Calculating Euler angles using quaternion
        """
        qx2 = self.q[1] ** 2
        qy2 = self.q[2] ** 2
        qz2 = self.q[3] ** 2
        self.euler[0] = np.arctan2(2 * self.q[1] * self.q[0] - 2 * self.q[2] * self.q[3],
                                   1 - 2 * qx2 - 2 * qz2) * 180 / math.pi
        self.euler[2] = -np.arcsin(2 * self.q[1] * self.q[2] - 2 * self.q[3] * self.q[0]) * 180 / math.pi
        self.euler[1] = np.arctan2(2 * self.q[2] * self.q[0] - 2 * self.q[1] * self.q[3],
                                   1 - 2 * qy2 - 2 * qz2) * 180 / math.pi

    def q_to_angle_vector(self):
        """
        Calculating of vector and rotation angle around this vector
        """
        self.angle_val = np.rad2deg(2 * np.arccos(self.q_norm[0]))

        power_sum = self.q_norm[1]**2 + self.q_norm[2]**2 + self.q_norm[3]**2
        vector_len = np.linalg.norm(np.sqrt(power_sum if power_sum else 1))

        self.vector_val[0] = self.q_norm[1] / vector_len
        self.vector_val[1] = self.q_norm[2] / vector_len
        self.vector_val[2] = self.q_norm[3] / vector_len

    # ========== CALLBACKS ==========

    def callback_euler(self):
        """
        Callback which callings when changing Euler double spin boxes
        """
        roll = np.deg2rad(self.roll.value())
        pitch = np.deg2rad(self.pitch.value())
        yaw = np.deg2rad(self.yaw.value())

        self.q_from_euler(roll, pitch, yaw)

        self.norm_val = np.around(np.linalg.norm(self.q), decimals=4)

        self.q_norm = self.q / self.norm_val
        self.q_norm = np.around(self.q_norm, decimals=4)

        self.q_to_dcm()
        self.q_to_angle_vector()

        self.update_q()
        self.update_angle_vector()
        self.update()

    def callback_angle_vector(self):
        """
        Callback which callings when changing angle or vector double spin boxes
        """
        angle = np.deg2rad(self.angle.value())
        vx = self.vx.value()
        vy = self.vy.value()
        vz = self.vz.value()

        self.angle_val = angle
        self.vector_val[0] = vx
        self.vector_val[1] = vy
        self.vector_val[2] = vz

        self.q[0] = np.cos(angle / 2)
        self.q[1] = np.sin(angle / 2) * vx
        self.q[2] = np.sin(angle / 2) * vy
        self.q[3] = np.sin(angle / 2) * vz

        self.q = np.around(self.q, decimals=4)

        self.norm_val = np.around(np.linalg.norm(self.q), decimals=4)

        self.q_norm = self.q / self.norm_val
        self.q_norm = np.around(self.q_norm, decimals=4)

        self.q_to_dcm()
        self.q_to_euler()
        self.q_to_angle_vector()

        self.update_euler()
        self.update_q()
        self.update()

    def callback_q(self):
        """
        Callback which callings when changing quaternion double spin boxes
        """
        self.q[0] = self.qw.value()
        self.q[1] = self.qx.value()
        self.q[2] = self.qy.value()
        self.q[3] = self.qz.value()

        self.norm_val = np.around(np.linalg.norm(self.q), decimals=4)

        self.q_norm = self.q / (self.norm_val if self.norm_val else 1)
        self.q_norm = np.around(self.q_norm, decimals=4)

        self.q_to_dcm()
        self.q_to_euler()
        self.q_to_angle_vector()

        self.update_euler()
        self.update_angle_vector()
        self.update()

    # ========== UPDATING UI ==========
    def update_euler(self):
        """
        Update Euler angles values in double spin boxes
        """
        self.roll.blockSignals(True)
        self.pitch.blockSignals(True)
        self.yaw.blockSignals(True)

        self.roll.setValue(self.euler[0])
        self.yaw.setValue(self.euler[1])
        self.pitch.setValue(self.euler[2])

        self.roll.blockSignals(False)
        self.pitch.blockSignals(False)
        self.yaw.blockSignals(False)

    def update_q(self):
        """
        Update quaternion values in double spin boxes
        """
        self.qw.blockSignals(True)
        self.qx.blockSignals(True)
        self.qy.blockSignals(True)
        self.qz.blockSignals(True)

        self.qw.setValue(self.q[0])
        self.qx.setValue(self.q[1])
        self.qy.setValue(self.q[2])
        self.qz.setValue(self.q[3])

        self.qw.blockSignals(False)
        self.qx.blockSignals(False)
        self.qy.blockSignals(False)
        self.qz.blockSignals(False)

    def update_angle_vector(self):
        """
        Update angle and vector values in double spin boxes
        """
        self.angle.blockSignals(True)
        self.vx.blockSignals(True)
        self.vy.blockSignals(True)
        self.vz.blockSignals(True)

        self.angle.setValue(self.angle_val)
        self.vx.setValue(self.vector_val[0])
        self.vy.setValue(self.vector_val[1])
        self.vz.setValue(self.vector_val[2])

        self.angle.blockSignals(False)
        self.vx.blockSignals(False)
        self.vy.blockSignals(False)
        self.vz.blockSignals(False)

    def update(self):
        """
        Update normalized quaternion, norm of quaternion, direct cosine matrix and 3D visualisation
        """
        self.teznor.setText(str(self.norm_val))

        self.qwn.setText(str(self.q_norm[0]))
        self.qxn.setText(str(self.q_norm[1]))
        self.qyn.setText(str(self.q_norm[2]))
        self.qzn.setText(str(self.q_norm[3]))

        self.Xz.setText(str(self.dcm[0][1]))
        self.Xx.setText(str(self.dcm[1][1]))
        self.Xy.setText(str(self.dcm[2][1]))

        self.Yz.setText(str(self.dcm[0][2]))
        self.Yx.setText(str(self.dcm[1][2]))
        self.Yy.setText(str(self.dcm[2][2]))

        self.Zz.setText(str(self.dcm[0][0]))
        self.Zx.setText(str(self.dcm[1][0]))
        self.Zy.setText(str(self.dcm[2][0]))

        # Set X vector
        self.x_axis.setData(pos=np.array([np.zeros(3), self.vector_len * self.dcm[:, 1].copy()]))
        self.text_item_x.setData(pos=self.text_vector_distance * self.dcm[:, 1].copy())

        # Set Y vector
        self.y_axis.setData(pos=np.array([np.zeros(3), self.vector_len * self.dcm[:, 2].copy()]))
        self.text_item_y.setData(pos=self.text_vector_distance * self.dcm[:, 2].copy())

        # Set Z vector
        self.z_axis.setData(pos=np.array([np.zeros(3), self.vector_len * self.dcm[:, 0].copy()]))
        self.text_item_z.setData(pos=self.text_vector_distance * self.dcm[:, 0].copy())

        vector = np.array([self.vector_val[2], self.vector_val[0], self.vector_val[1]])

        print("dcm")
        print(self.dcm[:, 1])
        print(self.dcm[:, 2])
        print(self.dcm[:, 0])
        print("end dcm")

        print(np.array([self.vector_val[1], self.vector_val[0], 0]))

        self.rot_vector_axis.setData(pos=np.array([np.zeros(3), self.vector_len * vector]))
