import numpy as np

MSG_Q_ALLOW_ONLY = "Quaternion allowed only"
MSG_INT_FLOAT_Q_ALLOW_ONLY = "Int, float or Quaternion allowed only"


class Quaternion:
    def __init__(self, q: np.array = np.zeros(4), euler: np.array = np.zeros(3)):
        if q.any() and q.shape[0] == 4:
            self._q = q
            self._euler = euler
            self._to_euler()
        elif euler.any() and euler.shape[0] == 3:
            self._q = q
            self._euler = euler
            self._from_euler(euler)
        else:
            self._q = np.array([1, 0, 0, 0], dtype=np.float64)
            self._euler = euler
            self._to_euler()

        self._q_len = np.linalg.norm(self._q)
        self._q_norm = self._q / self._q_len
        self._vector = np.zeros(3, dtype=np.float64)
        self._angle = 0
        self._dcm = np.eye(3, dtype=np.float64)
        self._dcm_for_qt = np.eye(3, dtype=np.float64)

        self._to_rotation_vector()
        self._to_dcm()

    def w(self) -> np.float64:
        return self._q[0]

    def x(self) -> np.float64:
        return self._q[1]

    def y(self) -> np.float64:
        return self._q[2]

    def z(self) -> np.float64:
        return self._q[3]

    def roll(self) -> np.float64:
        return self._euler[0]

    def pitch(self) -> np.float64:
        return self._euler[1]

    def yaw(self) -> np.float64:
        return self._euler[2]

    def get_q(self) -> np.array:
        return self._q.copy()

    def get_q_norm(self) -> np.array:
        return self._q_norm.copy()

    def get_euler(self) -> np.array:
        return self._euler

    def get_rotation_vector(self) -> np.array:
        """
        Returns rotation vector
        :return: np.array([<angle>, <vector x>, <vector y>, <vector z>])
        """
        return np.append(self._angle, self._vector)

    def get_dcm(self) -> np.array:
        return self._dcm.copy()

    def _from_euler(self, euler: np.array):
        """
        Calculating quaternion using Euler angles
        """
        roll, pitch, yaw = euler

        c1 = np.cos(yaw * 0.5)
        c2 = np.cos(pitch * 0.5)
        c3 = np.cos(roll * 0.5)
        s1 = np.sin(yaw * 0.5)
        s2 = np.sin(pitch * 0.5)
        s3 = np.sin(roll * 0.5)

        w = c1 * c2 * c3 - s1 * s2 * s3
        x = s1 * s2 * c3 + c1 * c2 * s3
        y = c1 * s2 * c3 - s1 * c2 * s3
        z = s1 * c2 * c3 + c1 * s2 * s3

        self._q = np.around(np.array([w, x, y, z], dtype=np.float64), decimals=4)
        
    def _to_euler(self):
        """
        Calculating Euler angles using quaternion
        """
        qx2 = self._q[1] ** 2
        qy2 = self._q[2] ** 2
        qz2 = self._q[3] ** 2
        self._euler[0] = np.arctan2(2 * self._q[1] * self._q[0] - 2 * self._q[2] * self._q[3], 1 - 2 * qx2 - 2 * qz2) * 180 / np.pi
        self._euler[2] = -np.arcsin(2 * self._q[1] * self._q[2] - 2 * self._q[3] * self._q[0]) * 180 / np.pi
        self._euler[1] = np.arctan2(2 * self._q[2] * self._q[0] - 2 * self._q[1] * self._q[3], 1 - 2 * qy2 - 2 * qz2) * 180 / np.pi

    def _to_rotation_vector(self):
        """
        Calculating of vector and rotation angle around this vector
        """
        self._angle = np.rad2deg(2 * np.arccos(self._q_norm[0]))

        power_sum = self._q_norm[1]**2 + self._q_norm[2]**2 + self._q_norm[3]**2
        vector_len = np.linalg.norm(np.sqrt(power_sum if power_sum else 1))

        self._vector[0] = self._q_norm[1] / vector_len
        self._vector[1] = self._q_norm[2] / vector_len
        self._vector[2] = self._q_norm[3] / vector_len

    def _to_dcm(self):
        """
        Calculating DCM using quaternion
        """
        w, x, y, z = self._q

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
            [mXx, mYx, mZx],
            [mXy, mYy, mZy],
            [mXz, mYz, mZz]
        ])

        dcm_for_qt = np.array([
            [mZz, mXz, mYz],
            [mZx, mXx, mYx],
            [mZy, mXy, mYy]
        ])

        self._dcm = np.around(dcm, decimals=4)
        self._dcm_for_qt = np.around(dcm_for_qt, decimals=4)

    def quaternion_multiply(self, q2):
        """
        Multiplying current quaternions and q2 in arguments
        :param q2: second quaternion
        :return: new quaternion
        """
        res = np.zeros(4, dtype=np.float64)
        res[0] = self.w() * q2.w() - self.x() * q2.x() - self.y() * q2.y() - self.z() * q2.z()
        res[1] = self.w() * q2.x() + self.x() * q2.w() + self.y() * q2.z() - self.z() * q2.y()
        res[2] = self.w() * q2.y() - self.x() * q2.z() + self.y() * q2.w() + self.z() * q2.x()
        res[3] = self.w() * q2.z() + self.x() * q2.y() - self.y() * q2.x() + self.z() * q2.w()
        return Quaternion(res)

    def __eq__(self, other) -> bool:
        if isinstance(other, Quaternion):
            return self._q == other._q
        else:
            raise ValueError(MSG_Q_ALLOW_ONLY)

    def __add__(self, other):
        if isinstance(other, Quaternion):
            return Quaternion(self._q + other._q)
        else:
            raise ValueError(MSG_Q_ALLOW_ONLY)

    def __sub__(self, other):
        if isinstance(other, Quaternion):
            return Quaternion(self._q - other._q)
        else:
            raise ValueError(MSG_Q_ALLOW_ONLY)

    def __mul__(self, other):
        if isinstance(other, (int, float)):
            return Quaternion(self._q * other)
        elif isinstance(other, Quaternion):
            return self.quaternion_multiply(other)
        else:
            raise ValueError(MSG_INT_FLOAT_Q_ALLOW_ONLY)