import numpy as np

MSG_NDARRAY_ALLOW_ONLY = "Numpy ndarray allowed only with 4 elements!"
MSG_Q_ALLOW_ONLY = "Quaternion allowed only"
MSG_INT_FLOAT_Q_ALLOW_ONLY = "Int, float or Quaternion allowed only"
MSG_INT_FLOAT_ALLOW_ONLY = "Int or float allowed only"
MSG_NP_ARRAY_EULER = "Numpy array with 3 elements allowed only"
MSG_NORMALIZE_FIRST = "First step you need to normalize quaternion"


class Quaternion:
    def __init__(self, q: np.array = np.zeros(4)):
        if not isinstance(q, np.ndarray):
            raise ValueError(MSG_NORMALIZE_FIRST)

        self._q = q if q.any() and q.shape[0] == 4 else np.array([1, 0, 0, 0], dtype=np.float64)
        self._q_len = np.linalg.norm(self._q)
        self._is_normalized = False

    def w(self) -> np.float64:
        return self._q[0]

    def x(self) -> np.float64:
        return self._q[1]

    def y(self) -> np.float64:
        return self._q[2]

    def z(self) -> np.float64:
        return self._q[3]

    def get_q_array(self) -> np.array:
        return self._q.copy()

    def get_q_len(self) -> float:
        return self._q_len

    def normalize(self, q=None):
        if isinstance(q, Quaternion):
            return q / np.linalg.norm(q.get_q_array())
        elif q:
            raise ValueError(MSG_Q_ALLOW_ONLY)

        self._q_len = np.linalg.norm(self._q)
        self._q /= self._q_len
        self._is_normalized = True

    def set_using_rotation_vector(self, rotation_vector: np.array, norm_q: bool = True):
        """

        :param rotation_vector:
        :return:
        """
        if not isinstance(rotation_vector, np.ndarray):
            return

        angle, vx, vy, vz = rotation_vector
        q = np.zeros(4, dtype=np.float64)
        q[0] = np.cos(angle / 2)
        q[1] = np.sin(angle / 2) * vx
        q[2] = np.sin(angle / 2) * vy
        q[3] = np.sin(angle / 2) * vz

        q = np.around(q, decimals=4)
        self._q = self.normalize(q) if norm_q else q

    def set_using_euler(self, euler: np.array = np.zeros(3, dtype=np.float64), norm_q: bool = True):
        """
        Calculating quaternion using Euler angles
        """
        if not (euler.any() and euler.shape[0] == 3):
            raise ValueError(MSG_NP_ARRAY_EULER)

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

        q = np.around(np.array([w, x, y, z], dtype=np.float64), decimals=4)
        self._q = self.normalize(q) if norm_q else q

    def get_euler(self):
        """
        Calculating Euler angles using quaternion
        """
        euler = np.zeros(3, dtype=np.float64)

        qx2 = self._q[1] ** 2
        qy2 = self._q[2] ** 2
        qz2 = self._q[3] ** 2
        euler[0] = np.arctan2(2 * self._q[1] * self._q[0] - 2 * self._q[2] * self._q[3], 1 - 2 * qx2 - 2 * qz2) * 180 / np.pi
        euler[2] = -np.arcsin(np.around(2 * self._q[1] * self._q[2] - 2 * self._q[3] * self._q[0], decimals=4)) * 180 / np.pi
        a = np.around(2 * self._q[2] * self._q[0] - 2 * self._q[1] * self._q[3], decimals=4)
        b = np.around(1 - 2 * qy2 - 2 * qz2, decimals=4)
        euler[1] = np.arctan2(a, b) * 180 / np.pi

        return euler

    def get_rotation_vector(self):
        """
        Calculating of vector and rotation angle around this vector
        """
        res = np.zeros(4, dtype=np.float64)

        # angle
        res[0] = np.rad2deg(2 * np.arccos(self._q[0]))

        power_sum = self._q[1] ** 2 + self._q[2] ** 2 + self._q[3] ** 2
        vector_len = np.linalg.norm(np.sqrt(power_sum if power_sum else 1))

        res[1] = self._q[1] / vector_len
        res[2] = self._q[2] / vector_len
        res[3] = self._q[3] / vector_len
        return res

    def get_dcm(self, get_dcm_for_qt: bool = False):
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

        if get_dcm_for_qt:
            dcm_for_qt = np.array([
                [mZz, mXz, mYz],
                [mZx, mXx, mYx],
                [mZy, mXy, mYy]
            ])
            return np.around(dcm_for_qt, decimals=4)

        dcm = np.array([
            [mXx, mYx, mZx],
            [mXy, mYy, mZy],
            [mXz, mYz, mZz]
        ])
        return np.around(dcm, decimals=4)

    def quaternion_multiply(self, q2, normalize_result: bool = True):
        """
        Multiplying current quaternions and q2 in arguments
        :param q2: second quaternion
        :param normalize_result:
        :return: new quaternion
        """
        res = np.zeros(4, dtype=np.float64)
        res[0] = self.w() * q2.w() - self.x() * q2.x() - self.y() * q2.y() - self.z() * q2.z()
        res[1] = self.w() * q2.x() + self.x() * q2.w() + self.y() * q2.z() - self.z() * q2.y()
        res[2] = self.w() * q2.y() - self.x() * q2.z() + self.y() * q2.w() + self.z() * q2.x()
        res[3] = self.w() * q2.z() + self.x() * q2.y() - self.y() * q2.x() + self.z() * q2.w()
        _q = Quaternion(res)
        if normalize_result:
            return self.normalize(_q)
        return _q

    def __eq__(self, other) -> bool:
        if isinstance(other, Quaternion):
            return np.array_equal(self._q, other._q)
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
        if isinstance(other, (float, int)) :
            return Quaternion(self._q * other)
        elif isinstance(other, Quaternion):
            return self.quaternion_multiply(other)
        else:
            raise ValueError(MSG_INT_FLOAT_Q_ALLOW_ONLY)

    def __rmul__(self, other):
        if isinstance(other, (int, float)):
            return Quaternion(self._q * other)
        else:
            raise ValueError(MSG_INT_FLOAT_Q_ALLOW_ONLY)

    def __truediv__(self, other):
        if isinstance(other, (int, float)):
            return Quaternion(self._q / other)
        else:
            raise ValueError(MSG_INT_FLOAT_ALLOW_ONLY)

    def __repr__(self):
        return f"[w={self.w()}\tx={self.x()}\ty={self.y()}\tz={self.z()}]"

    def __str__(self):
        return self.__repr__()


if __name__ == "__main__":
    q = Quaternion()
