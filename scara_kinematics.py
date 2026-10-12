import numpy as np

class ScaraKinematics:
    def __init__(self, l1=0.40, l2=0.35, l3=0.15):
        self.l1 = l1
        self.l2 = l2
        self.l3 = l3

    def forward_kinematics(self, q):
        """
        Computes wafer center position and orientation.
        q: [theta1, theta2, theta3] (radians)
        Returns: [x, y, phi]
        """
        th1, th2, th3 = q
        th12 = th1 + th2
        th123 = th12 + th3

        x = self.l1 * np.cos(th1) + self.l2 * np.cos(th12) + self.l3 * np.cos(th123)
        y = self.l1 * np.sin(th1) + self.l2 * np.sin(th12) + self.l3 * np.sin(th123)
        phi = th123
        return np.array([x, y, phi])

    def inverse_kinematics(self, target_pose, elbow_up=True):
        """
        Computes joint angles for desired [x, y, phi].
        Returns: q = [theta1, theta2, theta3]
        """
        x, y, phi = target_pose
        # 1. Wrist center position
        xw = x - self.l3 * np.cos(phi)
        yw = y - self.l3 * np.sin(phi)

        # 2. Law of Cosines for theta2
        r_sq = xw**2 + yw**2
        d = (r_sq - self.l1**2 - self.l2**2) / (2.0 * self.l1 * self.l2)
        if np.abs(d) > 1.0:
            raise ValueError(f"Target pose {target_pose} is out of reachable workspace.")

        sign = 1.0 if elbow_up else -1.0
        th2 = np.arctan2(sign * np.sqrt(max(0.0, 1.0 - d**2)), d)

        # 3. Shoulder angle theta1
        k1 = self.l1 + self.l2 * np.cos(th2)
        k2 = self.l2 * np.sin(th2)
        th1 = np.arctan2(yw, xw) - np.arctan2(k2, k1)

        # 4. Wrist angle theta3 for zero relative orientation
        th3 = phi - (th1 + th2)
        return np.array([th1, th2, th3])

    def jacobian(self, q):
        """
        Computes the 2x3 positional Jacobian J(q) for wafer center.
        """
        th1, th2, th3 = q
        th12 = th1 + th2
        th123 = th12 + th3

        s1 = np.sin(th1)
        s12 = np.sin(th12)
        s123 = np.sin(th123)

        c1 = np.cos(th1)
        c12 = np.cos(th12)
        c123 = np.cos(th123)

        j11 = -self.l1 * s1 - self.l2 * s12 - self.l3 * s123
        j12 = -self.l2 * s12 - self.l3 * s123
        j13 = -self.l3 * s123

        j21 = self.l1 * c1 + self.l2 * c12 + self.l3 * c123
        j22 = self.l2 * c12 + self.l3 * c123
        j23 = self.l3 * c123

        return np.array([
            [j11, j12, j13],
            [j21, j22, j23]
        ])

    def jacobian_derivative(self, q, q_dot):
        """
        Computes 2x3 J_dot(q, q_dot).
        """
        th1, th2, th3 = q
        dth1, dth2, dth3 = q_dot
        w1 = dth1
        w12 = dth1 + dth2
        w123 = dth1 + dth2 + dth3

        th12 = th1 + th2
        th123 = th12 + th3

        s1 = np.sin(th1)
        s12 = np.sin(th12)
        s123 = np.sin(th123)

        c1 = np.cos(th1)
        c12 = np.cos(th12)
        c123 = np.cos(th123)

        jd11 = -self.l1 * c1 * w1 - self.l2 * c12 * w12 - self.l3 * c123 * w123
        jd12 = -self.l2 * c12 * w12 - self.l3 * c123 * w123
        jd13 = -self.l3 * c123 * w123

        jd21 = -self.l1 * s1 * w1 - self.l2 * s12 * w12 - self.l3 * s123 * w123
        jd22 = -self.l2 * s12 * w12 - self.l3 * s123 * w123
        jd23 = -self.l3 * s123 * w123

        return np.array([
            [jd11, jd12, jd13],
            [jd21, jd22, jd23]
        ])

    def cartesian_acceleration(self, q, q_dot, q_ddot):
        """
        Computes a_tip = J(q) * q_ddot + J_dot(q, q_dot) * q_dot
        """
        J = self.jacobian(q)
        J_dot = self.jacobian_derivative(q, q_dot)
        return J @ q_ddot + J_dot @ q_dot