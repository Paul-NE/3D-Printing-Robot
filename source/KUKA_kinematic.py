from generic_robot import *

class KUKA(GenericSerialRobot, Inverse):
    def __init__(self, lengths):
        def rx(a): return Transform(
            Vector.zero(),
            Quaternion.from_angle_axis(a, Vector(1, 0, 0))
        )

        def ry(a): return Transform(
            Vector.zero(),
            Quaternion.from_angle_axis(a, Vector(0, 1, 0))
        )

        def rz(a): return Transform(
            Vector.zero(),
            Quaternion.from_angle_axis(a, Vector(0, 0, 1))
        )

        def rotate(Vec):
            def rotate(a): return Transform(
                    Vector.zero(),
                    Quaternion.from_angle_axis(a, Vec)
            )
            return rotate

        def no_motion(*args): return Transform(
            Vector.zero(),
            Quaternion.from_angle_axis(0, Vector(1, 0, 0))
        )

        providers = [
            ( # Column
                Transform(Vector(0, 0, lengths[0]), Quaternion.identity()),
                rz
            ),
            (
                Transform(Vector(0, 0, lengths[1]), Quaternion.identity()),
                rx
            ),
            (
                Transform(Vector(0, 0, lengths[2]), Quaternion.identity()),
                rx
            ),
            (
                Transform(Vector(0, 0, lengths[3]), Quaternion.identity()),
                rz
            ),
            (
                Transform(Vector(0, 0, lengths[4]), Quaternion.identity()),
                rx
            ),
            ( # Flange
                Transform(Vector(0, 0, lengths[5]), Quaternion.identity()),
                rz
            )
        ]
        self.to_flange = Transform(Vector(0, 0, 0), Quaternion.identity())
        super().__init__(providers)

    def forward(self, generalized):
        result = super().forward(generalized)
        result.append(result[-1] + self.to_flange)
        return result


if __name__ == "__main__":
    l = [1, 1, 1, 1, 1, 1]
    robot = KUKA(l)
    print(robot.forward([0, math.pi/2, 0, 0, 0, 0]))