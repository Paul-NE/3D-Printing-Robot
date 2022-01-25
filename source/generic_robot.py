from kinematics import Transform, Quaternion, Vector
import numpy as np
import math


class Inverse:
    def inverse(self, target):
        raise NotImplementedError()


class GenericSerialRobot(object):
    def __init__(self, bone_generalized_providers):
        super().__init__()
        self.bones = []
        self.generalized_providers = []
        for bone_generalized_provider in bone_generalized_providers:
            self.bones.append(bone_generalized_provider[0])
            self.generalized_providers.append(bone_generalized_provider[1])

    def forward(self, generalized):
        origin = Transform.identity()
        current = origin
        frames = [origin]
        for i in range(len(generalized)):
            bone = self.bones[i]
            provider = self.generalized_providers[i]
            current += bone + provider(generalized[i])
            frames.append(current)
        return frames
