import sys

sys.path.append('../build')
import libry as ry

class Environment:
    def __init__(self, g_file):
        self.RealWorld = ry.Config()
        self.RealWorld.addFile(g_file)

        self.V = ry.ConfigurationViewer()
        self.V.setConfiguration(self.RealWorld)

        self.S = self.RealWorld.simulation(ry.SimulatorEngine.physx, True)

        self.C = ry.Config()
        self.C.addFile(g_file)

        self.C.setFrameState(self.RealWorld.getFrameState())
        self.V.setConfiguration(self.C)

        self.intial_q = self.S.get_q()
        self.S.addSensor("kitchen_camera")
        self.S.addSensor("table_camera")

    def add_dyna_mass(self, mass):
        for frame_name in self.RealWorld.getFrameNames():
            if "dyna_" in frame_name:
                print("Mass of ", frame_name, "set to", mass, " kgs")
                dyna_obj = self.RealWorld.frame(frame_name)
                dyna_obj.setMass(mass)

    def set_contact(self,frame_name):
        obj = self.RealWorld.frame(frame_name)
        obj.setContact(1)

    def get_position(self, frame_name):
        obj = self.RealWorld.frame(frame_name)
        return obj.getPosition()

    def get_quaternion(self,frame_name):
        obj = self.RealWorld.frame(frame_name)
        return obj.getQuaternion(1)



