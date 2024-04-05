import time
from utils.PSM import NewPSM
from ambf_client import Client


def sample_jp_motion():
    sim_name = "PSM_Sim"
    client = Client(sim_name)
    client.connect()

    psm1 = NewPSM(client, "psm1")
    psm1.servo_jp([0.1176, -0.1176, 0.0882, 0.5, 0.5, 0.5])
    psm1.set_jaw_angle(0.5)

    psm2 = NewPSM(client, "psm2")
    psm2.servo_jp([0.0294, 0.0, 0.1176, 0.5, 0.5, 0.5])
    psm2.set_jaw_angle(0.5)

    time.sleep(1.0)
    input("Press enter to finish ...")


if __name__ == "__main__":
    sample_jp_motion()
