import torch
import time
import numpy as np

_real_torch_load = torch.load
def cpu_torch_load(*args, **kwargs):
    if 'map_location' not in kwargs:
        kwargs['map_location'] = 'cpu'
    return _real_torch_load(*args, **kwargs)
torch.load = cpu_torch_load

from Go2Py.sim import mujoco

def main():
    print("Starting Go2 Walking Test")
    sim = mujoco.Go2Sim(mode='highlevel', render=True)

    print("Stand up")
    sim.standUpReset()
    time.sleep(2)

    print("Walking forward 0.3 m/s")
    start_time = time.time()
    while time.time() - start_time < 10:
        sim.step(vx=0.3, vy=0.0, omega_z=0.0)
        time.sleep(sim.dt)
        
    stand_start = time.time()
    while time.time() - stand_start < 1.0:
        sim.step(0, 0, 0)
        time.sleep(sim.dt)

    print("Sitting down")
    sim.sitDownReset()
    settle_start = time.time()
    while time.time() - settle_start < 4.0:
        sim.stepLowlevel()
        time.sleep(sim.dt)
    
    print("Complete")
    sim.close()

if __name__ == "__main__":
    main()
  
