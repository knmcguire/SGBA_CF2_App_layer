## SGBA App layer

Hi and welcome at the Crazyflie firmware app layer implementation of the Swarm Gradient Bug Algorithm (SGBA), accomendating the publication of:

> K.N. McGuire, C. De Wagter, K. Tuyls, H.J. Kappen, G.C.H.E. de Croon,
> 'Minimal navigation solution for a swarm of tiny flying robots to explore an unknown environment'
> Science Robotics, 23 October 2019 
> DOI: http://robotics.sciencemag.org/lookup/doi/10.1126/scirobotics.aaw9710

This repo will undergo several updates in the coming period, so please keep checking the status. Also if you have any questions, please use the issue list of this repository.

#### Build instructions

Make sure your crazyflie has the newest NRF firmware with bluetooth disabled! (https://github.com/bitcraze/crazyflie2-nrf-firmware). Also give your crazyflies different address with an unique ID: E7E7E701, E7E7E702, etc. and make sure they are listining all on the same channel. Maximum is until 09 for now.

#### Clone this repo

```git clone --recurse-submodules -j8 git://github.com/THISREPO.git```

#### Build and flash
Make sure to do this in the directory of the applayer (this repo) and not the directory of the crazyflie firmware)

```
make clean
make
make cload
```

#### Scripts
Use 2 crazyradio PAs: 
 - 1 to make the crazyflies take off (python3 take_off.py) and land (python3 land_all.py). Use this as helper scripts, but make sure to close take_off.py once all crazyflies has taken off.
 - 1 to act as the home beacon (python3 RSSI_beacon.py)

