# manipulative-arm
Packages for compliant motion under force control of an ABB IRB 120 arm. ABB's EGM is used for its low latency. 
Multiple "behaviors" of the arm are being developed, for example, move until touch, peg in hole
Simultaneously, different control algorithms for achieving compliance are being tested, currently under accommodation control


## Nodes to run:
`roscore`

`rosrun robotiq_ft_sensor rq_sensor`

`rosrun irb120_accommodation_control custom_egm_test`

`rosrun irb120_accommodation_control accommodation_controller`

`python gui/new_gui.py`


## Assumptions about the system and robot:

When cutting, the blade is aimed in the positive x direction of the tool frame. To then cut in the direction of the blade, use PTEL_x with some positive input parameter of the target distance using the cutting parameter preset. 

## Explanations of major programs:



TODO
