world {}

table (world){
    shape:ssBox, Q:<t(0 0. 0)>, size:[9. 9. .1 .02], color:[.3 .3 .3]
    contact, logical:{ }
    friction:.1
}

Prefix: "L_"
Include: 'panda_moveGripper.g'


Prefix: "1_"
Include: 'panda_moveGripper.g'

Prefix: "R_"
Include: 'panda_moveGripper.g'



Prefix!

Edit L_panda_link0 (table)  { Q:<t( -0.8 0.2 .1) d(90 0 0 1)> }
Edit 1_panda_link0 (table)  { Q:<t( -0.8 -1.6 .1) d(90 0 0 1)> }



Edit R_panda_link0 (table)  { Q:<t(2. 2.63 .1) d(-90 0 0 1)> }
#Edit R_finger1  { Q:< d(90 0 0 1)> }
#Edit R_finger2  { Q:< d(90 0 0 1)> }