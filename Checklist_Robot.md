
# Phase 1: Preparation

## Step 1.1: Inspection
Inspect whether the ram extruder, nozzle head and the extrusion hose have been cleaned properly. Make sure all needed objects are available and in proper condition:
- extrusion motor and connected screw + nut
- ram extruder barrel
- control box
- hose
- extruder head + nozzle

Additionally, you will need the following tools:
- screwdriver
- small screwdriver for nozzle head
- ducktape

## Step 1.2 Clay Insertion
In this guide, we assume the clay to already be prepared. Insert the clay into the ram extruder.

Process steps:
1. Move the plunger to the top section of the barrel.
2. Insert clay into the barrel until the gap is filled.
3. Move the plunger down slightly, so that the clay is being pulled into the extruder.
4. Repeat steps 2.-4. until enough clay has been loaded.

> Air Pockets
> Make sure when loading the barrel with clay, that no air pockets or bubbles are introduced into the barrel. These will manifest as defects in the printed object later on.


## Step 1.3: Assembly of Extruder
Assemble the nozzle head and mount the extruder barrel.

Nozzle head:
1. Add vaseline to the small extruder screw, then insert it into the red nozzle head.
2. Mount the nozzle head to the robot and fix it with the two small screws. (tool: second smallest outer wrench)
3. Fix the extruder screw using the small embedded screws. Make sure that the flat side is aligned with the small embedded screws in order to fix it properly. (tool: second smallest inner wrench)
4. Fix the nozzle to the nozzle head (tool: largest outer wrench)

Extruder Barrel:
1. Connect the barrel with the plunge.
2. Mount the barrel and the plunge onto the robot arm. (tool: screwdriver)
3. Move the nut from the ram extruder into a position that more or less aligns with the position of the plunger.
4. Insert the ram extruder into the second barrel. 
5. Connect the two barrels with each other. Potentially the position of the nut needs to be readjusted if it does not fit.
6. Connect the tube between the extruder and the nozzle head. Add duck tape to ensure a strong connection.

## Step 1.4: Load Extruder
Use the `communicate2arduino` program to load the hose and the nozzle with clay. As soon as clay is being extruded, this process is complete. Remove the extruded clay.
1. Run the `communicate2arduino` program.
2. Start with a delay of `1000`. Lower gradually in steps, for example with a new delay of `500` and eventually `300`, to speed up the process.
3. Wait for the clay to fill the hose and eventually extrude through the nozzle.

> In case the clay is taking a long time before being extruded, this likely means that the extrusion motor first needs to connect the nut to the plunger in order to apply pressure on the clay. This can be prevented by properly preparing placing the nut during the extrusion motor setup.


# Phase 2: Printing
## Step 2.1: Z-Probing
Move the nozzle onto the printing platform until it touches it (be careful not to apply pressure on the platform). Move the nozzle along the boundary of the printing platform using the x- and y-axis controls. In case the nozzle starts to apply pressure on the platform (you can check the weight display), readjust the height using the screws on the bottom of the printing platform.

When the z-probing is complete, the roboter should be able to *glide* across the platform in all directions, ensuring a leveled printing platform.

> Speed control
> Make sure to lower the speed control % in order not to crash into the platform (approx. 10% speed).

## Step 2.2: Test Extrusion
Use the `communicate2arduino` program to extrude a small amount of clay. Similar to step 1.3, make sure to gradually decrease the `delay`. This step should make sure that everything is running correctly.

## Step 2.3: Run Print
Connect the extruder control box to the extruder and the nozzle head and to the computer. Run the `runPrint` program to initialise and execute the print.
1. Check if all the parameters are set correctly. (`debug` should be set to `False`)
2. Select whether FIM should be imported via file or via neo4j.
3. Run the program.
4. Now, a couple of windows pop up; the *speed profile*, the *waypoint* graph, the *transition* curve and the joint coordinates. Use these graphs to double-check whether everything looks good.
5. You are asked now to start a program from the teachpanel. Load the `rtde_control_loop_improved_start` program and run it.
6. Additionally, the starting or layer transition point needs to be set. The choice here has a direct impact on the print outcome.

# Phase 3: Cleaning
1. release pressure by loosening the middle part of the barrel (connection between barrel and plunger)
2. Remove the hose
3. remove the extruder motor and screw
4. dismount the barrel
5. Remove any remaining clay and clean the individual parts
6. Return the parts back to the closet where they are stored.