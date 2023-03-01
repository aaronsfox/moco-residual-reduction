# -*- coding: utf-8 -*-
'''
Created on Fri Feb 24 15:51:22 2023

@author:
    Aaron Fox
    Centre for Sport Research
    Deakin University
    aaron.f@deakin.edu.au
    
    Test code for Moco driven version of RRA.

'''

# %% Import packages

import opensim as osim
import numpy as np
from scipy.interpolate import interp1d

# %% Set-up

#Add OpenSim geometry path (weird issues with this on new laptop)
osim.ModelVisualizer.addDirToGeometrySearchPaths('C:\\OpenSim 4.3\\Geometry')

# %% Define functions

# %% addTorqueActuators

def addTorqueActuators(osimModel = None,
                       optForces = None,
					   minControl = -1,
					   maxControl = +1):
    
    """
    
    Convenience function for adding series of torque actuators to model
    
    Input:    osimModel - OpenSim model object for use
              optForces - dict of coordinates and their associated optimal forces to add
			  minControl - minimum control signal value for actuators (default = -1)
			  maxControl - maximum control signal value for actuators (default = +1)
              
    Output:   osimModel - updated torque driven model
                  
    """
    
    #Check inputs
    if osimModel is None or optForces is None:
            raise ValueError('All inputs for this function are required!')
    
    #Remove the original lumbar actuators to not apply force
    #Get the force indices to remove from the model
    forceRemove = []
    for forceInd in range(osimModel.updForceSet().getSize()):
        if 'lumbar' in osimModel.updForceSet().get(forceInd).getName():
            forceRemove.append(forceInd)
    #Each time the force is removed the indices reduce, so need to account for this
    for ind in range(len(forceRemove)):
        osimModel.updForceSet().remove(forceRemove[ind]-ind)
    
    #Intialise model system
    osimModel.initSystem()
    
    #Get coordinate list
    coordinatesList = list(optForces.keys())
    
    #Get coordinate set
    coordSet = osimModel.getCoordinateSet()
    
    #Loop through coordinates and add actuators
    for coordinate in coordinatesList:
        #Create actuator
        actu = osim.CoordinateActuator()
        #Set name
        actu.setName(f'{coordinate}_actuator')
        #Set coordinate
        actu.setCoordinate(coordSet.get(coordinate))
        #Set optimal force
        actu.setOptimalForce(optForces[coordinate])
        #Set min and max control
        actu.setMinControl(minControl)
        actu.setMaxControl(maxControl)
        #Append to model force set
        osimModel.updForceSet().cloneAndAppend(actu)
        # #Append to model components
        # osimModel.addComponent(actu)
    
    #Finalise model connections
    osimModel.finalizeConnections()
    
    #Return model
    return osimModel

# %% kinematicsToStates

def kinematicsToStates(kinematicsFileName = None, osimModelFileName = None,
                       outputFileName = 'coordinates.sto',
                       inDegrees = True, outDegrees = False):
    
    # Convenience function for converting IK results to a states storage.
    #
    # Input:    kinematicsFileName - file containing kinematic data. Header should only be coordinates name, rather than path to state
    #           osimModelFileName - opensim model filename that corresponds to kinematic data
    #           outputFileName - optional filename to output to (defaults to coordinates.sto)
    #           inDegrees - set to true if kinematics file is in degrees (defaults to True)
    #           outDegrees - set to true if desired output is in degrees (defaults to False)

    if kinematicsFileName is None:
        raise ValueError('Filename for kinematics is required')
    if osimModelFileName is None:
        raise ValueError('OpenSim model filename is required')

    #Load in the kinematic data
    kinematicsStorage = osim.Storage(kinematicsFileName)
    
    #Create a copy of the kinematics data to alter the column labels in
    statesStorage = osim.Storage(kinematicsFileName)
    
    #Resample the data points linearly to avoid any later issues with matching
    #time points. Use a time stamp for 250 Hz
    kinematicsStorage.resampleLinear(1/250)
    statesStorage.resampleLinear(1/250)
    
    #Get the column headers for the storage file
    angleNames = kinematicsStorage.getColumnLabels()
    
    #Get the corresponding full paths from the model to rename the
    #angles in the kinematics file
    kinematicModel = osim.Model(osimModelFileName)
    for ii in range(0,angleNames.getSize()):
        currAngle = angleNames.get(ii)
        if currAngle != 'time':
            #Get full path to coordinate
            fullPath = kinematicModel.updCoordinateSet().get(currAngle).getAbsolutePathString()+'/value'
            #Set angle name appropriately using full path
            angleNames.set(ii,fullPath)
    
    #Set the states storage object to have the updated column labels
    statesStorage.setColumnLabels(angleNames)
    
    #Appropriately set output in degrees or radians
    if inDegrees and not outDegrees:
        #Convert degrees values to radians for consistency with the current
        #file label (defaults back to inDegrees=no). Radians seem to work
        #better with the Moco process as well.
        kinematicModel.initSystem()
        kinematicModel.getSimbodyEngine().convertDegreesToRadians(statesStorage)
    elif inDegrees and outDegrees:
        #Change the storage label back to specifying indegrees=yes
        statesStorage.setInDegrees(True)
    elif not inDegrees and outDegrees:
        #Convert radians to degrees
        kinematicModel.initSystem()
        kinematicModel.getSimbodyEngine().convertRadiansToDegrees(statesStorage)
        #Reset labeling for degrees
        statesStorage.setInDegrees(True)
    
    #Write the states storage object to file
    statesStorage.printToXML(outputFileName)

# %% Set-up a coordinate tracking problem with IK data

#Create and name an instance of the MocoTrack tool.
track = osim.MocoTrack()
track.setName('torqueDrivenCoordinateTracking')

# Construct a ModelProcessor and set it on the tool.
modelProcessor = osim.ModelProcessor('subject01_simbody.osim')
modelProcessor.append(osim.ModOpAddExternalLoads('subject01_walk1_grf.xml'))
modelProcessor.append(osim.ModOpRemoveMuscles())

#Process model to edit
osimModel = modelProcessor.process()

#Add in torque actuators based on RRA actuators

#Create the dictionary of optimal forces to pass to function for torque actuators
#### NOTE: currently includes residual pelvis actuators
optForces = {'pelvis_tx': 4, 'pelvis_ty': 8, 'pelvis_tz': 2,
             'pelvis_tilt': 2, 'pelvis_list': 2, 'pelvis_rotation': 2,
             'hip_flexion_r': 300, 'hip_adduction_r': 200, 'hip_rotation_r': 100,
             'knee_angle_r': 300, 'ankle_angle_r': 300,
             'subtalar_angle_r': 100, 'mtp_angle_r': 100,
             'hip_flexion_l': 300, 'hip_adduction_l': 200, 'hip_rotation_l': 100,
             'knee_angle_l': 300, 'ankle_angle_l': 300,
             'subtalar_angle_l': 100, 'mtp_angle_l': 100,
             'lumbar_extension': 300, 'lumbar_bending': 200, 'lumbar_rotation': 200}

#Add the actuators to the torque driven model
osimModel = addTorqueActuators(osimModel = osimModel,
                               optForces = optForces,
                               minControl = np.inf * -1,
                               maxControl = np.inf)

#Set model in tracking tool
track.setModel(osim.ModelProcessor(osimModel))

#Convert kinematics to states
kinematicsToStates(kinematicsFileName = 'subject01_walk1_ik.mot',
                   osimModelFileName = 'subject01_simbody.osim',
                   outputFileName = 'coordinates.sto',
                   inDegrees = True, outDegrees = False)

# Construct a TableProcessor of the coordinate data and pass it to the 
# tracking tool. TableProcessors can be used in the same way as
# ModelProcessors by appending TableOperators to modify the base table.
# A TableProcessor with no operators, as we have here, simply returns the
# base table.
track.setStatesReference(osim.TableProcessor('coordinates.sto'))
track.set_states_global_tracking_weight(10)

# This setting allows extra data columns contained in the states
# reference that don't correspond to model coordinates.
track.set_allow_unused_references(True)

# Since there is only coordinate position data in the states references,
# this setting is enabled to fill in the missing coordinate speed data using
# the derivative of splined position data.
track.set_track_reference_position_derivatives(True)

# Initial time, final time, and mesh interval.
#Set times to variables (used later)
initialTime = 0.5
finalTime = 2.0
#Set in tracking tool
track.set_initial_time(initialTime)
track.set_final_time(finalTime)
track.set_mesh_interval(0.02)

#Create weight set for state tracking
stateWeights = osim.MocoWeightSet()

#Create a dictionary that provides the kinematic task weights for function
taskWeights = {'pelvis_tx': 5, 'pelvis_ty': 5, 'pelvis_tz': 5,
               'pelvis_tilt': 1000, 'pelvis_list': 500, 'pelvis_rotation': 100,
               'hip_flexion_r': 20, 'hip_adduction_r': 20, 'hip_rotation_r': 20,
               'knee_angle_r': 20, 'ankle_angle_r': 20,
               'subtalar_angle_r': 20, 'mtp_angle_r': 20,
               'hip_flexion_l': 20, 'hip_adduction_l': 20, 'hip_rotation_l': 20,
               'knee_angle_l': 20, 'ankle_angle_l': 20,
               'subtalar_angle_l': 20, 'mtp_angle_l': 20,
               'lumbar_extension': 50, 'lumbar_bending': 50, 'lumbar_rotation': 10}

#Set constant weight to scale tracking error speeds by
w = 0.001

#Loop through coordinates to apply weights
for coordInd in range(osimModel.updCoordinateSet().getSize()):
    
    #Get name and absolute path to coordinate
    coordName = osimModel.updCoordinateSet().get(coordInd).getName()
    coordPath = osimModel.updCoordinateSet().get(coordInd).getAbsolutePathString()

    #If a task weight is provided, add it in
    if coordName in list(taskWeights.keys()):
        #Append state into weight set
        #Coordinate value
        stateWeights.cloneAndAppend(osim.MocoWeight(f'{coordPath}/value',
                                                    taskWeights[coordName]))
        #Coordinate speed
        stateWeights.cloneAndAppend(osim.MocoWeight(f'{coordPath}/speed',
                                                    w * taskWeights[coordName])) 

#Add to tracking problem
track.set_states_weight_set(stateWeights)

# Instead of calling solve(), call initialize() to receive a pre-configured
# MocoStudy object based on the settings above. Use this to customize the
# problem beyond the MocoTrack interface.
study = track.initialize()
problem = study.updProblem()
   
#Regularization term on MocoTrack problem (minimize squared muscle excitations)
effort = osim.MocoControlGoal.safeDownCast(problem.updGoal('control_effort'))
effort.setWeight(0.001)

# Put a large weight on the pelvis CoordinateActuators, which act as the
# residual, or 'hand-of-god', forces which we would like to keep as small
# as possible.
for forceInd in range(osimModel.updForceSet().getSize()):
    if 'pelvis' in osimModel.updForceSet().get(forceInd).getAbsolutePathString():
        effort.setWeightForControl(osimModel.updForceSet().get(forceInd).getAbsolutePathString(), 10)
    else:
        effort.setWeightForControl(osimModel.updForceSet().get(forceInd).getAbsolutePathString(), 1)

##### Add bounds in problem --- time bounds should be the same as IK?

#Add torso mass as a parameter in the problem
massParam = osim.MocoParameter('torso_mass', '/bodyset/torso', 'mass', 
                               osim.MocoBounds(osimModel.updBodySet().get('torso').getMass() * 0.9, #10% reduction in mass allowed
                                               osimModel.updBodySet().get('torso').getMass() * 1.1) #10% increase in mass allowed
                               )
problem.addParameter(massParam)

# #Configure the tropter solver
# #### TODO: set mesh interval???
# solver = study.initTropterSolver()
# solver.set_optim_max_iterations(10) #leave it at 10 for now to see how it runs

#Or can we use Casadi solver without initSystem?
solver = osim.MocoCasADiSolver.safeDownCast(study.updSolver())
solver.set_optim_max_iterations(1000) #leave it at 10 for now to see how it runs
solver.set_parameters_require_initsystem(False)

### TODO: other solver parameters?

#Reset problem
solver.resetProblem(problem)

#Using MocoTrack with parameter optimisation doesn't work due to the guess not 
#having the parameter (I think). See here:
    #https://simtk.org/plugins/phpBB/viewtopicPhpbb.php?f=1815&t=11504&p=32366&start=0&view=

#Create a guess to fill and use
initialGuess = solver.createGuess()

#Read in tracked states file to fill guess
trackedStatesTable = osim.TimeSeriesTable('torqueDrivenCoordinateTracking_tracked_states.sto')

#Get start and end indices for time bounds
startInd = np.argmax(np.array(trackedStatesTable.getIndependentColumn()) >= initialTime)
stopInd = np.argmax(np.array(trackedStatesTable.getIndependentColumn()) >= finalTime) + 1

#Loop through tracked states and input to guess
for stateLabel in trackedStatesTable.getColumnLabels():
    #Get data from tracked states within time bounds
    trackedTime = np.array(trackedStatesTable.getIndependentColumn())[startInd:stopInd]
    trackedData = trackedStatesTable.getDependentColumn(stateLabel).to_numpy()[startInd:stopInd]
    #Create interpolation function
    interpFunc = interpFunc = interp1d(trackedTime, trackedData)
    #Interpolate data to new time steps
    interpTrackedData = interpFunc(initialGuess.getTime().to_numpy())
    #Fill in guess
    initialGuess.setState(stateLabel, interpTrackedData)

#Set guess in solver
solver.setGuess(initialGuess)

# Solve and visualize.
solution = study.solve()
study.visualize(solution)

#Write solution
# solution.write('solution_kinematicsOnly.sto')
solution.write('solution_kinematics_torsoMass.sto')

##### Note that mass didn't change for the torso at all in first run...


### TODO: remove tracked states file


# %%

track.setModel(modelProcessor)

# Construct a TableProcessor of the coordinate data and pass it to the 
# tracking tool. TableProcessors can be used in the same way as
# ModelProcessors by appending TableOperators to modify the base table.
# A TableProcessor with no operators, as we have here, simply returns the
# base table.
track.setStatesReference(osim.TableProcessor('coordinates.sto'))
track.set_states_global_tracking_weight(10)

# This setting allows extra data columns contained in the states
# reference that don't correspond to model coordinates.
track.set_allow_unused_references(True)

# Since there is only coordinate position data in the states references,
# this setting is enabled to fill in the missing coordinate speed data using
# the derivative of splined position data.
track.set_track_reference_position_derivatives(True)

# Initial time, final time, and mesh interval.
track.set_initial_time(0.81)
track.set_final_time(1.65)
track.set_mesh_interval(0.08)

# Instead of calling solve(), call initialize() to receive a pre-configured
# MocoStudy object based on the settings above. Use this to customize the
# problem beyond the MocoTrack interface.
study = track.initialize()

# Get a reference to the MocoControlCost that is added to every MocoTrack
# problem by default.
problem = study.updProblem()
effort = osim.MocoControlGoal.safeDownCast(problem.updGoal('control_effort'))

# Put a large weight on the pelvis CoordinateActuators, which act as the
# residual, or 'hand-of-god', forces which we would like to keep as small
# as possible.
model = modelProcessor.process()
model.initSystem()
forceSet = model.getForceSet()
for i in range(forceSet.getSize()):
    forcePath = forceSet.get(i).getAbsolutePathString()
    if 'pelvis' in str(forcePath):
        effort.setWeightForControl(forcePath, 10)

# Solve and visualize.
solution = study.solve()
study.visualize(solution)