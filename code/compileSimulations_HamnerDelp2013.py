# -*- coding: utf-8 -*-
"""
Created on Fri Feb 24 15:51:22 2023

@author:
    Aaron Fox
    Centre for Sport Research
    Deakin University
    aaron.f@deakin.edu.au
    
    This script runs through the process of compiling the RRA and Moco Tracking
    simulations run on the Hamner & Delp 2013 data to compare the solutions.

"""

# %% Import packages

import opensim as osim
# import osimFunctions as helper
import os
import pickle
import numpy as np
import matplotlib.pyplot as plt

# %% Set-up

#Set matplotlib parameters
from matplotlib import rcParams
# rcParams['font.family'] = 'sans-serif'
rcParams['font.sans-serif'] = 'Arial'
rcParams['font.weight'] = 'bold'
rcParams['axes.labelsize'] = 12
rcParams['axes.titlesize'] = 16
rcParams['axes.linewidth'] = 1.5
rcParams['axes.labelweight'] = 'bold'
rcParams['legend.fontsize'] = 10
rcParams['xtick.major.width'] = 1.5
rcParams['ytick.major.width'] = 1.5
rcParams['legend.framealpha'] = 0.0
rcParams['savefig.dpi'] = 300
rcParams['savefig.format'] = 'pdf'

#Add OpenSim geometry path (weird issues with this on new laptop)
osim.ModelVisualizer.addDirToGeometrySearchPaths('C:\\OpenSim 4.3\\Geometry')

#Get home path
homeDir = os.getcwd()

#Set subject list
subList = ['subject01',
           'subject02',
           'subject03',
           'subject04', #some noisy kinematics in Moco (arm kinematics)
           'subject08',
           'subject10', #some noisy kinematics in Moco (arm kinematics)
           'subject11',
           'subject17', #some noisy kinematics in Moco (arm kinematics)
           'subject19',
           'subject20'] #some noisy kinematics in Moco (arm kinematics)
    
#Set run names list
runList  = ['run2',
            'run3',
            'run4',
            'run5']
    
#Set the run label to examine
#Currently just one trial but could be adapted to a list
runLabel = 'run5'
runName = 'Run_5'
    
#Set run cycle list
cycleList = ['cycle1',
             'cycle2',
             'cycle3']

#Set a list for kinematic vars
kinematicVars = ['pelvis_tx', 'pelvis_ty', 'pelvis_tz',
                 'pelvis_tilt', 'pelvis_list', 'pelvis_rotation',
                 'hip_flexion_r', 'hip_adduction_r', 'hip_rotation_r',
                 'knee_angle_r', 'ankle_angle_r',
                 'hip_flexion_l', 'hip_adduction_l', 'hip_rotation_l',
                 'knee_angle_l', 'ankle_angle_l',
                 'lumbar_extension', 'lumbar_bending', 'lumbar_rotation',
                 'arm_flex_r', 'arm_add_r', 'arm_rot_r',
                 'elbow_flex_r', 'pro_sup_r',
                 'arm_flex_l', 'arm_add_l', 'arm_rot_l',
                 'elbow_flex_l', 'pro_sup_l'
                 ]

#Set a list for generic kinematic vars
kinematicVarsGen = ['pelvis_tx', 'pelvis_ty', 'pelvis_tz',
                    'pelvis_tilt', 'pelvis_list', 'pelvis_rotation',
                    'hip_flexion', 'hip_adduction', 'hip_rotation',
                    'knee_angle', 'ankle_angle',
                    'lumbar_extension', 'lumbar_bending', 'lumbar_rotation',
                    'arm_flex', 'arm_add', 'arm_rot',
                    'elbow_flex', 'pro_sup'
                    ]

#Set a list for residual variables
residualVars = ['FX', 'FY', 'FZ', 'MX', 'MY', 'MZ']

#Set dicitonary for plotting axes
kinematicAx = {'pelvis_tx': [0,0], 'pelvis_ty': [0,1], 'pelvis_tz': [0,2],
               'pelvis_tilt': [1,0], 'pelvis_list': [1,1], 'pelvis_rotation': [1,2],
               'hip_flexion_r': [2,0], 'hip_adduction_r': [2,1], 'hip_rotation_r': [2,2],
               'knee_angle_r': [3,0], 'ankle_angle_r': [3,1],
               'hip_flexion_l': [4,0], 'hip_adduction_l': [4,1], 'hip_rotation_l': [4,2],
               'knee_angle_l': [5,0], 'ankle_angle_l': [5,1],
               'lumbar_extension': [6,0], 'lumbar_bending': [6,1], 'lumbar_rotation': [6,2],
               'arm_flex_r': [7,0], 'arm_add_r': [7,1], 'arm_rot_r': [7,2],
               'elbow_flex_r': [8,0], 'pro_sup_r': [8,1],
               'arm_flex_l': [9,0], 'arm_add_l': [9,1], 'arm_rot_l': [9,2],
               'elbow_flex_l': [10,0], 'pro_sup_l': [10,1]
               }

# %% Extract solution times

#Set a place to store average solution times
solutionTimes = {'rra': np.zeros(len(subList)), 'moco': np.zeros(len(subList))}

#Loop through subject list
for subject in subList:
    
    #Load RRA and Moco solution time data
    with open(f'..\\data\\HamnerDelp2013\\{subject}\\rra\\{runLabel}\\{subject}_rraRunTimeData.pkl', 'rb') as openFile:
        rraRunTime = pickle.load(openFile)
    with open(f'..\\data\\HamnerDelp2013\\{subject}\\moco\\{runLabel}\\{subject}_mocoRunTimeData.pkl', 'rb') as openFile:
        mocoRunTime = pickle.load(openFile)
        
    #Calculate averages and append to dictionary
    solutionTimes['rra'][subList.index(subject)] = np.array([rraRunTime[runLabel][cycle]['rraRunTime'] for cycle in cycleList]).mean()
    solutionTimes['moco'][subList.index(subject)] = np.array([mocoRunTime[runLabel][cycle]['mocoRunTime'] for cycle in cycleList]).mean()
    
#Average and display these results
print(f'Average RRA run time (s): {np.round(solutionTimes["rra"].mean(),2)} +/- {np.round(solutionTimes["rra"].std(),2)}')
print(f'Average Moco run time (s): {np.round(solutionTimes["moco"].mean(),2)} +/- {np.round(solutionTimes["moco"].std(),2)}')

# %% Extract root mean square deviations versus kinematic data

#Set a place to store average solution times
kinematicsRMSD = {'rra': {var: np.zeros(len(subList)) for var in kinematicVarsGen},
                  'moco':{var: np.zeros(len(subList)) for var in kinematicVarsGen}}

#Loop through subject list
for subject in subList:
    
    #Load RRA and Moco solution RMSD data
    with open(f'..\\data\\HamnerDelp2013\\{subject}\\results\\outputs\\{subject}_rraKinematicsRMSE.pkl', 'rb') as openFile:
        rraRMSD = pickle.load(openFile)
    with open(f'..\\data\\HamnerDelp2013\\{subject}\\results\\outputs\\{subject}_mocoKinematicsRMSE.pkl', 'rb') as openFile:
        mocoRMSD = pickle.load(openFile)

    #Loop through and extract mean for generic kinematic variables
    for var in kinematicVarsGen:
        
        #Check for pelvis/lumbar variable
        if 'pelvis' in var or 'lumbar' in var:
            #Extract the mean and place in dictionary for rra and moco data
            kinematicsRMSD['rra'][var][subList.index(subject)] = rraRMSD[runLabel]['mean'][var]
            kinematicsRMSD['moco'][var][subList.index(subject)] = mocoRMSD[runLabel]['mean'][var]
        else:
            #Caclculate for both left and right variables
            kinematicsRMSD['rra'][var][subList.index(subject)] = np.array((rraRMSD[runLabel]['mean'][f'{var}_r'], rraRMSD[runLabel]['mean'][f'{var}_l'])).mean()
            kinematicsRMSD['moco'][var][subList.index(subject)] = np.array((mocoRMSD[runLabel]['mean'][f'{var}_r'], mocoRMSD[runLabel]['mean'][f'{var}_l'])).mean()

#Average and display results for variables
for var in kinematicVarsGen:
    if var in ['pelvis_tx', 'pelvis_ty', 'pelvis_tz']:
        #Convert to cm for translations
        print(f'Average RRA RMSD for {var}: {np.round(kinematicsRMSD["rra"][var].mean()*100,2)} +/- {np.round(kinematicsRMSD["rra"][var].std()*100,2)}')
        print(f'Average Moco RMSD for {var}: {np.round(kinematicsRMSD["moco"][var].mean()*100,2)} +/- {np.round(kinematicsRMSD["moco"][var].std()*100,2)}')
    else:
        #Present in degrees
        print(f'Average RRA RMSD for {var}: {np.round(kinematicsRMSD["rra"][var].mean(),2)} +/- {np.round(kinematicsRMSD["rra"][var].std(),2)}')
        print(f'Average Moco RMSD for {var}: {np.round(kinematicsRMSD["moco"][var].mean(),2)} +/- {np.round(kinematicsRMSD["moco"][var].std(),2)}')

# %% Extract peak residual forces and moments

#Set a place to store average solution times
peakResiduals = {'rra': {var: np.zeros(len(subList)) for var in residualVars},
                 'moco':{var: np.zeros(len(subList)) for var in residualVars}}

#Loop through subject list
for subject in subList:
    
    #Load RRA and Moco solution RMSD data
    with open(f'..\\data\\HamnerDelp2013\\{subject}\\results\\outputs\\{subject}_rraResiduals.pkl', 'rb') as openFile:
        rraResiduals = pickle.load(openFile)
    with open(f'..\\data\\HamnerDelp2013\\{subject}\\results\\outputs\\{subject}_mocoResiduals.pkl', 'rb') as openFile:
        mocoResiduals = pickle.load(openFile)

    #Loop through and extract peak residuals and average
    for var in residualVars:
        
        #Extract the peak from each cycle and place in dictionary for rra and moco data
        peakResiduals['rra'][var][subList.index(subject)] = np.array([np.abs(rraResiduals[runLabel][cycle][var]).max() for cycle in cycleList]).mean()
        peakResiduals['moco'][var][subList.index(subject)] = np.array([np.abs(mocoResiduals[runLabel][cycle][var]).max() for cycle in cycleList]).mean()

#Average and display results for residual variables
for var in residualVars:
    print(f'Average RRA residuals for {var}: {np.round(peakResiduals["rra"][var].mean(),3)} +/- {np.round(peakResiduals["rra"][var].std(),3)}')
    print(f'Average Moco resdiauls for {var}: {np.round(peakResiduals["moco"][var].mean(),3)} +/- {np.round(peakResiduals["moco"][var].std(),3)}')

#Check X-fold increases in RRA vs. Moco residuals
for var in residualVars:
    print(f'X-fold increase in RRA residuals for {var}: {np.round(peakResiduals["rra"][var].mean() / peakResiduals["moco"][var].mean(),3)}')









    
# %% ----- end of compileSimulations_HamnerDelp2013.py ----- %% #