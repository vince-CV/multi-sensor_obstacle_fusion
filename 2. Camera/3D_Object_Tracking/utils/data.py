import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from collections import Counter
import seaborn as sns
import csv
import re
import os
import seaborn as sns
from prettytable import PrettyTable
import pyCompare
import matplotlib.pyplot as plt
from matplotlib.pyplot import MultipleLocator




image_path = "C:/Users/xwen2/Desktop/projects/16. Sensor Fusion Nanodegree/2. Camera/3D_Object_Tracking/utils/"
file_name = "data_ttc.csv"


dfs = pd.read_csv(image_path+file_name)

detectorList = [ "SHITOMASI", "HARRIS", "FAST", "BRISK", "ORB", "AKAZE", "SIFT"]
descriptorList = [ "BRISK", "BRIEF", "ORB", "FREAK", "AKAZE", "SIFT" ]


plt.figure()
savepath = 'C:/Users/xwen2/Desktop/projects/16. Sensor Fusion Nanodegree/2. Camera/3D_Object_Tracking/utils/'

for descriptorType in descriptorList:
    df_temp1 = dfs.loc[(dfs['descriptor']==descriptorType)]
    df_temp1.loc[df_temp1['ttc_camera']>20] = 0
    df_temp1.loc[df_temp1['ttc_camera']<=0] = 0

    plt.xlim((0,19))
    ax = sns.lineplot(x = 'frame', y ='ttc_camera' , hue = 'detector', data = df_temp1, dashes=False, markers = True)
    plt.title("descriptor: " + descriptorType, fontsize=12)
    plt.xlabel("Frame index")
    plt.ylabel('TTC_Camera / s')
    my_x_ticks = np.arange(0, 19, 1)
    plt.xticks(my_x_ticks)
    #sns.set(style="darkgrid")
    plt.legend()
    plt.grid()
    #plt.show()
    plt.savefig(savepath+descriptorType+'_detector_data.png')
    plt.clf()


for detectorType in detectorList:
    #df_temp = dfs.loc[(dfs['detector']==detectorType)]
    df_temp = dfs.loc[(dfs['detector']==detectorType)]

    df_temp.loc[df_temp['ttc_camera']>20] = 0
    df_temp.loc[df_temp['ttc_camera']<=0] = 0

    plt.xlim((0,19))
    ax = sns.lineplot(x = 'frame', y ='ttc_camera' , hue = 'descriptor', data = df_temp, dashes=False, markers = True)
    plt.title("detector: " +detectorType, fontsize=12)
    plt.xlabel("Frame index")
    plt.ylabel('TTC_Camera / s')
    my_x_ticks = np.arange(0, 19, 1)
    plt.xticks(my_x_ticks)
    #sns.set(style="darkgrid")
    plt.legend()
    plt.grid()
    #plt.show()
    plt.savefig(savepath+detectorType+'_descriptor_data.png')
    plt.clf()

    for descriptorType in descriptorList:

        if ((detectorType == "SIFT") and (descriptorType == "ORB")):
            continue
        if ((detectorType != "AKAZE") and (descriptorType == "AKAZE")):
            continue

        df = df_temp.loc[(df_temp['descriptor']==descriptorType)]
        df = df.reset_index(drop=True)

        data_plot = [df['ttc_lidar'], df['ttc_camera']]

        #data_plot.reset_index(drop=True)

        #print(data_plot)
        #ax = sns.lineplot(x = 'frame', y = 'tcc_lidar', markers = True, data = df)
        #ax = sns.lineplot(x = 'frame', y = 'tcc_camera', markers = True, data = df)
        
        plt.xlim((-1,18))
        ax = sns.lineplot( data = data_plot, dashes=False,markers = True)
        plt.title(detectorType+"+"+descriptorType, fontsize=12)
        plt.xlabel("Frame index")
        plt.ylabel('TTC / s')
        my_x_ticks = np.arange(0, 18, 1)
        plt.xticks(my_x_ticks)
        #sns.set(style="darkgrid")
        plt.legend()
        plt.grid()
        #plt.show()
        #plt.savefig(savepath+detectorType+"+"+descriptorType+'_data.png')
        plt.clf()
        

        #pyCompare.blandAltman(df['ttc_lidar'], df['ttc_camera'], savePath=savepath+detectorType+"+"+descriptorType+'_agreement.png' )

