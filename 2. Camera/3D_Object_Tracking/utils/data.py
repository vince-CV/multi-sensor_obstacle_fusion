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




image_path = "C:/Users/xwen2/Desktop/New folder/"
file_name = "data_test.csv"


dfs = pd.read_csv(image_path+file_name)

detectorList = [ "SHITOMASI", "HARRIS", "FAST", "BRISK", "ORB", "AKAZE", "SIFT"]
descriptorList = [ "BRISK", "BRIEF", "ORB", "FREAK", "AKAZE", "SIFT" ]


plt.figure()
savepath = 'C:/Users/xwen2/Desktop/New folder/plots/'

for detectorType in detectorList:
    df_temp = dfs.loc[(dfs['detector']==detectorType)]
    df_temp = dfs.loc[(dfs['detector']==detectorType)]

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
    plt.savefig(savepath+detectorType+'_data.png')
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

        print(data_plot)
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
        plt.savefig(savepath+detectorType+"+"+descriptorType+'_data.png')
        plt.clf()
        

        pyCompare.blandAltman(df['ttc_lidar'], df['ttc_camera'], savePath=savepath+detectorType+"+"+descriptorType+'_agreement.png' )






    #input_size = np.float(row[1][6])
    #print(guess_name)
    #print(input_size)

    #data_temp = {'t_processing': [t],
    #            'thread': [i],
    #            'server': [server_item]
    #}

    #df_temp = pd.DataFrame(data_temp)
    #print(df_temp)
    #dft = pd.concat([dft, df_temp], axis=0)


'''

dft = {'t_processing': [], 'thread': [], 'server': []}

dft = pd.DataFrame(dft)

for thread_item in thread:
    for server_item in server:
        
        csv_name = image_path +"210329_art.E11577_FF_threading" + thread_item + server_item + ".csv"
        #print(csv_name)
        
        i = 0
        if thread_item == '_ON':
            i = 1

        dfs = pd.read_csv(csv_name)

        for row in dfs.iterrows():

           t = np.float(row[1][9])

           data_temp = {'t_processing': [t],
                        'thread': [i],
                        'server': [server_item]
           }

           df_temp = pd.DataFrame(data_temp)
           #print(df_temp)
           dft = pd.concat([dft, df_temp], axis=0)



sns.set(style="darkgrid")
(fx,fy)=(3500,2400)
my_dpi=300
fig=plt.figure(figsize=(fx/my_dpi, fy/my_dpi), dpi=my_dpi)
plt.ylim((1000,3000))

sns.boxplot(x="server", y="t_processing", hue="thread", data=dft)  # RUN PLOT   
plt.savefig(image_path+"output.jpg", bbox_inches='tight',pad_inches = 0)
plt.close(fig)
plt.clf()


dfs_baseline = pd.read_csv("C:/Users/xwen2/Desktop/Library/PythonApplication1/csv_import/210329_art.E11577_FF_threading_baseline.csv")

sns.set(style="darkgrid")
(fx,fy)=(3500,2400)
my_dpi=300
fig=plt.figure(figsize=(fx/my_dpi, fy/my_dpi), dpi=my_dpi)
plt.ylim((1000,3000))

sns.boxplot(data=dfs_baseline["processing_time"])  # RUN PLOT   
plt.savefig(image_path+"output_baseline.jpg", bbox_inches='tight',pad_inches = 0)
plt.close(fig)
plt.clf()
'''








'''


        df.columns = ["value"]

        df_1 = pd.DataFrame(int(layer_item)*np.ones(len(df)), columns=list('layer'))
#        df_2 = pd.DataFrame("layer") 

        
        if thread == "generic":
            df_generic = pd.concat([df_generic, df], axis=1)
            #df_generic = pd.melt(df_generic, id_vars=None, var_name=None) 
        if thread == "thread":
            df_thread = pd.concat([df_thread, df])

        print(df_generic)



    
'''

'''
for image_name in imagelist:
    (nameWithoutExtention, extention) = os.path.splitext(os.path.basename(image_name))
    rect_name = image_path+nameWithoutExtention+"_rect.txt"
    

dfs = pd.read_csv(csv_file)

#size_item = [192, 256, 384, 416, 512, 640, 768]
size_item = [1, 0.8, 0.5, 0.3, 0.1]

name_guess_accuracy = []
conf_guess_accuracy = []
iou_guess_accuracy  = []
time = []

for size in size_item:

    df = dfs.loc[dfs['downsample']==size]


    metric = df[['label_guess', 'confidence', 'bbx_IOU', 'time']]

    avg = metric.mean(axis=0)

    name_guess_accuracy.append(avg['label_guess'])
    conf_guess_accuracy.append(avg['confidence'])
    iou_guess_accuracy.append(avg['bbx_IOU'])
    time.append(avg['time'])


plt.plot(size_item, name_guess_accuracy, label = "name_guess")
plt.plot(size_item, conf_guess_accuracy, label = "conf_guess")
plt.plot(size_item, iou_guess_accuracy, label = "iou_metric")
plt.plot(size_item, time, label = "inference_time")

plt.plot(size_item, name_guess_accuracy, "k.")
plt.plot(size_item, conf_guess_accuracy, "k.")
plt.plot(size_item, iou_guess_accuracy, "k.")
plt.plot(size_item, time, "k.")

plt.legend()
plt.xticks(size_item, rotation = 45)
plt.grid(True)
plt.savefig("C:/Users/xwen2/Desktop/data_test.jpg")
plt.close()
plt.clf()






'''


