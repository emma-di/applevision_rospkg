#!/usr/bin/env python2
# For making data visualizations
# To run this:
    # src/applevision_rospkg/bin/data_visualizations.py
    
import csv
import matplotlib.pyplot as plt
import numpy as np
import os

# for angle/vector visualization (copied (modified) from https://matplotlib.org/stable/gallery/lines_bars_and_markers/scatter_hist.html#sphx-glr-gallery-lines-bars-and-markers-scatter-hist-py)
def scatter_hist(x1, y1, x2, y2, x3, y3, ax, ax_histx, ax_histy, outlier, size, c1, c2, c3):
    # no labels
    # ax_histx.tick_params(axis=str(x1), labelbottom=False)
    # ax_histy.tick_params(axis=str(y1), labelleft=False)
    ax.axes.xaxis.set_visible(False)
    ax.axes.yaxis.set_visible(False)

    # the scatter plot:
    ax.scatter(x1, y1, s=size, color = str(c1), label = 'Successful overall & angle')
    ax.scatter(x2, y2, s=size, color = str(c2), label = 'Successful overall, failed angle')
    if outlier == True:
        ax.scatter(x3, y3, s=size, color = str(c3), marker = '^', label = 'Failure')
    
    # ax.set(xlim=(min(x1+x2+x3), max(x1+x2+x3)), ylim = (min(y1+y2+y3)+max(y1+y2+y3)))
    ax.set_aspect('equal', adjustable = 'datalim')
    ax.autoscale(enable=True) 
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    # LEGEND LOCATION
    ax.legend( loc = 'lower left') #bbox_to_anchor=(.1, 0.1), loc="upper left")

    def binsamount(l):
        return int((max(l)-min(l))*500)
    ax_histx.hist((x1+x2), bins=abs(binsamount(x1+x2)),  alpha=0.5, histtype='stepfilled', color= c1, edgecolor='none')
    ax_histy.hist((y1+y2), bins=abs(binsamount(y1+y2)), orientation='horizontal',  alpha=0.5, histtype='stepfilled', color=c1, edgecolor='none')
    # OUTLIER HISTOGRAMS??
    # ax_histx.hist(x3, bins=50, alpha=0.5, histtype='stepfilled', color= c3, edgecolor='none')
    # ax_histy.hist(x3, bins=50, orientation='horizontal',  alpha=0.5, histtype='stepfilled', color=c3, edgecolor='none')

# class for data visualization
# takes one parameter: data spreadsheet location as a string (spreadsheet from stagex testing files)
class DataVis():
    # label and sort data
    def __init__(self, datasheet):
        self.name = datasheet[11:27]
        with open (datasheet) as f:
            reader = csv.reader(f)
            self.data = list(reader)
            self.end = len(self.data[0])
            self.trials = (self.data[0])[2:self.end]
            self.results = (self.data[1])[2:self.end]
            self.angles = (self.data[2])[2:self.end]
            self.vectors = (self.data[3])[2:self.end]
            times = (self.data[4])[2:self.end]
            self.times = [eval(i) for i in times]
            self.avg_time = eval(((self.data[4])[0])[0:5])
            #self.start_coords = (self.data[5])[2:self.end]
    # visualization for approach times (scatter plot + avg time line)
    def time_vis(self):
        # search for outliers
        normal_times = []
        normal_trials = []
        strange_times = []
        strange_trials = []
        outlier_times = []
        outlier_trials = []
        for i in range(len(self.times)):
            # failures
            if self.results[i] == 'fail':
                outlier_times.append(self.times[i])
                outlier_trials.append(i)
            # successes, but with strange times
            elif self.times[i] > 60 or self.times[i]<13:
                strange_times.append(self.times[i])
                strange_trials.append(i)
            # normal success
            else:
                normal_times.append(self.times[i])
                normal_trials.append(i)
                
        fig,ax = plt.subplots(1)
        ax.scatter(normal_trials, normal_times, c='skyblue', label = 'Success') # plot normal success times
        # ax.scatter(strange_trials, strange_times, c = 'red', label = 'Success - Abnormal') # plot strange successes times
        ax.scatter(outlier_trials, outlier_times, c = 'red', marker = 'X', label = 'Failure') # plot failure times
        # ax.scatter([0,106], [self.avg_time+15, 10], c = 'white') # makes space to display average time and legend
        # ax.plot(np.linspace(self.avg_time, self.avg_time, 100), c= 'blue', label = 'Average Time') # plot average time
        # ax.text(101, self.avg_time-.4,self.avg_time) # display average time text
        # ax.scatter([0,106], [self.avg_time+ 15, 10], c = 'white') # makes space to display average time and legend
        ax.plot(np.linspace(self.avg_time, self.avg_time, 100), c= 'blue', label = 'Average Time') # plot average time
        
        ax.set_xticks([1, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100])
        plt.title('Approach Times')
        plt.ylabel('Seconds')
        plt.savefig('/root/data/time_vis_noleg.png')
        ax.legend(loc="lower right")
        # plt.savefig('/root/data/{}/time_vis.png'.format(self.name))
        plt.savefig('/root/data/time_vis.png')
        plt.show()
    # copied (modified) from https://matplotlib.org/stable/gallery/lines_bars_and_markers/scatter_hist.html#sphx-glr-gallery-lines-bars-and-markers-scatter-hist-py
    def angle_vis(self, outlier): # outlier is a boolean: do you want the vis to include outliers?
        x = []
        y = []
        badx = []
        bady = []
        failx = []
        faily = []
        for i in range (len(self.vectors)):
            coord = (self.vectors[i])[1:(len(self.vectors[i])-1)].split(' ') # remove spaces and brackets
            while '' in coord:
                coord.remove('')
            if self.results[i] == 'success':
                if eval(self.angles[i]) <= 5:
                    x.append(float(coord[0]))
                    y.append(float(coord[1]))
                else:
                    badx.append(float(coord[0]))
                    bady.append(float(coord[1]))
            elif outlier == True:
                failx.append(float(coord[0]))
                faily.append(float(coord[1]))
        
        # definitions for the axes
        left, width = 0.1, 0.65
        bottom, height = 0.1, 0.65
        spacing = 0.005

        rect_scatter = [left, bottom, width, height]
        rect_histx = [left, bottom + height + spacing, width, 0.2]
        rect_histy = [left + width + spacing, bottom, 0.2, height]

        # start with a square Figure
        fig = plt.figure(figsize=(8, 8))

        # plot
        ax = fig.add_axes(rect_scatter)
        ax_histx = fig.add_axes(rect_histx, sharex=ax)
        ax_histy = fig.add_axes(rect_histy, sharey=ax)

        # use the previously defined function
        scatter_hist(x, y, badx, bady, failx, faily, ax, ax_histx, ax_histy, outlier, 30, 'steelblue', 'skyblue', 'red')
        if outlier == True:
            plt.savefig('/root/data/{}/outlier_angles.png'.format(self.name))
        else:
            plt.savefig('/root/data/{}/angle_vis.png'.format(self.name), bbox_inches='tight')
        plt.show()
    
    def start_vis(self):
            startx = []
            starty = []
            fstartx = []
            fstarty = []
            for i in range(len(self.start_coords)):
            # for coord in self.start_coords:
                coord = self.start_coords[i]
                if self.results[i] == "fail":
                    x = coord[1:21]
                    x = x.replace(",",' ')
                    x=eval(x)
                    fstartx.append(x)
                    y = coord[22:41]
                    y = y.replace(",",' ')
                    y= eval(y)
                    fstarty.append(y)
                else:
                    x = coord[1:21]
                    x = x.replace(",",' ')
                    x=eval(x)
                    startx.append(x)
                    y = coord[22:41]
                    y = y.replace(",",' ')
                    y= eval(y)
                    starty.append(y)
            fig,ax = plt.subplots(1)
            ax.scatter(startx, starty, 30, c='skyblue', label = 'Success')
            ax.scatter(fstartx, fstarty, 30, c='red', marker = 'X', label = 'Fail')
            ax.set_xticks([-.10, -.05, 0, .05, .10])
            plt.savefig('/root/data/start_vis_noleg.png')
            ax.legend(loc="lower right")

            # plt.savefig('/root/data/{}/start_vis.png'.format(self.name), bbox_inches='tight')
            plt.savefig('/root/data/start_vis.png')
            plt.show()

# creates and saves all visualizations
# spreadsheet is the location of the csv file (/root/data/...100.csv)
def visualizations(spreadsheet):
    data = DataVis(str(spreadsheet))
    data.time_vis()
    #data.angle_vis(True)
    #data.angle_vis(False)
    #data.start_vis()
    
def occlusion_levels(light75, light60, med75, med60, heavy75, heavy60, veryheavy75, veryheavy60):

    # Numbers of pairs of bars you want
    N = 4

    # Data on X-axis ------

    # Specify the values of 75% confidence success (height)
    conf75 = (light75, med75, heavy75, veryheavy75)
    # Specify the values of 60% condifence success (height)
    conf60 = (light60, med60, heavy60, veryheavy60)

    # Position of bars on x-axis
    ind = np.arange(N)

    # Figure size
    plt.figure(figsize=(10,5))

    # Width of a bar 
    width = 0.3       

    # Plotting
    plt.bar(ind, conf75 , width, label='75% Confidence Identification Success', color = 'skyblue')
    plt.bar(ind + width, conf60, width, label='60% Confidence Identification Success', color = 'blue')

    plt.xlabel('Occlusion Level')
    plt.ylabel('Percent Success')
    plt.title('Apple Identification Success Rates')

    # xticks()
    # First argument - A list of positions at which ticks should be placed
    # Second argument -  A list of labels to place at the given locations
    plt.xticks(ind + width / 2, ('Light', 'Medium', 'Heavy', 'Very Heavy'))
    
    def autolabel(rects):
        """Attach a text label above each bar in *rects*, displaying its height."""
        for rect in rects:
            height = rect.get_height()
            ax.annotate('{}'.format(height),
                        xy=(rect.get_x() + rect.get_width() / 2, height),
                        xytext=(0, 3),  # 3 points vertical offset
                        textcoords="offset points",
                        ha='center', va='bottom')

    # Finding the best position for legends and putting it
    plt.legend(loc='best')
    plt.show()
 
def occlusion_graph(light75, light60, med75, med60, heavy75, heavy60, veryheavy75, veryheavy60):

    labels = ['Light', 'Medium', 'Heavy', 'Very Heavy']
    # Specify the values of 75% confidence success (height)
    conf75 = (light75, med75, heavy75, veryheavy75)
    # Specify the values of 60% condifence success (height)
    conf60 = (light60, med60, heavy60, veryheavy60)
    filler = 110 # to make space for the legend

    x = np.arange(len(labels))  # the label locations
    width = 0.35  # the width of the bars

    fig, ax = plt.subplots()
    filler = ax.bar(x, filler, color = 'white')
    rects1 = ax.bar(x - width/2, conf75, width, label='75% Confidence Threshold', color = 'skyblue')
    rects2 = ax.bar(x + width/2, conf60, width, label='60% Confidence Threshold', color = 'blue')

    # Add some text for labels, title and custom x-axis tick labels, etc.
    ax.set_ylabel('Percent Success')
    ax.set_title('Apple Identification Success Rates')
    ax.set_xticks(x)
    ax.set_xticklabels(labels)
    ax.legend()

    def autolabel(rects):
        """Attach a text label above each bar in *rects*, displaying its height."""
        for rect in rects:
            height = rect.get_height()
            ax.annotate('{}'.format(height),
                        xy=(rect.get_x() + rect.get_width() / 2, height),
                        xytext=(0, 3),  # 3 points vertical offset
                        textcoords="offset points",
                        ha='center', va='bottom')


    autolabel(rects1)
    autolabel(rects2)

    fig.tight_layout()

    plt.show()
    
 
# visualizations('/root/catkin_ws/src/applevision_rospkg/bin/stage2data.csv')
# visualizations('/root/data/2023-03-08 17:01/stage1_10.csv')

#occlusion_graph(100, 100, 100, 100, 85, 90, 60, 75)