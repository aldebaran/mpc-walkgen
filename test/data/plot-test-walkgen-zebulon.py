import pylab
import numpy
import matplotlib.pyplot as plt

def readFile(src, header):
    return numpy.genfromtxt(src, dtype=None, names=header, delimiter=None)

def plotTime(data, name, newFig=True, splot=111):
    dt = 0.02
    duration = 0.0
    durationList = []
    for i in range(len(data[data.dtype.names[0]])):
        durationList.append(duration)
        duration += dt
    if (newFig):
        fig = plt.figure(figsize=(11,11))
    pylab.subplot(splot)
    color=['b','g','r','c','m','k']
    pylab.title(name)
    for i in [0,1,2,3,4,5]:
        if len(data.dtype.names)>i:
            pylab.plot(durationList, data[data.dtype.names[i]], color[i], label=data.dtype.names[i])
    pylab.legend(data.dtype.names)
    pylab.xlabel('t [s]');
    pylab.ylabel('[m]');

def plotTrajectory(data0, data1, name, newFig=True, color=0):
    if (newFig):
        fig = plt.figure(figsize=(11,11))
    colorTab=['b','g','r','c','m','k']
    pylab.title(name)
    pylab.plot(data0[data0.dtype.names[0]], data1[data1.dtype.names[0]], colorTab[color])
    pylab.xlabel('x [m]');
    pylab.ylabel('y [m]');

def plotDiffTrajectory(data0, data0r, data1, data1r, name, newFig=True, color=0):
    if (newFig):
        fig = plt.figure(figsize=(11,11))
    colorTab=['b','g','r','c','m','k']
    pylab.title(name)
    pylab.plot(data0[data0.dtype.names[0]]-data0r[data0r.dtype.names[0]], data1[data1.dtype.names[0]]-data1r[data1r.dtype.names[0]], colorTab[color])
    pylab.xlabel('x [m]');
    pylab.ylabel('y [m]');


if __name__ == '__main__':

    header1 = ['Position', 'Velocity', 'Acceleration']
    header2 = ['Position']
    src = './'
    testName = ''

    CoM_X = readFile(src+testName+"CoM_X.data",header1);
    CoM_Y = readFile(src+testName+"CoM_Y.data",header1);
    CoM_Yaw = readFile(src+testName+"CoM_Yaw.data",header1);
    CoP_X = readFile(src+testName+"CoP_X.data",header2);
    CoP_Y = readFile(src+testName+"CoP_Y.data",header2);
    BAS_X = readFile(src+testName+"BAS_X.data",header1);
    BAS_Y = readFile(src+testName+"BAS_Y.data",header1);


    plotTime(CoM_X, "CoM X", True, 311)
    plotTime(CoM_Y, "CoM Y", False, 312)
    plotTime(CoM_Yaw, "CoM Yaw", False, 313)

    plotTime(CoP_X, "CoP X", True, 211)
    plotTime(CoP_Y, "CoP Y", False, 212)

    plotTime(BAS_X, "Base X", True, 211)
    plotTime(BAS_Y, "Base Y", False, 212)

    plotTrajectory(CoM_X, CoM_Y, "")
    plotTrajectory(CoP_X, CoP_Y, "", False, 1)
    plotTrajectory(BAS_X, BAS_Y, "CoM,CoP and Base trajectory", False, 2)
    pylab.legend(('CoM', 'CoP', 'Base'))

    plotDiffTrajectory(CoM_X, BAS_X, CoM_Y, BAS_Y, "")
    plotDiffTrajectory(CoP_X, BAS_X, CoP_Y, BAS_Y, "Relative position between CoP/CoM and base", False, 1)
    pylab.legend(('CoM', 'CoP', ))

    pylab.show()
