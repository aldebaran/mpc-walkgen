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
    CoP_X = readFile(src+testName+"CoP_X.data",header2);
    CoP_Y = readFile(src+testName+"CoP_Y.data",header2);
    LF_X  = readFile(src+testName+"LF_X.data",header1);
    LF_Y  = readFile(src+testName+"LF_Y.data",header1);
    RF_X  = readFile(src+testName+"RF_X.data",header1);
    RF_Y  = readFile(src+testName+"RF_Y.data",header1);

    plotTime(CoM_X, "CoM X", True, 211)
    plotTime(CoM_Y, "CoM Y", False, 212)

    plotTime(CoP_X, "CoP X", True, 211)
    plotTime(CoP_Y, "CoP Y", False, 212)

    plotTime(LF_X, "Left foot X", True, 221)
    plotTime(LF_Y, "Left foot Y", False, 222)
    plotTime(RF_X, "Right foot X", False, 223)
    plotTime(RF_Y, "Right foot Y", False, 224)

    plotTrajectory(CoM_X, CoM_Y, "")
    plotTrajectory(CoP_X, CoP_Y, "", False, 1)
    plotTrajectory(LF_X, LF_Y, "", False, 2)
    plotTrajectory(RF_X, RF_Y, "CoM,CoP and feet trajectory", False, 3)
    pylab.legend(('CoM', 'CoP', 'Base'))


    pylab.show()
