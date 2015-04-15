import numpy as np
import pylab

def GaussianSmooth(list, degree=5):

     window=degree*2-1

     weight=np.array([1.0]*window)  

     weightGauss=[]  

     for i in range(window):

         i=i-degree+1  

         frac=i/float(window)  

         gauss=1/(np.exp((4*(frac))**2))  

         weightGauss.append(gauss)  

     weight=np.array(weightGauss)*weight  

     smoothed=[0.0]*(len(list)-window)  

     for i in range(len(smoothed)):  

         smoothed[i]=sum(np.array(list[i:i+window])*weight)/sum(weight)  
         print smoothed

     return smoothed  
data=[0]*30
data[15]=1
GaussianSmooth(data)

# # Plotting stuff

# ylab.figure(figsize=(550/80,700/80))  

# #original data
# pylab.subplot(2,1,1)  

# p1=pylab.plot(data,".k")  

# p1=pylab.plot(data,"-k")  

# a=pylab.axis()  

# pylab.axis([a[0],a[1],-.1,1.1])  

# pylab.text(2,.8,"raw data",fontsize=14)  

# # smooth curve
# pylab.subplot(2,1,2)  

# p1=pylab.plot(GaussianSmooth(data),".k")  

# p1=pylab.plot(GaussianSmooth(data),"-k")  

# pylab.axis([a[0],a[1],-.1,.4])  

# pylab.text(2,.3,"moving gaussian",fontsize=14)  

# pylab.savefig("smooth.png",dpi=80) 