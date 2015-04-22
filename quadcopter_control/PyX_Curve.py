from pyx import *

def Pyx_smooth(list, degree= 1):
	c = canvas.canvas()

	for i in range(len(list)):
		if i ==0:
			p=path.line(list[i][0],list[i][1],list[i+1][0],list[i+1][1])
		else:
			p.append(path.lineto(list[i][0],list[i][1]))

	c.stroke(p)
	c.stroke(p, [deformer.smoothed(1.0), color.rgb.red])
	# c.writeEPSfile("smoothed")
	# c.writePDFfile("smoothed")
	ps = deformer.smoothed(2.0).deform(p)

	return ps



# Pyx_smooth([[0,0],[2,2],[3,2],[3,4]])
