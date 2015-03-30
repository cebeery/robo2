import numpy as np

def rotation(angles):
	phi = angles[2]
	theta = angles[1]
	psi = angles[0]

	R = np.empty((3,3,))
	
	R[0] = (np.cos(phi)*np.cos(theta), np.cos(phi) * np.sin(theta) * np.sin(psi) - np.cos(psi) * np.sin(phi), np.sin(phi) * np.sin(psi) + np.cos(phi) * np.cos(psi) * np.sin(theta))
	R[1] = (np.cos(theta) * np.sin(phi), np.cos(phi) * np.cos(psi) + np.sin(phi) * np.sin(theta) * np.sin(psi),np.cos(psi) * np.sin(phi) * np.sin(theta) - np.cos(phi) * np.sin(psi))
	R[2] = (-np.sin(theta),np.cos(theta)*np.sin(psi),np.cos(theta)*np.cos(phi))

	return R


print rotation((2,4,5))