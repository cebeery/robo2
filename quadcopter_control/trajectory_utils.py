from pyx import path, unit
import copy

unit.set(defaultunit="m")
metersPerPoint = 0.00035277

class Trajectory:
	def __init__(self):
		self.orients = []
		self.times = []
		self.positions = [(0,0), (1,0), (2,0), (3,0), (4,0), (5,0), (6,0), (7,0), (8,0), (9,0)]

	def arclen(self):		
		# so we can use pop() w/o modifying self.positions
		positions = copy.copy(self.positions)
		positions.reverse()

		p = None

		# BEZIER CURVES
		while len(positions) > 4: # need at least 4 points to make bezier curve
			# make a bezier curve
			points = []
			for _ in range(3):
				points += positions.pop()
			points += positions[-1] # so next curve will start at this point

			if p:
				p += path.curve(*points)
			else:
				p = path.curve(*points)


		# LINES
		while len(positions) > 1: # use up rest of points with lines
			points = []
			points += positions.pop()
			points += positions[-1]

			if p:
				p += path.line(*points)
			else:
				p = path.line(*points)


		# ARC LENGTH
		print p.arclen_pt()*metersPerPoint

			

def test():
	t = Trajectory()
	t.arclen()