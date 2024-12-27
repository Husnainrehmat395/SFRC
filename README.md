# SFRC
# coding=UTF-8
from abaqusConstants import *
from caeModules import *
import numpy as np
import math

def distance_between_lines(p1, q1, p2, q2):
    """
    Calculates the shortest distance between two line segments in 3D space.
    Args:
    - p1, q1: Start and end points of the first line segment.
    - p2, q2: Start and end points of the second line segment.
    Returns:
    - Minimum distance between the two line segments.
    """
    def norm(v):
        return np.linalg.norm(v)

    def dot(v1, v2):
        return np.dot(v1, v2)

    u = np.array(q1) - np.array(p1)
    v = np.array(q2) - np.array(p2)
    w = np.array(p1) - np.array(p2)

    a = dot(u, u)
    b = dot(u, v)
    c = dot(v, v)
    d = dot(u, w)
    e = dot(v, w)

    denom = a * c - b * b
    if denom == 0:  # Lines are parallel
        return norm(np.cross(w, u)) / norm(u)

    s = (b * e - c * d) / denom
    t = (a * e - b * d) / denom

    s = max(0, min(1, s))
    t = max(0, min(1, t))

    closest_point_1 = np.array(p1) + s * u
    closest_point_2 = np.array(p2) + t * v

    return norm(closest_point_1 - closest_point_2)


def generate_random_fiber(existing_fibers, min_distance):
    """
    Generates a random fiber (start and end points) such that it stays within the concrete beam
    and does not overlap with existing fibers.
    """
    while True:
        # Generate a random starting point within the concrete beam
        ipoint = (np.random.uniform(-H / 2, H / 2),
                  np.random.uniform(-B / 2, B / 2),
                  np.random.uniform(-L / 2, L / 2))

        # Generate a random orientation
        Psi = np.random.uniform(0, 2 * math.pi)
        Theta = np.random.uniform(0, math.pi)  # Limit Theta to [0, π] for full 3D space

        # Compute the endpoint of the fiber
        x = ipoint[0] + fiberlength * math.sin(Theta) * math.cos(Psi)
        y = ipoint[1] + fiberlength * math.sin(Theta) * math.sin(Psi)
        z = ipoint[2] + fiberlength * math.cos(Theta)
        epoint = (x, y, z)

        # Check if the entire fiber is within the concrete beam
        if (-H / 2 <= epoint[0] <= H / 2 and
            -B / 2 <= epoint[1] <= B / 2 and
            -L / 2 <= epoint[2] <= L / 2):

            # Check for minimum distance from existing fibers
            is_valid = True
            for fiber in existing_fibers:
                ip_existing, ep_existing = fiber
                dist = distance_between_lines(ipoint, epoint, ip_existing, ep_existing)
                if dist < min_distance:
                    is_valid = False
                    break

            if is_valid:
                return ipoint, epoint


L = 150.0  # Length of the concrete beam
H = 150.0  # Height of the concrete beam
B = 750.0  # Width of the concrete beam
fiberlength = 10  # Length of each fiber
fibernumber = 100  # Number of fibers
min_distance = 2.5  # Minimum distance between fibers

# Generate all fibers with random positions and orientations
fibers = []
for _ in range(fibernumber):
    ipoint, epoint = generate_random_fiber(fibers, min_distance)
    fibers.append((ipoint, epoint))

myModel = mdb.Model(name='Model-1')

# Create fibers as solid cylinders
for j in range(fibernumber):
    # Create a sketch for the fiber cylinder
    s = myModel.ConstrainedSketch(name='Fiber_Sketch_' + str(j), sheetSize=200.0)
    s.setPrimaryObject(option=STANDALONE)

    # Calculate fiber start and end points
    ipoint = fibers[j][0]
    epoint = fibers[j][1]

    # Calculate the direction vector of the cylinder
    direction = np.array(epoint) - np.array(ipoint)
    length = np.linalg.norm(direction)
    direction_unit = direction / length

    # Define the circle cross-section of the cylinder
    s.CircleByCenterPerimeter(center=(0.0, 0.0), point1=(fiberlength / 8.0, 0.0))  # Adjusted circle radius

    # Create a 3D part and extrude the cylinder along its length
    part_name = 'Fiber_' + str(j)
    fiber_part = myModel.Part(name=part_name, dimensionality=THREE_D, type=DEFORMABLE_BODY)
    fiber_part.BaseSolidExtrude(sketch=s, depth=length)
    s.unsetPrimaryObject()

    # Position and orient the fiber correctly
    a = myModel.rootAssembly
    instance_name = 'Fiber_Instance_' + str(j)
    fiber_instance = a.Instance(name=instance_name, part=fiber_part, dependent=ON)

    # Translate the fiber to the starting point
    a.translate(instanceList=(instance_name,), vector=ipoint)

    # Rotate the fiber to align with its direction vector
    axis = (0.0, 0.0, 1.0)  # Default extrusion is along the Z-axis
    angle = math.degrees(math.acos(direction_unit[2]))  # Angle with the Z-axis
    rotation_axis = (-direction_unit[1], direction_unit[0], 0.0)  # Rotation axis perpendicular to Z and direction
    if np.linalg.norm(rotation_axis) > 0:  # Avoid rotation if already aligned
        a.rotate(instanceList=(instance_name,),
                 axisPoint=ipoint, axisDirection=rotation_axis, angle=angle)

# Merge only the fibers into a single part for topology optimization
a.InstanceFromBooleanMerge(name='FibersOnly',
                           instances=tuple(a.instances.values()),
                           keepIntersections=ON, originalInstances=SUPPRESS)

# Create the concrete part
s = myModel.ConstrainedSketch(name='Concrete_Sketch', sheetSize=1000.0)
s.rectangle(point1=(-H / 2, -B / 2), point2=(H / 2, B / 2))
concrete_part = myModel.Part(name='Concrete', dimensionality=THREE_D, type=DEFORMABLE_BODY)
concrete_part.BaseSolidExtrude(sketch=s, depth=L)
s.unsetPrimaryObject()

# Add the concrete part to the assembly
concrete_instance = a.Instance(name='Concrete_Instance', part=concrete_part, dependent=ON)

# Translate the concrete part to align with the fibers
a.translate(instanceList=('Concrete_Instance',), vector=(0.0, 0.0, -L / 2))

# Merge fibers and concrete into a single part for topology optimization
a.InstanceFromBooleanMerge(name='FiberReinforcedConcrete',
                           instances=(a.instances['Concrete_Instance'], a.instances['FibersOnly']),
                           keepIntersections=ON, originalInstances=SUPPRESS)
