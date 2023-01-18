# Library functionality
from math import radians, degrees, cos, sin, pow, sqrt, atan, tan, pi
from sys import exit

# Pile information
class pile_info:
    def __init__(self, helix_dia, helix_num, shaft_length):
        self.helix_dia = helix_dia
        self.helix_num = helix_num
        self.shaft_length = shaft_length

# Jacking force information
class jacking_force_info:
    def __init__(self, jacking_force_x, jacking_force_y):
        self.jacking_force_x = jacking_force_x
        self.jacking_force_y = jacking_force_y

# Defined values
jacking_force = 37000.0
frame_weight = 5000.0
batter_angle = 45.0
launch_angle = 12.0
shaft_outer_dia = 3.5
friction_coefficient = 0.65
pile_num = 4
bearing_capacity = 9
soil_cohesion = 2.4
soil_adhesion = (2 / 3) * soil_cohesion
ft_in_conv = 12

def main():
    # Pile characteristics
    helix_dia = float(input("Helix diameter (inches): "))
    shaft_length = float(input("Shaft length (ft): "))
    helix_num = int(input("Number of helices: "))
    pile = pile_info(helix_dia, helix_num, shaft_length)
    # Force characteristics
    force_x = jacking_force * cos(radians(launch_angle))
    force_y = jacking_force * sin(radians(launch_angle))
    force = jacking_force_info(force_x, force_y)
    # Obtain values from helper functions
    shaft_plate_depths = plate_depths_shaft(pile)
    surface_plate_depths = plate_depths_surface(shaft_plate_depths)
    forces_list = pile_forces(force)
    axial_sf = axial_safety_factor(forces_list, shaft_plate_depths, pile)
    lateral_sf = lateral_safety_factor(forces_list, surface_plate_depths, pile)
    # Check geometric constraint
    first_plate_depth = shaft_plate_depths[0]
    helix_radius_limit = first_plate_depth * tan(radians(batter_angle))
    helix_radius = pile.helix_dia / 2
    if helix_radius_limit < helix_radius:
        print("Input valid dimensions")
        exit(1)
    # Display safety factors
    print(f"Axial safety factor: {axial_sf}")
    print(f"Lateral safety factor: {lateral_sf}")
    exit(0)


# Given a number of helicals, find their respective depths
def plate_depths_shaft(pile):
    plate_depths = []
    plate_depth = 0
    shaft_length_inches = pile.shaft_length * ft_in_conv
    even_spacing = shaft_length_inches / pile.helix_num
    for plate in range(pile.helix_num + 1):
        if plate != 0:
            plate_depth = plate_depth + even_spacing
            plate_depths.append(round(plate_depth, 4))

    return plate_depths

def plate_depths_surface(shaft_plate_depths):
    plate_depths_surface = []
    for plate in range(len(shaft_plate_depths)):
        plate_depth_surface = shaft_plate_depths[plate] * sin(radians(batter_angle))
        plate_depths_surface.append(plate_depth_surface)
    
    return plate_depths_surface

# Calculate pile forces
def pile_forces(force):
    pile_forces = {
        # Total axial force on helical piles
        "total_axial": 0,
        # Total lateral force on helical piles
        "total_lateral": 0,
        # Axial force on each pile
        "axial": 0,
        # Lateral force on each pile
        "lateral": 0
    }

    # Resultant force
    resultant = sqrt(pow(force.jacking_force_x, 2) + pow(force.jacking_force_y, 2))

    # Angle of force on helical piles from horizontal, degrees
    phi = degrees(atan(force.jacking_force_y / force.jacking_force_x))
    # Angle between resultant force and piles, degrees
    gamma = phi + batter_angle
    gamma_radians = radians(gamma)

    # Update dictionary
    pile_forces["total_axial"] = resultant * cos(gamma_radians)
    pile_forces["total_lateral"] = resultant * sin(gamma_radians)
    pile_forces["axial"] = pile_forces["total_axial"] / pile_num
    pile_forces["lateral"] = pile_forces["total_lateral"] / pile_num

    # Return the pile forces to main
    return pile_forces

def axial_safety_factor(forces_list, shaft_plate_depths, pile):
    # Axial force on each pile
    pile_axial_force = forces_list["axial"]
    # Calculate helix area
    helix_area = pi * pow(pile.helix_dia / 2, 2)
    # Calculate ultimate_bearing_pressure
    ultimate_bearing_pressure = 9 * soil_cohesion
    # Calculate axial capacity
    axial_capacity = (pile.helix_num * ultimate_bearing_pressure * helix_area) + soil_adhesion * (shaft_plate_depths[-1]) * pi * shaft_outer_dia
    # Calculate axial safety factor
    axial_safety_factor = axial_capacity / pile_axial_force

    return axial_safety_factor


def axial_safety_factor(forces_list, shaft_plate_depths, pile):
    # Axial force on each pile
    pile_axial_force = forces_list["axial"]
    # Calculate helix area
    helix_area = pi * pow(pile.helix_dia / 2, 2)
    # Calculate ultimate_bearing_pressure
    ultimate_bearing_pressure = 9 * soil_cohesion
    # Calculate axial capacity
    axial_capacity = (pile.helix_num * ultimate_bearing_pressure * helix_area) + soil_adhesion * (shaft_plate_depths[-1]) * pi * shaft_outer_dia
    # Calculate axial safety factor
    axial_safety_factor = axial_capacity / pile_axial_force

    return axial_safety_factor


def lateral_safety_factor(forces_list, surface_plate_depths, pile):
    length_above_ground = 0
    shaft_length_inches = pile.shaft_length * ft_in_conv
    general_case = bearing_capacity * soil_cohesion
    helix_radius = pile.helix_dia / 2
    shaft_outer_radius = shaft_outer_dia / 2

    bearing_resistance = []
    uplift_resistance = []

    for plate in range(len(surface_plate_depths)):
        if surface_plate_depths[plate] < 5 * pile.helix_dia:
            bearing_resistance_plate = ((bearing_capacity - 6.2)/(5 * pile.helix_dia) * surface_plate_depths[plate] + 6.2) * soil_cohesion
            bearing_resistance.append(bearing_resistance_plate)
        else:
            bearing_resistance.append(general_case)
        if surface_plate_depths[plate] < 2 * pile.helix_dia:
            uplift_resistance_plate = (bearing_capacity * surface_plate_depths[plate] * soil_cohesion)/(2 * pile.helix_dia)
            uplift_resistance.append(uplift_resistance_plate)
        else:
            uplift_resistance.append(general_case)

    R = helix_radius
    r = shaft_outer_radius
    K_1 = 2 * pi * (pow(R, 2) - pow(r,2))
    K_2 = 2 * (pow(R, 4) / 4 + pow(r, 4) / 3 - pow(r, 4) / 4 - r * pow(R, 3) / 3) / (R - r)

    L = shaft_length_inches
    e = length_above_ground
    d = shaft_outer_dia
    even_case = pile.helix_num // 2

    expr1 = 10.5 * e * pow(d, 2) + 9 * (L - e) * e * d + 10.5 * pow(d, 3) + 4.5 * d * pow((L - e), 2)
    expr2 = K_2 * (sum(bearing_resistance) + sum(uplift_resistance))
    if pile.helix_num % 2 == 0:
        expr3 = soil_cohesion * K_1 * soil_adhesion * (sum(surface_plate_depths[even_case:]) - sum(surface_plate_depths[:-even_case]))
    else:
        odd_case = pile.helix_num // 2 + 1
        expr3 = soil_cohesion * K_1 * soil_adhesion * (sum(surface_plate_depths[odd_case:]) - sum(surface_plate_depths[:-odd_case]))
    M = expr1 + (expr2 / expr3)

    X = -e + (sqrt(324 * d ** (2 * e ** 2) + 36 * d * M) / (18 * d))

    ultimate_lateral_load = soil_cohesion * d * (18 * X - 10.5 * d - 9 * (L - e))
    total_ultimate_lateral_load = ultimate_lateral_load * pile_num
    lateral_sf = total_ultimate_lateral_load / forces_list["total_lateral"]

    return lateral_sf


if __name__ == "__main__":
    main()