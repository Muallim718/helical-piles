from math import radians, degrees, cos, sin, pow, sqrt, atan, tan, pi
from sys import exit


# English Engineering units (lb and inches)
helical_piles = 4
jacking_force = 37000
reaction_frame_weight = 5000
# Soil on concrete
friction_coefficient = 0.65
# Bearing capacity factor for cohesive component of soil
bearing_capacity = 9
soil_cohesion = 2.4
soil_adhesion = (2 / 3) * soil_cohesion
ft_in_conversion = 12
 

def main():
    # Prompt user for dimensions
    helix_diameter = float(input("Helix diameter (inches): "))
    # shaft_outer_diameter = float(input("Outer shaft diameter (inches): "))
    shaft_outer_diameter = 3.5
    # launch_angle = float(input("Launch angle (degrees): "))
    launch_angle = 12.0
    shaft_length = float(input("Shaft length (ft): "))
    helices_number = int(input("Number of helices: "))

    # Convert shaft length into inches
    shaft_length_inches = ft_in_conversion * shaft_length
    # Calculate angles
    batter_angle = 45
    launch_angle_radians = radians(launch_angle)

    # Calculate jacking force components
    jacking_force_x = jacking_force * cos(launch_angle_radians)
    jacking_force_y = jacking_force * sin(launch_angle_radians)

    # Get values from functions
    shaft_plate_depths = plate_depths_shaft(shaft_length_inches, helices_number)
    surface_plate_depths = plate_depths_surface(shaft_plate_depths, batter_angle)
    pile_forces_list = pile_forces(jacking_force_x, jacking_force_y, batter_angle)
    axial_sf = axial_safety_factor(pile_forces_list, helix_diameter, helices_number, shaft_plate_depths, shaft_outer_diameter)
    lateral_sf = lateral_safety_factor(helix_diameter, pile_forces_list, shaft_outer_diameter, shaft_length_inches, surface_plate_depths, helices_number)
    first_plate_depth = shaft_plate_depths[0]
    helix_radius_limit = first_plate_depth * tan(radians(batter_angle))
    helix_radius = helix_diameter / 2

    if helix_radius_limit < helix_radius:
        print("Input valid dimensions")
        exit(0)
    
    # Let the user know whether their design will fail or not
    print(f"Axial safety factor: {axial_sf}")
    print(f"Lateral safety factor: {lateral_sf}")
    print(pile_forces_list["F_hpat"], pile_forces_list["F_hplt"])
    exit(0)


# Given a number of helicals, find their respective depths
def plate_depths_shaft(shaft_length, helices_number):
    plate_depths = []
    plate_depth = 0
    even_spacing = shaft_length / helices_number
    for plate in range(helices_number + 1):
        if plate != 0:
            plate_depth = plate_depth + even_spacing
            plate_depths.append(round(plate_depth, 4))

    return plate_depths

def plate_depths_surface(shaft_plate_depths, batter_angle):
    plate_depths_surface = []
    for plate in range(len(shaft_plate_depths)):
        plate_depth_surface = shaft_plate_depths[plate] * sin(radians(batter_angle))
        plate_depths_surface.append(plate_depth_surface)
    
    return plate_depths_surface


# Calculate pile forces
def pile_forces(jacking_force_x, jacking_force_y, batter_angle):
    pile_forces = {
        # Total axial force on helical piles
        "F_hpat": 0,
        # Total lateral force on helical piles
        "F_hplt": 0,
        # Axial force on each pile
        "F_hpa": 0,
        # Lateral force on each pile
        "F_hpl": 0
    }

    # Resultant force
    F_hp = sqrt(pow(jacking_force_x, 2) + pow(jacking_force_y, 2))

    # Angle of force on helical piles from horizontal, degrees
    phi = degrees(atan(jacking_force_y / jacking_force_x))
    # Angle between resultant force and piles, degrees
    gamma = phi + batter_angle
    gamma_radians = radians(gamma)

    # Update dictionary
    pile_forces["F_hpat"] = F_hp * cos(gamma_radians)
    pile_forces["F_hplt"] = F_hp * sin(gamma_radians)
    pile_forces["F_hpa"] = pile_forces["F_hpat"] / helical_piles
    pile_forces["F_hpl"] = pile_forces["F_hplt"] / helical_piles

    # Return the pile forces to main
    return pile_forces


def axial_safety_factor(pile_forces, helix_diameter, helices_number, shaft_plate_depths, shaft_outer_diameter):
    # Axial force on each pile
    pile_axial_force = pile_forces["F_hpa"]
    # Calculate helix area
    helix_area = pi * pow(helix_diameter / 2, 2)
    # Calculate ultimate_bearing_pressure
    ultimate_bearing_pressure = 9 * soil_cohesion
    # Calculate axial capacity
    axial_capacity = (helices_number * ultimate_bearing_pressure * helix_area) + soil_adhesion * (shaft_plate_depths[-1]) * pi * shaft_outer_diameter
    # Calculate axial safety factor
    axial_safety_factor = axial_capacity / pile_axial_force

    return axial_safety_factor


def lateral_safety_factor(helix_diameter, pile_forces, shaft_outer_diameter, shaft_length_inches, surface_plate_depths, helices_number):
    length_above_ground = 0
    general_case = bearing_capacity * soil_cohesion
    helix_radius = helix_diameter / 2
    shaft_outer_radius = shaft_outer_diameter / 2

    bearing_resistance = []
    uplift_resistance = []

    for plate in range(len(surface_plate_depths)):
        if surface_plate_depths[plate] < 5 * helix_diameter:
            bearing_resistance_plate = ((bearing_capacity - 6.2)/(5 * helix_diameter) * surface_plate_depths[plate] + 6.2) * soil_cohesion
            bearing_resistance.append(bearing_resistance_plate)
        else:
            bearing_resistance.append(general_case)
        if surface_plate_depths[plate] < 2 * helix_diameter:
            uplift_resistance_plate = (bearing_capacity * surface_plate_depths[plate] * soil_cohesion)/(2 * helix_diameter)
            uplift_resistance.append(uplift_resistance_plate)
        else:
            uplift_resistance.append(general_case)

    R = helix_radius
    r = shaft_outer_radius
    K_1 = 2 * pi * (pow(R, 2) - pow(r,2))
    K_2 = 2 * (pow(R, 4) / 4 + pow(r, 4) / 3 - pow(r, 4) / 4 - r * pow(R, 3) / 3) / (R - r)

    L = shaft_length_inches
    e = length_above_ground
    d = shaft_outer_diameter
    even_case = helices_number // 2

    expr1 = 10.5 * e * pow(d, 2) + 9 * (L - e) * e * d + 10.5 * pow(d, 3) + 4.5 * d * pow((L - e), 2)
    expr2 = K_2 * (sum(bearing_resistance) + sum(uplift_resistance))
    if helices_number % 2 == 0:
        expr3 = soil_cohesion * K_1 * soil_adhesion * (sum(surface_plate_depths[even_case:]) - sum(surface_plate_depths[:-even_case]))
    else:
        odd_case = helices_number // 2 + 1
        expr3 = soil_cohesion * K_1 * soil_adhesion * (sum(surface_plate_depths[odd_case:]) - sum(surface_plate_depths[:-odd_case]))
    M = expr1 + (expr2 / expr3)

    X = -e + (sqrt(324 * d ** (2 * e ** 2) + 36 * d * M) / (18 * d))

    ultimate_lateral_load = soil_cohesion * d * (18 * X - 10.5 * d - 9 * (L - e))
    total_ultimate_lateral_load = ultimate_lateral_load * helical_piles
    lateral_sf = total_ultimate_lateral_load / pile_forces["F_hplt"]

    return lateral_sf


if __name__ == "__main__":
    main()