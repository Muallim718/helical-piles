# Import the math library
import math


# English Engineering units (lb and inches)
helical_piles = 4
concrete_blocks = 20
shaft_outer_diameter = 3.5
safety_factor = 1.5
applied_force = 120000
reaction_frame_weight = 5000
concrete_weight = 2200
# Soil on concrete
friction_coefficient = 0.65
# Bearing capacity factor for cohesive component of soil
bearing_capacity = 9

def main():
    # Get launch angle from user's input
    launch_agnle = float(input("Launch angle: "))
    # Find batter angle
    batter_angle = 45 - launch_agnle
    # Find jacking force
    jacking_force = safety_factor * applied_force
    # Save launch agnle in radians
    launch_angle_radians = math.radians(launch_agnle)
    # Find x and y components of jacking force
    jacking_force_x = jacking_force * math.cos(launch_angle_radians)
    jacking_force_y = jacking_force * math.sin(launch_angle_radians)
    # Get list of pile forces
    pile_forces_list = pile_forces(jacking_force_x, jacking_force_y, batter_angle)
    # Get min helix diameter
    min_helix_diameter = helix_diameter(pile_forces_list)
    shaft_length = optimatal_length(min_helix_diameter, pile_forces_list)
    print(shaft_length / 12)


def pile_forces(jacking_force_x, jacking_force_y, batter_angle):
    # Initialize pile forces dictionary
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

    # Horizontal friction force of concrete blocks
    F_cb = concrete_blocks * concrete_weight * friction_coefficient;
    # Horizontal force on helical pile
    F_hpx = jacking_force_x - F_cb
    # Vertical force on helical pile
    F_hpy = jacking_force_y - reaction_frame_weight
    # Resultant force
    F_hp = math.sqrt((F_hpx ** 2) + (F_hpy ** 2))

    # Angle of force on helical piles from horizontal, degrees
    phi = math.degrees(math.atan(F_hpy / F_hpx))
    # Angle between resultant force and piles, degrees
    gamma = phi + batter_angle
    # Gamma in radians
    gamma_radians = math.radians(gamma)

    # Update dictionary
    pile_forces["F_hpat"] = F_hp * math.cos(gamma_radians)
    pile_forces["F_hplt"] = F_hp * math.sin(gamma_radians)
    pile_forces["F_hpa"] = pile_forces["F_hpat"] / helical_piles
    pile_forces["F_hpl"] = pile_forces["F_hplt"] / helical_piles

    # Return the pile forces to main
    return pile_forces


def helix_diameter(pile_forces):
    soil_cohesion = 2.4
    # Number of helices for each pile
    helix_amount = 2

    # Calculate min_helix_diameter
    helix_axial_force = pile_forces["F_hpa"] / helix_amount
    helix_area = helix_axial_force / (soil_cohesion * bearing_capacity)
    helix_diameter = 2 * math.sqrt(helix_area / math.pi)

    return helix_diameter


def optimatal_length(helix_diameter, pile_forces):
    soil_shear_strength = 2.4
    soil_adhesion_factor = 0.5
    length_above_ground = 0
    helix_radius = helix_diameter / 2
    shaft_outer_radius = shaft_outer_diameter / 2

    embedded_shaft_length = 60
    total_ultimate_lateral_load = 0
    F_hplt = pile_forces["F_hplt"]

    K_1 = 2 * math.pi *(math.pow(helix_radius, 2) - math.pow(shaft_outer_radius, 2));
    top1 = math.pow(helix_radius, 4) / 4 + math.pow(shaft_outer_radius, 4) / 3
    top2 = math.pow(shaft_outer_radius, 4) / 4
    top3 = shaft_outer_radius * math.pow(helix_radius, 3) / 3
    top = 2 * (top1 - top2 - top3)
    bottom = helix_radius - shaft_outer_radius
    K_2 = top / bottom

    while total_ultimate_lateral_load < F_hplt:
        embedded_shaft_length = embedded_shaft_length + 6
        first_plate_depth = embedded_shaft_length / 2
        second_plate_depth = embedded_shaft_length

        bearing_resistance_case_plate1 = ((bearing_capacity - 6.2)/(5 * helix_diameter) * (first_plate_depth + 6.2)) * soil_shear_strength
        bearing_resistance_case_plate2 = ((bearing_capacity - 6.2)/(5 * helix_diameter) * (second_plate_depth + 6.2)) * soil_shear_strength
        uplift_resistance_case_plate1 = (bearing_capacity * first_plate_depth * soil_shear_strength)/(2 * helix_diameter)
        uplift_resistance_case_plate2 = (bearing_capacity * second_plate_depth * soil_shear_strength)/(2 * helix_diameter)
        general_case = bearing_capacity * soil_shear_strength
        
        if first_plate_depth < 5 * helix_diameter:
            pb_1 = bearing_resistance_case_plate1
        else: 
            pb_1 = general_case
        
        if second_plate_depth < 5 * helix_diameter: 
            pb_2 = bearing_resistance_case_plate2
        else:
            pb_2 = general_case

        if first_plate_depth < 2 * helix_diameter:
            pu_1 = uplift_resistance_case_plate1
        else:
            pu_1 = general_case
        
        if second_plate_depth < 2 * helix_diameter:
            pu_2 = uplift_resistance_case_plate2
        else:
            pu_2 = general_case

        expr1 = 10.5 * length_above_ground * math.pow(shaft_outer_diameter, 2) + 9 * embedded_shaft_length * length_above_ground * shaft_outer_diameter
        expr2 = 10.5 * math.pow(shaft_outer_diameter, 3) + 4.5 * shaft_outer_diameter * math.pow(embedded_shaft_length, 2)
        expr3 = K_2 * (pb_1 + pb_2 + pu_1 + pu_2) / soil_shear_strength
        expr4 = K_1 * soil_adhesion_factor * (second_plate_depth - first_plate_depth)
        M = expr1 + expr2 + expr3 + expr4

        expr5 = -length_above_ground + math.sqrt(324 * shaft_outer_diameter ** (2 * length_above_ground ** 2) + 36 * shaft_outer_diameter * M)
        expr6 = 18 * shaft_outer_diameter
        X = expr5 / expr6

        ultimate_lateral_load = soil_shear_strength * shaft_outer_diameter * (18 * X - 10.5 * shaft_outer_diameter - 9 * embedded_shaft_length)
        total_ultimate_lateral_load = ultimate_lateral_load * helical_piles
    return embedded_shaft_length
    

main()